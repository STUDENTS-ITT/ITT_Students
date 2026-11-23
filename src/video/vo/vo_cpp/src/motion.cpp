#include "vo/motion.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <numeric>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vo/util.hpp"

namespace vo {

using Mat3 = cv::Matx33d;
using V3 = cv::Vec3d;

namespace {

// ---------------------------------------------------------------------------
// Соглашения о системах координат (самое тонкое место порта).
//
// OpenCV decomposeHomographyMat возвращает R, t, переводящие точки ИЗ
// камеры 1 В камеру 2: x2 = R x1 + t. Накопление траектории (как в MATLAB
// estrelpose) использует позу камеры 2 в системе камеры 1:
//     R_rel = R^T,   t_rel = -R^T t.
// Затем отражение _FLIP_Y переводит систему "y вниз по кадру" в систему
// накопления "y вперёд (вверх по кадру)" - без него трек получается
// зеркальным.
// ---------------------------------------------------------------------------
const Mat3 kFlipY(1, 0, 0, 0, -1, 0, 0, 0, 1);

int count_true(const std::vector<char>& m) {
    return static_cast<int>(std::count(m.begin(), m.end(), 1));
}

// Нормированные координаты камеры.
std::vector<cv::Point2d> normalize_points(const std::vector<cv::Point2d>& xy, const Mat3& K) {
    std::vector<cv::Point2d> out;
    if (xy.empty()) return out;
    cv::undistortPoints(xy, out, cv::Mat(K), cv::noArray());
    return out;
}

bool finish_pose(MotionResult& result, const Mat3& R_rel_in, const V3& t_rel_in,
                 std::optional<double> step_length) {
    const double norm_t = cv::norm(t_rel_in);
    bool finite = std::isfinite(norm_t);
    for (int i = 0; i < 9; ++i) finite = finite && std::isfinite(R_rel_in.val[i]);
    if (!finite || norm_t < DBL_EPSILON) {
        result.message = "invalid_pose";
        return false;
    }
    const Mat3 R_rel = kFlipY * R_rel_in * kFlipY;
    const V3 t_rel = kFlipY * t_rel_in;
    result.rotation = orthonormalize_rotation(R_rel);
    result.translation = t_rel * (1.0 / norm_t);
    result.model = "homography";
    result.step_length = step_length;
    result.success = true;
    result.message = "ok_homography";
    return true;
}

void to_matlab_pose(const Mat3& R_cv, const V3& t_cv, Mat3& R_rel, V3& t_rel) {
    R_rel = R_cv.t();
    t_rel = -(R_cv.t() * t_cv);
}

bool motion_from_homography(const std::vector<cv::Point2d>& xy1,
                            const std::vector<cv::Point2d>& xy2, const Intrinsics& intr,
                            const Config& cfg, MotionResult& result,
                            std::optional<double> altitude) {
    const RansacCfg& rs = cfg.ransac;
    const Mat3 K = intr.K();
    const size_t num_matches = xy1.size();

    cv::Mat m;
    cv::Mat Hm = cv::findHomography(xy1, xy2, cv::RANSAC, rs.homography_threshold, m,
                                    rs.max_num_trials, rs.confidence / 100.0);
    if (Hm.empty() || m.empty()) {
        result.message = "homography_failed";
        return false;
    }

    result.inlier_mask.assign(num_matches, 0);
    for (size_t i = 0; i < num_matches && i < m.total(); ++i)
        result.inlier_mask[i] = m.at<uchar>(static_cast<int>(i)) != 0;
    result.status_code = 0;
    result.num_inliers = count_true(result.inlier_mask);
    result.inlier_ratio = static_cast<double>(result.num_inliers) /
                          static_cast<double>(std::max<size_t>(num_matches, 1));
    if (result.num_inliers < rs.min_inliers) {
        result.message = "too_few_inliers";
        return false;
    }

    std::vector<cv::Mat> rotations, translations, normals;
    const int count = cv::decomposeHomographyMat(Hm, cv::Mat(K), rotations, translations, normals);
    if (count <= 0) {
        result.message = "homography_not_decomposable";
        return false;
    }

    std::vector<cv::Point2d> in1, in2;
    for (size_t i = 0; i < num_matches; ++i)
        if (result.inlier_mask[i]) {
            in1.push_back(xy1[i]);
            in2.push_back(xy2[i]);
        }

    std::vector<int> candidates(static_cast<size_t>(count));
    std::iota(candidates.begin(), candidates.end(), 0);

    // Отсев решений, при которых плоскость оказывается за камерой. Точки
    // обязаны быть float32 И в нормированных координатах.
    try {
        const auto n1 = normalize_points(in1, K), n2 = normalize_points(in2, K);
        std::vector<cv::Point2f> r1, r2;
        for (const auto& p : n1) r1.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
        for (const auto& p : n2) r2.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
        cv::Mat possible;
        cv::filterHomographyDecompByVisibleRefpoints(rotations, normals, r1, r2, possible);
        if (!possible.empty()) {
            candidates.clear();
            for (int i = 0; i < static_cast<int>(possible.total()); ++i)
                candidates.push_back(possible.at<int>(i));
        }
    } catch (const cv::Exception& e) {
        std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Отсев решений гомографии недоступен (" << e.what()
                  << "); выбор ветви разложения опирается только на нормаль плоскости.\n";
    }
    if (candidates.empty()) {
        result.message = "homography_not_decomposable";
        return false;
    }

    // Если осталось несколько решений - берём то, где нормаль ближе к
    // оптической оси: для надира земля лежит прямо перед объективом.
    int best = candidates[0];
    double best_nz = -1.0;
    for (int i : candidates) {
        const double nz = std::abs(normals[static_cast<size_t>(i)].at<double>(2));
        if (nz > best_nz) {
            best_nz = nz;
            best = i;
        }
    }
    result.valid_fraction = 1.0 / static_cast<double>(std::max<size_t>(candidates.size(), 1));

    const Mat3 R_cv(rotations[static_cast<size_t>(best)]);
    const cv::Mat& tm = translations[static_cast<size_t>(best)];
    const V3 t_over_d(tm.at<double>(0), tm.at<double>(1), tm.at<double>(2));

    // Расстояние до плоскости - это высота съёмки: ||t/d|| * d = шаг в метрах.
    std::optional<double> distance;
    if (altitude && *altitude != 0.0) distance = altitude;
    else if (cfg.vo.plane_distance) distance = cfg.vo.plane_distance;
    std::optional<double> step_length;
    if (distance)
        step_length = cv::norm(t_over_d) * *distance * cfg.geometry.scale_correction;

    Mat3 R_rel;
    V3 t_rel;
    to_matlab_pose(R_cv, t_over_d, R_rel, t_rel);
    return finish_pose(result, R_rel, t_rel, step_length);
}

}  // namespace

// ============================================================================

Mat3 orthonormalize_rotation(const Mat3& R) {
    cv::Mat w, u, vt;
    cv::SVD::compute(cv::Mat(R), w, u, vt);
    cv::Mat out = u * vt;
    if (cv::determinant(out) < 0) {
        u.col(2) *= -1.0;
        out = u * vt;
    }
    return Mat3(out);
}

Mat3 rot_z(double psi) {
    const double c = std::cos(psi), s = std::sin(psi);
    return Mat3(c, -s, 0, s, c, 0, 0, 0, 1);
}

double turn_from_rotation(const Mat3& R) {
    for (int i = 0; i < 9; ++i)
        if (!std::isfinite(R.val[i])) return 0.0;
    return std::atan2(R(1, 0), R(0, 0));
}

cv::Point2d pixels_to_meters(const cv::Point2d& shift_px, double altitude_m,
                             const Intrinsics& intr, const Config& cfg) {
    // L = px * H / f. Камера движется ПРОТИВОПОЛОЖНО видимому смещению сцены -
    // отсюда минус у первой компоненты; ось Y изображения направлена вниз,
    // поэтому смещение сцены вниз означает движение вперёд.
    const double gain = altitude_m * cfg.geometry.scale_correction;
    return {-shift_px.x / intr.fx * gain, shift_px.y / intr.fy * gain};
}

MotionResult estimate_motion(const std::vector<cv::Point2d>& xy1,
                             const std::vector<cv::Point2d>& xy2, const Intrinsics& intr,
                             const Config& cfg, std::optional<double> altitude) {
    MotionResult result;
    const size_t num_matches = xy1.size();
    result.inlier_mask.assign(num_matches, 0);

    if (static_cast<int>(num_matches) < cfg.ransac.min_matches) {
        result.message = "too_few_matches";
        return result;
    }
    const double alt = altitude ? *altitude : cfg.altitude.value;

    const double t0 = now_sec();
    try {
        motion_from_homography(xy1, xy2, intr, cfg, result, alt);
    } catch (const cv::Exception& e) {
        result.message = std::string("error: ") + e.what();
    } catch (const std::exception& e) {
        result.message = std::string("error: ") + e.what();
    }
    result.time_sec = now_sec() - t0;
    return result;
}

}  // namespace vo
