#include "vo/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "vo/altitude.hpp"
#include "vo/body_yaw.hpp"
#include "vo/calibration.hpp"
#include "vo/features.hpp"
#include "vo/ins_export.hpp"
#include "vo/matching.hpp"
#include "vo/motion.hpp"
#include "vo/plot.hpp"
#include "vo/preprocess.hpp"
#include "vo/selection.hpp"
#include "vo/tracking.hpp"
#include "vo/viewer.hpp"

namespace vo {

namespace fs = std::filesystem;

namespace {

double positive_mod(double a, double m) {
    double r = std::fmod(a, m);
    if (r < 0) r += m;
    return r;
}

std::string join(const std::string& a, const std::string& b) {
    return (fs::u8path(a) / fs::u8path(b)).u8string();
}

BodyYawTrack load_body_yaw(const Config& cfg) {
    BodyYawTrack body;
    std::string cam = cfg.telemetry.camera_timestamps;
    std::string att = cfg.telemetry.attitude;
    if (cam.empty()) {
        const fs::path images = fs::u8path(cfg.data_folder);
        const fs::path guess = images.parent_path() / "timestamps.csv";
        if (fs::exists(guess)) cam = guess.u8string();
    }
    if (att.empty()) {
        const fs::path images = fs::u8path(cfg.data_folder);
        const fs::path guesses[] = {fs::u8path(cfg.project_root) / "other" / "dji_osdk_ros_attitude.csv",
                                    images.parent_path().parent_path() / "other" /
                                        "dji_osdk_ros_attitude.csv"};
        for (const fs::path& guess : guesses)
            if (fs::exists(guess)) {
                att = guess.u8string();
                break;
            }
    }
    if (!cam.empty() && !att.empty()) body.load(cam, att);
    return body;
}

size_t cap_step_by_body_yaw(size_t prev_id, size_t curr_id, const BodyYawTrack& body,
                            double max_body_deg) {
    if (!body.ok() || curr_id <= prev_id + 1) return curr_id;
    size_t capped = prev_id + 1;
    for (size_t f = prev_id + 1; f <= curr_id; ++f) {
        if (std::abs(rad2deg(body.delta_rad(prev_id, f))) <= max_body_deg)
            capped = f;
        else
            break;
    }
    return capped;
}

}  // namespace

// ---------------------------------------------------------------------------
// Плоскости графиков
// ---------------------------------------------------------------------------

PlaneInfo trajectory_plane(const Config& cfg) {
    const std::string p = upper(cfg.vis.trajectory_plane);
    // СК: X=север, Z=восток, Y=вверх.
    // На поле: снизу восток (Z), слева север (X).
    // positions: [0]=восток, [1]=север, [2]=вверх.
    // Подписи латиницей: шрифт OpenCV не рисует кириллицу.
    // Смысл осей: снизу восток (Z), слева север (X), вверх — Y.
    if (p == "XY") return {0, 1, "east", "north", "xy", "east-north"};
    if (p == "XZ") return {0, 2, "east", "up", "xz", "east-up"};
    if (p == "YZ") return {1, 2, "north", "up", "yz", "north-up"};
    throw std::runtime_error("Неизвестная плоскость \"" + p +
                             "\". Задайте trajectory_plane = XY, XZ или YZ.");
}

// ---------------------------------------------------------------------------
// Таблицы и экспорт
// ---------------------------------------------------------------------------

void write_trajectory_csv(const std::vector<TrajectoryRow>& rows, const std::string& path) {
    CsvTable t;
    t.columns = {"Frame",   "File",      "X",       "Y",        "Z",
                 "Altitude", "SpeedMps", "HeadingDeg", "TurnDeg", "StepM",
                 "ShiftPct", "FrameStep", "Matches", "Inliers",  "InlierRatioPct",
                 "StepTimeMs", "Status",  "Model",   "RotationMode", "UsedFramePct", "LkTracks"};
    for (const TrajectoryRow& r : rows)
        t.rows.push_back({std::to_string(r.frame), r.file, fmt_g15(r.x), fmt_g15(r.y),
                          fmt_g15(r.z), fmt_g15(r.altitude), fmt_g15(r.speed_mps),
                          fmt_g15(r.heading_deg), fmt_g15(r.turn_deg), fmt_g15(r.step_m),
                          fmt_g15(r.shift_pct), std::to_string(r.frame_step),
                          std::to_string(r.matches), std::to_string(r.inliers),
                          fmt_g15(r.inlier_ratio_pct), fmt_g15(r.step_time_ms), r.status, r.model,
                          std::to_string(r.rotation_mode), fmt_g15(r.used_frame_pct),
                          std::to_string(r.lk_tracks)});
    t.write(path);
}

namespace {

constexpr double kEarthA = 6378137.0;          // большая полуось WGS-84, м
constexpr double kEarthE2 = 6.69437999014e-3;  // квадрат первого эксцентриситета

void meters_per_degree(double lat_deg, double& m_lat, double& m_lon) {
    const double lat = deg2rad(lat_deg);
    const double sn = std::sin(lat);
    const double w = 1.0 - kEarthE2 * sn * sn;
    m_lat = kPi / 180.0 * kEarthA * (1.0 - kEarthE2) / std::pow(w, 1.5);
    m_lon = std::max(kPi / 180.0 * kEarthA / std::sqrt(w) * std::cos(lat), 1e-9);
}

}  // namespace

std::string write_gps_track(const std::vector<cv::Vec3d>& P, const std::vector<int>& frame_ids,
                            double frame_rate, const std::string& folder, const Config& cfg,
                            const std::vector<double>* altitude, bool one_based) {
    const ExportCfg& exp = cfg.export_cfg;
    const size_t n = P.size();
    if (n == 0) return "";

    const double fps = frame_rate > 0 ? frame_rate : exp.frame_rate;

    // Какие оси идут в восток и север.
    PlaneInfo plane;
    if (upper(exp.plane) == "AUTO") {
        plane = trajectory_plane(cfg);
    } else {
        Config tmp = cfg;
        tmp.vis.trajectory_plane = exp.plane;
        plane = trajectory_plane(tmp);
    }

    std::vector<double> t(n), east(n), north(n), lat(n), lon(n), alt(n), speed(n, 0.0), heading(n, 0.0);
    double m_lat, m_lon;
    meters_per_degree(exp.ref_lat, m_lat, m_lon);
    for (size_t i = 0; i < n; ++i) {
        double id = i < frame_ids.size() ? frame_ids[i] : static_cast<double>(i);
        if (one_based) id -= 1.0;
        t[i] = id / fps;
        east[i] = P[i][plane.col_x] * exp.scale;
        north[i] = P[i][plane.col_y] * exp.scale * (exp.north_flip ? -1.0 : 1.0);
        lat[i] = exp.ref_lat + north[i] / m_lat;
        lon[i] = exp.ref_lon + east[i] / m_lon;
    }
    bool alt_ok = altitude && altitude->size() >= n;
    if (alt_ok)
        for (size_t i = 0; i < n; ++i) alt_ok = alt_ok && std::isfinite((*altitude)[i]);
    for (size_t i = 0; i < n; ++i) alt[i] = alt_ok ? (*altitude)[i] : exp.ref_alt;

    // Скорость и курс по горизонтальной проекции: курс 0 = север, по часовой.
    double last = 0.0;
    for (size_t i = 1; i < n; ++i) {
        const double de = east[i] - east[i - 1], dn = north[i] - north[i - 1];
        const double dt = t[i] - t[i - 1];
        const double step = std::hypot(de, dn);
        speed[i] = dt > 0 ? step / dt : 0.0;
        if (step > 1e-12) last = positive_mod(rad2deg(std::atan2(de, dn)), 360.0);
        heading[i] = last;
    }
    if (n > 1) heading[0] = heading[1];

    make_dirs(folder);
    const std::string path = join(folder, exp.gps_file);
    std::ofstream out(fs::u8path(path), std::ios::binary);
    out << "timestamp,lat,lon,alt,speed,heading\n";
    for (size_t i = 0; i < n; ++i)
        out << fmt("%.3f,%.8f,%.8f,%.3f,%.4f,%.3f\n", t[i], lat[i], lon[i], alt[i], speed[i], heading[i]);
    std::cout << "Трек в формате GPS (" << plane.name << " -> восток/север, " << fmt("%.2f", fps)
              << " fps, масштаб " << fmt("%g", exp.scale) << " м/ед.): " << path << " (" << n
              << " строк)\n";
    return path;
}

// ---------------------------------------------------------------------------
// Графики
// ---------------------------------------------------------------------------

void plot_trajectory(const std::vector<cv::Vec3d>& P, const std::string& folder,
                     const std::string& alg_name, const Config& cfg) {
    if (P.empty()) return;
    make_dirs(folder);
    const cv::Size size(cfg.vis.plot_width, cfg.vis.plot_height);

    auto series_for = [&](int cx, int cy) {
        Series s;
        s.color = tab_color(0);
        s.width = 1.5;
        for (const auto& p : P) {
            s.x.push_back(p[cx]);
            s.y.push_back(p[cy]);
        }
        Series start;
        start.x = {s.x.front()};
        start.y = {s.y.front()};
        start.color = kBlack;
        start.style = Series::Style::Marker;
        return std::vector<Series>{s, start};
    };

    // Основной график: плоскость из cfg.
    const PlaneInfo plane = trajectory_plane(cfg);
    Axes ax;
    ax.title = "Trajectory in plane " + plane.name + ": " + alg_name;
    ax.xlabel = plane.label_x + ", m";
    ax.ylabel = plane.label_y + ", m";
    ax.equal = true;
    ax.legend = false;
    ax.series = series_for(plane.col_x, plane.col_y);
    save_figure(join(folder, "trajectory_" + plane.suffix + ".png"), render_figure({ax}, 1, 1, size));

    // Три проекции (замена 3D-графика matplotlib).
    std::vector<Axes> views;
    const int cols[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    const char* names[3] = {"X-Y", "X-Z", "Y-Z"};
    const char* labels[3] = {"X", "Y", "Z"};
    for (int k = 0; k < 3; ++k) {
        Axes a;
        a.title = names[k];
        a.xlabel = std::string(labels[cols[k][0]]) + ", m";
        a.ylabel = std::string(labels[cols[k][1]]) + ", m";
        a.equal = true;
        a.legend = false;
        a.series = series_for(cols[k][0], cols[k][1]);
        views.push_back(a);
    }
    save_figure(join(folder, "trajectory_views.png"),
                render_figure(views, 1, 3, {size.width * 3 / 2, size.height / 2 + 60},
                              "Visual odometry: " + alg_name));
}

void plot_trajectory_comparison(const std::vector<std::string>& names,
                                const std::vector<std::vector<cv::Vec3d>>& positions,
                                const std::string& folder, const Config& cfg) {
    if (names.empty()) return;
    make_dirs(folder);
    const PlaneInfo plane = trajectory_plane(cfg);
    const cv::Size size(cfg.vis.plot_width + 160, cfg.vis.plot_height);

    auto make_series = [&](size_t i, double width) {
        Series s;
        s.color = tab_color(static_cast<int>(i));
        s.label = names[i];
        s.width = width;
        for (const auto& p : positions[i]) {
            s.x.push_back(p[plane.col_x]);
            s.y.push_back(p[plane.col_y]);
        }
        return s;
    };

    Axes all;
    all.title = "Trajectory comparison (plane " + plane.name + ")";
    all.xlabel = plane.label_x + ", m";
    all.ylabel = plane.label_y + ", m";
    all.equal = true;
    for (size_t i = 0; i < names.size(); ++i) all.series.push_back(make_series(i, 1.5));
    save_figure(join(folder, "trajectories_" + plane.suffix + "_compare.png"),
                render_figure({all}, 1, 1, size));

    const int n = static_cast<int>(names.size());
    const int n_cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(n))));
    const int n_rows = (n + n_cols - 1) / n_cols;
    std::vector<Axes> grid(static_cast<size_t>(n_rows * n_cols));
    for (size_t i = 0; i < grid.size(); ++i) {
        if (i >= names.size()) {
            grid[i].hide = true;
            continue;
        }
        Axes& a = grid[i];
        a.title = names[i];
        a.xlabel = plane.label_x;
        a.ylabel = plane.label_y;
        a.equal = true;
        a.legend = false;
        a.series.push_back(make_series(i, 1.3));
        a.series.back().label.clear();
    }
    save_figure(join(folder, "trajectories_" + plane.suffix + "_grid.png"),
                render_figure(grid, n_rows, n_cols, {1200, 900},
                              "Trajectories by algorithm (" + plane.name + ")"));
}

void save_match_figure(const cv::Mat& image1, const cv::Mat& image2,
                       const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2,
                       const std::string& filename, const std::string& title) {
    make_dirs(parent_path(filename));
    const int h = std::max(image1.rows, image2.rows);
    cv::Mat canvas = cv::Mat::zeros(h, image1.cols + image2.cols, CV_8UC3);
    auto to_bgr = [](const cv::Mat& m) {
        cv::Mat out;
        if (m.channels() == 1) cv::cvtColor(m, out, cv::COLOR_GRAY2BGR);
        else out = m;
        return out;
    };
    to_bgr(image1).copyTo(canvas(cv::Rect(0, 0, image1.cols, image1.rows)));
    to_bgr(image2).copyTo(canvas(cv::Rect(image1.cols, 0, image2.cols, image2.rows)));

    const size_t n = std::min(p1.size(), p2.size());
    // Больше 300 линий превращают картинку в кашу - показываем подмножество.
    const size_t step = std::max<size_t>(1, n / 300);
    size_t shown = 0;
    for (size_t i = 0; i < n; i += step) {
        const cv::Point a(static_cast<int>(std::lround(p1[i].x)), static_cast<int>(std::lround(p1[i].y)));
        const cv::Point b(static_cast<int>(std::lround(p2[i].x)) + image1.cols,
                          static_cast<int>(std::lround(p2[i].y)));
        cv::line(canvas, a, b, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
        cv::circle(canvas, a, 3, cv::Scalar(60, 255, 60), 1, cv::LINE_AA);
        cv::circle(canvas, b, 3, cv::Scalar(60, 255, 60), 1, cv::LINE_AA);
        ++shown;
    }
    const std::string caption = title + " | shown " + std::to_string(shown) + " of " + std::to_string(n);
    cv::rectangle(canvas, {0, 0}, {canvas.cols, 34}, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(canvas, caption, {10, 23}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
    save_figure(filename, canvas);
}

// ---------------------------------------------------------------------------
// Траектория одним методом
// ---------------------------------------------------------------------------

namespace {

bool on_shadow_mask(const cv::Mat& mask, const cv::Point2f& p) {
    if (mask.empty()) return false;
    const int x = cvRound(p.x), y = cvRound(p.y);
    if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows) return false;
    return mask.at<uchar>(y, x) != 0;
}

// Точки шасси/тени почти не двигаются в кадре, в отличие от земли.
// Если медиана смещения уже большая, пары с крошечным потоком отбрасываем.
void reject_static_pairs(std::vector<cv::Point2f>& m1, std::vector<cv::Point2f>& m2,
                         double static_px) {
    if (static_px <= 0.0 || m1.size() < 8 || m1.size() != m2.size()) return;
    std::vector<double> d;
    d.reserve(m1.size());
    for (size_t i = 0; i < m1.size(); ++i)
        d.push_back(std::hypot(m2[i].x - m1[i].x, m2[i].y - m1[i].y));
    const double med = median(d);
    if (med < 4.0) return;   // зависание / почти нет движения сцены
    const double thr = std::max(static_px, 0.05 * med);
    std::vector<cv::Point2f> k1, k2;
    k1.reserve(m1.size());
    k2.reserve(m2.size());
    for (size_t i = 0; i < m1.size(); ++i)
        if (d[i] >= thr) {
            k1.push_back(m1[i]);
            k2.push_back(m2[i]);
        }
    if (k1.size() >= 8) {
        m1.swap(k1);
        m2.swap(k2);
    }
}

// Кадр-якорь: пока его земля видна, поза текущего кадра считается прямо от него,
// а не суммой промежуточных шагов.
struct AnchorSnap {
    size_t id = 0;
    FeatureData data;
    cv::Mat shadow;
    double x = 0, y = 0;
    cv::Matx33d R = cv::Matx33d::eye();
    double altitude = 0;
};

bool motion_against(const FeatureData& older, const cv::Mat& shadow_older, const FeatureData& newer,
                    const cv::Mat& shadow_newer, size_t id_from, size_t id_to, const BodyYawTrack& body,
                    double altitude, const Config& cfg, const Intrinsics& intr, MotionResult& motion,
                    double& shift_px, std::vector<cv::Point2f>& p1, std::vector<cv::Point2f>& p2) {
    const MatchResult match = match_feature_sets(older, newer, cfg);
    p1.clear();
    p2.clear();
    p1.reserve(match.pairs.size());
    p2.reserve(match.pairs.size());
    for (const auto& pr : match.pairs) {
        p1.push_back(older.points[static_cast<size_t>(pr.first)]);
        p2.push_back(newer.points[static_cast<size_t>(pr.second)]);
    }
    if (cfg.shadow.enable) {
        std::vector<cv::Point2f> k1, k2;
        for (size_t i = 0; i < p1.size(); ++i)
            if (!on_shadow_mask(shadow_older, p1[i]) && !on_shadow_mask(shadow_newer, p2[i])) {
                k1.push_back(p1[i]);
                k2.push_back(p2[i]);
            }
        p1.swap(k1);
        p2.swap(k2);
        reject_static_pairs(p1, p2, cfg.shadow.static_flow_px);
    }
    if (static_cast<int>(p1.size()) < cfg.ransac.min_matches) {
        motion = MotionResult();
        motion.message = "too_few_matches";
        shift_px = 0;
        return false;
    }
    std::vector<cv::Point2d> a(p1.begin(), p1.end()), b(p2.begin(), p2.end());
    motion = estimate_motion(a, b, intr, cfg, altitude);
    std::vector<double> d;
    int cnt = 0;
    for (char c : motion.inlier_mask) cnt += c ? 1 : 0;
    const bool use_mask = cnt >= 3;
    for (size_t i = 0; i < p2.size(); ++i)
        if (!use_mask || (i < motion.inlier_mask.size() && motion.inlier_mask[i]))
            d.push_back(std::hypot(p2[i].x - p1[i].x, p2[i].y - p1[i].y));
    shift_px = median(d);
    return motion.success;
}

double drop_vertical_step(double step_len, const cv::Vec3d& translation, double alt_from, double alt_to) {
    const double dh = std::abs(alt_to - alt_from);
    const double horizontal_m = step_len * std::hypot(translation[0], translation[1]);
    if (dh >= 1.0 && dh >= horizontal_m) return 0.0;
    return step_len;
}

VoRun run_body(const Config& cfg, Sequence* sequence_in, const std::string& algorithm,
               bool verbose, ConsoleLog& log) {
    std::unique_ptr<Sequence> owned;
    Sequence* sequence = sequence_in;
    if (!sequence) {
        owned = open_sequence(cfg.data_folder, cfg);
        sequence = owned.get();
    }

    const size_t total = sequence->size();
    const size_t limit = cfg.max_frames < 0 ? total
                                            : std::min<size_t>(total, static_cast<size_t>(cfg.max_frames));
    if (limit < 2)
        throw std::runtime_error("Требуется не менее двух кадров, доступно " + std::to_string(limit) + ".");

    cv::Mat first_raw = sequence->read(0);
    if (first_raw.empty()) throw std::runtime_error("Не удалось прочитать первый кадр источника.");
    const Calibration calib = load_calibration(cfg, first_raw.rows, first_raw.cols);
    const Intrinsics& intr = calib.intrinsics;
    const double frame_w = static_cast<double>(intr.width);

    const Algorithm alg = pick_algorithm(cfg, algorithm);
    VoRun run;
    run.folder = join(join(cfg.results_folder, "visual_odometry"), safe_file_name(alg.name));
    const std::string match_folder = join(run.folder, "matches");
    make_dirs(match_folder);
    log.set_path(join(run.folder, cfg.export_cfg.console_file));

    if (verbose) {
        std::cout << "Метод визуальной одометрии: " << alg.name << "\n";
        if (calib.is_approximate)
            std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Используется приближенная калибровка. Масштаб траектории "
                         "будет неверен - см. camera.focal_length и geometry.scale_correction.\n";
    }

    std::unique_ptr<AlgorithmRunner> runner;
    try {
        runner = std::make_unique<AlgorithmRunner>(alg, cfg);
    } catch (const std::exception& e) {
        throw std::runtime_error("Метод " + alg.name + " недоступен: " + e.what());
    }

    // ------------------------------------------------------------- высота
    const std::vector<double> altitude = load_altitude_series(cfg, limit);
    const std::string alt_source = altitude_source(cfg);

    // Отбраковка единичных выбросов длины шага (см. vo.step_length_limit).
    const double step_limit = cfg.vo.step_length_limit;
    int num_clamped = 0;

    // ------------------------------------------------- выбор кадров и обрезка
    FrameSelector sel(cfg, frame_w);
    RotationCrop crop(cfg);
    int crop_toggled = 0;
    const BodyYawTrack body_yaw = load_body_yaw(cfg);
    int arc_splits = 0;
    constexpr size_t kArcMinGap = 8;
    constexpr double kArcMinBodyDeg = 12.0;
    constexpr double kSkipAnchorBodyDeg = 15.0;
    constexpr size_t kSkipAnchorMinGap = 8;
    constexpr double kMaxBodyTurnPerStepDeg = 9.0;
    if (verbose) {
        std::cout << sel.describe() << "\n";
        if (crop.enabled()) {
            const auto& c = crop.rotation_cut();
            std::cout << "Обрезка на поворотах: порог " << fmt("%.1f", rad2deg(crop.threshold()))
                      << " град/с, задержка " << crop.delay() << " шагов, остаётся "
                      << fmt("%.0f", 100 * (1 - c[0] - c[1]) * (1 - c[2] - c[3])) << "% кадра\n";
        }
        if (cfg.shadow.enable)
            std::cout << "Маска теней: тёмные пятна площади тени (размах "
                      << fmt("%.1f", cfg.shadow.aircraft_span_m) << " м), до "
                      << fmt("%.0f", cfg.shadow.max_altitude_m) << " м\n";
        if (body_yaw.ok())
            std::cout << "Рыскание корпуса загружено (attitude). Разворот режется на короткие шаги; "
                         "в траекторию курс корпуса не пишется.\n";
        // shift_from удалён (поддерживается только homography)
    }

    // ------------------------------------------------------------ состояние
    double x_m = cfg.geometry.initial_coords.x, y_m = cfg.geometry.initial_coords.y;
    double heading = deg2rad(cfg.geometry.initial_heading_deg);
    cv::Matx33d R_abs = rot_z(heading);
    const double fps = sequence->fps() > 0 ? sequence->fps() : cfg.export_cfg.frame_rate;
    double distance_m = 0.0;

    const bool profile = cfg.perf.profile;
    const bool verbose_frames = cfg.perf.verbose_per_frame;
    const bool save_figures = cfg.perf.save_match_figures;
    enum Stage { kRead, kPre, kDetect, kMatch, kTrack, kMotion, kFigures, kNumStages };
    const char* stage_names[kNumStages] = {"чтение кадров", "предобработка", "детектирование",
                                           "сопоставление", "трекинг LK", "оценка движения",
                                           "картинки соответствий"};
    double stage[kNumStages] = {0, 0, 0, 0, 0, 0, 0};
    const std::string lk_mode = lower(cfg.lk.mode);
    int lk_steps = 0, shadow_frames = 0, shadow_dropped = 0, static_dropped = 0;
    std::map<std::string, int> model_counts;
    int saved_figures = 0, retries = 0;

    size_t prev_id = 0;
    cv::Mat prev_image = preprocess_frame(first_raw, calib, cfg);
    cv::Mat prev_shadow = make_shadow_mask(prev_image, altitude[0], intr.fx, cfg.shadow);
    if (!prev_shadow.empty()) ++shadow_frames;
    FeatureData prev_data = detect_and_describe(prev_image, *runner, cfg, &crop, prev_shadow);

    run.frame_ids.push_back(0);
    run.positions.emplace_back(x_m, y_m, altitude_to_z(altitude, 0));
    run.rotations.push_back(R_abs);
    {
        TrajectoryRow r;
        r.frame = 1;
        r.file = sequence->name(0);
        r.x = x_m;
        r.y = y_m;
        r.z = run.positions[0][2];
        r.altitude = altitude[0];
        r.heading_deg = positive_mod(90.0 - rad2deg(heading), 360.0);
        r.status = "origin";
        r.rotation_mode = crop.active();
        r.used_frame_pct = 100.0 * crop.crop_fraction();
        run.rows.push_back(r);
    }

    LiveTrajectoryPlot live(cfg);
    FrameViewer viewer(cfg);
    const double run_start = now_sec();

    // Снимок, от которого считается поза, пока земля в кадре та же.
    AnchorSnap anchor;
    anchor.id = prev_id;
    anchor.data = prev_data;
    anchor.shadow = prev_shadow.clone();
    anchor.x = x_m;
    anchor.y = y_m;
    anchor.R = R_abs;
    anchor.altitude = altitude.empty() ? 0.0 : altitude[0];
    int anchor_hits = 0, anchor_refresh = 0;

    // -------------------------------------------------------------- главный цикл
    cv::Mat curr_image;
    FeatureData curr_data;
    int weak_turns = 0;
    while (true) {
        if (prev_id < altitude.size()) sel.set_altitude(altitude[prev_id]);
        size_t curr_id = prev_id + static_cast<size_t>(sel.step());
        if (curr_id >= limit) {
            // До конца остался неполный шаг: берём последний доступный кадр,
            // если он достаточно далеко от предыдущего.
            if (limit - 1 - prev_id < static_cast<size_t>(std::max(1, sel.step() / 4))) break;
            curr_id = limit - 1;
        }
        // Не укорачиваем шаг по рысканью корпуса: иначе меняется выбор кадров
        // относительно эталонного прогона одометрии (RMS ~18.7 м на HKairport03).
        const double loop_start = now_sec();
        double t_stage = now_sec();
        cv::Mat curr_raw = sequence->read(curr_id);
        stage[kRead] += now_sec() - t_stage;
        if (curr_raw.empty()) {
            std::cout << "Источник закончился на кадре " << curr_id + 1 << ".\n";
            break;
        }

        t_stage = now_sec();
        curr_image = preprocess_frame(curr_raw, calib, cfg);
        stage[kPre] += now_sec() - t_stage;

        t_stage = now_sec();
        cv::Mat curr_shadow = make_shadow_mask(curr_image, altitude[curr_id], intr.fx, cfg.shadow);
        if (!curr_shadow.empty()) ++shadow_frames;
        curr_data = detect_and_describe(curr_image, *runner, cfg, &crop, curr_shadow);
        stage[kDetect] += now_sec() - t_stage;

        t_stage = now_sec();
        const MatchResult match = match_feature_sets(prev_data, curr_data, cfg);
        stage[kMatch] += now_sec() - t_stage;

        MotionResult motion;
        const double alt_here = altitude[curr_id];
        std::vector<cv::Point2f> matched1, matched2;
        std::vector<int> matched_prev;
        matched1.reserve(match.pairs.size());
        matched2.reserve(match.pairs.size());
        for (const auto& pr : match.pairs) {
            matched_prev.push_back(pr.first);
            matched1.push_back(prev_data.points[static_cast<size_t>(pr.first)]);
            matched2.push_back(curr_data.points[static_cast<size_t>(pr.second)]);
        }

        int lk_count = 0;
        if (lk_mode == "only") {
            // Соответствия только из пирамидального LK. FREAK-пары не используются.
            // Отсев выбросов — RANSAC внутри оценки гомографии.
            t_stage = now_sec();
            const LkTracks lk = track_points_lk(prev_image, curr_image, prev_data.points, cfg.lk);
            matched1 = lk.p1;
            matched2 = lk.p2;
            lk_count = static_cast<int>(lk.p1.size());
            stage[kTrack] += now_sec() - t_stage;
        } else if (lk_mode == "always" || (lk_mode == "rotation" && crop.active())) {
            t_stage = now_sec();
            cv::Mat H_guess;
            if (matched1.size() >= 8)
                H_guess = cv::findHomography(matched1, matched2, cv::RANSAC, cfg.ransac.homography_threshold);
            const LkTracks lk = track_points_lk(prev_image, curr_image, prev_data.points, cfg.lk, H_guess);
            // Для точки, прослеженной LK, берётся субпиксельное положение из
            // потока; пары FREAK остаются только для непрослеженных точек.
            std::vector<char> tracked(prev_data.points.size(), 0);
            for (int i : lk.index) tracked[static_cast<size_t>(i)] = 1;
            std::vector<cv::Point2f> m1 = lk.p1, m2 = lk.p2;
            for (size_t k = 0; k < matched1.size(); ++k)
                if (!tracked[static_cast<size_t>(matched_prev[k])]) {
                    m1.push_back(matched1[k]);
                    m2.push_back(matched2[k]);
                }
            matched1.swap(m1);
            matched2.swap(m2);
            lk_count = static_cast<int>(lk.p1.size());
            stage[kTrack] += now_sec() - t_stage;
        }

        if (cfg.shadow.enable && (!prev_shadow.empty() || !curr_shadow.empty())) {
            const size_t before = matched1.size();
            std::vector<cv::Point2f> k1, k2;
            k1.reserve(matched1.size());
            k2.reserve(matched2.size());
            for (size_t i = 0; i < matched1.size(); ++i)
                if (!on_shadow_mask(prev_shadow, matched1[i]) && !on_shadow_mask(curr_shadow, matched2[i])) {
                    k1.push_back(matched1[i]);
                    k2.push_back(matched2[i]);
                }
            shadow_dropped += static_cast<int>(before - k1.size());
            matched1.swap(k1);
            matched2.swap(k2);
        }
        if (cfg.shadow.enable) {
            const size_t before_static = matched1.size();
            reject_static_pairs(matched1, matched2, cfg.shadow.static_flow_px);
            static_dropped += static_cast<int>(before_static - matched1.size());
        }
        const size_t num_matches = matched1.size();
        if (static_cast<int>(num_matches) >= cfg.ransac.min_matches) {
            std::vector<cv::Point2d> a(matched1.begin(), matched1.end()), b(matched2.begin(), matched2.end());
            t_stage = now_sec();
            motion = estimate_motion(a, b, intr, cfg, alt_here);
            stage[kMotion] += now_sec() - t_stage;
        } else {
            motion.message = "too_few_matches";
        }

        // Поза от снимка, если он ещё перекрывает текущий кадр. Иначе снимок
        // заменяется последним принятым кадром, и этот шаг идёт обычной цепочкой.
        bool from_anchor = false;
        MotionResult anchored;
        double anchor_shift = 0.0;
        std::vector<cv::Point2f> anchor_p1, anchor_p2;
        constexpr double kAnchorMinRatio = 0.35;
        constexpr double kAnchorMaxShift = 0.40;
        constexpr double kAnchorMaxAlt = 20.0;
        if (anchor.id != prev_id) {
            t_stage = now_sec();
            const bool got = motion_against(anchor.data, anchor.shadow, curr_data, curr_shadow, anchor.id,
                                            curr_id, body_yaw, anchor.altitude, cfg, intr, anchored,
                                            anchor_shift, anchor_p1, anchor_p2);
            stage[kMotion] += now_sec() - t_stage;
            const bool overlap = got && anchored.inlier_ratio >= kAnchorMinRatio &&
                                 anchored.num_inliers >= cfg.ransac.min_inliers &&
                                 anchor_shift < kAnchorMaxShift * frame_w &&
                                 std::abs(alt_here - anchor.altitude) <= kAnchorMaxAlt;
            from_anchor = overlap;
            if (from_anchor && body_yaw.ok() && curr_id > anchor.id + kSkipAnchorMinGap) {
                const double span = std::abs(rad2deg(body_yaw.delta_rad(anchor.id, curr_id)));
                if (span > kSkipAnchorBodyDeg) from_anchor = false;
            }
        }

        // Большой поворот при малой доле инлаеров в траекторию не пишется,
        // если этот поворот вообще идёт в позу. От снимка поза берётся целиком.
        const double turn_try = motion.success ? std::abs(rad2deg(turn_from_rotation(motion.rotation))) : 0.0;
        const bool weak_turn = !from_anchor && motion.success &&
                               turn_try >= cfg.frame_select.turn_reject_min_deg &&
                               motion.inlier_ratio < cfg.frame_select.weak_turn_inlier_ratio;
        if (weak_turn && sel.can_shrink()) {
            sel.shrink();
            ++retries;
            ++weak_turns;
            if (verbose && verbose_frames)
                std::cout << "  кадр " << curr_id + 1 << ": поворот " << fmt("%.1f", turn_try)
                          << " град при " << fmt("%.0f", 100.0 * motion.inlier_ratio)
                          << "% инлаеров, уменьшаю шаг до " << sel.step() << "\n";
            continue;
        }
        if (weak_turn) {
            motion.rotation = cv::Matx33d::eye();
            ++weak_turns;
        }

        // --- Неудача: отступаем назад по шагу и пробуем ближний кадр ---------
        if (!motion.success && !from_anchor && sel.can_shrink()) {
            sel.shrink();
            ++retries;
            if (verbose && verbose_frames)
                std::cout << "  кадр " << curr_id + 1 << ": " << motion.message
                          << ", уменьшаю шаг до " << sel.step() << "\n";
            continue;
        }

        // ------------------------------------------------------- накопление
        double shift_px = 0.0, step_m = 0.0;
        const bool posed = from_anchor || motion.success;
        if (motion.success) {
            // Смещение сцены для регулятора шага — по соседним кадрам, не по снимку:
            // от снимка смещение растёт и регулятор схлопнул бы шаг.
            std::vector<double> d;
            int cnt = 0;
            for (char c : motion.inlier_mask) cnt += c ? 1 : 0;
            const bool use_mask = cnt >= 3;
            for (size_t i = 0; i < matched2.size(); ++i)
                if (!use_mask || motion.inlier_mask[i])
                    d.push_back(std::hypot(matched2[i].x - matched1[i].x, matched2[i].y - matched1[i].y));
            shift_px = median(d);
        }

        if (from_anchor) {
            ++anchor_hits;
            ++model_counts["anchor"];
            double step_len = anchored.step_length ? *anchored.step_length : cfg.vo.translation_scale;
            if (anchored.step_length && step_limit > 0 && anchor_shift > 1e-6) {
                const double bound = step_limit * anchor_shift * anchor.altitude *
                                     cfg.geometry.scale_correction / std::max(intr.fx, 1e-9);
                if (step_len > bound) {
                    step_len = bound;
                    ++num_clamped;
                }
            }
            if (alt_here < cfg.vo.min_translate_alt_m) step_len = 0.0;
            step_len = drop_vertical_step(step_len, anchored.translation, anchor.altitude, alt_here);
            const cv::Vec3d delta = anchor.R * (anchored.translation * step_len);
            const double x0 = x_m, y0 = y_m;
            const size_t gap = curr_id > anchor.id ? curr_id - anchor.id : 0;
            const double body_turn_deg =
                std::abs(rad2deg(body_yaw.delta_rad(anchor.id, curr_id)));
            const bool use_arc = body_yaw.ok() && gap >= kArcMinGap && body_turn_deg >= kArcMinBodyDeg;
            if (use_arc) {
                ++arc_splits;
                const double h_anchor = std::atan2(anchor.R(1, 0), anchor.R(0, 0));
                integrate_arc_xy(anchor.x, anchor.y, h_anchor, step_len, anchor.id, curr_id, body_yaw,
                                 x_m, y_m);
            } else {
                x_m = anchor.x + delta[0];
                y_m = anchor.y + delta[1];
            }
            step_m = std::hypot(x_m - x0, y_m - y0);
            distance_m += step_m;
            R_abs = orthonormalize_rotation(anchor.R * anchored.rotation);
            heading = std::atan2(R_abs(1, 0), R_abs(0, 0));
            if (!motion.success && alt_here > 1e-6)
                shift_px = step_m * intr.fx / alt_here;
        } else if (motion.success) {
            ++model_counts[motion.model];
            if (anchor.id != prev_id) {
                anchor.id = prev_id;
                anchor.data = prev_data;
                anchor.shadow = prev_shadow.clone();
                anchor.x = x_m;
                anchor.y = y_m;
                anchor.R = R_abs;
                anchor.altitude = prev_id < altitude.size() ? altitude[prev_id] : alt_here;
                ++anchor_refresh;
            }
            double step_len;
            if (!motion.step_length) {
                // Эпиполярная модель без метрики: условная единица.
                step_len = cfg.vo.translation_scale;
            } else {
                step_len = *motion.step_length;
                if (step_limit > 0 && shift_px > 1e-6) {
                    // Сцена сместилась на shift_px пикселей - камера прошла
                    // около shift_px * H / f метров: независимая от модели
                    // верхняя оценка.
                    const double bound = step_limit * shift_px * alt_here *
                                         cfg.geometry.scale_correction / std::max(intr.fx, 1e-9);
                    if (step_len > bound) {
                        step_len = bound;
                        ++num_clamped;
                    }
                }
            }
            // На взлёте/посадке высота мала, плоскость земли не держится, и
            // шаг ||t/d||*H даёт ложный снос. Курс из гомографии оставляем.
            if (alt_here < cfg.vo.min_translate_alt_m) step_len = 0.0;
            const double h_prev = prev_id < altitude.size() ? altitude[prev_id] : alt_here;
            step_len = drop_vertical_step(step_len, motion.translation, h_prev, alt_here);

            const cv::Vec3d step_vec = motion.translation * step_len;
            const cv::Vec3d delta = R_abs * step_vec;
            const double x0 = x_m, y0 = y_m;
            x_m += delta[0];
            y_m += delta[1];
            step_m = std::hypot(x_m - x0, y_m - y0);
            distance_m += step_m;
            R_abs = orthonormalize_rotation(R_abs * motion.rotation);
            heading = std::atan2(R_abs(1, 0), R_abs(0, 0));

            if (save_figures && saved_figures < cfg.vo.max_saved_match_figures) {
                ++saved_figures;
                std::vector<cv::Point2f> in1, in2;
                for (size_t i = 0; i < matched1.size(); ++i)
                    if (motion.inlier_mask[i]) {
                        in1.push_back(matched1[i]);
                        in2.push_back(matched2[i]);
                    }
                t_stage = now_sec();
                save_match_figure(prev_image, curr_image, in1, in2,
                                  join(match_folder, fmt("frame_%06zu.png", curr_id + 1)),
                                  alg.name + ", frames " + std::to_string(prev_id + 1) + "-" +
                                      std::to_string(curr_id + 1));
                stage[kFigures] += now_sec() - t_stage;
            }
        }

        const double turn_rad = motion.success ? turn_from_rotation(motion.rotation) : 0.0;
        const double dt = fps > 0 ? static_cast<double>(curr_id - prev_id) / fps : 0.0;

        // Слишком резкий манёвр для такого шага: берём кадр ближе.
        if (motion.success && sel.too_sharp(turn_rad, dt) && sel.can_shrink()) {
            sel.shrink();
            ++retries;
            if (verbose && verbose_frames)
                std::cout << "  кадр " << curr_id + 1 << ": поворот "
                          << fmt("%.0f", rad2deg(std::abs(turn_rad)) / std::max(dt, 1e-9))
                          << " град/с слишком резкий, уменьшаю шаг до " << sel.step() << "\n";
            continue;
        }

        // Снимок обрезки, действовавшей ПРИ ОБРАБОТКЕ этого кадра.
        const double used_pct = 100.0 * crop.crop_fraction();
        const bool was_active = crop.active();
        CropUsed crop_used{was_active, crop.bounds(curr_image.size()), crop.crop_fraction()};
        crop.update(turn_rad, dt);
        if (crop.enabled() && crop.active() != was_active) ++crop_toggled;

        const double z_m = altitude_to_z(altitude, curr_id);
        run.frame_ids.push_back(static_cast<int>(curr_id));
        run.positions.emplace_back(x_m, y_m, z_m);
        run.rotations.push_back(R_abs);

        TrajectoryRow row;
        row.frame = static_cast<int>(curr_id) + 1;
        row.file = sequence->name(curr_id);
        row.x = x_m;
        row.y = y_m;
        row.z = z_m;
        row.altitude = alt_here;
        row.speed_mps = dt > 0 ? step_m / dt : 0.0;
        row.heading_deg = positive_mod(90.0 - rad2deg(heading), 360.0);
        row.turn_deg = rad2deg(turn_rad);
        row.step_m = step_m;
        row.shift_pct = 100.0 * shift_px / frame_w;
        row.frame_step = static_cast<int>(curr_id - prev_id);
        row.matches = from_anchor ? static_cast<int>(anchor_p1.size()) : static_cast<int>(num_matches);
        row.inliers = from_anchor ? anchored.num_inliers : motion.num_inliers;
        row.inlier_ratio_pct = 100.0 * (from_anchor ? anchored.inlier_ratio : motion.inlier_ratio);
        row.step_time_ms = 1000.0 * (now_sec() - loop_start);
        row.status = from_anchor ? "ok" : motion.message;
        row.model = from_anchor ? "anchor" : motion.model;
        row.rotation_mode = was_active;
        row.used_frame_pct = used_pct;
        row.lk_tracks = lk_count;
        if (lk_count > 0) ++lk_steps;
        run.rows.push_back(row);

        bool shown = false;
        const double done = now_sec() - run_start;
        const double rate = static_cast<double>(run.rows.size() - 1) / std::max(done, 1e-6);
        if (verbose && verbose_frames) {
            const double pct = 100.0 * static_cast<double>(curr_id + 1) / std::max<size_t>(limit, 1);
            shown = log.progress(fmt("[%5.1f%%] кадр %zu/%zu  шаг %3zu  смещ %4.1f%%  инлаеров %4d  "
                                     "путь %7.1f м  %5.1f шаг/с",
                                     pct, curr_id + 1, limit, curr_id - prev_id,
                                     100 * shift_px / frame_w, motion.num_inliers, distance_m, rate));
        }
        // Обновляемая строка работает только в терминале; при перенаправлении
        // печатаем сводку раз в полсотни шагов.
        if (verbose && !shown && run.rows.size() % 50 == 0)
            std::cout << "Кадр " << curr_id + 1 << "/" << limit << ": шагов " << run.rows.size() - 1
                      << ", путь " << fmt("%.0f", distance_m) << " м, " << fmt("%.1f", rate)
                      << " шаг/с\n";

        // Управление шагом по измеренному смещению
        sel.update(shift_px, turn_rad, dt, posed);
        live.update(run.positions, run.rows.size() - 1, distance_m);

        if (viewer.enabled()) {
            ViewInfo info;
            info.frame = static_cast<int>(curr_id) + 1;
            info.limit = static_cast<int>(limit);
            info.step = static_cast<int>(curr_id - prev_id);
            info.shift = 100 * shift_px / frame_w;
            info.points = curr_data.num_valid;
            info.matches = static_cast<int>(num_matches);
            info.inliers = motion.num_inliers;
            info.rate = rad2deg(std::abs(turn_rad)) / std::max(dt, 1e-9);
            info.distance = distance_m;
            info.rate_steps = rate;
            const bool keep = viewer.show(curr_image, curr_data.points,
                                          motion.success ? &matched1 : nullptr,
                                          motion.success ? &matched2 : nullptr,
                                          motion.success ? &motion.inlier_mask : nullptr, crop_used, info);
            if (!keep) {
                std::cout << "Прервано пользователем (q).\n";
                break;
            }
        }

        prev_id = curr_id;
        prev_image = curr_image;
        prev_shadow = curr_shadow;
        prev_data = std::move(curr_data);
        curr_data = FeatureData();
        if (curr_id >= limit - 1) break;
    }

    log.end_progress();
    viewer.close();
    live.update(run.positions, run.rows.size() - 1, distance_m, true);
    live.finish();

    const size_t n = run.rows.size();
    run.elapsed_sec = now_sec() - run_start;

    // -------------------------------------------------------------- сохранение
    write_trajectory_csv(run.rows, join(run.folder, "trajectory.csv"));
    plot_trajectory(run.positions, run.folder, alg.name, cfg);
    std::vector<double> alt_rows;
    for (const auto& r : run.rows) alt_rows.push_back(r.altitude);
    write_gps_track(run.positions, run.frame_ids, fps, run.folder, cfg, &alt_rows);
    
    // Экспорт для INS/Kalman
    if (cfg.ins_export.enable) {
        export_for_ins(run.rows, run.positions, run.frame_ids, cfg);
    }

    // ---------------------------------------------------------------- сводка
    const size_t steps = std::max<size_t>(n - 1, 1);
    std::vector<double> speeds, shifts, turns, rates;
    std::vector<double> used_steps;
    for (size_t i = 1; i < n; ++i) {
        const TrajectoryRow& r = run.rows[i];
        if (starts_with(r.status, "ok")) speeds.push_back(r.speed_mps);
        if (r.frame_step > 0) {
            used_steps.push_back(r.frame_step);
            rates.push_back(std::abs(r.turn_deg) / std::max(r.frame_step / std::max(fps, 1e-9), 1e-9));
        }
        if (r.shift_pct > 0) shifts.push_back(r.shift_pct);
    }
    const double mean_speed = speeds.empty() ? 0.0 : std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
    std::cout << "Путь " << fmt("%.1f", distance_m) << " м, средняя скорость " << fmt("%.2f", mean_speed)
              << " м/с, конечная точка X=" << fmt("%.1f", x_m) << " м, Y=" << fmt("%.1f", y_m) << " м\n";
    if (num_clamped)
        std::cout << "Ограничение длины шага сработало на " << num_clamped << " шагах из " << n - 1
                  << ": длина превышала оценку по смещению сцены более чем в " << fmt("%g", step_limit)
                  << " раза. Единичные срабатывания нормальны; если их много, масштаб шага "
                     "восстанавливается неустойчиво - проверьте модель движения и калибровку.\n";

    double alt_min = 1e300, alt_max = -1e300;
    for (const auto& r : run.rows) {
        alt_min = std::min(alt_min, r.altitude);
        alt_max = std::max(alt_max, r.altitude);
    }
    std::cout << "Высота (" << alt_source << "): " << fmt("%.1f..%.1f", alt_min, alt_max)
              << " м, поправка масштаба " << fmt("%g", cfg.geometry.scale_correction) << "\n";

    if (!used_steps.empty()) {
        const auto mm = std::minmax_element(used_steps.begin(), used_steps.end());
        std::cout << "Шаг по кадрам: " << static_cast<int>(*mm.first) << ".." << static_cast<int>(*mm.second)
                  << " (медиана " << static_cast<int>(median(used_steps)) << "), смещение сцены медиана "
                  << fmt("%.1f", shifts.empty() ? 0.0 : median(shifts)) << " % кадра, отступов назад "
                  << retries;
        if (body_yaw.ok()) std::cout << ", разрезов дуги " << arc_splits;
        std::cout << ", отброшено слабых поворотов " << weak_turns << ", шагов от снимка " << anchor_hits
                  << ", смен снимка " << anchor_refresh << "\n";
    }
    if (crop.enabled())
        std::cout << "Режим поворота: активен на " << crop.steps_active() << " шагах из " << steps << " ("
                  << fmt("%.1f", 100.0 * crop.steps_active() / static_cast<double>(steps))
                  << " %), переключений " << crop_toggled << "; угловая скорость - медиана "
                  << fmt("%.2f", rates.empty() ? 0.0 : median(rates)) << ", максимум "
                  << fmt("%.2f", rates.empty() ? 0.0 : *std::max_element(rates.begin(), rates.end()))
                  << " град/с\n";
    if (lk_mode != "off")
        std::cout << "Трекинг Лукаса-Канаде (" << lk_mode << "): " << lk_steps << " шагов из " << steps << "\n";
    if (cfg.shadow.enable)
        std::cout << "Маска теней: кадры с пятном " << shadow_frames << ", снято пар " << shadow_dropped
                  << ", статичных пар " << static_dropped << "\n";
    std::cout << "Обработано шагов: " << steps << " за " << fmt("%.1f", run.elapsed_sec) << " с ("
              << fmt("%.2f", static_cast<double>(steps) / std::max(run.elapsed_sec, 1e-6))
              << " шаг/с) из " << limit << " кадров источника\n";
    if (!model_counts.empty()) {
        std::string parts;
        for (const auto& kv : model_counts)
            parts += (parts.empty() ? "" : ", ") + kv.first + ": " + std::to_string(kv.second);
        std::cout << "Модель движения по шагам - " << parts << "\n";
    }
    if (profile) {
        double total_stage = 1e-9;
        for (double s : stage) total_stage += s;
        std::vector<int> order(kNumStages);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b) { return stage[a] > stage[b]; });
        std::cout << "Время по этапам (на обработанный кадр):\n";
        auto u8len = [](const std::string& s) {
            size_t k = 0;
            for (unsigned char c : s) k += ((c & 0xC0) != 0x80);
            return k;
        };
        for (int i : order)
            if (stage[i] > 0) {
                const std::string name = stage_names[i];
                std::cout << "  " << name << std::string(24 - std::min<size_t>(24, u8len(name)), ' ')
                          << fmt("%8.1f мс %5.1f %%", stage[i] / static_cast<double>(steps) * 1000,
                                 100 * stage[i] / total_stage)
                          << "\n";
            }
    }
    if (verbose) std::cout << "Траектория сохранена в " << run.folder << "\n";
    return run;
}

}  // namespace

VoRun run_visual_odometry(const Config& cfg, Sequence* sequence, const std::string& algorithm,
                          bool verbose) {
    // Весь консольный вывод прогона дублируется в console_log.txt в папке метода.
    ConsoleLog log;
    return run_body(cfg, sequence, algorithm, verbose, log);
}

// ---------------------------------------------------------------------------
// Траектории всеми методами
// ---------------------------------------------------------------------------

std::vector<TrajectorySummary> run_all_visual_odometry(Config cfg, bool verbose) {
    const std::vector<Algorithm> algorithms = select_algorithms(cfg);
    if (algorithms.empty()) throw std::runtime_error("Нет доступных алгоритмов детектирования/описания.");
    if (!cfg.vo.algorithms.empty()) {
        std::string names;
        for (const auto& a : algorithms) names += (names.empty() ? "" : ", ") + a.name;
        std::cout << "Отобрано методов: " << names << "\n";
    }

    const std::string compare_folder = join(join(cfg.results_folder, "visual_odometry"), "comparison");
    make_dirs(compare_folder);
    ConsoleLog log(join(compare_folder, cfg.export_cfg.console_file));

    cfg.vo.max_saved_match_figures = std::min(cfg.vo.max_saved_match_figures, 2);

    // Источник открывается один раз на все методы - для видео это критично.
    std::unique_ptr<Sequence> sequence = open_sequence(cfg.data_folder, cfg);
    if (!sequence->length_known())
        std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Источник не поддерживает перемотку назад, поэтому прогнать по "
                     "нему несколько методов подряд нельзя. Используйте режим vo (один метод) или "
                     "скачайте видео в файл.\n";

    std::vector<std::string> names;
    std::vector<std::vector<cv::Vec3d>> positions_list;
    std::vector<TrajectorySummary> rows;
    std::cout << "\n=== ТРАЕКТОРИИ ДЛЯ " << algorithms.size() << " АЛГОРИТМОВ ===\n";

    for (size_t i = 0; i < algorithms.size(); ++i) {
        const Algorithm& alg = algorithms[i];
        std::cout << "\n--- [" << i + 1 << "/" << algorithms.size() << "] " << alg.name << " ---\n";
        const double t0 = now_sec();
        VoRun run;
        try {
            run = run_visual_odometry(cfg, sequence.get(), alg.name, false);
        } catch (const std::exception& e) {
            std::cout << "[ПРЕДУПРЕЖДЕНИЕ] " << alg.name << " пропущен: " << e.what() << "\n";
            continue;
        }
        const double elapsed = now_sec() - t0;

        int steps_ok = 0;
        std::vector<double> m, in, ratio;
        for (size_t k = 0; k < run.rows.size(); ++k) {
            if (starts_with(run.rows[k].status, "ok")) ++steps_ok;
            if (k > 0) {
                m.push_back(run.rows[k].matches);
                in.push_back(run.rows[k].inliers);
                ratio.push_back(run.rows[k].inlier_ratio_pct);
            }
        }
        double path = 0.0;
        for (size_t k = 1; k < run.positions.size(); ++k) path += cv::norm(run.positions[k] - run.positions[k - 1]);

        TrajectorySummary s;
        s.algorithm = alg.name;
        s.num_frames = static_cast<int>(run.rows.size());
        s.successful_steps = steps_ok;
        s.mean_matches = safe_mean(m);
        s.mean_inliers = safe_mean(in);
        s.mean_inlier_ratio_pct = safe_mean(ratio);
        s.path_length = path;
        s.final_x = run.positions.back()[0];
        s.final_y = run.positions.back()[1];
        s.final_z = run.positions.back()[2];
        s.elapsed_sec = elapsed;
        rows.push_back(s);
        names.push_back(alg.name);
        positions_list.push_back(run.positions);

        std::cout << "Готово за " << fmt("%.1f", elapsed) << " с: успешных шагов " << steps_ok << "/"
                  << run.rows.size() - 1 << ", длина пути " << fmt("%.2f", path) << "\n";
    }

    CsvTable table;
    table.columns = {"Algorithm", "NumFrames", "SuccessfulSteps", "MeanMatches", "MeanInliers",
                     "MeanInlierRatioPct", "PathLength", "FinalX", "FinalY", "FinalZ", "ElapsedSec"};
    CsvTable pretty = table;
    for (const auto& s : rows) {
        table.rows.push_back({s.algorithm, std::to_string(s.num_frames), std::to_string(s.successful_steps),
                              fmt_g15(s.mean_matches), fmt_g15(s.mean_inliers), fmt_g15(s.mean_inlier_ratio_pct),
                              fmt_g15(s.path_length), fmt_g15(s.final_x), fmt_g15(s.final_y), fmt_g15(s.final_z),
                              fmt_g15(s.elapsed_sec)});
        pretty.rows.push_back({s.algorithm, std::to_string(s.num_frames), std::to_string(s.successful_steps),
                               fmt("%.2f", s.mean_matches), fmt("%.2f", s.mean_inliers),
                               fmt("%.2f", s.mean_inlier_ratio_pct), fmt("%.2f", s.path_length),
                               fmt("%.2f", s.final_x), fmt("%.2f", s.final_y), fmt("%.2f", s.final_z),
                               fmt("%.2f", s.elapsed_sec)});
    }
    if (!table.rows.empty()) table.write(join(compare_folder, "trajectory_summary.csv"));
    plot_trajectory_comparison(names, positions_list, compare_folder, cfg);

    if (verbose) {
        std::cout << "\n";
        pretty.print(std::cout);
        std::cout << "\nСравнительные графики: " << compare_folder << "\n";
    }
    return rows;
}

}  // namespace vo
