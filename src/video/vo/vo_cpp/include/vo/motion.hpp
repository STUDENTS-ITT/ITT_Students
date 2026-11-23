// Оценка движения между двумя кадрами по гомографии (порт раздела 9 vo_final2.py).
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vo/calibration.hpp"
#include "vo/config.hpp"

namespace vo {

struct MotionResult {
    bool success = false;
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d translation{0, 0, 0};          // единичное направление в СК накопления
    cv::Matx33d essential = cv::Matx33d::all(std::nan(""));
    std::vector<char> inlier_mask;           // 1 - инлаер
    int num_inliers = 0;
    double inlier_ratio = 0.0;
    double valid_fraction = 0.0;
    double time_sec = 0.0;
    int status_code = -1;
    std::string message = "not_estimated";
    std::string model;                       // similarity | homography | essential
    std::optional<double> step_length;       // метры, если удалось получить
    double scale_ratio = 1.0;                // масштаб сцены между кадрами
};

// xy1, xy2 - координаты сопоставленных точек в пикселях (0-based).
// altitude - высота съёмки, м (нужна для шага в метрах).
MotionResult estimate_motion(const std::vector<cv::Point2d>& xy1,
                             const std::vector<cv::Point2d>& xy2,
                             const Intrinsics& intrinsics, const Config& cfg,
                             std::optional<double> altitude = std::nullopt);

// Проекция матрицы на SO(3).
cv::Matx33d orthonormalize_rotation(const cv::Matx33d& R);

// Вращение вокруг вертикали (оптической оси надирной камеры).
cv::Matx33d rot_z(double psi);

// Поворот вокруг вертикали из матрицы шага, рад.
double turn_from_rotation(const cv::Matx33d& R);

// Смещение сцены в пикселях -> перемещение камеры (вправо, вперёд), м.
cv::Point2d pixels_to_meters(const cv::Point2d& shift_px, double altitude_m,
                             const Intrinsics& intrinsics, const Config& cfg);

}  // namespace vo
