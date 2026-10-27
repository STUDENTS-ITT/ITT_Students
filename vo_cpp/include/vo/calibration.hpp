// Внутренние параметры камеры и их загрузка (порт раздела 3 vo_final2.py).
//
// Формат файла - JSON (или YAML/XML OpenCV FileStorage), ключи те же, что у
// Python-версии:
//   {"fx":..., "fy":..., "cx":..., "cy":..., "imageSize":[h,w],
//    "distortion":[k1,k2,p1,p2,k3]}
// либо матрица "K" (3x3 или 9 чисел), либо "focal" + "principal".
// Координаты главной точки - 0-based (OpenCV).
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vo/config.hpp"

namespace vo {

struct Intrinsics {
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int height = 0, width = 0;
    std::vector<double> distortion = std::vector<double>(5, 0.0);

    cv::Matx33d K() const {
        return cv::Matx33d(fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }
    // Масштабирование под другой размер кадра (дисторсия сбрасывается).
    Intrinsics scaled(int new_height, int new_width) const;
    std::string str() const;
};

struct Calibration {
    Intrinsics intrinsics;
    bool is_approximate = false;
    std::string source;
    bool has_distortion() const;
};

// Бросает std::runtime_error, если калибровки нет и приближение запрещено.
Calibration load_calibration(const Config& cfg, int image_height, int image_width);

}  // namespace vo
