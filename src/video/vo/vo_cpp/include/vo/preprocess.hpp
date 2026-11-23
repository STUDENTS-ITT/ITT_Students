// Предобработка кадра: полутон uint8, дисторсия, CLAHE, заливка OSD
// (порт раздела 5 vo_final2.py).
#pragma once

#include <opencv2/core.hpp>

#include "vo/calibration.hpp"
#include "vo/config.hpp"

namespace vo {

// rgb2gray + приведение к uint8.
cv::Mat to_gray_uint8(const cv::Mat& raw);

// Замена областей OSD ровным фоном (средняя яркость) с размытием границы.
cv::Mat blank_osd(const cv::Mat& image, const Config& cfg);

// Полутоновый uint8-кадр, готовый к детектированию.
cv::Mat preprocess_frame(const cv::Mat& raw, const Calibration& calib, const Config& cfg);

}  // namespace vo
