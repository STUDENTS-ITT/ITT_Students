// Трекинг точек оптическим потоком Лукаса-Канаде между соседними кадрами.
#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "vo/config.hpp"

namespace vo {

struct LkTracks {
    std::vector<int> index;            // номер исходной точки в seeds
    std::vector<cv::Point2f> p1, p2;   // положение в предыдущем и текущем кадре
};

// Пирамидальный LK с проверкой прямого-обратного трекинга. Если задана
// гомография H_guess (3x3, CV_64F), начальное положение точек в текущем кадре
// берётся из неё - так LK выдерживает большие повороты.
LkTracks track_points_lk(const cv::Mat& prev, const cv::Mat& curr,
                         const std::vector<cv::Point2f>& seeds, const LkCfg& cfg,
                         const cv::Mat& H_guess = cv::Mat());

}  // namespace vo
