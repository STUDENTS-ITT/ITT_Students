// Окна просмотра: обрабатываемый видеопоток и живая траектория.
#pragma once

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vo/config.hpp"

namespace vo {

struct CropUsed {
    bool active = false;
    std::array<double, 4> bounds{0, 0, 0, 0};   // x_min, x_max, y_min, y_max
    double fraction = 1.0;
};

struct ViewInfo {
    int frame = 0, limit = 0, step = 0;
    double shift = 0;        // % ширины кадра
    int points = 0, matches = 0, inliers = 0;
    double rate = 0;         // угловая скорость, град/с
    double distance = 0;     // путь, м
    double rate_steps = 0;   // шагов в секунду
};

// Показывает кадр ПОСЛЕ предобработки, отобранные точки (синие), инлаеры
// (зелёные, со следами) и отбракованные (красные), зону обрезки на поворотах.
// q - выход, пробел - пауза.
class FrameViewer {
public:
    explicit FrameViewer(const Config& cfg);
    ~FrameViewer();
    bool enabled() const { return enabled_; }

    // false - пользователь просит прервать прогон.
    bool show(const cv::Mat& image, const std::vector<cv::Point2f>& points,
              const std::vector<cv::Point2f>* matched1, const std::vector<cv::Point2f>* matched2,
              const std::vector<char>* inliers, const CropUsed& crop, const ViewInfo& info);
    void close();

private:
    void draw_hud(cv::Mat& vis, const ViewInfo& info, bool crop_active, double crop_fraction) const;
    bool enabled_ = false;
    bool paused_ = false;
    int width_ = 1100;
    bool draw_points_ = true, draw_tracks_ = true, dim_cropped_ = true;
    std::string window_ = "Visual odometry - current frame";
};

// Траектория в плоскости (XY|XZ|YZ), обновляемая по ходу работы.
class LiveTrajectoryPlot {
public:
    explicit LiveTrajectoryPlot(const Config& cfg);
    ~LiveTrajectoryPlot();
    bool enabled() const { return enabled_; }
    void update(const std::vector<cv::Vec3d>& positions, size_t steps, double distance, bool force = false);
    void finish();

private:
    bool enabled_ = false;
    int every_ = 10;
    int col_x_ = 0, col_y_ = 1;
    std::string plane_;
    std::string window_ = "Visual odometry - trajectory";
};

}  // namespace vo
