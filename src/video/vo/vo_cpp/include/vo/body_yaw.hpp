// Рыскание корпуса по кадрам камеры (кватернион DJI OSDK или |ω_z|).
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace vo {

class BodyYawTrack {
public:
    bool load(const std::string& camera_csv, const std::string& attitude_csv);
    bool ok() const { return ok_; }
    size_t frames() const { return yaw_.size(); }
    // Развёрнутый курс корпуса на кадре, рад.
    double yaw_rad(size_t frame) const;
    // yaw(frame_b) - yaw(frame_a), рад.
    double delta_rad(size_t frame_a, size_t frame_b) const;
    // Пик |d(yaw)/dt| на (frame_a, frame_b], град/с.
    double rate_dps(size_t frame_a, size_t frame_b) const;

private:
    bool ok_ = false;
    std::vector<double> yaw_;
    std::vector<double> cam_t_;
};

// Поворот 2D-точки вокруг (cx, cy), рад (положительный = против часовой).
void rotate_points_2d(std::vector<cv::Point2f>& pts, double cx, double cy, double rad);

// Распределить step_m вдоль дуги по приращениям рыскания между кадрами [a, b].
// h0 — курс VO в начале шага (рад); x0,y0 — поза якоря. Возвращает конечные x,y.
void integrate_arc_xy(double x0, double y0, double h0, double step_m, size_t frame_a, size_t frame_b,
                      const BodyYawTrack& body, double& x_out, double& y_out);

}  // namespace vo
