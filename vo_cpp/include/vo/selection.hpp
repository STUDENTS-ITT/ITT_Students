// Выбор кадров и обрезка на поворотах (порт FrameSelector и RotationCrop).
#pragma once

#include <array>
#include <string>

#include <opencv2/core.hpp>

#include "vo/config.hpp"

namespace vo {

// Адаптивный выбор следующего обрабатываемого кадра: шаг подстраивается так,
// чтобы смещение сцены между кадрами держалось около TargetShiftPct ширины
// кадра; отдельный контур ограничивает угловую скорость; неудача -> откат.
class FrameSelector {
public:
    FrameSelector(const Config& cfg, double frame_width);

    int step() const;
    // На крейсерской высоте потолок шага уменьшается (см. cruise_max_step).
    void set_altitude(double altitude_m);
    std::string describe() const;
    bool can_shrink() const { return mode_ != "fixed" && step() > min_step_; }
    void shrink();
    bool too_sharp(double turn_rad, double dt) const;
    void update(double shift_px, double turn_rad, double dt, bool success);

private:
    std::string mode_;
    double width_;
    int min_step_, max_step_, cruise_max_step_;
    double cruise_altitude_m_;
    int active_max_;
    double target_, damping_, max_growth_;
    double turn_limit_, turn_reject_, turn_reject_min_, turn_noise_floor_, turn_gate_frac_;
    int fixed_step_;
    // Шаг ведётся вещественным: иначе из значения 1 он не может вырасти.
    double step_f_;
};

class RotationCrop {
public:
    explicit RotationCrop(const Config& cfg);

    bool enabled() const { return enabled_; }
    bool active() const { return active_; }
    int steps_active() const { return steps_active_; }
    double threshold() const { return threshold_; }   // рад/с
    int delay() const { return delay_; }
    const std::array<double, 4>& rotation_cut() const { return rotation_cut_; }

    // Доли (слева, справа, сверху, снизу), которые сейчас не используются.
    std::array<double, 4> current_cut() const;
    // (x_min, x_max, y_min, y_max) в пикселях.
    std::array<double, 4> bounds(const cv::Size& image_size) const;
    double crop_fraction() const;

    // Угол поворота между кадрами, рад. R - матрица 3x3 (row-major).
    static double angle_from_rotation(const cv::Matx33d& R, const std::string& source);

    // Учесть поворот очередного шага (turn_rad за dt секунд).
    double update(double turn_rad, double dt);

private:
    bool enabled_;
    double threshold_;
    int delay_;
    std::array<double, 4> rotation_cut_, normal_cut_;
    std::string angle_source_;
    bool active_;
    int since_change_ = 0;
    int steps_active_ = 0;
    double last_angle_ = 0.0;
};

}  // namespace vo
