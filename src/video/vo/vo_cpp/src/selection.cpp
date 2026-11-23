#include "vo/selection.hpp"

#include <algorithm>
#include <cmath>

#include "vo/util.hpp"

namespace vo {

FrameSelector::FrameSelector(const Config& cfg, double frame_width) {
    const FrameSelectCfg& sel = cfg.frame_select;
    mode_ = lower(sel.mode);
    width_ = frame_width;
    min_step_ = std::max(1, sel.min_step);
    max_step_ = std::max(min_step_, sel.max_step);
    cruise_max_step_ = std::max(min_step_, std::min(sel.cruise_max_step, max_step_));
    cruise_altitude_m_ = sel.cruise_altitude_m;
    active_max_ = max_step_;
    target_ = sel.target_shift_pct / 100.0;
    damping_ = std::min(std::max(sel.damping, 0.0), 1.0);
    max_growth_ = sel.max_growth;
    turn_limit_ = deg2rad(sel.turn_limit_dps);
    turn_reject_ = deg2rad(sel.turn_reject_dps);
    turn_reject_min_ = deg2rad(sel.turn_reject_min_deg);
    turn_noise_floor_ = deg2rad(sel.turn_noise_floor_deg);
    turn_gate_frac_ = sel.turn_gate_fraction;
    fixed_step_ = std::max(1, cfg.frame_step);
    step_f_ = (mode_ == "fixed")
                  ? static_cast<double>(fixed_step_)
                  : static_cast<double>(std::min(std::max(sel.start_step, min_step_), max_step_));
}

int FrameSelector::step() const {
    const double r = round_half_even(step_f_);
    return static_cast<int>(std::min(std::max(r, static_cast<double>(min_step_)),
                                     static_cast<double>(active_max_)));
}

void FrameSelector::set_altitude(double altitude_m) {
    const int cap = (altitude_m >= cruise_altitude_m_) ? cruise_max_step_ : max_step_;
    active_max_ = cap;
    if (step_f_ > static_cast<double>(cap)) step_f_ = static_cast<double>(cap);
}

std::string FrameSelector::describe() const {
    if (mode_ == "fixed") return fmt("Выбор кадров: постоянный шаг %d", fixed_step_);
    return fmt("Выбор кадров: адаптивный, цель по смещению сцены %.0f %% ширины кадра, "
               "шаг в пределах %d..%d, старт %d; на высоте от %.0f м не больше %d кадров",
               100 * target_, min_step_, max_step_, step(), cruise_altitude_m_, cruise_max_step_);
}

void FrameSelector::shrink() {
    step_f_ = std::max(static_cast<double>(min_step_), step_f_ / 2.0);
}

bool FrameSelector::too_sharp(double turn_rad, double dt) const {
    // По умолчанию выключено (turn_reject_dps = 0): при развороте отбраковка
    // дробит шаг до минимума, и оценки поворота тонут в шуме.
    if (mode_ == "fixed" || dt <= 0 || turn_reject_ <= 0) return false;
    const double turn = std::abs(turn_rad);
    return turn / dt > turn_reject_ && turn > turn_reject_min_;
}

void FrameSelector::update(double shift_px, double turn_rad, double dt, bool success) {
    if (mode_ == "fixed") {
        step_f_ = static_cast<double>(fixed_step_);
        return;
    }
    if (!success) {
        shrink();
        return;
    }

    const double frac = shift_px / std::max(width_, 1e-9);
    double gain;
    if (frac <= 1e-6) {
        // Камера практически стоит: наращиваем шаг осторожно.
        gain = max_growth_;
    } else {
        gain = target_ / frac;
    }

    // Ограничение по угловой скорости включается, только когда оценка поворота
    // достоверна: сцена сместилась заметно И сам поворот больше шума.
    const double turn = std::abs(turn_rad);
    const bool observable = frac >= turn_gate_frac_ * target_;
    if (observable && turn > turn_noise_floor_) {
        const double rate = turn / std::max(dt, 1e-9);
        if (rate > 1e-9) gain = std::min(gain, turn_limit_ / rate);
    }

    gain = std::min(std::max(gain, 1.0 / max_growth_), max_growth_);
    gain = 1.0 + damping_ * (gain - 1.0);   // демпфирование

    step_f_ = std::min(std::max(step_f_ * gain, static_cast<double>(min_step_)),
                       static_cast<double>(active_max_));
}

// ----------------------------------------------------------------------------

RotationCrop::RotationCrop(const Config& cfg) {
    const RotationCropCfg& s = cfg.rotation_crop;
    enabled_ = s.enable;
    threshold_ = deg2rad(s.threshold_dps);   // рад/с
    delay_ = s.delay_steps;
    rotation_cut_ = s.rotation_cut;
    normal_cut_ = s.normal_cut;
    angle_source_ = lower(s.angle_source);
    active_ = s.at_start && enabled_;
}

std::array<double, 4> RotationCrop::current_cut() const {
    if (!enabled_) return normal_cut_;
    return active_ ? rotation_cut_ : normal_cut_;
}

std::array<double, 4> RotationCrop::bounds(const cv::Size& size) const {
    const auto c = current_cut();
    const double w = size.width, h = size.height;
    return {w * c[0], w * (1.0 - c[1]), h * c[2], h * (1.0 - c[3])};
}

double RotationCrop::crop_fraction() const {
    const auto c = current_cut();
    return std::max(0.0, 1.0 - c[0] - c[1]) * std::max(0.0, 1.0 - c[2] - c[3]);
}

double RotationCrop::angle_from_rotation(const cv::Matx33d& R, const std::string& source) {
    for (int i = 0; i < 9; ++i)
        if (!std::isfinite(R.val[i])) return 0.0;
    if (source == "total") {
        const double c = (R(0, 0) + R(1, 1) + R(2, 2) - 1.0) / 2.0;
        return std::acos(std::min(1.0, std::max(-1.0, c)));
    }
    return std::atan2(R(1, 0), R(0, 0));
}

double RotationCrop::update(double turn_rad, double dt) {
    last_angle_ = turn_rad;
    if (!enabled_) return turn_rad;

    const double rate = std::abs(turn_rad) / std::max(dt, 1e-9);
    if (rate > threshold_) {
        if (!active_) {
            active_ = true;
            since_change_ = 0;
        }
        ++since_change_;
    } else if (active_) {
        ++since_change_;
        if (since_change_ > delay_) {
            active_ = false;
            since_change_ = 0;
        }
    }
    if (active_) ++steps_active_;
    return turn_rad;
}

}  // namespace vo
