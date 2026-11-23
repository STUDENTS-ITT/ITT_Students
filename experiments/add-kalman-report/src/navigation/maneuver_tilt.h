// maneuver_tilt.h — коррекция pitch/roll после манёвра (без angle.dat).

#pragma once

#include "../ins/attitude_calc.h"
#include "../ins/ins_filter.h"
#include "../utils/constants.h"
#include "../utils/types.h"

namespace nav
{

struct TiltManeuverPolicy
{
    double gyro_off_rad_s = 0.08;
    double settle_s = 0.35;
    double brake_body_accel = 0.7; // м/с², |V̇_body x/y| — торможение/разгон

    // Апериодическое сведение к акселю: быстрое при большой ошибке,
    // медленное у нуля — иначе шум акселя попадает прямо в угол.
    double tau_fast_s = 0.3;
    double tau_slow_s = 4.0;
    double fast_err_rad = 1.0 * DEG_TO_RAD;
    double level_acc_rad = 1.5 * DEG_TO_RAD;

    // Интегральная оценка дрейфа θ̇/φ̇: без неё пропорциональная коррекция
    // уравновешивает дрейф на постоянной ошибке ω_дрейф · tau.
    double bias_gain = 0.2;                        // 1/с²
    double bias_max_rad_s = 0.3 * DEG_TO_RAD;

    double divergence_deadband_rad = 0.25 * DEG_TO_RAD;
};

// Оценка дрейфа угловых скоростей тангажа/крена (интегральный член).
struct TiltTrimState
{
    double bias_pitch = 0.0;
    double bias_roll = 0.0;
};

inline bool isBrakingManeuver(const Vector &V_dot_body, const TiltManeuverPolicy &pol = {})
{
    return fabs(V_dot_body[0]) > pol.brake_body_accel ||
           fabs(V_dot_body[1]) > pol.brake_body_accel;
}

inline bool isLevelTiltSettled(double gyro_mag,
                               const Vector &V_dot_body,
                               const TiltManeuverPolicy &pol = {})
{
    return gyro_mag <= pol.gyro_off_rad_s && !isBrakingManeuver(V_dot_body, pol);
}

// Тангаж/крен по сырому акселю: без компенсации V̇_body, которая при
// ошибочной C(θ) «подтверждает» ошибку угла.
inline double rawAccelPitch(const Vector &f_body)
{
    return atan2(f_body[0], f_body[1]);
}

inline double rawAccelRoll(const Vector &f_body)
{
    return -atan2(f_body[2], f_body[1]);
}

// Вычитание оценённого дрейфа из скоростей Эйлера перед интегрированием.
inline void applyTiltRateBias(ins::AttitudeRates &rates, const TiltTrimState &trim)
{
    rates.pitch_dot -= trim.bias_pitch;
    rates.roll_dot -= trim.bias_roll;
}

// Запрет наращивать расхождение с акселем: скорость, уводящая угол от
// показаний акселя, обнуляется (сведение к акселю при этом не мешает).
inline void limitAxisDivergence(double &rate, double att, double raw,
                                const TiltManeuverPolicy &pol)
{
    const double delta = normalize_angle(raw - att);
    if (fabs(delta) < pol.divergence_deadband_rad)
        return;
    if (delta * rate < 0.0)
        rate = 0.0;
}

inline bool limitTiltDivergence(ins::AttitudeRates &rates,
                                const ins::Attitude &att,
                                const Vector &f_body,
                                double gyro_mag,
                                const Vector &V_dot_body,
                                const TiltManeuverPolicy &pol = {})
{
    if (!isLevelTiltSettled(gyro_mag, V_dot_body, pol))
        return false;

    const double raw_pitch = rawAccelPitch(f_body);
    const double raw_roll = rawAccelRoll(f_body);

    if (fabs(raw_pitch) >= pol.level_acc_rad || fabs(raw_roll) >= pol.level_acc_rad)
        return false;

    limitAxisDivergence(rates.pitch_dot, att.pitch, raw_pitch, pol);
    limitAxisDivergence(rates.roll_dot, att.roll, raw_roll, pol);
    return true;
}

inline void trimTiltAxis(double &att, double &bias, double raw, double dt,
                         double trust, const TiltManeuverPolicy &pol)
{
    const double delta = normalize_angle(raw - att);
    const double tau = (fabs(delta) > pol.fast_err_rad) ? pol.tau_fast_s
                                                        : pol.tau_slow_s;
    att = normalize_angle(att + fmin(1.0, trust * dt / tau) * delta);

    bias = fmax(-pol.bias_max_rad_s,
                fmin(pol.bias_max_rad_s,
                     bias - trust * pol.bias_gain * delta * dt));
}

// Сведение углов к горизонту, когда манёвр закончился и аксель показывает level.
inline bool trimManeuverTilt(ins::Attitude &att,
                             TiltTrimState &trim,
                             const Vector &f_body,
                             double gyro_mag,
                             const Vector &V_dot_body,
                             double dt,
                             const TiltManeuverPolicy &pol = {})
{
    if (!isLevelTiltSettled(gyro_mag, V_dot_body, pol))
        return false;

    const double raw_pitch = rawAccelPitch(f_body);
    const double raw_roll = rawAccelRoll(f_body);

    if (fabs(raw_pitch) >= pol.level_acc_rad || fabs(raw_roll) >= pol.level_acc_rad)
        return false;

    // Чем тише гироскоп, тем достовернее «аксель = горизонт»: на хвосте
    // манёвра машина ещё доворачивает, и гнаться за акселем нельзя.
    const double trust = 1.0 - gyro_mag / pol.gyro_off_rad_s;

    trimTiltAxis(att.pitch, trim.bias_pitch, raw_pitch, dt, trust, pol);
    trimTiltAxis(att.roll, trim.bias_roll, raw_roll, dt, trust, pol);
    return true;
}

inline bool shouldApplyAccelTilt(double time_s,
                                 double maneuver_time_s,
                                 double gyro_mag,
                                 bool acc_ok,
                                 double acc_pitch,
                                 double acc_roll,
                                 const Vector &V_dot_body,
                                 const ins::Attitude &att,
                                 const ins::AttitudeRates &rates,
                                 const TiltManeuverPolicy &pol = {})
{
    if (!acc_ok)
        return false;

    if (gyro_mag > pol.gyro_off_rad_s)
        return false;

    if (isBrakingManeuver(V_dot_body, pol))
        return false;

    if ((time_s - maneuver_time_s) < pol.settle_s)
        return false;

    const double dp = fabs(normalize_angle(acc_pitch - att.pitch));
    const double dr = fabs(normalize_angle(acc_roll - att.roll));

    // Глубокий «нос вниз» на торможении: аксель ≈ 0° не должен тянуть INS вверх.
    const bool pitch_dynamic = (att.pitch < -3.0 * DEG_TO_RAD) &&
                               (acc_pitch > -1.0 * DEG_TO_RAD) &&
                               (dp > DEG_TO_RAD);
    const bool roll_dynamic = (fabs(att.roll) > DEG_TO_RAD) &&
                              (fabs(acc_roll) < 2.0 * DEG_TO_RAD) &&
                              (dr > DEG_TO_RAD);

    if (pitch_dynamic || roll_dynamic)
        return false;

    if (fabs(rates.pitch_dot) > 2.0 * DEG_TO_RAD ||
        fabs(rates.roll_dot) > 2.0 * DEG_TO_RAD)
        return false;

    return true;
}

inline void applyAccelTiltCorrection(const ins::Attitude &att,
                                     double pitch_meas,
                                     double roll_meas,
                                     double sig_pitch,
                                     double sig_roll,
                                     Vector &x,
                                     Matrix &P)
{
    ins::correctTilt(att.pitch, att.roll,
                     pitch_meas, roll_meas,
                     sig_pitch, sig_roll,
                     x, P);
}

inline void clearTiltKalmanErrors(Vector &x)
{
    x[7] = 0.0;
    x[8] = 0.0;
}

} // namespace nav
