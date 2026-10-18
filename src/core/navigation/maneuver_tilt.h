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
    // Порог «тихого» гироскопа: ниже него акселерометру можно верить как
    // горизонту, выше — угол ведём интегрированием гироскопа. Для
    // согласованного аппарата в развороте-разгоне аксель «слеп» к наклону
    // (f_б ≈ (0, |f|, 0)), поэтому порог должен быть у шума ДУС (~2σ), а не
    // «на глаз»: при большом пороге акселерометрное сведение держало тангаж
    // у нуля вместо истинных 5–7°.
    double gyro_off_rad_s = 0.005;
    double settle_s = 0.35;
    double brake_body_accel = 0.7; // м/с², |V̇_body x/y| — торможение/разгон

    // Порог «горизонтальности» по акселю: за ним limitTiltDivergence не
    // работает (в манёвре он бесполезен и вреден).
    double level_acc_rad = 1.5 * DEG_TO_RAD;

    double divergence_deadband_rad = 0.25 * DEG_TO_RAD;
};

// ЭТАП 1 (R1) — УДАЛЕНО: TiltTrimState, trimDriftIncrement, trimManeuverTilt.
//
// Это был скрытый оцениватель дрейфа тангажа/крена: интеграл bias_pitch/
// bias_roll вычитался из скоростей Эйлера (applyTiltRateBias) и подтягивал
// угол напрямую (trimTiltAxis). Ветка жила вне фильтра — конкурировала с
// x[12..14], не имела ковариации и не выводилась ни в телеметрию, ни в
// runs.db, ни в отчёт.
//
// Опыт по 9 наборам показал, что трим в оценке смещений не участвовал вовсе:
// счётчики «сходится/не сходится/не наблюдаемо» после его удаления совпали с
// baseline на всех наборах, а на самом жёстком (square_harsh) ошибка тангажа
// упала на 30 %, крена — на 13 %, широты — на 39 %. Инжекция его интеграла в
// bg (вариант «b» плана) оказалась вредной: оценка уходила к клампу
// bias_max_rad_s, превышая истину в 10–20 раз.
//
// Поля политики tau_fast_s/tau_slow_s/fast_err_rad/bias_gain/bias_max_rad_s
// удалены вместе с механикой — они обслуживали только прямую подтяжку угла.

// Глобальная политика сведения тангажа/крена.
//
// Значения по умолчанию совпадают с TiltManeuverPolicy, поэтому поведение
// не меняется. Переопределяется из settings.ini (nav::applySettings) или из
// GUI. Объявлена здесь, чтобы шаг счисления (trajectory.h) не зависел от
// filter_settings.h.
TiltManeuverPolicy &tiltPolicy();

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

// Интегральная оценка дрейфа одной оси без прямой подтяжки угла (Этап 1).
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
