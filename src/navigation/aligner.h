// aligner.h — Формирование начального состояния навигации и ковариации P₀.
//
// Содержит:
//   - passport initialCovariance — ковариационная матрица по приорам/floors
//   - initialCovarianceFromAveraging — P₀ по окну усреднения (σ_sample/σ_mean, k(N))
//   - initialErrorsFromAveraging — x₀ = (последний отсчёт окна − среднее)
//   - referenceAttitude / initialAlignment — совместимость со старой схемой

#pragma once

#include <algorithm>
#include <cmath>

#include "../ins/attitude_calc.h"
#include "../ins/ins_filter.h"
#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "averaging.h"
#include "gps_processor.h"
#include "trajectory.h"

namespace nav
{

// Паспортная (fallback) ковариационная матрица P₀.
// sig_heading <= 0 — формула gyro-compass: cfg.sig_bg / (cos(lat)*U_EARTH).
inline Matrix initialCovariance(const ins::KalmanConfig &cfg, double lat_rad,
                                double sig_heading = -1.0)
{
    // Паспортные неопределённости смещений (floors).
    const double sig_ba = 5.0e-2;  // 5 мг, м/с²
    const double sig_bg = 5.0e-4;  // ~103 °/ч, рад/с

    const double sig_hdg0 = (sig_heading > 0.0)
                                ? sig_heading
                                : cfg.sig_bg / fmax(fabs(cos(lat_rad)) * U_EARTH, 1e-10);

    const double sigma[ins::KF_STATE] = {
        2.0 * cfg.sig_pos, 2.0 * cfg.sig_pos, 2.0 * cfg.sig_h,
        2.5, 2.5, 2.5,
        sig_hdg0, 1.0 * DEG_TO_RAD, 1.0 * DEG_TO_RAD,
        sig_ba, sig_ba, sig_ba,
        sig_bg, sig_bg, sig_bg};

    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = sigma[i] * sigma[i];

    // Корреляции «наклон ↔ горизонтальное смещение акселерометра».
    const double g = normalGravity(lat_rad, 0.0);
    at(P0, 7, 9, ins::KF_STATE) = sigma[9] * sigma[9] / g;
    at(P0, 9, 7, ins::KF_STATE) = sigma[9] * sigma[9] / g;
    at(P0, 8, 11, ins::KF_STATE) = -sigma[11] * sigma[11] / g;
    at(P0, 11, 8, ins::KF_STATE) = -sigma[11] * sigma[11] / g;

    return P0;
}

// Совместимость: P₀ по текущей глобальной конфигурации.
inline Matrix initialCovariance(double lat_rad)
{
    return initialCovariance(ins::kalman_cfg, lat_rad);
}

// Совместимость со старой сигнатурой без широты (использует cfg.sig_pos).
inline Matrix initialCovariance()
{
    const double sig_ba = 5.0e-2;
    const double sig_bg = 5.0e-4;
    const double sigma[ins::KF_STATE] = {
        2.0 * ins::kalman_cfg.sig_pos, 2.0 * ins::kalman_cfg.sig_pos, 2.0 * ins::kalman_cfg.sig_h,
        2.5, 2.5, 2.5,
        ins::kalman_cfg.sig_hdg, 1.0 * DEG_TO_RAD, 1.0 * DEG_TO_RAD,
        sig_ba, sig_ba, sig_ba,
        sig_bg, sig_bg, sig_bg};
    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = sigma[i] * sigma[i];
    return P0;
}

// P₀ по окну усреднения (перенос из ITT_Students_NoQT).
// Номинал pos/vel/hdg — последний отсчёт окна => σ_sample; наклон/ba/bg —
// оценки по среднему => σ_mean. При !rest_ok — паспортный fallback.
inline Matrix initialCovarianceFromAveraging(const ins::KalmanConfig &cfg,
                                             const AveragingResult &avg,
                                             double lat_rad, double sig_heading)
{
    if (!avg.rest_ok)
    {
        return initialCovariance(cfg, lat_rad, sig_heading);
    }

    const double sig_ba_hor = 5.0e-2;   // паспорт: 5 мг, м/с²
    const double sig_bg_pass = 5.0e-4;  // паспорт: ~103 °/ч, рад/с

    const double g = normalGravity(lat_rad, 0.0);

    // 1/cos(lat) — защита от полюса.
    double cos_lat = fabs(cos(lat_rad));
    if (cos_lat < 0.05)
        cos_lat = 0.05;

    // Позиция/скорость/высота: номинал — последний отсчёт => σ_sample.
    const double sig_pos_lat = fmax(avg.sig_pos_sample / R_EARTH, cfg.sig_pos);
    const double sig_pos_lon = fmax(avg.sig_pos_lon_sample / (R_EARTH * cos_lat),
                                    cfg.sig_pos / cos_lat);
    const double sig_h = fmax(avg.sig_alt_sample, cfg.sig_h);
    const double sig_v = fmax(avg.sig_v_sample, cfg.sig_v);
    const double sig_hdg = (sig_heading > 0.0)
                               ? sig_heading
                               : cfg.sig_bg / (cos_lat * U_EARTH);

    // Наклон: шумовая (σ_a_mean/g) + аппаратная (σ_ba_hor/g) составляющие.
    const double sig_a_hor = fmax(avg.sig_a_mean[0], avg.sig_a_mean[2]);
    const double sig_tilt = fmax(sqrt(sig_ba_hor * sig_ba_hor
                                      + sig_a_hor * sig_a_hor) / g,
                                 2.0 * sig_ba_hor / g);

    // Вертикальное смещение акселерометра: оценка по среднему с floor.
    const double sig_ba_vert = fmax(avg.sig_a_mean[1], sig_ba_hor / 10.0);

    // Дрейф гироскопов: горизонтальные по σ_w_mean с floor, вертикальный — паспорт.
    const double sig_w_hor = fmax(avg.sig_w_mean[0], avg.sig_w_mean[2]);
    const double sig_bg_hor = fmax(sig_w_hor, sig_bg_pass / 5.0);
    const double sig_bg_vert = sig_bg_pass;

    const double sigma[ins::KF_STATE] = {
        sig_pos_lat, sig_pos_lon, sig_h,
        sig_v, sig_v, sig_v,
        sig_hdg, sig_tilt, sig_tilt,
        sig_ba_hor, sig_ba_vert, sig_ba_hor,
        sig_bg_hor, sig_bg_vert, sig_bg_hor};

    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = sigma[i] * sigma[i];

    // Корреляции «наклон ↔ горизонтальное смещение акселерометра».
    // Конвенция состояний: x[7]=pitch, x[8]=roll, x[9]=ba_x, x[11]=ba_z.
    at(P0, 7, 9, ins::KF_STATE) = sigma[9] * sigma[9] / g;
    at(P0, 9, 7, ins::KF_STATE) = sigma[9] * sigma[9] / g;
    at(P0, 8, 11, ins::KF_STATE) = -sigma[11] * sigma[11] / g;
    at(P0, 11, 8, ins::KF_STATE) = -sigma[11] * sigma[11] / g;

    return P0;
}

// Начальные ошибки x₀ по окну усреднения (перенос из ITT_Students_NoQT).
// Номинал pos/vel/hdg — последний отсчёт окна, поэтому x₀ = last - mean.
// ba не вносится в x₀ (номинал state.ba = ba_est), bg вносится в x₀[12..14]
// (номинал state.bg = 0, оценка применяется первой коррекцией).
// apply=false: окно не доверено (rest_ok=false) => x₀=0.
inline Vector initialErrorsFromAveraging(const AveragingResult &avg, bool apply)
{
    Vector x(ins::KF_STATE, 0.0);

    if (!apply)
    {
        return x;
    }

    if (avg.has_sns)
    {
        x[0] = avg.sns_last.lat - avg.lat_mean;
        x[1] = avg.sns_last.lon - avg.lon_mean;
        x[2] = avg.sns_last.alt - avg.alt_mean;
        x[3] = avg.sns_last.vn - avg.vn_mean;
        x[4] = avg.sns_last.vh - avg.vh_mean;
        x[5] = avg.sns_last.ve - avg.ve_mean;
    }

    if (avg.has_heading)
    {
        x[6] = normalize_angle(avg.heading_last - avg.heading_circ_mean);
    }

    for (int k = 0; k < 3; k++)
        x[12 + k] = avg.bg_est[k];

    return x;
}

// Начальная ориентация из первого отсчёта СНС (совместимость).
inline ins::Attitude referenceAttitude(const SnsSample &first)
{
    ins::Attitude att;
    att.heading = first.heading;
    att.roll = first.roll;
    att.pitch = first.pitch;
    return att;
}

// Формирование начального состояния (совместимость со старой схемой).
inline NavState initialAlignment(const SnsSample &first, const ins::Attitude &att)
{
    NavState st;
    st.att = att;
    st.lat = first.lat;
    st.lon = first.lon;
    st.alt = first.alt;
    st.V = {first.vn, first.vh, first.ve};
    st.P = initialCovariance(st.lat);
    return st;
}

// Перегрузка: углы из автономной выставки (совместимость).
inline NavState initialAlignment(const SnsSample &first, double yaw, double pitch, double roll)
{
    ins::Attitude att;
    att.heading = yaw;
    att.pitch = pitch;
    att.roll = roll;
    return initialAlignment(first, att);
}

} // namespace nav
