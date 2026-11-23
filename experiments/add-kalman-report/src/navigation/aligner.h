// aligner.h — Формирование начального состояния навигации.
//
// Содержит:
//   - initialCovariance — начальная ковариационная матрица фильтра Калмана P₀
//   - referenceAttitude — начальная ориентация из первого отсчёта СНС
//   - initialAlignment — формирование NavState: координаты из СНС,
//     углы — либо из СНС, либо из автономной выставки (перегрузка по yaw, pitch, roll)

#pragma once

#include "../ins/attitude_calc.h"
#include "../ins/ins_filter.h"
#include "../math_lib/matrix_ops.h"
#include "../utils/constants.h"
#include "gps_processor.h"
#include "trajectory.h"

namespace nav
{

// Начальная ковариационная матрица P₀ (диагональная).
// C11: P0 больше не захардкожен — калибруется по настроенным σ из kalman_cfg:
//   pos = 2·σ_pos, alt = 2·σ_h, а курс = σ_bg / Ω_h (неснимаемое смещение гиро
//   искажает проекцию скорости вращения Земли, Ω_h = ω_E·cos(lat)), т.е. курс
//   определяется точностью курсовой выставки, а не свободным масштабом.
inline Matrix initialCovariance(double lat_rad)
{
    const double sigma[ins::KF_STATE] = {
        2.0 * ins::kalman_cfg.sig_pos, 2.0 * ins::kalman_cfg.sig_pos, 2.0 * ins::kalman_cfg.sig_h,
        2.5, 2.5, 2.5,
        ins::kalman_cfg.sig_bg / fmax(fabs(cos(lat_rad)) * U_EARTH, 1e-10), 1.0 * DEG_TO_RAD, 1.0 * DEG_TO_RAD,
        3e-3, 3e-3, 3e-3,
        1e-4, 1e-4, 1e-4};

    const double pdiag[ins::KF_STATE] = {
        sigma[0] * sigma[0], sigma[1] * sigma[1], sigma[2] * sigma[2],
        sigma[3] * sigma[3], sigma[4] * sigma[4], sigma[5] * sigma[5],
        sigma[6] * sigma[6], sigma[7] * sigma[7], sigma[8] * sigma[8],
        sigma[9] * sigma[9], sigma[10] * sigma[10], sigma[11] * sigma[11],
        sigma[12] * sigma[12], sigma[13] * sigma[13], sigma[14] * sigma[14]};

    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = pdiag[i];
    return P0;
}

inline ins::Attitude referenceAttitude(const SnsSample &first)
{
    ins::Attitude att;
    att.heading = first.heading;
    att.roll = first.roll;
    att.pitch = first.pitch;
    return att;
}

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

inline NavState initialAlignment(const SnsSample &first, double yaw, double pitch, double roll)
{
    ins::Attitude att;
    att.heading = yaw;
    att.pitch = pitch;
    att.roll = roll;
    return initialAlignment(first, att);
}

inline NavState initialAlignment(double lat_rad, double lon_rad, double alt,
                                 double yaw, double pitch, double roll,
                                 const Vector &ba_init = {0.0, 0.0, 0.0})
{
    NavState st;

    st.att.heading = yaw;
    st.att.pitch = pitch;
    st.att.roll = roll;
    st.lat = lat_rad;
    st.lon = lon_rad;
    st.alt = alt;
    st.V = {0.0, 0.0, 0.0};
    st.ba = ba_init;
    st.bg = {0.0, 0.0, 0.0};
    st.P = initialCovariance(st.lat);
    return st;
}

} // namespace nav
