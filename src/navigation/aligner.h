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
// Отражает неопределённость начальных оценок:
//   - координаты ~50 м
//   - высота ~10 м
//   - скорость ~2.5 м/с
//   - курс ~15°
//   - крен/тангаж ~0.1°
//   - смещения акселя/гиро — малые начальные оценки
inline Matrix initialCovariance()
{
    const double p_pos = (50.0 / R_EARTH) * (50.0 / R_EARTH);
    const double p_h = 10.0;
    const double p_v = 2.5;
    const double p_hdg = (15.0 * DEG_TO_RAD) * (15.0 * DEG_TO_RAD);
    const double p_tilt = (0.1 * DEG_TO_RAD) * (0.1 * DEG_TO_RAD);
    const double p_ba = 9e-6;
    const double p_bg = 1e-8;

    const double pdiag[ins::KF_STATE] = {
        p_pos, p_pos, p_h,
        p_v, p_v, p_v,
        p_hdg, p_tilt, p_tilt,
        p_ba, p_ba, p_ba,
        p_bg, p_bg, p_bg};

    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = pdiag[i];
    return P0;
}

// Начальная ориентация из первого отсчёта СНС (эталон).
inline ins::Attitude referenceAttitude(const SnsSample &first)
{
    ins::Attitude att;
    att.heading = first.heading;
    att.roll = first.roll;
    att.pitch = first.pitch;
    return att;
}

// Формирование начального состояния: углы из СНС, координаты и скорости — из СНС.
inline NavState initialAlignment(const SnsSample &first, const ins::Attitude &att)
{
    NavState st;

    st.att = att;

    st.lat = first.lat;
    st.lon = first.lon;
    st.alt = first.alt;

    st.V = {first.vn, first.vh, first.ve};

    st.P = initialCovariance();
    return st;
}

// Перегрузка: начальное состояние с углами из автономной выставки.
// Координаты и скорости — из первого отсчёта СНС.
inline NavState initialAlignment(const SnsSample &first, double yaw, double pitch, double roll)
{
    ins::Attitude att;
    att.heading = yaw;
    att.pitch = pitch;
    att.roll = roll;
    return initialAlignment(first, att);
}

} // namespace nav
