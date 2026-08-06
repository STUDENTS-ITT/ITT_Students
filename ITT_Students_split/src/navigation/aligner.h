#pragma once

// Формирование стартового состояния БИНС: углы берутся из автономной выставки
// (bins_alignment.h), координаты и скорости — из первого отсчёта СНС.

#include "../ins/attitude_calc.h"
#include "../ins/ins_filter.h"
#include "../math_lib/matrix_ops.h"
#include "../utils/constants.h"
#include "gps_processor.h"
#include "trajectory.h"

namespace nav
{

// P0: φ, λ в рад²; ba/bg из σ_stab² = (1e-4)²
inline Matrix initialCovariance()
{
    const double p_pos = (50.0 / R_EARTH) * (50.0 / R_EARTH); // ~50 м
    const double p_h = 10.0;                                  // м²
    const double p_v = 2.5;                                   // (м/с)²
    const double p_att = (10.0 * DEG_TO_RAD) * (10.0 * DEG_TO_RAD);
    const double p_ba = 1e-8;
    const double p_bg = 1e-8;

    const double pdiag[ins::KF_STATE] = {
        p_pos, p_pos, p_h,
        p_v, p_v, p_v,
        p_att, p_att, p_att,
        p_ba, p_ba, p_ba,
        p_bg, p_bg, p_bg};

    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = pdiag[i];
    return P0;
}

// Эталонные углы из angle.dat (рад): ψ учебника = yaw файла со знаком минус.
// Запасной вариант, если автономная выставка не отработала.
inline ins::Attitude referenceAttitude(const SnsSample &first)
{
    ins::Attitude att;
    att.heading = first.heading;
    att.roll = first.roll;
    att.pitch = first.pitch;
    return att;
}

// Углы задаются выставкой, координаты и скорости — первым отсчётом эталона.
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

} // namespace nav
