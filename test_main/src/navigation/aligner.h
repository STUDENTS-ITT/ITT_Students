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
    // Горизонт и курс выставка получает по-разному: крен и тангаж — по
    // проекциям g, их ошибка есть смещение акселерометра, делённое на g
    // (единицы угловых минут); курс — гирокомпасированием, его ошибка есть
    // смещение гироскопа, делённое на U·cos φ, что на три порядка хуже.
    const double p_hdg = (15.0 * DEG_TO_RAD) * (15.0 * DEG_TO_RAD);
    const double p_tilt = (0.1 * DEG_TO_RAD) * (0.1 * DEG_TO_RAD);
    // σ_ba0 = 3e-3 м/с²: прежнее 1e-4 было втрое меньше реального смещения
    // вертикального акселерометра (~1,1e-3), и фильтр не имел права его
    // оценить — оставалось дотягиваться блужданием, на что уходило 300 с.
    const double p_ba = 9e-6;
    const double p_bg = 1e-8; // σ_bg0 = 1e-4 рад/с, измеренные дрейфы до 6,5e-5

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
