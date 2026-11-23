// ins_filter.h - модели прогноза и коррекции фильтра Калмана 15-го порядка.
//
// Вектор состояния (15 компонент):
//   x[0..2]   - ошибки координат (долгота, широта, h)
//   x[3..5]   - ошибки скорости (Vn, Vh, Ve)
//   x[6..8]   - ошибки углов ориентации (курс, тангаж, крен)
//   x[9..11]  - смещения акселерометров (ba)
//   x[12..14] - смещения гироскопов (bg)
//
// Вектор измерений (9 компонент):
//   z[0..2] - измеренные координаты (относительно модели)
//   z[3..5] - измеренные скорости (относительно модели)
//   z[6..8] - измеренные углы ориентации (относительно модели)

#pragma once

#include <cmath>

#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../common/constants.h"
#include "../math_lib/gravity.h"
#include "../common/types.h"
#include "attitude_calc.h"

namespace ins
{

// Размерности вектора состояния и измерений.
constexpr int KF_STATE = 15;
constexpr int KF_MEAS = 9;

// Конфигурация фильтра Калмана и выставки/инициализации.
// Содержит шумы Q/R, приоры для P0/fallback и параметры защиты СНС.
struct KalmanConfig
{
    // Шумы процесса Q (используются Qj_matrix и как приоры, где применимо).
    double sig_g = 3.394e-4;   // шум гироскопов, рад/с/√Гц
    double sig_a = 3.05e-3;    // шум акселерометров, м/с^2/√Гц
    double sig_bg = 1.16e-5;   // шум дрейфа смещений гироскопов, рад/с/√Гц
    double sig_ba = 2e-5;      // шум дрейфа смещений акселерометров, м/с^2/√Гц

    // Шумы/приоры измерений и начального состояния (R, P0/fallback, гейт покоя).
    double sig_pos = 5.0 / R_EARTH;   // приор позиции, рад (горизонт, по широте)
    double sig_h = 5.0;               // приор/шум высоты, м
    double sig_v = 0.1;               // приор/шум скорости, м/с
    double sig_hdg = 1.0 * DEG_TO_RAD; // приор/шум курса, рад

    // Выставка (AveragingWindow).
    double align_time_s = 1.0;         // длительность окна выставки, с (по умолчанию)

    // Защита СНС (gnss_guard).
    double gnss_guard_on = 1.0;           // 1.0 - включить защиту СНС, 0.0 - отключить
    double gnss_vel_plaus = 500.0;        // максимальная правдоподобная скорость по СНС, м/с
    double gnss_silence_timeout_s = 5.0;  // таймаут пропадания СНС для перехода в STALE, с
    double gnss_alt_band_m = 5000.0;      // допустимое отклонение по высоте от первой принятой, м
    double gnss_dt_max_s = 3600.0;        // максимальный допустимый dt между принятыми кадрами, с
    double gnss_jump_pos_m = 1000.0;      // порог скачка по позиции между принятыми кадрами, м
    double gnss_jump_vel_ms = 100.0;      // порог скачка по скорости между принятыми кадрами, м/с
};

// Глобальная конфигурация по умолчанию.
inline KalmanConfig kalman_cfg{};

// Параметры процесса Q (дискретизированная, упрощённая форма).
inline Matrix Qj_matrix(double T)
{
    const double sig_g = kalman_cfg.sig_g;
    const double sig_a = kalman_cfg.sig_a;
    const double sig_bg = kalman_cfg.sig_bg;
    const double sig_ba = kalman_cfg.sig_ba;

    const double q_v = sig_a * sig_a * T;
    const double q_att = sig_g * sig_g * T;
    const double q_ba = sig_ba * sig_ba * T;
    const double q_bg = sig_bg * sig_bg * T;

    Matrix Qj(KF_STATE * KF_STATE, 0);
    for (int i = 0; i < 3; i++)
        at(Qj, i, i, KF_STATE) = 0.0;    // координаты без шума
    for (int i = 3; i < 6; i++)
        at(Qj, i, i, KF_STATE) = q_v;    // скорость
    for (int i = 6; i < 9; i++)
        at(Qj, i, i, KF_STATE) = q_att;  // углы
    for (int i = 9; i < 12; i++)
        at(Qj, i, i, KF_STATE) = q_ba;   // смещения ба
    for (int i = 12; i < 15; i++)
        at(Qj, i, i, KF_STATE) = q_bg;   // смещения бг
    return Qj;
}

// Параметры измерений R (диагональная).
inline Matrix Rj_matrix()
{
    const double sig_pos = kalman_cfg.sig_pos;
    const double sig_h = kalman_cfg.sig_h;
    const double sig_v = kalman_cfg.sig_v;
    const double sig_ang = kalman_cfg.sig_hdg;

    Matrix Rj(KF_MEAS * KF_MEAS, 0);
    const double rdiag[KF_MEAS] = {
        sig_pos * sig_pos, sig_pos * sig_pos, sig_h * sig_h,
        sig_v * sig_v, sig_v * sig_v, sig_v * sig_v,
        sig_ang * sig_ang, sig_ang * sig_ang, sig_ang * sig_ang};
    for (int i = 0; i < KF_MEAS; i++)
        at(Rj, i, i, KF_MEAS) = rdiag[i];
    return Rj;
}

// Матрица перехода состояний F (15x15).
inline Matrix Fj_matrix(double T, double lat, double alt, const Matrix &C,
                        const Vector &f_nav, const Attitude &att)
{
    const double g = normalGravity(lat, alt);

    Matrix Fj(KF_STATE * KF_STATE, 0);
    for (int i = 0; i < KF_STATE; i++)
        at(Fj, i, i, KF_STATE) = 1.0;

    at(Fj, 0, 3, KF_STATE) = T / R_EARTH;
    at(Fj, 1, 5, KF_STATE) = T / (R_EARTH * fmax(fabs(cos(lat)), 1e-10));
    at(Fj, 2, 4, KF_STATE) = T;

    at(Fj, 4, 2, KF_STATE) = T * 2.0 * g / R_EARTH;

    const Vector n_psi = {0.0, 1.0, 0.0};
    const Vector n_theta = {sin(att.heading), 0.0, cos(att.heading)};
    const Vector n_gamma = {at(C, 0, 0, 3), at(C, 1, 0, 3), at(C, 2, 0, 3)};

    const Vector d_psi = vector_product(n_psi, f_nav);
    const Vector d_theta = vector_product(n_theta, f_nav);
    const Vector d_gamma = vector_product(n_gamma, f_nav);
    for (int r = 0; r < 3; r++)
    {
        at(Fj, 3 + r, 6, KF_STATE) = T * d_psi[r];
        at(Fj, 3 + r, 7, KF_STATE) = T * d_theta[r];
        at(Fj, 3 + r, 8, KF_STATE) = T * d_gamma[r];
    }

    for (int j = 0; j < 3; j++)
    {
        at(Fj, 3, 9 + j, KF_STATE) = T * at(C, 0, j, 3);
        at(Fj, 4, 9 + j, KF_STATE) = T * at(C, 1, j, 3);
        at(Fj, 5, 9 + j, KF_STATE) = T * at(C, 2, j, 3);
    }

    const double cg = cos(att.roll);
    const double sg = sin(att.roll);
    const double cth = cos(att.pitch);
    const double tth = tan(att.pitch);
    const double e_rate[3][3] = {
        {0.0, cg / cth, -sg / cth},
        {0.0, sg, cg},
        {1.0, -tth * cg, tth * sg}};
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            at(Fj, 6 + r, 12 + c, KF_STATE) = T * e_rate[r][c];

    return Fj;
}

// Прогноз.
inline void predict(double T, double lat, double alt, const Matrix &C,
                    const Vector &f_nav, const Attitude &att, Vector &x, Matrix &P)
{
    const Matrix Fj = Fj_matrix(T, lat, alt, C, f_nav, att);
    const Matrix FTj = transpose_m(Fj, KF_STATE);

    x = multiply_m(Fj, x, KF_STATE);
    P = matrix_sum(
        multiply_matrix(multiply_matrix(Fj, P, KF_STATE, KF_STATE), FTj, KF_STATE, KF_STATE),
        Qj_matrix(T), KF_STATE);
}

// Коррекция.
inline void correct(const Vector &bins, const Vector &sns, Vector &x, Matrix &P)
{
    Vector zj = vector_diff(bins, sns);
    zj[6] = normalize_angle(zj[6]);
    zj[7] = normalize_angle(zj[7]);
    zj[8] = normalize_angle(zj[8]);

    const Matrix Hj = H_matrix(KF_MEAS, KF_STATE);
    const Matrix HTj = transpose_m(Hj, KF_STATE);

    const Matrix P_HTj = multiply_matrix(P, HTj, KF_STATE, KF_MEAS);
    const Matrix S = matrix_sum(
        multiply_matrix(multiply_matrix(Hj, P, KF_STATE, KF_STATE), HTj, KF_STATE, KF_MEAS),
        Rj_matrix(), KF_MEAS);

    const Matrix Kj = multiply_matrix(P_HTj, return_matrix(S, KF_MEAS), KF_MEAS, KF_MEAS);

    Vector innov = vector_diff(zj, multiply_m(Hj, x, KF_STATE));
    innov[6] = normalize_angle(innov[6]);
    innov[7] = normalize_angle(innov[7]);
    innov[8] = normalize_angle(innov[8]);
    x = vector_sum(x, multiply_m(Kj, innov, KF_MEAS));

    const Matrix E_KH = matrix_diff(E_matrix(KF_STATE),
                                    multiply_matrix(Kj, Hj, KF_MEAS, KF_STATE), KF_STATE);
    P = multiply_matrix(E_KH, P, KF_STATE, KF_STATE);
}

} // namespace ins
