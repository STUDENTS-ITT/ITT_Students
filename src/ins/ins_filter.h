// ins_filter.h — Угловатый фильтр Калмана 15-го порядка.
//
// Вектор состояний (15 компонент):
//   x[0..2]   — ошибки координат (δφ, δλ, δh)
//   x[3..5]   — ошибки скорости (δVn, δVh, δVe)
//   x[6..8]   — ошибки углов ориентации (δψ, δθ, δφ)
//   x[9..11]  — смещения акселерометра (δba)
//   x[12..14] — смещения гироскопа (δbg)
//
// Вектор измерений (9 компонент):
//   z[0..2] — разность координат (БИНС − СНС)
//   z[3..5] — разность скоростей (БИНС − СНС)
//   z[6..8] — разность углов ориентации (БИНС − СНС)

#pragma once

#include <cmath>

#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "../utils/types.h"
#include "attitude_calc.h"

namespace ins
{

// Размерность вектора состояний и вектора измерений.
constexpr int KF_STATE = 15;
constexpr int KF_MEAS = 9;

// Матрица шума процесса Q (диагональная, по моделям дрейфа).
// σ_g, σ_a — шумы гироскопа и акселерометра;
// σ_bg, σ_ba — дрейфы смещений гироскопа и акселерометра.
inline Matrix Qj_matrix(double T)
{
    const double sig_g = 3.394e-4;   // шум гироскопа, рад/√с
    const double sig_a = 3.05e-3;    // шум акселерометра, м/с²/√с
    const double sig_bg = 1.16e-5;   // дрейф смещения гироскопа, рад/с/√с
    const double sig_ba = 2e-5;      // дрейф смещения акселерометра, м/с²/√с

    const double q_v = sig_a * sig_a * T;
    const double q_att = sig_g * sig_g * T;
    const double q_ba = sig_ba * sig_ba * T;
    const double q_bg = sig_bg * sig_bg * T;

    Matrix Qj(KF_STATE * KF_STATE, 0);
    for (int i = 0; i < 3; i++)
        at(Qj, i, i, KF_STATE) = 0.0;    // координаты — без шума
    for (int i = 3; i < 6; i++)
        at(Qj, i, i, KF_STATE) = q_v;    // скорости
    for (int i = 6; i < 9; i++)
        at(Qj, i, i, KF_STATE) = q_att;  // углы
    for (int i = 9; i < 12; i++)
        at(Qj, i, i, KF_STATE) = q_ba;   // смещения акселя
    for (int i = 12; i < 15; i++)
        at(Qj, i, i, KF_STATE) = q_bg;   // смещения гиро
    return Qj;
}

// Матрица шума измерений R (диагональная).
// Погрешности СНС: координаты ~5 м, высота ~5 м, скорости ~0.1 м/с.
// Углы: курс — из СНС (sig_hdg), крен/тангаж — из акселя (sig_pitch, sig_roll).
inline Matrix Rj_matrix(double sig_hdg, double sig_pitch, double sig_roll)
{
    const double sig_pos = 5.0 / R_EARTH;   // позиция в радианах
    const double sig_h = 5.0;               // высота, м
    const double sig_v = 0.1;               // скорость, м/с

    Matrix Rj(KF_MEAS * KF_MEAS, 0);
    const double rdiag[KF_MEAS] = {
        sig_pos * sig_pos, sig_pos * sig_pos, sig_h * sig_h,
        sig_v * sig_v, sig_v * sig_v, sig_v * sig_v,
        sig_hdg * sig_hdg, sig_pitch * sig_pitch, sig_roll * sig_roll};
    for (int i = 0; i < KF_MEAS; i++)
        at(Rj, i, i, KF_MEAS) = rdiag[i];
    return Rj;
}

// Матрица моста F (15×15): связь ошибок ориентации/скорости/координат
// и дрейфов смещений ДУС/ акселерометра.
//
// Блок-схема:
//   δφ̇ = δVn / R                        (ошибки координат ← ошибки скорости)
//   δλ̇ = δVe / (R·cos(φ))               (ошибки координат ← ошибки скорости)
//   δḣ = δVh                             (ошибки координат ← ошибки скорости)
//   δVė = 2ω_Zem·sin(φ)·δVn + ...       (ошибки скорости ← ошибки координат)
//   δVė += -f_nav × δψ,θ,φ              (ошибки скорости ← ошибки ориентации)
//   δVė += C·δba                         (ошибки скорости ← смещения акселя)
//   δψ̇,θ̇,φ̇ = Ω(ψ,θ,φ)·δbg              (ошибки ориентации ← смещения гиро)
inline Matrix Fj_matrix(double T, double lat, double alt, const Matrix &C,
                        const Vector &f_nav, const Attitude &att)
{
    const double g = normalGravity(lat, alt);

    Matrix Fj(KF_STATE * KF_STATE, 0);
    for (int i = 0; i < KF_STATE; i++)
        at(Fj, i, i, KF_STATE) = 1.0;   // единичная диагональ

    // δφ̇ = δVn / R
    at(Fj, 0, 3, KF_STATE) = T / R_EARTH;
    // δλ̇ = δVe / (R·cos(φ))
    at(Fj, 1, 5, KF_STATE) = T / (R_EARTH * fmax(fabs(cos(lat)), 1e-10));
    // δḣ = δVh
    at(Fj, 2, 4, KF_STATE) = T;

    // Гравитационный градиент: δVė ← δVn (влияние ошибки высоты на вертикальную скорость)
    at(Fj, 4, 2, KF_STATE) = T * 2.0 * g / R_EARTH;

    // δVė ← ошибки ориентации (через крестовое произведение f_nav × n̂)
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

    // δVė ← смещения акселерометра (через матрицу направляющих косинусов C)
    for (int j = 0; j < 3; j++)
    {
        at(Fj, 3, 9 + j, KF_STATE) = T * at(C, 0, j, 3);
        at(Fj, 4, 9 + j, KF_STATE) = T * at(C, 1, j, 3);
        at(Fj, 5, 9 + j, KF_STATE) = T * at(C, 2, j, 3);
    }

    // δψ̇,θ̇,φ̇ ← смещения гироскопа (связывающая матрица Эйлера)
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

// Этап предсказания (time update):
//   x = F·x,  P = F·P·F^T + Q
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

// Этап коррекции (measurement update):
//   z = БИНС − СНС (вектор инновации)
//   K = P·H^T·(H·P·H^T + R)^{-1}
//   x = x + K·(z − H·x)
//   P = (I − K·H)·P
inline void correct(const Vector &bins, const Vector &sns, Vector &x, Matrix &P,
                    double sig_hdg, double sig_pitch, double sig_roll)
{
    // Инновация: разность БИНС и СНС.
    Vector zj = vector_diff(bins, sns);
    zj[6] = normalize_angle(zj[6]);
    zj[7] = normalize_angle(zj[7]);
    zj[8] = normalize_angle(zj[8]);

    // Матрица наблюдения H: измеряем первые 9 компонент состояния.
    const Matrix Hj = H_matrix(KF_MEAS, KF_STATE);
    const Matrix HTj = transpose_m(Hj, KF_STATE);

    // Ковариация инновации: S = H·P·H^T + R.
    const Matrix P_HTj = multiply_matrix(P, HTj, KF_STATE, KF_MEAS);
    const Matrix S = matrix_sum(
        multiply_matrix(multiply_matrix(Hj, P, KF_STATE, KF_STATE), HTj, KF_STATE, KF_MEAS),
        Rj_matrix(sig_hdg, sig_pitch, sig_roll), KF_MEAS);

    // Коэффициент усиления Калмана: K = P·H^T·S^{-1}.
    const Matrix Kj = multiply_matrix(P_HTj, return_matrix(S, KF_MEAS), KF_MEAS, KF_MEAS);

    // Коррекция вектора состояния: x += K·(z − H·x).
    Vector innov = vector_diff(zj, multiply_m(Hj, x, KF_STATE));
    innov[6] = normalize_angle(innov[6]);
    innov[7] = normalize_angle(innov[7]);
    innov[8] = normalize_angle(innov[8]);
    x = vector_sum(x, multiply_m(Kj, innov, KF_MEAS));

    // Коррекция ковариации: P = (I − K·H)·P.
    const Matrix E_KH = matrix_diff(E_matrix(KF_STATE),
                                    multiply_matrix(Kj, Hj, KF_MEAS, KF_STATE), KF_STATE);
    P = multiply_matrix(E_KH, P, KF_STATE, KF_STATE);
}

// Размерность вектора измерений для коррекции только по тангажу/крену.
constexpr int KF_TILT_MEAS = 2;

inline Matrix R_tilt_matrix(double sig_pitch, double sig_roll)
{
    Matrix R(KF_TILT_MEAS * KF_TILT_MEAS, 0);
    at(R, 0, 0, KF_TILT_MEAS) = sig_pitch * sig_pitch;
    at(R, 1, 1, KF_TILT_MEAS) = sig_roll * sig_roll;
    return R;
}

inline Matrix H_tilt_matrix()
{
    Matrix H(KF_TILT_MEAS * KF_STATE, 0);
    at(H, 0, 7, KF_STATE) = 1.0;
    at(H, 1, 8, KF_STATE) = 1.0;
    return H;
}

// Коррекция только pitch/roll (на каждом такте ИМУ, 400 Гц).
inline void correctTilt(double pitch_bins, double roll_bins,
                        double pitch_meas, double roll_meas,
                        double sig_pitch, double sig_roll,
                        Vector &x, Matrix &P)
{
    Vector z = {normalize_angle(pitch_bins - pitch_meas),
                normalize_angle(roll_bins - roll_meas)};

    const Matrix H = H_tilt_matrix();
    const Matrix HT = transpose_m(H, KF_STATE);

    const Matrix P_HT = multiply_matrix(P, HT, KF_STATE, KF_TILT_MEAS);
    const Matrix S = matrix_sum(
        multiply_matrix(multiply_matrix(H, P, KF_STATE, KF_STATE), HT, KF_STATE, KF_TILT_MEAS),
        R_tilt_matrix(sig_pitch, sig_roll), KF_TILT_MEAS);

    const Matrix K = multiply_matrix(P_HT, return_matrix(S, KF_TILT_MEAS), KF_TILT_MEAS, KF_TILT_MEAS);

    Vector innov = vector_diff(z, multiply_m(H, x, KF_STATE));
    innov[0] = normalize_angle(innov[0]);
    innov[1] = normalize_angle(innov[1]);

    x = vector_sum(x, multiply_m(K, innov, KF_TILT_MEAS));

    const Matrix E_KH = matrix_diff(
        E_matrix(KF_STATE),
        multiply_matrix(K, H, KF_TILT_MEAS, KF_STATE),
        KF_STATE);
    P = multiply_matrix(E_KH, P, KF_STATE, KF_STATE);
}

} // namespace ins
