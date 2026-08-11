#pragma once

// Дискретный фильтр Калмана для коррекции БИНС по данным СНС (§5.4).
//
// Вектор состояния (15):
// [δφ, δλ, δh, δVn, δVh, δVe, δψ, δθ, δγ, ba_x, ba_y, ba_z, bg_x, bg_y, bg_z]

#include <cmath>

#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "../utils/types.h"
#include "attitude_calc.h"

namespace ins
{

// размерность вектора состояния
constexpr int KF_STATE = 15;

// размерность вектора измерений
constexpr int KF_MEAS = 9;

// Дискретный Q: q = σ² * T (учебник §5.4, дискретизация белого шума).
// σ_g, σ_a — плотность белого шума датчиков, σ_bg, σ_ba — блуждание смещений.
inline Matrix Qj_matrix(double T)
{
    // Белый шум гироскопов по неподвижному участку imu.dat равен 1,1e-4, но
    // заниженное значение здесь ухудшает углы: запас покрывает остаточную
    // погрешность линеаризации Φ, которая иначе ничем не учтена.
    const double sig_g = 3.394e-4;
    const double sig_a = 3.05e-3; // измерено по неподвижному участку

    // Смещения датчиков в этой записи практически постоянны, поэтому
    // блуждание задаётся малым: при прежних значениях оценка смещения
    // вертикального акселерометра гуляла на 2,5e-3 м/с² и не устанавливалась.
    const double sig_bg = 1.16e-5;
    const double sig_ba = 2e-5;

    const double q_v = sig_a * sig_a * T;
    const double q_att = sig_g * sig_g * T;
    const double q_ba = sig_ba * sig_ba * T;
    const double q_bg = sig_bg * sig_bg * T;

    Matrix Qj(KF_STATE * KF_STATE, 0);
    for (int i = 0; i < 3; i++)
        at(Qj, i, i, KF_STATE) = 0.0; // φ, λ, h
    for (int i = 3; i < 6; i++)
        at(Qj, i, i, KF_STATE) = q_v; // V
    for (int i = 6; i < 9; i++)
        at(Qj, i, i, KF_STATE) = q_att; // ψ, θ, γ
    for (int i = 9; i < 12; i++)
        at(Qj, i, i, KF_STATE) = q_ba; // ba
    for (int i = 12; i < 15; i++)
        at(Qj, i, i, KF_STATE) = q_bg; // bg
    return Qj;
}

// R: φ, λ в радианах → σ_поз задаётся в метрах и делится на радиус Земли;
// h, V, углы — в своих единицах
inline Matrix Rj_matrix()
{
    const double sig_pos = 5.0 / R_EARTH;  // рад
    const double sig_h = 5.0;             // м
    const double sig_v = 0.1;               // м/с
    const double sig_ang = 1.0 * DEG_TO_RAD; // рад

    Matrix Rj(KF_MEAS * KF_MEAS, 0);
    const double rdiag[KF_MEAS] = {
        sig_pos * sig_pos, sig_pos * sig_pos, sig_h * sig_h,
        sig_v * sig_v, sig_v * sig_v, sig_v * sig_v,
        sig_ang * sig_ang, sig_ang * sig_ang, sig_ang * sig_ang};
    for (int i = 0; i < KF_MEAS; i++)
        at(Rj, i, i, KF_MEAS) = rdiag[i];
    return Rj;
}

// Переходная матрица Φ ≈ I + A*T (модель ошибок БИНС для ЛА).
// f_nav — кажущееся ускорение в географических осях на этом такте, att —
// углы, по которым построена C: обе величины задают точку линеаризации.
inline Matrix Fj_matrix(double T, double lat, double alt, const Matrix &C,
                        const Vector &f_nav, const Attitude &att)
{
    const double g = normalGravity(lat, alt);

    Matrix Fj(KF_STATE * KF_STATE, 0);
    for (int i = 0; i < KF_STATE; i++)
        at(Fj, i, i, KF_STATE) = 1.0;

    // φ̇ = Vn/R, λ̇ = Ve/(R cos φ), ḣ = Vh
    at(Fj, 0, 3, KF_STATE) = T / R_EARTH;
    at(Fj, 1, 5, KF_STATE) = T / (R_EARTH * cos(lat));
    at(Fj, 2, 4, KF_STATE) = T;

    // Градиент силы тяжести: счисление вычитает g(h), поэтому завышенная
    // высота даёт завышенное ускорение вверх. Канал от этого неустойчив с
    // постоянной времени √(R/2g) ≈ 570 с, и фильтр должен это учитывать.
    at(Fj, 4, 2, KF_STATE) = T * 2.0 * g / R_EARTH;

    // Ошибка углов разворачивает вектор кажущегося ускорения: δ(C·f) = n × f,
    // где n — ось соответствующего поворота в географических осях. Курс идёт
    // вокруг вертикали места, тангаж — вокруг поперечной оси после разворота
    // по курсу, крен — вокруг продольной оси (нулевой столбец C).
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

    // смещения акс. в связанной СК → навигационная через C (тело→геогр.)
    for (int j = 0; j < 3; j++)
    {
        at(Fj, 3, 9 + j, KF_STATE) = T * at(C, 0, j, 3); // Vn
        at(Fj, 4, 9 + j, KF_STATE) = T * at(C, 1, j, 3); // Vh
        at(Fj, 5, 9 + j, KF_STATE) = T * at(C, 2, j, 3); // Ve
    }

    // Смещения гироскопов входят в ошибки углов через ту же матрицу, что и в
    // eulerRates (3.30): на ровном месте ψ ← bg_y, θ ← bg_z, γ ← bg_x.
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

// Прогноз (экстраполяция): x̂⁻ = Φx̂, P⁻ = ΦPΦᵀ + Q.
// Выполняется каждый такт ИМУ с шагом T, иначе ковариация не успевает
// накопить неопределённость за прошедшее время. Точка линеаризации (широта,
// высота и матрица C) берётся на начало такта.
inline void predict(double T, double lat, double alt, const Matrix &C,
                    const Vector &f_nav, const Attitude &att, Vector &x, Matrix &P)
{
    const Matrix Fj = Fj_matrix(T, lat, alt, C, f_nav, att);
    const Matrix FTj = transpose_m(Fj, KF_STATE);

    x = multiply_m(Fj, x, KF_STATE);
    P = matrux_sum(
        multiply_matrix(multiply_matrix(Fj, P, KF_STATE, KF_STATE), FTj, KF_STATE, KF_STATE),
        Qj_matrix(T), KF_STATE);
}

// Коррекция по эталону: K = P⁻Hᵀ(HP⁻Hᵀ + R)⁻¹, x̂⁺ = x̂⁻ + K(z − Hx̂⁻),
// P⁺ = (I − KH)P⁻. Выполняется только когда пришёл отсчёт СНС.
inline void correct(const Vector &bins, const Vector &sns, Vector &x, Matrix &P)
{
    Vector zj = vector_diff(bins, sns);
    zj[6] = normolize_angle(zj[6]);
    zj[7] = normolize_angle(zj[7]);
    zj[8] = normolize_angle(zj[8]);

    const Matrix Hj = H_matrix(KF_MEAS, KF_STATE);
    const Matrix HTj = transpose_m(Hj, KF_STATE);

    const Matrix P_HTj = multiply_matrix(P, HTj, KF_STATE, KF_MEAS);
    const Matrix S = matrux_sum(
        multiply_matrix(multiply_matrix(Hj, P, KF_STATE, KF_STATE), HTj, KF_STATE, KF_MEAS),
        Rj_matrix(), KF_MEAS);
    const Matrix Kj = multiply_matrix(P_HTj, return_matrix(S, KF_MEAS), KF_MEAS, KF_MEAS);

    Vector innov = vector_diff(zj, multiply_m(Hj, x, KF_STATE));
    innov[6] = normolize_angle(innov[6]);
    innov[7] = normolize_angle(innov[7]);
    innov[8] = normolize_angle(innov[8]);
    x = vector_sum(x, multiply_m(Kj, innov, KF_MEAS));

    const Matrix E_KH = matrux_diff(E_matrix(KF_STATE),
                                    multiply_matrix(Kj, Hj, KF_MEAS, KF_STATE), KF_STATE);
    P = multiply_matrix(E_KH, P, KF_STATE, KF_STATE);
}

} // namespace ins
