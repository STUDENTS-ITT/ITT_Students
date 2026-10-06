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

// Настройки шумов фильтра (переопределяются из params.ini при подборе).
struct KalmanConfig
{
    double sig_g   = 2.12428e-02;           // шум гироскопа, рад/√с (run_hk №6)
    double sig_a   = 1.5276e+00;            // шум акселерометра, м/с²/√с (run_hk №6)
    double sig_bg  = 1.05526e-05;           // дрейф смещения гироскопа, рад/с/√с (run_hk №6)
    double sig_ba  = 1.10162e-05;           // дрейф смещения акселерометра, м/с²/√с (run_hk №6)
    double sig_pos = 2.64401e-08;           // СНС позиция, рад (run_hk №6)
    double sig_h   = 5.0;                   // СНС высота, м
    double sig_v   = 4.81061e-02;           // СНС скорость, м/с (run_hk №6)
    double sig_hdg = 1.23106e-03;           // СНС курс, рад (run_hk №6)
    double outage_start_s = 0.0;             // начало окна без коррекций СНС, с
    double outage_end_s = 0.0;               // конец окна без коррекций СНС, с (0 — выкл)
};

// Глобальная (настраиваемая) конфигурация шумов.
inline KalmanConfig kalman_cfg;

// Матрица шума процесса Q (диагональная, по моделям дрейфа).
// σ_g, σ_a — шумы гироскопа и акселерометра;
// σ_bg, σ_ba — дрейфы смещений гироскопа и акселерометра.
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
    const double sig_pos = kalman_cfg.sig_pos;   // позиция в радианах
    const double sig_h = kalman_cfg.sig_h;       // высота, м
    const double sig_v = kalman_cfg.sig_v;       // скорость, м/с

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

    // Кориолисовы связи ошибок скоростей (2·Ω×δV).
    // Из уравнения V̇ = f_н − a_вред, где a_вред содержит Кориолис (2·Ω×V):
    //   δV̇n ← δVe: −2Ω·sin(φ)
    //   δV̇h ← δVe: +2Ω·cos(φ)
    //   δV̇e ← δVn: +2Ω·sin(φ)
    // Мало для низкоскоростного аппарата, но учитывается для полноты модели
    // (соответствует блочной схеме «δVė = 2ω_Zem·sin(φ)·δVn»).
    const double two_w = 2.0 * U_EARTH;
    at(Fj, 3, 5, KF_STATE) += T * (-two_w * sin(lat));
    at(Fj, 4, 5, KF_STATE) += T * (two_w * cos(lat));
    at(Fj, 5, 3, KF_STATE) += T * (two_w * sin(lat));

    // δVė ← ошибки ориентации (через крестовое произведение f_nav × n̂)
    // Курс растёт по часовой стрелке, поэтому его ось поворота направлена вниз.
    const Vector n_psi = {0.0, -1.0, 0.0};
    const Vector n_theta = {-sin(att.heading), 0.0, cos(att.heading)};
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
        {0.0, -cg / cth, sg / cth},
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

// Диагностика согласованности фильтра на кадре коррекции.
//   NIS  (Normalized Innovation Squared) — насколько велика инновация
//        относительно её ожидаемой дисперсии S = H·P·Hᵀ + R.
//   NEES (Normalized Estimation Error Squared) — насколько велика оценка
//        ошибки относительно её ковариации P (без шума измерения R).
// Оба — хи-квадратные величины: в согласованном фильре среднее значение
// равно числу степеней свободы (9), а отношение NIS/NEES близко к 1.
struct CorrectionStats
{
    double nis = 0.0;   // innovᵀ·S⁻¹·innov
    double nees = 0.0;  // innovᵀ·P_meas⁻¹·innov
    int dof = KF_MEAS;  // число степеней свободы (размерность H)
    bool valid = false; // заполнено ли (кадр коррекции с обратимой S)
};

// Этап коррекции (measurement update):
//   z = БИНС − СНС (вектор инновации)
//   K = P·H^T·(H·P·H^T + R)^{-1}
//   x = x + K·(z − H·x)
//   P = (I − K·H)·P
//
// stats — необязательный выход для NIS/NEES. Передаётся nullptr в консольном
// варианте, поэтому вычислительный путь и выходные файлы не меняются.
inline void correct(const Vector &bins, const Vector &sns, Vector &x, Matrix &P,
                    double sig_hdg, double sig_pitch, double sig_roll,
                    CorrectionStats *stats = nullptr)
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

    // Диагностика согласованности (только по запросу). Считаем до обновления x и P,
    // поэтому используем именно предсказанные P и инновацию.
    //   NIS  = innovᵀ·S⁻¹·innov      — с учётом шума измерения R
    //   NEES = innovᵀ·P_meas⁻¹·innov — по ковариации состояния, без R
    if (stats != nullptr)
    {
        const Matrix Sinv = return_matrix(S, KF_MEAS);

        double nis = 0.0;
        const Vector Sinv_innov = multiply_m(Sinv, innov, KF_MEAS);
        for (int i = 0; i < KF_MEAS; i++)
            nis += innov[i] * Sinv_innov[i];

        // Ковариация измеряемого подпространства: верхний блок P 9×9.
        Matrix P_meas(KF_MEAS * KF_MEAS, 0);
        for (int i = 0; i < KF_MEAS; i++)
            for (int j = 0; j < KF_MEAS; j++)
                at(P_meas, i, j, KF_MEAS) = at(P, i, j, KF_STATE);

        const Matrix P_meas_inv = return_matrix(P_meas, KF_MEAS);
        const Vector P_meas_inv_innov = multiply_m(P_meas_inv, innov, KF_MEAS);
        double nees = 0.0;
        for (int i = 0; i < KF_MEAS; i++)
            nees += innov[i] * P_meas_inv_innov[i];

        stats->nis = nis;
        stats->nees = nees;
        stats->dof = KF_MEAS;
        stats->valid = std::isfinite(nis) && std::isfinite(nees);
    }

    x = vector_sum(x, multiply_m(Kj, innov, KF_MEAS));

    // Коррекция ковариации в устойчивой Joseph-форме:
    //   P = (I − K·H)·P·(I − K·H)ᵀ + K·R·Kᵀ.
    // Симметрична по построению и сохраняет положительную полуопределённость
    // (в отличие от упрощённой (I − K·H)·P, где вычитания теряют симметрию
    // и могут давать отрицательные диагонали).
    const Matrix E_KH = matrix_diff(E_matrix(KF_STATE),
                                    multiply_matrix(Kj, Hj, KF_MEAS, KF_STATE), KF_STATE);
    const Matrix E_KH_T = transpose_m(E_KH, KF_STATE);
    const Matrix Rj = Rj_matrix(sig_hdg, sig_pitch, sig_roll);
    const Matrix KR = multiply_matrix(Kj, Rj, KF_MEAS, KF_MEAS);
    P = matrix_sum(
        multiply_matrix(multiply_matrix(E_KH, P, KF_STATE, KF_STATE), E_KH_T, KF_STATE, KF_STATE),
        multiply_matrix(KR, transpose_m(Kj, KF_MEAS), KF_MEAS, KF_STATE),
        KF_STATE);
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

    Matrix K = multiply_matrix(P_HT, return_matrix(S, KF_TILT_MEAS), KF_TILT_MEAS, KF_TILT_MEAS);

    // Развязка tilt-контура от координат/скорости/курса (П2):
    // акселерометрическая коррекция должна трогать только ошибки углов и
    // смещений датчиков (x[7..14]).  Строки K для x[0..6] обнуляем, чтобы
    // корреляционные члены P·Hᵀ не «протекали» в непогашенные ошибки
    // pos/vel/hdg, которые живут между кадрами СНС.
    for (int r = 0; r < 7; r++)
        for (int c = 0; c < KF_TILT_MEAS; c++)
            at(K, r, c, KF_TILT_MEAS) = 0.0;

    Vector innov = vector_diff(z, multiply_m(H, x, KF_STATE));
    innov[0] = normalize_angle(innov[0]);
    innov[1] = normalize_angle(innov[1]);

    x = vector_sum(x, multiply_m(K, innov, KF_TILT_MEAS));

    // Коррекция ковариации в Joseph-форме (см. correct): симметрична и
    // сохраняет положительную полуопределённость P.
    const Matrix E_KH = matrix_diff(
        E_matrix(KF_STATE),
        multiply_matrix(K, H, KF_TILT_MEAS, KF_STATE),
        KF_STATE);
    const Matrix E_KH_T = transpose_m(E_KH, KF_STATE);
    const Matrix R_tilt = R_tilt_matrix(sig_pitch, sig_roll);
    const Matrix KR = multiply_matrix(K, R_tilt, KF_TILT_MEAS, KF_TILT_MEAS);
    P = matrix_sum(
        multiply_matrix(multiply_matrix(E_KH, P, KF_STATE, KF_STATE), E_KH_T, KF_STATE, KF_STATE),
        multiply_matrix(KR, transpose_m(K, KF_TILT_MEAS), KF_TILT_MEAS, KF_STATE),
        KF_STATE);
}

// Согласование ковариации после ESKF-reset (inject + zero δx).
//
// В каноническом ESKF после инъекции ошибок в номинал ковариация обновляется:
//   P ← G·P·Gᵀ,
// где G — якобиан операции reset (Solà, 2017).
// Для аддитивных групп (координаты, скорости, смещения датчиков)
// композиция линейна и G = I.  Для Эйлеровых углов (additive, mod 2π,
// с normalize_angle)
// G = I при малых δx, что верно при любой корректной
// выставке (рассинхронизация углов не превышает π/2).  Практический
// вклад функции — числовое согласование P: predict и коррекции накапливают
// ошибки округления, на недиагонали может потеряться симметрия, а на
// диагонали появиться отрицательные дисперсии → раздуть K и «смонтировать»
// режим копии СНС.
inline void apply_reset_covariance(Matrix &P)
{
    const int n = KF_STATE;

    // 1) Симметризация: P ← (P + Pᵀ)/2.
    //    После predict/коррекций симметрия нарушается на ε ~ 10⁻¹⁵.
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
        {
            const double s = 0.5 * (at(P, i, j, n) + at(P, j, i, n));
            at(P, i, j, n) = s;
            at(P, j, i, n) = s;
        }

    // 2) Страховка диагонали: отрицательные дисперсии → 1e-15.
    //    Источник: хвостовые вычитания в (I−KH)·P при малых σ.
    for (int i = 0; i < n; i++)
    {
        if (at(P, i, i, n) < 0.0)
            at(P, i, i, n) = 1e-15;
    }
}

} // namespace ins
