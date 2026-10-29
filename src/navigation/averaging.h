// averaging.h — Окно усреднения выставки (1 с) и оценки начального состояния.
//
// Схема (docs/fast_alignment_plan.md, §3-4):
//   - по каждому каналу окна считаются среднее mean и максимум отклонения
//     Δ = max|x(t) − mean|;
//   - СКО отсчёта  σ̂_отсчёта  = Δ / k(N),  СКО среднего  σ̂_среднего =
//     σ̂_отсчёта / √N,  где k(N) = E[max|N(0,1)|] — ожидаемый максимум модуля
//     N стандартных нормальных;
//   - номинал СНС берётся из последнего отсчёта окна, поэтому в P₀ идёт
//     σ̂_отсчёта; среднее окна (наклон, ba, bg) — σ̂_среднего;
//   - floors (паспортные значения) обязательны: в наборе square_noisefree все
//     максимумы нулевые, и без полов P₀ = 0 ⇒ K = 0 ⇒ фильтр не работает.
//
// Окно буферизуется: максимум отклонения от *итогового* среднего известен
// только после прохода по всем отсчётам. Окно — весь отрезок файла от его
// начала до T_align (потоки не перематываются); для 4-й строки StartupNav.ini
// 1.0 с при 200 Гц это 201 отсчёт ИМУ и 9 отсчётов СНС.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "navigation/gps_processor.h"
#include "utils/types.h"

namespace nav
{

// Ожидаемый максимум модуля N стандартных нормальных величин:
// k(N) = E[max|Z₁..Z_N|]. Нужен, чтобы перевести максимум отклонения от
// среднего в СКО отсчёта. Опорные значения (Монте-Карло, 4·10⁵ выборок):
// N = 9 → 1.84, N = 201 → 2.97 (docs/fast_alignment_plan.md, §3.2).
double maxDeviationFactor(int n);

// Результат окна усреднения.
struct AveragingResult
{
    double t_start = 0.0;   // время первого отсчёта окна, с
    double t_end = 0.0;     // конец окна (T_align), с
    int imu_count = 0;      // число отсчётов ИМУ в окне
    int sns_count = 0;      // число отсчётов СНС в окне

    // --- ИМУ: средние ---
    Vector f_mean = {0.0, 0.0, 0.0};   // средняя удельная сила, м/с²
    Vector w_mean = {0.0, 0.0, 0.0};   // средняя угловая скорость, рад/с

    // --- ИМУ: максимумы отклонения от среднего ---
    Vector df_max = {0.0, 0.0, 0.0};   // удельная сила, м/с²
    Vector dw_max = {0.0, 0.0, 0.0};   // угловая скорость, рад/с

    // --- ИМУ: СКО из максимумов отклонения ---
    Vector sig_a_sample = {0.0, 0.0, 0.0};  // СКО отсчёта акселя, м/с²
    Vector sig_w_sample = {0.0, 0.0, 0.0};  // СКО отсчёта гиро, рад/с
    Vector sig_a_mean = {0.0, 0.0, 0.0};    // СКО среднего акселя, м/с²
    Vector sig_w_mean = {0.0, 0.0, 0.0};    // СКО среднего гиро, рад/с

    // --- СНС ---
    bool has_sns = false;
    SnsSample sns_last;                 // последний отсчёт окна (номинал)
    double lat_mean = 0.0;              // средние по окну, рад
    double lon_mean = 0.0;              // рад
    double alt_mean = 0.0;              // м
    double vn_mean = 0.0;               // м/с
    double vh_mean = 0.0;               // м/с
    double ve_mean = 0.0;               // м/с
    double d_lat_max = 0.0;             // максимумы отклонения, м
    double d_lon_max = 0.0;             // м
    double d_alt_max = 0.0;             // м
    double d_v_max = 0.0;               // м/с
    double sig_pos_sample = 0.0;        // СКО отсчёта позиции (широта), м
    double sig_pos_lon_sample = 0.0;    // СКО отсчёта позиции (долгота), м
    double sig_alt_sample = 0.0;        // м
    double sig_v_sample = 0.0;          // м/с

    // --- Курс (angle.dat) ---
    bool has_heading = false;
    double heading_last = 0.0;          // последний отсчёт курса, рад (номинал)
    double heading_circ_mean = 0.0;     // круговое среднее курсов окна, рад
    double d_heading_max = 0.0;         // максимум отклонения, рад
    double sig_hdg_sample = 0.0;        // СКО отсчёта курса, рад

    // --- Ориентация и смещения ---
    double pitch = 0.0;                 // уровень по средней удельной силе, рад
    double roll = 0.0;                  // рад
    double gyro_compass_heading = 0.0;  // курс по вращению Земли, рад
    Vector ba_est = {0.0, 0.0, 0.0};    // оценка смещения акселя, м/с²
    Vector bg_est = {0.0, 0.0, 0.0};    // оценка смещения гиро, рад/с
    double omega_h_meas = -1.0;         // средний модуль горизонтальной ω, рад/с

    // --- Гейт покоя ---
    bool rest_ok = false;               // окно признано стоянкой
    std::string rest_reason;            // причина провала (пусто при rest_ok)
    double rest_speed = 0.0;            // модуль средней скорости СНС, м/с
    double rest_omega = 0.0;            // |ω̄ − Cᵀ·ω_Земли|, рад/с
    double rest_fy = 0.0;               // |f̄_Y − g|, м/с²
    double rest_speed_limit = 0.0;      // порог по скорости, м/с
    double rest_omega_limit = 0.0;      // порог по угловой скорости, рад/с
};

// Накопитель окна усреднения: буферизует отсчёты ИМУ и СНС, по завершении
// считает средние, максимумы отклонения, оценки смещений и вердикт гейта
// покоя.
class AveragingWindow
{
public:
    // t_end — конец окна, с (T_align, 4-я строка StartupNav.ini). Окно —
    // весь отрезок файла от начала чтения до t_end: файлы всегда читаются
    // с начала, перемотки нет. has_heading_ref — есть ли эталон курса
    // (angle.dat): без него курс берётся по вращению Земли.
    AveragingWindow(double t_end, bool has_heading_ref);

    // Отсчёт ИМУ (время, гиро, аксель). Отсчёты с t > t_end не принимаются.
    void addImu(double time, double wx, double wy, double wz,
                double ax, double ay, double az);

    // Отсчёт СНС (вместе с курсом эталона, если он есть).
    void addSns(const SnsSample &s);

    // Итог окна: средние, максимумы отклонений, смещения, гейт покоя.
    AveragingResult finish(double lat_rad, double alt);

    // Признак переполнения буфера (защита от неадекватного T_align).
    bool overflowed() const { return overflow_; }
    std::size_t imuCount() const { return imu_.size(); }
    std::size_t snsCount() const { return sns_.size(); }

private:
    struct ImuRow
    {
        double t;
        double w[3];
        double a[3];
    };

    struct SnsRow
    {
        double t;
        SnsSample s;
    };

    double t_end_;     // конец окна (T_align), с
    bool has_heading_;
    bool overflow_ = false;
    std::vector<ImuRow> imu_;
    std::vector<SnsRow> sns_;
};

} // namespace nav
