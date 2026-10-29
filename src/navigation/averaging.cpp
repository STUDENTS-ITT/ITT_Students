// averaging.cpp — Окно усреднения выставки: статистики, смещения, гейт покоя.
//
// Схема (docs/fast_alignment_plan.md, §3-4):
//   - среднее по окну и максимум отклонения от него Δ = max|x − mean|;
//   - СКО отсчёта σ̂ = Δ / k(N), СКО среднего σ̂/√N, k(N) = E[max|N(0,1)|];
//   - номинал позиции/скорости/курса — последний отсчёт окна (σ̂_отсчёта),
//     наклон и смещения — средние (σ̂_среднего);
//   - гейт покоя: скорость, отклонение ω̄ от проекции вращения Земли,
//     отклонение вертикальной удельной силы от g.
//
// Все углы внутри — радианы, метры — метры. Никаких зависимостей от Qt.

#include "navigation/averaging.h"

#include <cmath>

#include "ins/ins_filter.h"  // ins::kalman_cfg (порог скорости гейта)
#include "math_lib/transformations.h"
#include "utils/constants.h"

namespace nav
{

double maxDeviationFactor(int n)
{
    if (n <= 1)
    {
        return 1.0;
    }

    // Опорные значения E[max|Z₁..Z_N|], Монте-Карло (4·10⁵ выборок).
    static const double n_tab[] = {1.0, 2.0, 3.0, 5.0, 9.0, 10.0,
                                   20.0, 50.0, 100.0, 200.0, 500.0};
    static const double k_tab[] = {1.0, 1.1275, 1.3261, 1.5717, 1.8360, 1.8818,
                                   2.1675, 2.5096, 2.7478, 2.9686, 3.2411};
    const int m = static_cast<int>(sizeof(n_tab) / sizeof(n_tab[0]));

    if (n <= static_cast<int>(n_tab[0]))
    {
        return k_tab[0];
    }
    if (static_cast<double>(n) >= n_tab[m - 1])
    {
        // Асимптотика E[max|Z|] ~ √(2·ln N): для N = 500 даёт 3.26 против
        // измеренных 3.24 (расхождение ~0.6 %), дальше — только лучше.
        return sqrt(2.0 * log(static_cast<double>(n)));
    }

    // Линейная интерполяция между соседними опорными точками.
    for (int i = 0; i < m - 1; i++)
    {
        if (static_cast<double>(n) >= n_tab[i] && static_cast<double>(n) <= n_tab[i + 1])
        {
            const double t = (static_cast<double>(n) - n_tab[i]) / (n_tab[i + 1] - n_tab[i]);
            return k_tab[i] + t * (k_tab[i + 1] - k_tab[i]);
        }
    }
    return 1.0;
}

namespace
{

// Предохранители буфера: окно 1 с — это 201 отсчёт ИМУ и 9 СНС, но
// StartupNav.ini может задать и больше (старое значение 120 с).
constexpr std::size_t MAX_IMU_ROWS = 400000;
constexpr std::size_t MAX_SNS_ROWS = 20000;

// Порог гейта покоя (docs/fast_alignment_plan.md, §4.4).
//
// Вращение: |bg_est| = |ω̄ − Cᵀ·ω_Земли|. За окно 1 с постоянное вращение и
// постоянное смещение гироскопа принципиально неразличимы, поэтому порог
// складывается из запаса на смещение и 4σ̂ оценки среднего. Запас взят широким
// (≈2060 °/ч): у реальных данных run_hk оценка |bg| ≈ 590 °/ч, у square_harsh
// истинное смещение 215 °/ч. Смысл проверки — отсечь грубое вращение на окне
// (>~1 °/с), а не измерить смещение; медленный поворот за 1 с даёт ошибку
// уровня/курса меньше флора P₀.
constexpr double REST_OMEGA_BIAS_LIMIT = 1e-2;  // рад/с, ≈ 2060 °/ч
constexpr double REST_FY_LIMIT_MG = 10.0;       // мг, отклонение f̄_Y от g

// Множитель порогов: σ̂ по максимуму отклонения на 9 отсчётах СНС сама имеет
// разброс ~20 %, при 3σ ложные провалы гейта слишком часты.
constexpr double REST_SIGMA_FACTOR = 4.0;

} // namespace

AveragingWindow::AveragingWindow(double t_end, bool has_heading_ref)
    : t_end_(t_end),
      has_heading_(has_heading_ref)
{
    imu_.reserve(2048);
    sns_.reserve(64);
}

void AveragingWindow::addImu(double time, double wx, double wy, double wz,
                             double ax, double ay, double az)
{
    // Окно — от начала файла до t_end: отбрасываются только отсчёты за окном,
    // их принимает основной цикл (потоки не перематываются).
    if (time > t_end_)
    {
        return;
    }
    if (imu_.size() >= MAX_IMU_ROWS)
    {
        overflow_ = true;
        return;
    }

    ImuRow r;
    r.t = time;
    r.w[0] = wx;
    r.w[1] = wy;
    r.w[2] = wz;
    r.a[0] = ax;
    r.a[1] = ay;
    r.a[2] = az;
    imu_.push_back(r);
}

void AveragingWindow::addSns(const SnsSample &s)
{
    if (s.time > t_end_)
    {
        return;
    }
    if (sns_.size() >= MAX_SNS_ROWS)
    {
        overflow_ = true;
        return;
    }

    SnsRow r;
    r.t = s.time;
    r.s = s;
    sns_.push_back(r);
}

AveragingResult AveragingWindow::finish(double lat_rad, double alt)
{
    AveragingResult res;
    res.t_end = t_end_;
    res.imu_count = static_cast<int>(imu_.size());
    res.sns_count = static_cast<int>(sns_.size());

    // ------------------------------------------------------------------
    // 1. ИМУ: средние, максимумы отклонения, СКО
    // ------------------------------------------------------------------
    const int n_imu = res.imu_count;
    if (n_imu > 0)
    {
        res.t_start = imu_.front().t;

        for (const ImuRow &r : imu_)
        {
            for (int k = 0; k < 3; k++)
            {
                res.f_mean[k] += r.a[k];
                res.w_mean[k] += r.w[k];
            }
        }
        const double inv_n = 1.0 / static_cast<double>(n_imu);
        for (int k = 0; k < 3; k++)
        {
            res.f_mean[k] *= inv_n;
            res.w_mean[k] *= inv_n;
        }

        for (const ImuRow &r : imu_)
        {
            for (int k = 0; k < 3; k++)
            {
                res.df_max[k] = fmax(res.df_max[k], fabs(r.a[k] - res.f_mean[k]));
                res.dw_max[k] = fmax(res.dw_max[k], fabs(r.w[k] - res.w_mean[k]));
            }
        }

        // Максимум отклонения → СКО отсчёта и СКО среднего.
        const double kf = maxDeviationFactor(n_imu);
        const double sq_n = sqrt(static_cast<double>(n_imu));
        for (int k = 0; k < 3; k++)
        {
            res.sig_a_sample[k] = res.df_max[k] / kf;
            res.sig_w_sample[k] = res.dw_max[k] / kf;
            res.sig_a_mean[k] = res.sig_a_sample[k] / sq_n;
            res.sig_w_mean[k] = res.sig_w_sample[k] / sq_n;
        }
    }

    // ------------------------------------------------------------------
    // 2. СНС: средние, круговое среднее курса, максимумы отклонения
    // ------------------------------------------------------------------
    const int n_sns = res.sns_count;
    if (n_sns > 0)
    {
        res.has_sns = true;
        res.sns_last = sns_.back().s;

        double sum_lat = 0.0, sum_lon = 0.0, sum_alt = 0.0;
        double sum_vn = 0.0, sum_vh = 0.0, sum_ve = 0.0;
        double sum_sin = 0.0, sum_cos = 0.0;
        for (const SnsRow &r : sns_)
        {
            sum_lat += r.s.lat;
            sum_lon += r.s.lon;
            sum_alt += r.s.alt;
            sum_vn += r.s.vn;
            sum_vh += r.s.vh;
            sum_ve += r.s.ve;
            sum_sin += sin(r.s.heading);
            sum_cos += cos(r.s.heading);
        }
        const double inv_n = 1.0 / static_cast<double>(n_sns);
        res.lat_mean = sum_lat * inv_n;
        res.lon_mean = sum_lon * inv_n;
        res.alt_mean = sum_alt * inv_n;
        res.vn_mean = sum_vn * inv_n;
        res.vh_mean = sum_vh * inv_n;
        res.ve_mean = sum_ve * inv_n;

        // Круговое среднее курса: оценка истины для ошибки номинала. Берётся
        // только при наличии angle.dat — иначе heading в SnsSample нулевой.
        res.has_heading = has_heading_;
        if (has_heading_)
        {
            res.heading_circ_mean = atan2(sum_sin, sum_cos);
            res.heading_last = res.sns_last.heading;
        }

        const double cos_lat = fmax(fabs(cos(res.lat_mean)), 1e-10);
        for (const SnsRow &r : sns_)
        {
            res.d_lat_max = fmax(res.d_lat_max,
                                 fabs(r.s.lat - res.lat_mean) * R_EARTH);
            res.d_lon_max = fmax(res.d_lon_max,
                                 fabs(r.s.lon - res.lon_mean) * R_EARTH * cos_lat);
            res.d_alt_max = fmax(res.d_alt_max, fabs(r.s.alt - res.alt_mean));

            const double dv = fmax(fmax(fabs(r.s.vn - res.vn_mean),
                                        fabs(r.s.vh - res.vh_mean)),
                                   fabs(r.s.ve - res.ve_mean));
            res.d_v_max = fmax(res.d_v_max, dv);

            if (res.has_heading)
            {
                res.d_heading_max = fmax(
                    res.d_heading_max,
                    fabs(normalize_angle(r.s.heading - res.heading_circ_mean)));
            }
        }

        const double kf = maxDeviationFactor(n_sns);
        res.sig_pos_sample = res.d_lat_max / kf;
        res.sig_pos_lon_sample = res.d_lon_max / kf;
        res.sig_alt_sample = res.d_alt_max / kf;
        res.sig_v_sample = res.d_v_max / kf;
        res.sig_hdg_sample = res.d_heading_max / kf;
    }

    // ------------------------------------------------------------------
    // 3. Уровень по средней удельной силе, курс по вращению Земли
    // ------------------------------------------------------------------
    const double g = normalGravity(lat_rad, alt);

    if (n_imu > 0)
    {
        double ax_norm = res.f_mean[0] / g;
        ax_norm = fmax(-1.0, fmin(1.0, ax_norm));
        res.pitch = asin(ax_norm);
        res.roll = atan2(-res.f_mean[2], res.f_mean[1]);

        // Горизонтальные оси СК с нулевым курсом: n_N = C^T·[1,0,0],
        // n_E = C^T·[0,0,1] при ψ = 0. Проекции дают курс независимо от него
        // самого: ψ = atan2(−W_E, W_N) (вывод — Aligner.cpp, alignFromImuProvider).
        const double cp = cos(res.pitch), sp = sin(res.pitch);
        const double cr = cos(res.roll), sr = sin(res.roll);
        const double wn = res.w_mean[0] * cp
                        - res.w_mean[1] * sp * cr
                        + res.w_mean[2] * sp * sr;
        const double we = res.w_mean[1] * cr + res.w_mean[2] * sr;
        res.gyro_compass_heading = normalize_angle(atan2(-we, wn));
        res.omega_h_meas = sqrt(wn * wn + we * we);
    }

    // Номинал курса: последний отсчёт эталона, иначе — гирокомпасирование.
    const double heading = res.has_heading ? res.heading_last : res.gyro_compass_heading;

    // ------------------------------------------------------------------
    // 4. Смещения: ba = f̄ − C^T·[0,g,0], bg = ω̄ − C^T·ω_Земли
    // ------------------------------------------------------------------
    if (n_imu > 0)
    {
        const Matrix C = bodyToNavMatrix(heading, res.pitch, res.roll);

        const Vector g_nav = {0.0, g, 0.0};
        const Vector f_expected = navToBody(C, g_nav);
        for (int k = 0; k < 3; k++)
        {
            res.ba_est[k] = res.f_mean[k] - f_expected[k];
        }

        // Вращение Земли в навигационной СК (Север, Вверх, Восток); при покое
        // транспортная составляющая нулевая.
        const Vector w_earth = {U_EARTH * cos(lat_rad), U_EARTH * sin(lat_rad), 0.0};
        const Vector w_expected = navToBody(C, w_earth);
        for (int k = 0; k < 3; k++)
        {
            res.bg_est[k] = res.w_mean[k] - w_expected[k];
        }
    }

    // ------------------------------------------------------------------
    // 5. Гейт покоя
    // ------------------------------------------------------------------
    res.rest_ok = true;
    auto fail = [&res](const char *why)
    {
        if (!res.rest_reason.empty())
        {
            res.rest_reason += ",";
        }
        res.rest_reason += why;
    };

    if (n_imu == 0)
    {
        fail("no_imu");
    }
    if (n_sns == 0)
    {
        fail("no_sns");
    }
    else
    {
        res.rest_speed = sqrt(res.vn_mean * res.vn_mean
                              + res.vh_mean * res.vh_mean
                              + res.ve_mean * res.ve_mean);
        // Порог по скорости СНС: 4 СКО среднего по трём осям. σ̂ среднего
        // берётся из максимума отклонения окна (фактический шум набора), но не
        // ниже паспортного cfg.sig_v/√N: у square_harsh шум СНС 0.5 м/с при
        // паспортных 0.048 — по паспорту гейт ложно проваливался бы на стоянке.
        const double sq_n = sqrt(static_cast<double>(n_sns));
        const double sig_v_mean = fmax(res.sig_v_sample / sq_n,
                                       ins::kalman_cfg.sig_v / sq_n);
        res.rest_speed_limit = REST_SIGMA_FACTOR * sig_v_mean * sqrt(3.0);
        if (res.rest_speed > res.rest_speed_limit)
        {
            fail("speed");
        }
    }

    if (n_imu > 0)
    {
        // Отклонение ω̄ от проекции вращения Земли — это и есть смещение
        // гироскопа (в покое); поворот или качка добавляют к нему угловую
        // скорость. За 1 с постоянное вращение от смещения неотделимо, поэтому
        // порог = паспортный запас на смещение + 4σ̂ оценки среднего.
        res.rest_omega = sqrt(res.bg_est[0] * res.bg_est[0]
                              + res.bg_est[1] * res.bg_est[1]
                              + res.bg_est[2] * res.bg_est[2]);
        const double sig_w_mean = sqrt(res.sig_w_mean[0] * res.sig_w_mean[0]
                                       + res.sig_w_mean[1] * res.sig_w_mean[1]
                                       + res.sig_w_mean[2] * res.sig_w_mean[2]);
        res.rest_omega_limit = REST_OMEGA_BIAS_LIMIT
                             + REST_SIGMA_FACTOR * sig_w_mean;
        if (res.rest_omega > res.rest_omega_limit)
        {
            fail("omega");
        }

        res.rest_fy = fabs(res.f_mean[1] - g);
        if (res.rest_fy > REST_FY_LIMIT_MG * 1e-3 * G_EQ)
        {
            fail("vertical_accel");
        }
    }

    res.rest_ok = res.rest_reason.empty();
    return res;
}

} // namespace nav
