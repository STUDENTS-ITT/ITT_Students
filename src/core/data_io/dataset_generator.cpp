// dataset_generator.cpp — Синтез тестовых наборов данных БИНС/СНС.
//
// Модель построена так, чтобы «чистые» (без шума) показания ИМУ, прогнанные
// через ядро счисления, уходили от сгенерированной истины лишь на ошибку
// дискретизации интеграторов. Для этого используются ровно те же формулы, что
// в step() (trajectory.h):
//   - f_body = C^(T)·(V̇ + a_harm), где a_harm = harmfulAccel(...) (Кориолис +
//     центробежное + гравитация), C = bodyToNavMatrix(heading, pitch, roll).
//   - ω_gyro = C^(T)·ω_н + ω_отн, где ω_н = navAngularRate(...), а ω_отн —
//     угловые скорости тела из скоростей Эйлера (инверсия eulerRates).
// Смещения/шумы добавляются в файлы КАК измерения (файлы imu.dat/gps.dat/
// angle.dat), истина (truth.txt) пишется без них.

#include "dataset_generator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>

#include "../ins/attitude_calc.h"
#include "../utils/constants.h"

namespace dataset
{
namespace
{

// Согласованная с еulerRates трансформация скоростей Эйлера в угловые скорости
// тела (относительно навигационной СК): wx = φ̇ − ψ̇·sin p,
// wy = −ψ̇·cos p·cos φ + θ̇·sin φ, wz = ψ̇·cos p·sin φ + θ̇·cos φ.
Vector eulerToBodyRates(double heading_dot, double pitch_dot, double roll_dot,
                        double pitch, double roll)
{
    const double s_p = sin(pitch);
    const double c_p = cos(pitch);
    const double s_r = sin(roll);
    const double c_r = cos(roll);
    return {
        roll_dot - heading_dot * s_p,
        -heading_dot * c_p * c_r + pitch_dot * s_r,
        heading_dot * c_p * s_r + pitch_dot * c_r};
}

// Одна точка сгенерированной (чистой) траектории.
struct TrajPoint
{
    double t = 0;
    double lat = 0;       // рад
    double lon = 0;       // рад
    double alt = 0;       // м
    double vn = 0;        // м/с
    double vh = 0;        // м/с (вверх)
    double ve = 0;        // м/с
    double heading = 0;   // рад
    double pitch = 0;     // рад
    double roll = 0;      // рад
    double fx = 0;        // удельная сила (тело), м/с²
    double fy = 0;
    double fz = 0;
    double wx = 0;        // угловая скорость (тело), рад/с
    double wy = 0;
    double wz = 0;
};

// Генератор случайных чисел с гауссовым распределением.
struct Rng
{
    std::mt19937_64 mt;
    std::normal_distribution<double> gauss;

    explicit Rng(uint64_t seed) : mt(seed) {}

    double nextNormal() { return gauss(mt); }
    double nextUniform(double a, double b)
    {
        std::uniform_real_distribution<double> u(a, b);
        return u(mt);
    }
};

// Экспоненциальное приближение к цели (C-T процесс) за один шаг.
inline double approach(double current, double target, double tau, double dt)
{
    if (tau <= 0.0)
    {
        return target;
    }
    return current + (target - current) * (1.0 - exp(-dt / tau));
}

// Сумма векторов с записью в скаляры (утилита для ω).
void vector_sum_values(const Vector &a, const Vector &b, double &x, double &y, double &z)
{
    x = a[0] + b[0];
    y = a[1] + b[1];
    z = a[2] + b[2];
}

// Линейная интерполяция между двумя точками траектории.
TrajPoint interpolate(const TrajPoint &a, const TrajPoint &b, double t)
{
    const double s = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
    TrajPoint p;
    p.t = t;
    p.lat = a.lat + (b.lat - a.lat) * s;
    p.lon = a.lon + (b.lon - a.lon) * s;
    p.alt = a.alt + (b.alt - a.alt) * s;
    p.vn = a.vn + (b.vn - a.vn) * s;
    p.vh = a.vh + (b.vh - a.vh) * s;
    p.ve = a.ve + (b.ve - a.ve) * s;
    p.heading = a.heading + (b.heading - a.heading) * s;
    p.pitch = a.pitch + (b.pitch - a.pitch) * s;
    p.roll = a.roll + (b.roll - a.roll) * s;
    p.fx = a.fx + (b.fx - a.fx) * s;
    p.fy = a.fy + (b.fy - a.fy) * s;
    p.fz = a.fz + (b.fz - a.fz) * s;
    p.wx = a.wx + (b.wx - a.wx) * s;
    p.wy = a.wy + (b.wy - a.wy) * s;
    p.wz = a.wz + (b.wz - a.wz) * s;
    return p;
}

// Синтез чистой траектории + чистых показаний ИМУ на сетке ИМУ.
bool synthesize(const DatasetConfig &cfg, std::vector<TrajPoint> &out,
                DatasetReport *report, std::string *error)
{
    if (cfg.profile.empty())
    {
        *error = "profile is empty";
        return false;
    }
    double duration = 0.0;
    for (const ProfileSegment &s : cfg.profile)
    {
        duration += s.dur_s;
    }
    if (duration <= 0.0)
    {
        *error = "profile duration is zero";
        return false;
    }

    const double d2r = DEG_TO_RAD;
    const double dt = 1.0 / (cfg.imu_hz > 0.0 ? cfg.imu_hz : 200.0);
    const int n_imu = static_cast<int>(floor(duration / dt)) + 1;

    const double lat0 = cfg.lat_deg * d2r;
    const double lon0 = cfg.lon_deg * d2r;
    const double pitch0 = cfg.pitch_deg * d2r;
    const double roll0 = cfg.roll_deg * d2r;
    const double g0 = normalGravity(lat0, cfg.alt_m);

    double lat = lat0;
    double lon = lon0;
    double alt = cfg.alt_m;
    double heading = cfg.heading_deg * d2r;
    double pitch = pitch0;
    double roll = roll0;

    double v_h = 0.0;         // горизонтальная скорость
    double v_up = 0.0;        // вертикальная скорость
    double psi_dot = 0.0;     // текущая скорость разворота, рад/с

    double traveled = 0.0;
    double seg_time_left = cfg.profile[0].dur_s;
    size_t seg_idx = 0;

    out.reserve(static_cast<std::size_t>(n_imu));

    for (int i = 0; i < n_imu; i++)
    {
        const double t = static_cast<double>(i) * dt;

        // Переход к следующему сегменту, когда время текущего истекло.
        while (seg_time_left <= 0.0 && seg_idx + 1 < cfg.profile.size())
        {
            seg_idx++;
            seg_time_left += cfg.profile[seg_idx].dur_s;
        }

        const ProfileSegment &seg = cfg.profile[seg_idx];

        // Командные значения сегмента.
        const double v_h_tgt = seg.v_ms;
        const double v_up_tgt = seg.v_up_ms;
        const double psi_dot_tgt = seg.turn_deg_s * d2r;

        // Сглаженное приближение к командам.
        v_h = approach(v_h, v_h_tgt, cfg.tau_speed_s, dt);
        v_up = approach(v_up, v_up_tgt, cfg.tau_climb_s, dt);
        psi_dot = approach(psi_dot, psi_dot_tgt, cfg.tau_turn_s, dt);

        // Скорости в навигационной СК: V = v_h·(cos ψ, 0, sin ψ) + (0, v_up, 0).
        TrajPoint p;
        p.t = t;
        p.vn = v_h * cos(heading);
        p.vh = v_up;
        p.ve = v_h * sin(heading);

        // Производные скоростей (экспоненциальное приближение и вращение).
        const double dv_h = cfg.tau_speed_s > 0.0 ? (v_h_tgt - v_h) / cfg.tau_speed_s : 0.0;
        const double dv_up = cfg.tau_climb_s > 0.0 ? (v_up_tgt - v_up) / cfg.tau_climb_s : 0.0;
        const double an = dv_h * cos(heading) - v_h * psi_dot * sin(heading);
        const double aup = dv_up;
        const double ae = dv_h * sin(heading) + v_h * psi_dot * cos(heading);

        // Курс и крен обновляются до интерполяции скоростей.
        heading += psi_dot * dt;
        const double roll_target = atan2(v_h * psi_dot, g0) + roll0;
        roll = approach(roll, roll_target, cfg.tau_roll_s, dt);
        pitch = pitch0;

        // Позиция (приближение локальной плоскости).
        const double rh = R_EARTH + alt;
        lat += p.vn * dt / rh;
        lon += p.ve * dt / (rh * fmax(fabs(cos(lat)), 1e-10));
        alt += p.vh * dt;
        p.lat = lat;
        p.lon = lon;
        p.alt = alt;
        p.heading = heading;
        p.pitch = pitch;
        p.roll = roll;

        // Удельная сила: f_body = C^(T)·(V̇ + a_harm).
        const double lat_dot = p.vn / rh;
        const double lon_dot = p.ve / (rh * fmax(fabs(cos(lat)), 1e-10));
        const Vector V = {p.vn, p.vh, p.ve};
        const Vector a_harm = nav::harmfulAccel(lat, alt, V, lat_dot, lon_dot);
        const Matrix C = bodyToNavMatrix(heading, pitch, roll);
        const Vector Vdot = {an, aup, ae};
        const Vector f_nav = vector_sum(Vdot, a_harm);
        const Vector f_body = navToBody(C, f_nav);
        p.fx = f_body[0];
        p.fy = f_body[1];
        p.fz = f_body[2];

        // Угловые скорости гироскопа: ω_гиро = C^(T)·ω_н + ω_отн (см. imu_processor.h).
        const double roll_dot = cfg.tau_roll_s > 0.0
                                    ? (roll_target - roll) / cfg.tau_roll_s
                                    : 0.0;
        const Vector w_nav = nav::navAngularRate(lat, V, rh);
        const Vector w_abs_body = navToBody(C, w_nav);
        vector_sum_values(w_abs_body, eulerToBodyRates(psi_dot, 0.0, roll_dot, pitch, roll),
                          p.wx, p.wy, p.wz);

        out.push_back(p);
        traveled += sqrt(p.vn * p.vn + p.ve * p.ve) * dt;

        seg_time_left -= dt;
    }

    if (report != nullptr)
    {
        report->duration_s = duration;
        if (!out.empty())
        {
            report->final_alt_m = out.back().alt;
        }
        report->traveled_m = traveled;
    }
    return true;
}

// Запись imu.dat с шумами/смещениями/дрожанием/пропусками.
void writeImu(const std::vector<TrajPoint> &traj, const DatasetConfig &cfg,
              Rng &rng, std::ofstream &f, std::size_t &rows)
{
    const NoiseConfig &n = cfg.noise;
    const double gyro_bias[3] = {
        n.gyro_bias_deg_h.size() > 0 ? n.gyro_bias_deg_h[0] * DEG_TO_RAD / 3600.0 : 0.0,
        n.gyro_bias_deg_h.size() > 1 ? n.gyro_bias_deg_h[1] * DEG_TO_RAD / 3600.0 : 0.0,
        n.gyro_bias_deg_h.size() > 2 ? n.gyro_bias_deg_h[2] * DEG_TO_RAD / 3600.0 : 0.0};
    const double accel_bias[3] = {
        n.accel_bias_mg.size() > 0 ? n.accel_bias_mg[0] * 1e-3 * G_EQ : 0.0,
        n.accel_bias_mg.size() > 1 ? n.accel_bias_mg[1] * 1e-3 * G_EQ : 0.0,
        n.accel_bias_mg.size() > 2 ? n.accel_bias_mg[2] * 1e-3 * G_EQ : 0.0};
    const double gyro_rw = n.gyro_bias_rw_deg_h_sqrt_s * DEG_TO_RAD / 3600.0;
    const double accel_rw = n.accel_bias_rw_mg_sqrt_s * 1e-3 * G_EQ;

    double bg[3] = {0.0, 0.0, 0.0};  // текущее блуждающее смещение ДУС
    double ba[3] = {0.0, 0.0, 0.0};  // текущее блуждающее смещение акселя

    constexpr uint64_t BASE_NS = 1787824723000000000ULL;

    rows = 0;
    for (std::size_t k = 0; k < traj.size(); k++)
    {
        // Пропуск отсчёта с вероятностью imu_dropout_p.
        if (rng.nextUniform(0.0, 1.0) < cfg.noise.imu_dropout_p)
        {
            continue;
        }

        const TrajPoint &p = traj[k];
        double t = p.t;
        if (cfg.noise.imu_jitter_s > 0.0)
        {
            t += rng.nextUniform(-cfg.noise.imu_jitter_s, cfg.noise.imu_jitter_s);
            if (t < 0.0) t = 0.0;
        }

        // Блуждание смещений.
        const double dt = (k > 0) ? p.t - traj[k - 1].t : cfg.noise.imu_jitter_s;
        const double sqrt_dt = sqrt(fmax(dt, 0.0));
        for (int j = 0; j < 3; j++)
        {
            bg[j] += gyro_rw * sqrt_dt * rng.nextNormal();
            ba[j] += accel_rw * sqrt_dt * rng.nextNormal();
        }

        const double wx = p.wx + gyro_bias[0] + bg[0] + n.gyro_sigma * rng.nextNormal();
        const double wy = p.wy + gyro_bias[1] + bg[1] + n.gyro_sigma * rng.nextNormal();
        const double wz = p.wz + gyro_bias[2] + bg[2] + n.gyro_sigma * rng.nextNormal();
        const double ax = p.fx + accel_bias[0] + ba[0] + n.accel_sigma * rng.nextNormal();
        const double ay = p.fy + accel_bias[1] + ba[1] + n.accel_sigma * rng.nextNormal();
        const double az = p.fz + accel_bias[2] + ba[2] + n.accel_sigma * rng.nextNormal();

        const uint64_t ts = BASE_NS + static_cast<uint64_t>(llround(t * 1e9));
        f << std::fixed << std::setprecision(6) << t << '\t' << ts
          << std::defaultfloat << std::setprecision(17)
          << '\t' << wx << '\t' << wy << '\t' << wz
          << '\t' << ax << '\t' << ay << '\t' << az << '\n';
        rows++;
    }
}

// Запись gps.dat, angle.dat и truth.txt по сетке СНС с интерполяцией траектории.
// Использует один проход по точкам траектории, выдавая на каждый sns-такт
// измерение GPS (позиция + скорости), угол и истину.
bool writeSns(const std::vector<TrajPoint> &traj, const DatasetConfig &cfg,
              Rng &rng, std::ofstream &gps, std::ofstream &angle,
              std::ofstream &truth, std::size_t &gps_rows,
              std::size_t &angle_rows, std::size_t &truth_rows, std::string *error)
{
    const double sns_dt = 1.0 / (cfg.gps_hz > 0.0 ? cfg.gps_hz : 10.0);
    if (cfg.gps_hz <= 0.0)
    {
        *error = "gps_hz must be > 0";
        return false;
    }

    const NoiseConfig &n = cfg.noise;
    const double d2r = DEG_TO_RAD;
    const double sigma_h = n.gps_pos_h_sigma;
    const double sigma_v = n.gps_pos_v_sigma;
    const double sigma_vel = n.gps_vel_sigma;
    const double off[3] = {
        n.gps_pos_offset.size() > 0 ? n.gps_pos_offset[0] : 0.0,
        n.gps_pos_offset.size() > 1 ? n.gps_pos_offset[1] : 0.0,
        n.gps_pos_offset.size() > 2 ? n.gps_pos_offset[2] : 0.0};

    const double ang_sigma[3] = {
        n.angle_sigma_deg.size() > 0 ? n.angle_sigma_deg[0] * d2r : 0.0,
        n.angle_sigma_deg.size() > 1 ? n.angle_sigma_deg[1] * d2r : 0.0,
        n.angle_sigma_deg.size() > 2 ? n.angle_sigma_deg[2] * d2r : 0.0};

    constexpr uint64_t BASE_NS = 1787824723000000000ULL;

    // Коррелированная ошибка СНС (процесс Оренштейна–Уленбека) по N, Верх, E.
    const double tau = n.gps_pos_corr_tau_s > 0.0 ? n.gps_pos_corr_tau_s : 1e9;
    const double decay = exp(-sns_dt / tau);
    const double corr_scale = n.gps_pos_corr_sigma * sqrt(1.0 - decay * decay);
    double corr[3] = {0.0, 0.0, 0.0};

    double dl_total = 0.0;
    for (const ProfileSegment &s : cfg.profile) dl_total += s.dur_s;
    const int N = static_cast<int>(floor(dl_total * cfg.gps_hz));

    gps_rows = 0;
    angle_rows = 0;
    truth_rows = 0;

    for (int i = 0; i < N; i++)
    {
        const double t_nom = static_cast<double>(i + 1) / cfg.gps_hz;
        double t = t_nom;
        if (cfg.noise.sns_jitter_s > 0.0)
        {
            t += rng.nextUniform(-cfg.noise.sns_jitter_s, cfg.noise.sns_jitter_s);
            if (t < 0.0) t = 0.0;
        }

        // Интерполяция траектории в момент времени t.
        if (traj.empty())
        {
            *error = "empty trajectory";
            return false;
        }
        auto it = std::upper_bound(traj.begin(), traj.end(), t,
                                   [](double tt, const TrajPoint &p) { return tt < p.t; });
        const std::size_t idx = static_cast<std::size_t>(
            std::distance(traj.begin(), std::max(traj.begin(), it - 1)));
        const std::size_t idx_a = std::min(idx + 1, traj.size() - 1);
        const TrajPoint p = interpolate(traj[idx], traj[idx_a], t);

        // Истина.
        truth << std::left << std::fixed << std::setprecision(8) << std::setw(15) << t
              << std::setw(15) << p.lon * RAD_TO_DEG
              << std::setw(15) << p.lat * RAD_TO_DEG
              << std::setw(15) << p.alt
              << std::setw(15) << p.heading * RAD_TO_DEG
              << std::setw(15) << p.pitch * RAD_TO_DEG
              << std::setw(15) << p.roll * RAD_TO_DEG
              << std::setw(15) << p.vn
              << std::setw(15) << p.vh
              << std::setw(15) << p.ve << '\n';
        truth_rows++;

        // Коррелированная ошибка позиции.
        const double g_n[3] = {rng.nextNormal(), rng.nextNormal(), rng.nextNormal()};
        for (int j = 0; j < 3; j++)
        {
            corr[j] = corr[j] * decay + corr_scale * g_n[j];
        }

        // GPS: позиция с учётом смещения, корреляции и белого шума.
        const double rh = R_EARTH + p.alt;
        const double d_n = off[0] + corr[0] + sigma_h * rng.nextNormal();
        const double d_up = off[1] + corr[1] + sigma_v * rng.nextNormal();
        const double d_e = off[2] + corr[2] + sigma_h * rng.nextNormal();
        const double gps_lat = p.lat + d_n / rh;
        const double gps_lon = p.lon + d_e / (rh * fmax(fabs(cos(p.lat)), 1e-10));
        const double gps_alt = p.alt + d_up;

        const double gps_vn = p.vn + sigma_vel * rng.nextNormal();
        const double gps_vh = p.vh + sigma_vel * rng.nextNormal();
        const double gps_ve = p.ve + sigma_vel * rng.nextNormal();

        const uint64_t gps_ts = BASE_NS + static_cast<uint64_t>(llround(t * 1e9));
        gps << std::fixed << std::setprecision(6) << t << '\t' << gps_ts
            << std::defaultfloat << std::setprecision(17)
            << '\t' << gps_lat * RAD_TO_DEG << '\t' << gps_lon * RAD_TO_DEG << '\t' << gps_alt
            << '\t' << gps_vn << '\t' << gps_vh << '\t' << gps_ve << '\n';
        gps_rows++;

        // Angle: отдельная метка времени (собственное дрожание).
        double t_a = t_nom;
        if (cfg.noise.sns_jitter_s > 0.0)
        {
            t_a += rng.nextUniform(-cfg.noise.sns_jitter_s, cfg.noise.sns_jitter_s);
            if (t_a < 0.0) t_a = 0.0;
        }
        const TrajPoint pa = interpolate(traj[idx], traj[idx_a], t_a);
        const double roll = pa.roll + ang_sigma[0] * rng.nextNormal();
        const double pitch = pa.pitch + ang_sigma[1] * rng.nextNormal();
        const double yaw = pa.heading + ang_sigma[2] * rng.nextNormal();

        const uint64_t ang_ts = BASE_NS + static_cast<uint64_t>(llround(t_a * 1e9));
        angle << std::fixed << std::setprecision(6) << t_a << '\t' << ang_ts
              << std::fixed << std::setprecision(10)
              << '\t' << roll << '\t' << pitch << '\t' << yaw << '\n';
        angle_rows++;
    }

    return true;
}

} // namespace

bool generateDataset(const DatasetConfig &cfg, DatasetReport &report,
                     std::string *error)
{
    if (error == nullptr)
    {
        static std::string err_stub;
        error = &err_stub;
    }
    error->clear();

    if (cfg.out_dir.empty())
    {
        *error = "out_dir is empty";
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(cfg.out_dir, ec);
    if (ec)
    {
        *error = "cannot create output directory: " + ec.message();
        return false;
    }

    // Синтез чистой траектории.
    std::vector<TrajPoint> traj;
    if (!synthesize(cfg, traj, &report, error))
    {
        return false;
    }

    Rng rng(cfg.seed);

    const std::string imu_path = cfg.out_dir + "/imu.dat";
    const std::string gps_path = cfg.out_dir + "/gps.dat";
    const std::string angle_path = cfg.out_dir + "/angle.dat";
    const std::string truth_path = cfg.out_dir + "/truth.txt";
    const std::string ini_path = cfg.out_dir + "/StartupNav.ini";

    {
        std::ofstream f(imu_path);
        if (!f.is_open())
        {
            *error = "cannot open " + imu_path;
            return false;
        }
        f << "# time_s\ttimestamp_ns\twx\twy\twz\tax\tay\taz\n";
        writeImu(traj, cfg, rng, f, report.imu_rows);
    }

    report.gps_rows = 0;
    report.angle_rows = 0;
    report.truth_rows = 0;
    if (cfg.gps_hz > 0.0)
    {
        std::ofstream gps(gps_path);
        std::ofstream angle(angle_path);
        std::ofstream truth(truth_path);
        if (!gps.is_open() || !truth.is_open())
        {
            *error = "cannot open gps.dat / truth.txt";
            return false;
        }
        gps << "# time_s\ttimestamp_ns\tlatitude\tlongitude\taltitude\tvx\tvy\tvz\n";
        if (angle.is_open())
        {
            angle << "# time_s\ttimestamp_ns\troll\tpitch\tyaw\n";
        }
        truth << std::left
              << std::setw(15) << "time" << std::setw(15) << "lon"
              << std::setw(15) << "lat" << std::setw(15) << "alt"
              << std::setw(15) << "heading" << std::setw(15) << "pitch"
              << std::setw(15) << "roll" << std::setw(15) << "vn"
              << std::setw(15) << "vh" << std::setw(15) << "ve" << '\n';

        if (!writeSns(traj, cfg, rng, gps, angle, truth, report.gps_rows,
                      report.angle_rows, report.truth_rows, error))
        {
            return false;
        }
    }

    {
        std::ofstream f(ini_path);
        if (!f.is_open())
        {
            *error = "cannot open " + ini_path;
            return false;
        }
        f << std::fixed << std::setprecision(10)
          << cfg.lon_deg << '\t' << "; долгота начальной точки (град)" << '\n'
          << cfg.lat_deg << '\t' << "; широта начальной точки (град)" << '\n'
          << cfg.alt_m << '\t' << "; высота начальной точки (м)" << '\n'
          << cfg.align_time_s << '\t' << "; время выставки (сек)" << '\n';
    }

    return true;
}

} // namespace dataset