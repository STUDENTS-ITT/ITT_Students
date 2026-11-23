// main.cpp — Выставка по окну усреднения (AveragingWindow) + счисление БИНС
// с ESKF 15 и защитой СНС.
//
// Поток выполнения:
//   1. Разбор аргументов и конфигурации (StartupNav.ini, опциональный params.ini).
//   2. Окно усреднения T_align: средние/Δ ИМУ и СНС, наклон, ba/bg, гейт покоя.
//   3. Начальное состояние: номинал — последний отсчёт СНС окна (sns_last),
//      x0 = (last - mean), P0 по σ_sample/σ_mean и k(N)=E[max|N(0,1)|].
//      ba: state.ba=ba_est, x0[9..11]=0; bg: state.bg=0, x0[12..14]=bg_est.
//      При !rest_ok/недостатке данных — safe fallback (паспортный P0, x0=0).
//   4. Основной цикл: nav::step (прогноз+интегрирование каждый такт ИМУ,
//      коррекция по СНС раз в SNS_DECIMATION с SnsGuard).
//
// Использование:
//   imitator [каталог_данных] [params.ini]
//
//   каталог_данных по умолчанию — ../data/raw

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "data_io/data_reader.h"
#include "data_io/data_writer.h"
#include "ins/imu_processor.h"
#include "ins/ins_filter.h"
#include "navigation/aligner.h"
#include "navigation/averaging.h"
#include "navigation/gnss_guard.h"
#include "navigation/gps_processor.h"
#include "navigation/trajectory.h"
#include "common/constants.h"

namespace
{

constexpr double ALIGN_FALLBACK_S = 1.0;

void applyParamsFile(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        std::cerr << "params.ini: cannot open " << path << std::endl;
        return;
    }

    struct Entry
    {
        const char *name;
        double *field;
    };
    const Entry entries[] = {
        {"sigma_g", &ins::kalman_cfg.sig_g},
        {"sigma_a", &ins::kalman_cfg.sig_a},
        {"sigma_bg", &ins::kalman_cfg.sig_bg},
        {"sigma_ba", &ins::kalman_cfg.sig_ba},
        {"sigma_pos", &ins::kalman_cfg.sig_pos},
        {"sigma_h", &ins::kalman_cfg.sig_h},
        {"sigma_v", &ins::kalman_cfg.sig_v},
        {"sigma_hdg", &ins::kalman_cfg.sig_hdg},
        {"sigma_ang", &ins::kalman_cfg.sig_hdg},
        {"align_time_s", &ins::kalman_cfg.align_time_s},
        {"gnss_guard_on", &ins::kalman_cfg.gnss_guard_on},
        {"gnss_vel_plaus", &ins::kalman_cfg.gnss_vel_plaus},
        {"gnss_silence_timeout_s", &ins::kalman_cfg.gnss_silence_timeout_s},
        {"gnss_alt_band_m", &ins::kalman_cfg.gnss_alt_band_m},
        {"gnss_dt_max_s", &ins::kalman_cfg.gnss_dt_max_s},
        {"gnss_jump_pos_m", &ins::kalman_cfg.gnss_jump_pos_m},
        {"gnss_jump_vel_ms", &ins::kalman_cfg.gnss_jump_vel_ms},
    };

    std::string line;
    while (std::getline(f, line))
    {
        const auto eq = line.find('=');
        if (eq == std::string::npos)
            continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        while (!key.empty() && (key.back() == ' ' || key.back() == '\t' || key.back() == '\r'))
            key.pop_back();
        while (!key.empty() && (key.front() == ' ' || key.front() == '\t'))
            key.erase(key.begin());
        while (!val.empty() && (val.front() == ' ' || val.front() == '\t'))
            val.erase(val.begin());
        while (!val.empty() && (val.back() == ' ' || val.back() == '\t' || val.back() == '\r'))
            val.pop_back();
        if (key.empty() || val.empty() || key[0] == '#' || key[0] == ';')
            continue;

        for (const auto &e : entries)
        {
            if (key == e.name)
            {
                try
                {
                    *e.field = std::stod(val);
                }
                catch (const std::exception &)
                {
                    std::cerr << "params.ini: cannot parse " << key << " = " << val << std::endl;
                }
                break;
            }
        }
    }

    std::cout << "params.ini: loaded " << path << std::endl;
}

// Чтение StartupNav.ini: lon_deg, lat_deg, alt, T_align (с).
bool readStartupNav(const std::string &path, double &lon_deg, double &lat_deg,
                    double &alt, double &time_s)
{
    std::ifstream f(path);
    if (!f.is_open())
        return false;

    std::string line;
    if (!std::getline(f, line))
        return false;
    std::sscanf(line.c_str(), "%lf", &lon_deg);
    if (!std::getline(f, line))
        return false;
    std::sscanf(line.c_str(), "%lf", &lat_deg);
    if (!std::getline(f, line))
        return false;
    std::sscanf(line.c_str(), "%lf", &alt);
    if (!std::getline(f, line))
        return false;
    std::sscanf(line.c_str(), "%lf", &time_s);
    return true;
}

std::string pickFile(const std::string &dir, const std::string &preferred,
                     const std::string &fallback)
{
    const std::string first = dir + "/" + preferred;
    std::error_code ec;
    if (std::filesystem::exists(first, ec))
        return first;
    const std::string second = dir + "/" + fallback;
    if (std::filesystem::exists(second, ec))
        return second;
    return first;
}

void writeInitLog(const std::string &path, const std::string &data_dir,
                  const nav::AveragingResult &avg, const nav::NavState &st,
                  double sig_heading0, bool heading_from_sns)
{
    std::ofstream f(path);
    if (!f.is_open())
        return;

    f << std::fixed << std::setprecision(6);
    f << "# init.txt - итоги выставки (AveragingWindow)\n";
    f << "data: " << data_dir << "\n";
    f << "align_time_s: " << avg.t_end << "\n";
    f << "imu_samples: " << avg.imu_count << "\n";
    f << "sns_samples: " << avg.sns_count << "\n";
    f << "rest_ok: " << (avg.rest_ok ? 1 : 0) << "\n";
    f << "rest_reason: " << (avg.rest_reason.empty() ? "-" : avg.rest_reason) << "\n";
    f << "heading_source: " << (heading_from_sns ? "sns/angle" : "gyro-compass") << "\n";

    f << std::setprecision(9);
    f << "sig_a_mean_m_s2: " << avg.sig_a_mean[0] << " " << avg.sig_a_mean[1] << " " << avg.sig_a_mean[2] << "\n";
    f << "sig_w_mean_rad_s: " << avg.sig_w_mean[0] << " " << avg.sig_w_mean[1] << " " << avg.sig_w_mean[2] << "\n";
    f << "sig_pos_sample_m: " << avg.sig_pos_sample << " " << avg.sig_pos_lon_sample << "\n";
    f << "sig_alt_sample_m: " << avg.sig_alt_sample << "\n";
    f << "sig_v_sample_m_s: " << avg.sig_v_sample << "\n";
    f << "sig_hdg_sample_deg: " << avg.sig_hdg_sample * RAD_TO_DEG << "\n";
    f << "sig_heading0_deg: " << sig_heading0 * RAD_TO_DEG << "\n";

    const double mg = 1e-3 * G_EQ;
    const double deg_h = RAD_TO_DEG * 3600.0;
    f << "ba_est_mg: " << avg.ba_est[0] / mg << " " << avg.ba_est[1] / mg << " " << avg.ba_est[2] / mg << "\n";
    f << "bg_est_deg_h: " << avg.bg_est[0] * deg_h << " " << avg.bg_est[1] * deg_h << " " << avg.bg_est[2] * deg_h << "\n";
    f << "pitch_deg: " << avg.pitch * RAD_TO_DEG << "\n";
    f << "roll_deg: " << avg.roll * RAD_TO_DEG << "\n";
    f << "heading_nominal_deg: " << st.att.heading * RAD_TO_DEG << "\n";

    if (avg.has_sns)
    {
        f << "sns_last_deg: " << avg.sns_last.lat * RAD_TO_DEG << " "
          << avg.sns_last.lon * RAD_TO_DEG << " " << avg.sns_last.alt << "\n";
        f << "sns_last_v_m_s: " << avg.sns_last.vn << " " << avg.sns_last.vh << " " << avg.sns_last.ve << "\n";
    }
    f << "nominal_deg: " << st.lat * RAD_TO_DEG << " " << st.lon * RAD_TO_DEG << " " << st.alt << "\n";
    f << "nominal_v_m_s: " << st.V[0] << " " << st.V[1] << " " << st.V[2] << "\n";
    f << "nominal_ba_mg: " << st.ba[0] / mg << " " << st.ba[1] / mg << " " << st.ba[2] / mg << "\n";
    f << "nominal_bg_deg_h: " << st.bg[0] * deg_h << " " << st.bg[1] * deg_h << " " << st.bg[2] * deg_h << "\n";
    f << "x0:";
    for (int i = 0; i < ins::KF_STATE; i++)
        f << " " << st.x[i];
    f << "\n";
    f << "sigma0:";
    for (int i = 0; i < ins::KF_STATE; i++)
        f << " " << sqrt(fmax(0.0, at(st.P, i, i, ins::KF_STATE)));
    f << "\n";
}

} // namespace

int main(int argc, char **argv)
{
    // argv[1] — каталог данных (по умолчанию ../data/raw), argv[2] — params.ini.
    const std::string data_dir = (argc > 1) ? argv[1] : "../data/raw";
    if (argc > 2)
    {
        applyParamsFile(argv[2]);
    }

    // Имена файлов данных и результатов (совместимость с проектом).
    const std::string imu_file = pickFile(data_dir, "imu.dat", "imu.data");
    const std::string gps_file = pickFile(data_dir, "gps.dat", "gps.data");
    const std::string angle_file = pickFile(data_dir, "angle.dat", "angle.data");
    const std::string startup_file = pickFile(data_dir, "StartupNav.ini", "StartupNav.ini");
    const std::string out_file = "kalman15_line2.txt";
    const std::string err_file = "d_1.txt";
    const std::string init_file = "init.txt";

    const bool has_angle = std::filesystem::exists(angle_file);
    std::cout << "data: " << data_dir << std::endl;
    std::cout << "angle.dat: " << (has_angle ? "found" : "not found (gyro-compass)") << std::endl;

    const auto start_time = std::chrono::high_resolution_clock::now();

    // ------------------------------------------------------------------
    // 1. Точка и время выставки (StartupNav.ini)
    // ------------------------------------------------------------------
    double start_lon = 0.0, start_lat = 0.0, start_alt = 0.0;
    double align_time = ins::kalman_cfg.align_time_s;
    if (!(align_time > 0.0))
        align_time = ALIGN_FALLBACK_S;

    if (readStartupNav(startup_file, start_lon, start_lat, start_alt, align_time))
    {
        std::cout << "StartupNav: lon=" << start_lon << " lat=" << start_lat
                  << " alt=" << start_alt << " T_align=" << align_time << " s" << std::endl;
        if (!(align_time > 0.0))
        {
            std::cout << "WARNING: StartupNav T_align is not positive, using "
                      << ALIGN_FALLBACK_S << " s" << std::endl;
            align_time = ALIGN_FALLBACK_S;
        }
    }
    else
    {
        std::cout << "StartupNav not found, trying first SNS sample..." << std::endl;
        data_io::SnsReader sns_tmp;
        if (sns_tmp.open(gps_file, has_angle ? angle_file : ""))
        {
            nav::SnsSample first;
            if (sns_tmp.next(first))
            {
                start_lon = first.lon * RAD_TO_DEG;
                start_lat = first.lat * RAD_TO_DEG;
                start_alt = first.alt;
            }
            sns_tmp.close();
        }
        if (!(align_time > 0.0))
            align_time = ALIGN_FALLBACK_S;
        std::cout << "Using T_align=" << align_time << " s" << std::endl;
    }
    ins::kalman_cfg.align_time_s = align_time;

    // ------------------------------------------------------------------
    // 2. Окно усреднения выставки
    // ------------------------------------------------------------------
    std::cout << "=== Alignment: AveragingWindow " << align_time << " s ===" << std::endl;

    data_io::ImuReader imu;
    data_io::SnsReader sns;
    if (!imu.open(imu_file))
        return 1;
    if (!sns.open(gps_file, has_angle ? angle_file : ""))
        return 1;

    nav::AveragingWindow window(align_time, has_angle);

    std::vector<double> row;
    std::vector<double> pending_row;
    bool has_pending_row = false;

    while (imu.next(row))
    {
        if (!ins::isValidRow(row))
            continue;
        if (ins::sampleTime(row) > align_time)
        {
            pending_row = row;
            has_pending_row = true;
            break;
        }
        window.addImu(ins::sampleTime(row),
                      row[ins::IMU_COL_GYRO + 0], row[ins::IMU_COL_GYRO + 1], row[ins::IMU_COL_GYRO + 2],
                      row[ins::IMU_COL_ACCEL + 0], row[ins::IMU_COL_ACCEL + 1], row[ins::IMU_COL_ACCEL + 2]);
    }

    nav::SnsSample sns_sample;
    nav::SnsSample next_frame;
    bool has_next_frame = false;
    while (sns.next(sns_sample))
    {
        if (sns_sample.time > align_time)
        {
            next_frame = sns_sample;
            has_next_frame = true;
            break;
        }
        window.addSns(sns_sample);
    }

    const nav::AveragingResult avg = window.finish(start_lat * DEG_TO_RAD, start_alt);

    if (window.overflowed())
        std::cout << "WARNING: averaging window buffer overflow (T_align too large)" << std::endl;
    if (avg.imu_count == 0 || avg.sns_count == 0)
        std::cout << "WARNING: averaging window empty (imu=" << avg.imu_count
                  << ", sns=" << avg.sns_count << "), using safe fallback" << std::endl;

    std::cout << "averaging: imu=" << avg.imu_count << " sns=" << avg.sns_count
              << " t=[" << avg.t_start << ", " << avg.t_end << "]" << std::endl;
    std::cout << "Pitch=" << avg.pitch * RAD_TO_DEG << " deg, Roll=" << avg.roll * RAD_TO_DEG << " deg" << std::endl;
    std::cout << "ba [mg]: " << avg.ba_est[0] / (1e-3 * G_EQ) << ", "
              << avg.ba_est[1] / (1e-3 * G_EQ) << ", " << avg.ba_est[2] / (1e-3 * G_EQ) << std::endl;
    std::cout << "bg [deg/h]: " << avg.bg_est[0] * RAD_TO_DEG * 3600.0 << ", "
              << avg.bg_est[1] * RAD_TO_DEG * 3600.0 << ", " << avg.bg_est[2] * RAD_TO_DEG * 3600.0 << std::endl;
    if (avg.rest_ok)
        std::cout << "rest gate: passed" << std::endl;
    else
        std::cout << "WARNING: rest gate failed (" << avg.rest_reason
                  << "): safe fallback P0, x0=0" << std::endl;

    // ------------------------------------------------------------------
    // 3. Начальное состояние (номинал = sns_last, конвенция ba/bg/x0)
    // ------------------------------------------------------------------
    const bool heading_from_sns = avg.has_heading;
    double sig_heading0 = -1.0; // <= 0: gyro-compass формула в P0
    if (heading_from_sns)
        sig_heading0 = fmax(ins::kalman_cfg.sig_hdg, avg.sig_hdg_sample);

    nav::NavState state;
    if (avg.has_sns)
    {
        state.lat = avg.sns_last.lat;
        state.lon = avg.sns_last.lon;
        state.alt = avg.sns_last.alt;
        state.V = {avg.sns_last.vn, avg.sns_last.vh, avg.sns_last.ve};
    }
    else
    {
        state.lat = start_lat * DEG_TO_RAD;
        state.lon = start_lon * DEG_TO_RAD;
        state.alt = start_alt;
        state.V = {0.0, 0.0, 0.0};
    }

    state.att.heading = heading_from_sns ? avg.heading_last : avg.gyro_compass_heading;
    state.att.pitch = avg.pitch;
    state.att.roll = avg.roll;

    // Конвенция: ba в state (механизация), bg=0 в state и bg_est в x0[12..14].
    const bool apply_avg = avg.rest_ok && avg.imu_count > 0 && avg.sns_count > 0;
    if (apply_avg)
    {
        state.ba = avg.ba_est;
        state.bg = {0.0, 0.0, 0.0};
    }
    else
    {
        state.ba = {0.0, 0.0, 0.0};
        state.bg = {0.0, 0.0, 0.0};
    }
    state.bg_static = {0.0, 0.0, 0.0};

    state.P = nav::initialCovarianceFromAveraging(ins::kalman_cfg, avg, state.lat, sig_heading0);
    state.x = nav::initialErrorsFromAveraging(avg, apply_avg);
    state.time_prev = align_time;

    const double sig_heading_report = (sig_heading0 > 0.0)
                                          ? sig_heading0
                                          : sqrt(fmax(at(state.P, 6, 6, ins::KF_STATE), 0.0));
    std::cout << "heading source: " << (heading_from_sns ? "sns/angle" : "gyro-compass")
              << ", sigma(heading0)=" << sig_heading_report * RAD_TO_DEG << " deg" << std::endl;
    if (!has_angle)
        std::cout << "WARNING: angle.dat not found: initial heading from gyro-compass" << std::endl;

    writeInitLog(init_file, data_dir, avg, state, sig_heading_report, heading_from_sns);
    std::cout << "init log: " << init_file << std::endl;

    // ------------------------------------------------------------------
    // 4. Основной цикл
    // ------------------------------------------------------------------
    data_io::NavLogger log;
    if (!log.open(out_file, err_file))
        return 1;
    log.writeHeader();

    nav::SnsGuard sns_guard;

    // Текущий опорный кадр СНС (hold): последний принятый с time <= imu_time.
    nav::SnsSample hold_ref = avg.has_sns ? avg.sns_last : nav::SnsSample{};
    bool hold_valid = avg.has_sns;
    nav::SnsSample last_ref = next_frame;
    bool has_ref = has_next_frame;
    if (!hold_valid && has_next_frame)
    {
        hold_ref = next_frame;
        hold_valid = true;
    }

    int i = 0; // счётчик обработанных отсчётов ИМУ после окна

    auto process_row = [&](const std::vector<double> &r) {
        if (!ins::isValidRow(r))
            return;
        const double imu_time = ins::sampleTime(r);

        while (!has_ref || last_ref.time <= imu_time)
        {
            if (has_ref && last_ref.time <= imu_time)
            {
                hold_ref = last_ref;
                hold_valid = true;
            }
            if (!sns.next(last_ref))
            {
                has_ref = false;
                break;
            }
            has_ref = true;
            if (last_ref.time > imu_time)
                break;
        }

        if (!hold_valid)
            return;

        nav::step(i, r, hold_ref, state, log, sns_guard);
        i++;
    };

    if (has_pending_row)
        process_row(pending_row);

    while (imu.next(row))
        process_row(row);

    imu.close();
    sns.close();
    log.close();

    const auto end_time = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Output: " << out_file << ", " << err_file << std::endl;
    std::cout << "samples=" << i << ", time=" << elapsed.count() << " s" << std::endl;
    std::cout << "gnss: accepted=" << sns_guard.accepted_frames
              << " rejected=" << sns_guard.rejected_frames
              << " status=" << (sns_guard.status == nav::SnsStatus::VALID
                                    ? "VALID"
                                    : (sns_guard.status == nav::SnsStatus::STALE ? "STALE" : "INVALID"))
              << std::endl;

    return 0;
}
