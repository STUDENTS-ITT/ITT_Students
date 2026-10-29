// main.cpp — Консольная точка входа: выставка по окну усреднения + счисление
// БИНС с фильтром Калмана.
//
// Движок прогона целиком в этом файле (как в исходной консольной версии):
//   1. Разбор аргументов.
//   2. Точка и время выставки: StartupNav.ini, иначе — первый отсчёт СНС.
//   3. Окно усреднения T_align (4-я строка StartupNav.ini, обычно 1 с):
//      средние ИМУ/СНС, максимумы отклонения от среднего, наклон, ba, bg,
//      гейт покоя (docs/fast_alignment_plan.md).
//   4. Начальное состояние: номинал — последний отсчёт СНС окна, начальные
//      ошибки x0 — «последний отсчёт − среднее», P0 — по максимумам.
//   5. Основной цикл по строкам imu.dat: интегрирование, коррекция tilt,
//      коррекция по СНС, запись результатов. Первая коррекция СНС — по
//      первому кадру за окном; потоки ИМУ/СНС не перематываются.
//   6. Диагностика — только по явным флагам, на результат не влияет.
//
// Использование:
//   imitator [каталог_данных] [params.ini] [--bias-log] [--bias-report] [--consistency]
//
//   каталог_данных по умолчанию — ../data/raw
//   params.ini — необязательный плоский файл шумов фильтра (формат исходной
//                версии: ключи без секций)
//   результаты пишутся в tools/ рядом с исполняемым файлом
//
// Флаги:
//   --bias-log     писать bias.txt — журнал оценок смещений ДУС/акселя
//   --bias-report  прогнать набор и вывести сводную таблицу по смещениям
//                  (истина из params.json / оценка / σ(P) / норм. ошибка)
//   --consistency  писать consistency.txt — согласованность фильтра (NIS)
//                  на кадрах коррекции по СНС
//
// Выставка переведена на схему «окно 1 с»: вместо Median + EMA по 120 с
// считается среднее окна, а по максимуму отклонения от него — СКО. Это
// осознанное отличие от Qt-версии (docs/noqt_deviations.md): формат
// result.txt / reference.txt / errors.txt не меняется, но содержимое
// (начальные условия) — да.

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
#include "navigation/aligner.h"
#include "navigation/averaging.h"
#include "navigation/bias_log.h"
#include "navigation/bias_report.h"
#include "navigation/gps_processor.h"
#include "navigation/trajectory.h"
#include "utils/constants.h"
#include "utils/paths.h"

namespace
{

// Время выставки, если StartupNav.ini отсутствует или в нём мусор.
// Окно усреднения = время выставки, поэтому по умолчанию — 1 с.
constexpr double ALIGN_FALLBACK_S = 1.0;

// Порог годности курса по вращению Земли (R4) — теперь только диагностика:
// при наличии angle.dat курс берётся из эталона, без него — гирокомпасом
// с честным приором σ_bg/Ω_h.
constexpr double GYRO_COMPASS_LIMIT = 0.3;

// Ключи params.ini → поля KalmanConfig (плоский формат, как в исходной версии).
void applyParamsFile(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        std::cerr << "params.ini: cannot open " << path << std::endl;
        return;
    }

    struct Entry { const char *name; double *field; };
    const Entry entries[] = {
        {"sigma_g", &ins::kalman_cfg.sig_g},
        {"sigma_a", &ins::kalman_cfg.sig_a},
        {"sigma_bg", &ins::kalman_cfg.sig_bg},
        {"sigma_ba", &ins::kalman_cfg.sig_ba},
        {"sigma_pos", &ins::kalman_cfg.sig_pos},
        {"sigma_v", &ins::kalman_cfg.sig_v},
        {"sigma_ang", &ins::kalman_cfg.sig_hdg},
        {"outage_start_s", &ins::kalman_cfg.outage_start_s},
        {"outage_end_s", &ins::kalman_cfg.outage_end_s},
    };

    std::string line;
    while (std::getline(f, line))
    {
        // Пропускаем пустые строки и комментарии.
        std::string key, val;
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        key = line.substr(0, eq);
        val = line.substr(eq + 1);

        // Обрезаем пробелы.
        while (!key.empty() && key.back() == ' ') key.pop_back();
        while (!val.empty() && val.front() == ' ') val.erase(val.begin());
        while (!val.empty() && val.back() == ' ') val.pop_back();
        if (key.empty() || val.empty()) continue;

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
                    std::cerr << "params.ini: cannot parse " << key << " = " << val
                              << std::endl;
                }
                break;
            }
        }
    }

    std::cout << "params.ini: sigma_g=" << ins::kalman_cfg.sig_g
              << " sigma_a=" << ins::kalman_cfg.sig_a
              << " sigma_bg=" << ins::kalman_cfg.sig_bg
              << " sigma_ba=" << ins::kalman_cfg.sig_ba
              << " sigma_pos=" << ins::kalman_cfg.sig_pos
              << " sigma_v=" << ins::kalman_cfg.sig_v
              << " sigma_ang=" << ins::kalman_cfg.sig_hdg << std::endl;
}

// Чтение StartupNav.ini: долгота, широта, высота, время выставки — по строке.
bool readStartupNav(const std::string &path, double &lon_deg, double &lat_deg,
                    double &alt, double &time_s)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        return false;
    }

    std::string line;

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &lon_deg);

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &lat_deg);

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &alt);

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &time_s);

    return true;
}

// Выбор файла в каталоге данных: предпочтительное имя, иначе резервное.
// При отсутствии обоих возвращается предпочтительный путь (для сообщения об ошибке).
std::string pickFile(const std::string &dir, const std::string &preferred,
                     const std::string &fallback)
{
    const std::string first = dir + "/" + preferred;
    std::error_code ec;
    if (std::filesystem::exists(first, ec))
    {
        return first;
    }
    const std::string second = dir + "/" + fallback;
    if (std::filesystem::exists(second, ec))
    {
        return second;
    }
    return first;
}

// Запись итогов выставки в tools/init.txt. Файл диагностический: на прогон
// не влияет, нужен для разбора начальных условий и согласованности P0.
void writeInitLog(const std::string &path, const std::string &data_dir,
                  const nav::AveragingResult &avg, const nav::NavState &st,
                  double sig_heading0, bool heading_from_sns, double gc_ratio)
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        std::cerr << "cannot open " << path << std::endl;
        return;
    }

    f << std::fixed << std::setprecision(6);
    f << "# init.txt — итоги выставки (окно усреднения)\n";
    f << "data: " << data_dir << "\n";
    f << "align_time_s: " << avg.t_end << "\n";
    f << "window_s: " << avg.t_start << " .. " << avg.t_end << "\n";
    f << "imu_samples: " << avg.imu_count << "\n";
    f << "sns_samples: " << avg.sns_count << "\n";

    f << std::setprecision(9);
    f << "f_mean: " << avg.f_mean[0] << " " << avg.f_mean[1] << " "
      << avg.f_mean[2] << "\n";
    f << "w_mean: " << avg.w_mean[0] << " " << avg.w_mean[1] << " "
      << avg.w_mean[2] << "\n";
    f << "df_max: " << avg.df_max[0] << " " << avg.df_max[1] << " "
      << avg.df_max[2] << "\n";
    f << "dw_max: " << avg.dw_max[0] << " " << avg.dw_max[1] << " "
      << avg.dw_max[2] << "\n";
    f << "sig_a_mean: " << avg.sig_a_mean[0] << " " << avg.sig_a_mean[1] << " "
      << avg.sig_a_mean[2] << "\n";
    f << "sig_w_mean: " << avg.sig_w_mean[0] << " " << avg.sig_w_mean[1] << " "
      << avg.sig_w_mean[2] << "\n";

    f << "d_pos_max_m: " << avg.d_lat_max << " " << avg.d_lon_max << "\n";
    f << "d_alt_max_m: " << avg.d_alt_max << "\n";
    f << "d_v_max_m_s: " << avg.d_v_max << "\n";
    f << "sig_pos_sample_m: " << avg.sig_pos_sample << " "
      << avg.sig_pos_lon_sample << "\n";
    f << "sig_alt_sample_m: " << avg.sig_alt_sample << "\n";
    f << "sig_v_sample_m_s: " << avg.sig_v_sample << "\n";

    f << std::setprecision(9);
    f << "heading_last_deg: " << avg.heading_last * RAD_TO_DEG << "\n";
    f << "heading_circ_mean_deg: " << avg.heading_circ_mean * RAD_TO_DEG << "\n";
    f << "d_heading_max_deg: " << avg.d_heading_max * RAD_TO_DEG << "\n";
    f << "sig_heading_deg: " << avg.sig_hdg_sample * RAD_TO_DEG << "\n";
    f << std::setprecision(6);
    f << "heading_source: " << (heading_from_sns ? "angle.dat" : "gyro-compassing")
      << "\n";
    f << "pitch_deg: " << avg.pitch * RAD_TO_DEG << "\n";
    f << "roll_deg: " << avg.roll * RAD_TO_DEG << "\n";
    f << "gyro_compass_deg: " << avg.gyro_compass_heading * RAD_TO_DEG << "\n";
    f << "gyro_compass_ratio: " << gc_ratio << "\n";

    // Смещения: ba в мг (1 мг = 1e-3·g0), bg в °/ч.
    const double mg = 1e-3 * G_EQ;
    f << "ba_mg: " << avg.ba_est[0] / mg << " " << avg.ba_est[1] / mg << " "
      << avg.ba_est[2] / mg << "\n";
    const double deg_h = 180.0 / PI * 3600.0;
    f << "bg_deg_h: " << avg.bg_est[0] * deg_h << " " << avg.bg_est[1] * deg_h
      << " " << avg.bg_est[2] * deg_h << "\n";
    f << "omega_h_meas_deg_s: " << avg.omega_h_meas * RAD_TO_DEG << "\n";

    f << "rest_ok: " << (avg.rest_ok ? 1 : 0) << "\n";
    f << "rest_reason: " << (avg.rest_reason.empty() ? "-" : avg.rest_reason) << "\n";
    f << "rest_speed_m_s: " << avg.rest_speed << "\n";
    f << "rest_speed_limit_m_s: " << avg.rest_speed_limit << "\n";
    f << "rest_omega_rad_s: " << avg.rest_omega << "\n";
    f << "rest_omega_limit_rad_s: " << avg.rest_omega_limit << "\n";
    f << "rest_fy_m_s2: " << avg.rest_fy << "\n";

    if (avg.has_sns)
    {
        f << std::setprecision(9);
        f << "sns_last_deg: " << avg.sns_last.lat * RAD_TO_DEG << " "
          << avg.sns_last.lon * RAD_TO_DEG << " " << avg.sns_last.alt << "\n";
        f << "sns_last_v: " << avg.sns_last.vn << " " << avg.sns_last.vh << " "
          << avg.sns_last.ve << "\n";
    }

    // Начальное состояние и x0.
    f << std::setprecision(9);
    f << "nominal_deg: " << st.lat * RAD_TO_DEG << " " << st.lon * RAD_TO_DEG
      << " " << st.alt << "\n";
    f << "nominal_v: " << st.V[0] << " " << st.V[1] << " " << st.V[2] << "\n";
    f << "nominal_att_deg: " << st.att.heading * RAD_TO_DEG << " "
      << st.att.pitch * RAD_TO_DEG << " " << st.att.roll * RAD_TO_DEG << "\n";
    f << "nominal_ba_mg: " << st.ba[0] / mg << " " << st.ba[1] / mg << " "
      << st.ba[2] / mg << "\n";
    f << "x0:";
    for (int i = 0; i < ins::KF_STATE; i++)
        f << " " << st.x[i];
    f << "\n";
    f << "sigma0:";
    for (int i = 0; i < ins::KF_STATE; i++)
        f << " " << sqrt(fmax(0.0, at(st.P, i, i, ins::KF_STATE)));
    f << "\n";
    f << "sigma_heading0_deg: " << sig_heading0 * RAD_TO_DEG << "\n";
}

} // namespace

int main(int argc, char **argv)
{
    // Разбор аргументов: флаги идут перед позиционными, чтобы не сломать
    // существующий вызов `imitator <каталог> [params.ini]`.
    bool write_bias_log = false;
    bool bias_report = false;
    bool write_consistency = false;
    std::vector<std::string> positional;

    for (int i = 1; i < argc; i++)
    {
        if (std::strcmp(argv[i], "--bias-log") == 0) write_bias_log = true;
        else if (std::strcmp(argv[i], "--bias-report") == 0) bias_report = true;
        else if (std::strcmp(argv[i], "--consistency") == 0) write_consistency = true;
        else positional.push_back(argv[i]);
    }
    // Отчёт по смещениям без журнала бессмыслен — журнал включается принудительно.
    if (bias_report) write_bias_log = true;

    // argv[1] — каталог данных (по умолчанию ../data/raw), argv[2] — params.ini.
    const std::string data_dir = positional.empty() ? "../data/raw" : positional[0];
    if (positional.size() > 1)
    {
        applyParamsFile(positional[1]);
    }

    const std::filesystem::path tools_dir = utils::toolsDir(argv[0]);
    const std::string result_file = (tools_dir / "result.txt").string();
    const std::string reference_file = (tools_dir / "reference.txt").string();
    const std::string err_file = (tools_dir / "errors.txt").string();
    const std::string init_file = (tools_dir / "init.txt").string();

    const std::string imu_file = pickFile(data_dir, "imu.dat", "imu.data");
    const std::string gps_file = pickFile(data_dir, "gps.dat", "gps.data");
    const std::string angle_file = pickFile(data_dir, "angle.dat", "angle.data");
    const std::string startup_file = data_dir + "/StartupNav.ini";

    const bool has_angle = std::filesystem::exists(angle_file);
    std::cout << "angle.dat/angle.data: "
              << (has_angle ? "found" : "not found (angles will be zero)") << std::endl;

    const auto start_time = std::chrono::high_resolution_clock::now();

    // ------------------------------------------------------------------
    // 2. Точка и время выставки
    // ------------------------------------------------------------------
    double start_lon = 0.0, start_lat = 0.0, start_alt = 0.0;
    double align_time = ALIGN_FALLBACK_S;

    if (readStartupNav(startup_file, start_lon, start_lat, start_alt, align_time))
    {
        std::cout << "StartupNav: lon=" << start_lon << " lat=" << start_lat
                  << " alt=" << start_alt << " time=" << align_time << std::endl;
        if (!(align_time > 0.0))
        {
            std::cout << "WARNING: StartupNav time is not positive, using "
                      << ALIGN_FALLBACK_S << " s" << std::endl;
            align_time = ALIGN_FALLBACK_S;
        }
    }
    else
    {
        std::cout << "StartupNav not found, reading from gps.dat..." << std::endl;

        data_io::SnsReader sns_tmp;
        if (!sns_tmp.open(gps_file, has_angle ? angle_file : ""))
        {
            std::cerr << "no reference data" << std::endl;
            return 1;
        }
        nav::SnsSample first;
        if (!sns_tmp.next(first))
        {
            std::cerr << "no reference data" << std::endl;
            return 1;
        }
        sns_tmp.close();

        start_lon = first.lon * RAD_TO_DEG;
        start_lat = first.lat * RAD_TO_DEG;
        start_alt = first.alt;
        align_time = ALIGN_FALLBACK_S;
    }

    // ------------------------------------------------------------------
    // 3. Окно усреднения (быстрая выставка)
    // ------------------------------------------------------------------
    std::cout << "=== Alignment: averaging window " << align_time << " s ==="
              << std::endl;

    data_io::ImuReader imu;
    data_io::SnsReader sns;
    if (!imu.open(imu_file))
    {
        std::cerr << "cannot open " << imu_file << std::endl;
        return 1;
    }
    if (!sns.open(gps_file, has_angle ? angle_file : ""))
    {
        std::cerr << "cannot open " << gps_file << std::endl;
        return 1;
    }

    nav::AveragingWindow window(align_time, has_angle);

    // Потоки не перематываются: отсчёт ИМУ за окном («лишний») возвращается в
    // основной цикл как первый такт, кадр СНС — как уже прочитанный следующий.
    std::vector<double> row;
    std::vector<double> pending_row;
    bool has_pending_row = false;
    std::size_t invalid_imu_rows = 0;

    while (imu.next(row))
    {
        if (!ins::isValidRow(row))
        {
            // Битые строки imu.dat: раньше отбрасывались молча. Считаем и
            // логируем, чтобы предполётная проверка набора показывала, что
            // сравнения с истиной могут быть неполными.
            invalid_imu_rows++;
            continue;
        }
        if (row[0] > align_time)
        {
            pending_row = row;
            has_pending_row = true;
            break;
        }
        window.addImu(row[0], row[2], row[3], row[4], row[5], row[6], row[7]);
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
    {
        std::cout << "WARNING: averaging window buffer overflow (align time too large)"
                  << std::endl;
    }
    if (avg.imu_count == 0 || avg.sns_count == 0)
    {
        std::cout << "WARNING: averaging window is empty (imu=" << avg.imu_count
                  << ", sns=" << avg.sns_count << "): fallback to StartupNav prior"
                  << std::endl;
    }

    std::cout << "averaging: imu=" << avg.imu_count << " sns=" << avg.sns_count
              << " t=[" << avg.t_start << ", " << avg.t_end << "]" << std::endl;
    std::cout << "[Degree] Pitch: " << avg.pitch * RAD_TO_DEG
              << ", Roll: " << avg.roll * RAD_TO_DEG << std::endl;
    std::cout << "ba [mg]: " << avg.ba_est[0] / (1e-3 * G_EQ) << ", "
              << avg.ba_est[1] / (1e-3 * G_EQ) << ", "
              << avg.ba_est[2] / (1e-3 * G_EQ) << std::endl;
    std::cout << "bg [deg/h]: " << avg.bg_est[0] * RAD_TO_DEG * 3600.0 << ", "
              << avg.bg_est[1] * RAD_TO_DEG * 3600.0 << ", "
              << avg.bg_est[2] * RAD_TO_DEG * 3600.0 << std::endl;

    // Гейт покоя: доверие к средним окна и к оценке bg.
    if (avg.rest_ok)
    {
        std::cout << "rest gate: passed (|v|=" << avg.rest_speed
                  << " <= " << avg.rest_speed_limit
                  << " m/s, |w-w_earth|=" << avg.rest_omega
                  << " <= " << avg.rest_omega_limit
                  << " rad/s, |f_y-g|=" << avg.rest_fy << " m/s2)" << std::endl;
    }
    else
    {
        std::cout << "WARNING: rest gate failed (" << avg.rest_reason
                  << "): passport P0, bg estimate not applied" << std::endl;
    }

    // Курс: эталон (angle.dat) или гирокомпасирование.
    const bool heading_from_sns = avg.has_heading;
    double sig_heading0 = -1.0;  // <= 0 — формула выставки σ_bg/Ω_h
    if (heading_from_sns)
    {
        sig_heading0 = fmax(ins::kalman_cfg.sig_hdg, avg.sig_hdg_sample);
    }

    // Диагностика (R4): годность курса по вращению Земли. Ω_h — ожидаемая
    // проекция вращения Земли на горизонт; ||ω_h| − Ω_h| ≤ |bg_гор|, поэтому
    // отношение — нижняя оценка |bg_гор|/Ω_h. На курс больше не влияет.
    const double omega_h_expected = fabs(cos(start_lat * DEG_TO_RAD)) * U_EARTH;
    const double gc_ratio = (avg.omega_h_meas >= 0.0 && omega_h_expected > 0.0)
                                ? fabs(avg.omega_h_meas - omega_h_expected) / omega_h_expected
                                : -1.0;
    const bool gyro_compass_ok = gc_ratio >= 0.0 && gc_ratio <= GYRO_COMPASS_LIMIT;

    // ------------------------------------------------------------------
    // 4. Начальное состояние, ошибки x0 и ковариация P0
    // ------------------------------------------------------------------
    nav::NavState state;

    // Номинал координат и скоростей — последний отсчёт СНС окна (без СНС —
    // StartupNav и нулевые скорости).
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

    // Номинал ориентации: уровень — по средней удельной силе, курс — последний
    // отсчёт эталона (иначе гирокомпас). Номинал ba — оценка окна, bg — нуль
    // (оценка уходит в x0[12..14] и применяется первой коррекцией).
    state.att.heading = heading_from_sns ? avg.heading_last : avg.gyro_compass_heading;
    state.att.pitch = avg.pitch;
    state.att.roll = avg.roll;
    state.ba = avg.ba_est;
    state.bg = {0.0, 0.0, 0.0};

    state.P = nav::initialCovarianceFromAveraging(ins::kalman_cfg, avg, state.lat,
                                                  sig_heading0);
    state.x = nav::initialErrorsFromAveraging(avg, avg.rest_ok);

    // Первый такт цикла: dt = t_имп − T_align (номинал задан на момент T_align).
    state.time_prev = align_time;

    std::cout << "[Rad] Yaw: " << state.att.heading
              << ", Pitch: " << state.att.pitch
              << ", Roll: " << state.att.roll << std::endl;
    std::cout << "heading source: "
              << (heading_from_sns ? "angle.dat (last SNS sample in window)"
                                   : "gyro-compassing")
              << std::endl;

    const double sig_heading_report =
        (sig_heading0 > 0.0) ? sig_heading0 : sqrt(fmax(at(state.P, 6, 6, ins::KF_STATE), 0.0));
    std::cout << "sigma(heading0) = " << sig_heading_report << " rad" << std::endl;
    if (gc_ratio >= 0.0)
    {
        std::cout << "gyro-compassing check: ||w_h|-Omega_h|/Omega_h = " << gc_ratio
                  << " (lower bound of |bg_hor|/Omega_h, limit " << GYRO_COMPASS_LIMIT
                  << ")" << std::endl;
    }
    if (heading_from_sns && !gyro_compass_ok)
    {
        std::cout << "note: gyro-compassing would be unreliable, heading taken from angle.dat"
                  << std::endl;
    }
    if (!has_angle)
    {
        std::cout << "WARNING: angle.dat not found: initial heading from gyro-compassing"
                  << ", P0(heading) = sigma_bg/Omega_h"
                  << (gyro_compass_ok ? " (earth-rate check passed, but heading is unverified)"
                                      : " (earth-rate check failed: heading is unreliable)")
                  << std::endl;
    }
    std::cout << "x0: coords " << state.x[0] << ", " << state.x[1] << ", "
              << state.x[2] << " m; vel " << state.x[3] << ", " << state.x[4]
              << ", " << state.x[5] << " m/s; heading "
              << state.x[6] * RAD_TO_DEG << " deg; bg "
              << state.x[12] * RAD_TO_DEG * 3600.0 << ", "
              << state.x[13] * RAD_TO_DEG * 3600.0 << ", "
              << state.x[14] * RAD_TO_DEG * 3600.0 << " deg/h" << std::endl;
    std::cout << "Output: " << tools_dir.string() << std::endl;

    writeInitLog(init_file, data_dir, avg, state, sig_heading_report,
                 heading_from_sns, gc_ratio);

    // ------------------------------------------------------------------
    // 5. Основной цикл
    // ------------------------------------------------------------------
    data_io::NavLogger log;
    if (!log.open(result_file, reference_file, err_file))
    {
        std::cerr << "cannot open output files in " << tools_dir.string() << std::endl;
        return 1;
    }
    // Согласованность (NIS) — только по флагу: иначе ядро не считает
    // NIS/NEES вовсе и файл не создаётся.
    log.setConsistencyEnabled(write_consistency);
    if (write_consistency)
    {
        std::cout << "consistency log: " << (tools_dir / "consistency.txt").string()
                  << std::endl;
    }
    log.writeHeader();

    // Журнал смещений: пишется только по флагу, поэтому result/reference/
    // errors от него не зависят.
    nav::BiasLogWriter bias_log;
    const std::string bias_file = (tools_dir / "bias.txt").string();
    if (write_bias_log)
    {
        if (!bias_log.open(bias_file))
        {
            std::cerr << "cannot open " << bias_file << std::endl;
            return 1;
        }
        std::cout << "bias log: " << bias_file << std::endl;
    }

    // Время последней записи журнала — для декадирования и для дедупликации
    // финального кадра (ровная сетка окон 1/10/30/… с в отчёте).
    double bias_last_written = -1.0;
    std::size_t bias_rows = 0;
    std::size_t steps = 0;
    std::size_t corrections = 0;
    double time_last = 0.0;

    nav::SnsSample ref;
    // Текущий кадр СНС — последний отсчёт окна: первая коррекция произойдёт
    // на первом новом кадре (он независим от номинала), а не на этом.
    nav::SnsSample last_ref = next_frame;
    nav::SnsSample hold_ref = avg.sns_last;
    bool has_ref = has_next_frame;
    bool hold_valid = avg.has_sns;

    while (true)
    {
        if (has_pending_row)
        {
            row = pending_row;
            has_pending_row = false;
        }
        else if (!imu.next(row))
        {
            break;
        }

        if (!ins::isValidRow(row))
        {
            invalid_imu_rows++;
            continue;
        }

        const double imu_time = row[0];
        const double prev_gps_time = hold_valid ? hold_ref.time : -1.0;

        while (!has_ref || last_ref.time <= imu_time)
        {
            if (has_ref && last_ref.time <= imu_time)
            {
                hold_ref = last_ref;
                hold_valid = true;
            }
            if (!sns.next(last_ref))
            {
                break;
            }
            has_ref = true;
            if (last_ref.time > imu_time)
            {
                break;
            }
        }

        if (!hold_valid)
        {
            continue;
        }

        const bool in_outage =
            (ins::kalman_cfg.outage_end_s > ins::kalman_cfg.outage_start_s) &&
            (imu_time >= ins::kalman_cfg.outage_start_s) &&
            (imu_time <= ins::kalman_cfg.outage_end_s);

        // Во время «потери СНС» коррекции не применяются (проверка свободного счисления).
        const bool do_correction = !in_outage && (hold_ref.time != prev_gps_time);

        ref = hold_ref;

        nav::step(row, ref, state, log, do_correction, has_angle);

        steps++;
        if (do_correction) corrections++;
        time_last = imu_time;

        // Журнал смещений. Оценки ba/bg меняются на каждом такте (tilt-апдейт
        // 200 Гц), поэтому «записать при изменении» дал бы ~26 МБ на прогон.
        // Отчёту нужны окна 1/10/30/… с, а динамика смещений на порядки
        // медленнее такта: декадируем до 5 Гц. Финальный кадр дописывается
        // после цикла, чтобы отчёт всегда видел конечное состояние.
        if (bias_log.isOpen() &&
            (steps == 1 || (imu_time - bias_last_written) >= 0.2))
        {
            nav::BiasSample sample;
            sample.time = imu_time;
            for (int k = 0; k < 3; k++)
            {
                sample.ba[k] = state.ba[k];
                sample.bg[k] = state.bg[k];
                sample.sig_ba[k] = sqrt(fmax(0.0, at(state.P, 9 + k, 9 + k, ins::KF_STATE)));
                sample.sig_bg[k] = sqrt(fmax(0.0, at(state.P, 12 + k, 12 + k, ins::KF_STATE)));
            }
            bias_log.write(sample);
            bias_last_written = imu_time;
            bias_rows++;
        }
    }

    // Финальный кадр журнала: декадирование могло пропустить последнее
    // состояние, а отчёт всегда должен видеть конечные оценки и сигмы.
    if (bias_log.isOpen() && steps > 0)
    {
        nav::BiasSample sample;
        sample.time = time_last;
        for (int k = 0; k < 3; k++)
        {
            sample.ba[k] = state.ba[k];
            sample.bg[k] = state.bg[k];
            sample.sig_ba[k] = sqrt(fmax(0.0, at(state.P, 9 + k, 9 + k, ins::KF_STATE)));
            sample.sig_bg[k] = sqrt(fmax(0.0, at(state.P, 12 + k, 12 + k, ins::KF_STATE)));
        }
        // Дедупликация: если последний такт попал в декадирование, кадр уже есть.
        if (sample.time != bias_last_written)
        {
            bias_log.write(sample);
            bias_rows++;
        }
    }

    imu.close();
    sns.close();
    log.close();
    bias_log.close();

    if (bias_rows > 0)
    {
        std::cout << "bias log: " << bias_rows << " rows" << std::endl;
    }
    if (invalid_imu_rows > 0)
    {
        std::cout << "WARNING: skipped " << invalid_imu_rows
                  << " malformed imu.dat rows" << std::endl;
    }

    // ------------------------------------------------------------------
    // 6. Отчёт по смещениям (--bias-report)
    // ------------------------------------------------------------------
    if (bias_report)
    {
        std::vector<nav::BiasSample> bias_samples;
        std::string read_error;
        if (!nav::readBiasLog(bias_file, bias_samples, &read_error))
        {
            std::cerr << "imitator: bias-report failed: " << read_error << std::endl;
            return 1;
        }

        // Истина — из params.json набора. Если его нет (набор сгенерирован без
        // него или каталог чужой), отчёт всё равно строим, но помечаем истины.
        nav::BiasTruth truth;
        std::string truth_error;
        const std::string params_path = data_dir + "/params.json";
        nav::loadBiasTruth(params_path, truth, &truth_error);

        std::string report = nav::formatBiasReport(nav::buildBiasReport(bias_samples, truth),
                                                   truth);
        report += "\nlog: " + bias_file + " (" + std::to_string(bias_samples.size()) + " rows)\n";
        if (!truth.valid)
        {
            report += "truth: " + truth_error + "\n";
        }
        std::cout << report;
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << elapsed.count() << " s" << std::endl;

    return 0;
}
