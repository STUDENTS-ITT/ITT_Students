// simulation.cpp — Реализация прогона БИНС/СНС-комплексирования.
//
// Логика цикла перенесена из прежнего main.cpp без изменений порядка операций,
// поэтому консольный прогон даёт побитово те же файлы result/reference/errors.

#include "simulation.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include "../utils/constants.h"
#include "aligner.h"
#include "filter_settings.h"
#include "gps_processor.h"

namespace nav
{
namespace
{

// Приёмник, транслирующий поток результатов в файлы и/или подписчику.
class ForwardingSink : public data_io::NavSink
{
public:
    ForwardingSink(data_io::NavSink *file_sink, SimulationObserver *observer)
        : file_(file_sink), observer_(observer) {}

    void writeHeader() override
    {
        if (file_ != nullptr) file_->writeHeader();
    }

    void writeResult(const data_io::NavResult &r) override
    {
        last_result_ = r;
        has_result_ = true;
        if (file_ != nullptr) file_->writeResult(r);
        if (observer_ != nullptr) observer_->onResult(r);
    }

    void writeReference(const data_io::NavReference &r) override
    {
        if (file_ != nullptr) file_->writeReference(r);
        if (observer_ != nullptr && has_errors_)
        {
            observer_->onCorrection(r, last_errors_);
        }
    }

    void writeErrors(double time, const Vector &x) override
    {
        last_errors_ = x;
        has_errors_ = true;
        if (file_ != nullptr) file_->writeErrors(time, x);
    }

    void close() override
    {
        if (file_ != nullptr) file_->close();
    }

private:
    data_io::NavSink *file_;
    SimulationObserver *observer_;

    data_io::NavResult last_result_{};
    Vector last_errors_;
    bool has_result_ = false;
    bool has_errors_ = false;
};

} // namespace

bool readStartupNav(const std::string &path, double &lon_deg, double &lat_deg,
                    double &alt, double &time_s)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;

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

std::size_t countImuRows(const std::string &imu_path)
{
    std::ifstream f(imu_path);
    if (!f.is_open()) return 0;

    std::size_t rows = 0;
    std::string line;
    while (std::getline(f, line))
    {
        if (!line.empty()) rows++;
    }
    return rows > 0 ? rows - 1 : 0;  // первая строка — заголовок
}

bool runSimulation(const RunOptions &options, RunSummary &summary,
                   SimulationObserver *observer, CancellationToken *cancel,
                   std::ostream *log)
{
    summary = RunSummary{};

    const std::string data_dir = options.data_dir;
    const std::string imu_file = data_dir + "/imu.dat";
    const std::string gps_file = data_dir + "/gps.dat";
    const std::string angle_file = data_dir + "/angle.dat";
    const std::string startup_file = data_dir + "/StartupNav.ini";

    const auto wall_start = std::chrono::high_resolution_clock::now();

    // Настройки фильтра из settings.ini (если задан).
    FilterSettings settings;
    bool legacy_params = false;
    if (!options.settings_file.empty())
    {
        if (std::filesystem::exists(options.settings_file))
        {
            if (!loadFilterSettings(options.settings_file, settings))
            {
                summary.error = "settings.ini: cannot parse " + options.settings_file;
                if (log) *log << summary.error << std::endl;
                return false;
            }
            // Старый плоский params.ini без секций: его ключи читаются отдельно,
            // иначе applySettings() затрёт их значениями по умолчанию.
            legacy_params = !settingsFileHasSections(options.settings_file);
            if (legacy_params && log)
            {
                *log << "settings.ini: legacy format without sections, plain keys only"
                     << std::endl;
            }
        }
        else if (log)
        {
            // Отсутствующий файл — не ошибка: работаем на значениях по умолчанию.
            *log << "settings.ini not found (" << options.settings_file
                 << "), using default values" << std::endl;
        }
    }
    applySettings(settings);
    if (legacy_params) applyLegacyParamsFile(options.settings_file, log);

    const bool has_angle = std::filesystem::exists(angle_file);
    summary.has_angle = has_angle;
    if (log)
    {
        *log << "angle.dat: " << (has_angle ? "found" : "not found (angles will be zero)")
             << std::endl;
    }

    const std::filesystem::path tools_dir = options.output_dir;

    double start_lon = 0.0, start_lat = 0.0, start_alt = 0.0;
    double align_time = settings.align_fallback_s;

    if (readStartupNav(startup_file, start_lon, start_lat, start_alt, align_time))
    {
        if (log)
        {
            *log << "StartupNav: lon=" << start_lon << " lat=" << start_lat
                 << " alt=" << start_alt << " time=" << align_time << std::endl;
        }
    }
    else
    {
        if (log) *log << "StartupNav not found, reading from gps.dat..." << std::endl;

        data_io::SnsReader sns_tmp;
        if (!sns_tmp.open(gps_file, has_angle ? angle_file : ""))
        {
            summary.error = "no reference data";
            if (log) *log << summary.error << std::endl;
            return false;
        }
        SnsSample first;
        sns_tmp.next(first);
        sns_tmp.close();

        start_lon = first.lon * RAD_TO_DEG;
        start_lat = first.lat * RAD_TO_DEG;
        start_alt = first.alt;
        align_time = settings.align_fallback_s;
    }

    // Явное переопределение длительности выставки из настроек (0 — не задано).
    if (settings.align_time_override_s > 0.0)
    {
        align_time = settings.align_time_override_s;
    }

    summary.start_lon_deg = start_lon;
    summary.start_lat_deg = start_lat;
    summary.start_alt = start_alt;
    summary.align_time_s = align_time;

    if (cancel != nullptr && cancel->cancelled())
    {
        summary.error = "cancelled";
        return false;
    }

    double Yaw_0 = 0.0, Pitch_0 = 0.0, Roll_0 = 0.0;
    double ba_x = 0.0, ba_y = 0.0, ba_z = 0.0;

    if (log) *log << "=== Alignment ===" << std::endl;
    get_angle_start(&Yaw_0, &Pitch_0, &Roll_0, &ba_x, &ba_y, &ba_z,
                    imu_file.c_str(), start_lat, start_alt, align_time);

    if (log)
    {
        *log << "[Degree] Yaw: " << Yaw_0 * 180.0 / PI
             << ", Pitch: " << Pitch_0 * 180.0 / PI
             << ", Roll: " << Roll_0 * 180.0 / PI << std::endl;
        *log << "ba: " << ba_x << ", " << ba_y << ", " << ba_z << std::endl;
    }

    const Vector ba0 = {ba_x, ba_y, ba_z};

    NavState state = initialAlignment(start_lat * DEG_TO_RAD, start_lon * DEG_TO_RAD,
                                      start_alt, Yaw_0, Pitch_0, Roll_0, ba0);

    summary.yaw0_deg = Yaw_0 * 180.0 / PI;
    summary.pitch0_deg = Pitch_0 * 180.0 / PI;
    summary.roll0_deg = Roll_0 * 180.0 / PI;
    summary.ba[0] = ba_x;
    summary.ba[1] = ba_y;
    summary.ba[2] = ba_z;

    if (log)
    {
        *log << "[Rad] Yaw: " << state.att.heading << ", Pitch: " << state.att.pitch
             << ", Roll: " << state.att.roll << std::endl;
        *log << "Output: " << tools_dir.string() << std::endl;
    }

    if (observer != nullptr) observer->onAlignment(summary);

    data_io::ImuReader imu;
    if (!imu.open(imu_file))
    {
        summary.error = "cannot open " + imu_file;
        return false;
    }
    data_io::SnsReader sns;
    if (!sns.open(gps_file, has_angle ? angle_file : ""))
    {
        summary.error = "cannot open " + gps_file;
        imu.close();
        return false;
    }

    data_io::NavLogger file_logger;
    data_io::NavSink *file_sink = nullptr;
    if (options.write_files)
    {
        const std::string result_file = (tools_dir / "result.txt").string();
        const std::string reference_file = (tools_dir / "reference.txt").string();
        const std::string err_file = (tools_dir / "errors.txt").string();

        if (!file_logger.open(result_file, reference_file, err_file))
        {
            summary.error = "cannot open output files in " + tools_dir.string();
            imu.close();
            sns.close();
            return false;
        }
        file_sink = &file_logger;
    }

    ForwardingSink sink(file_sink, observer);
    sink.writeHeader();

    const bool want_telemetry = observer != nullptr && options.capture_telemetry;
    RunPacer *pacer = options.pacer;
    StepTelemetry tel;

    std::vector<double> row;
    SnsSample ref;
    SnsSample last_ref;
    SnsSample hold_ref;
    bool has_ref = false;
    bool hold_valid = false;

    const double total_rows =
        (observer != nullptr) ? static_cast<double>(countImuRows(imu_file)) : 0.0;
    double done_rows = 0.0;
    double last_report_time = -1.0;

    while (imu.next(row))
    {
        done_rows += 1.0;

        if (!ins::isValidRow(row))
        {
            continue;
        }

        const double imu_time = row[0];
        const double prev_gps_time = hold_valid ? hold_ref.time : -1.0;

        // Темп прогона: пауза и ограничение скорости (GUI). Здесь же
        // проверяется отмена, чтобы остановка была мгновенной и на паузе.
        if (pacer != nullptr)
        {
            pacer->beforeStep(imu_time, options.speed_factor);
            if (cancel != nullptr && cancel->cancelled())
            {
                imu.close();
                sns.close();
                sink.close();
                summary.error = "cancelled";
                return false;
            }
        }

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

        StepTelemetry *tp = want_telemetry ? &tel : nullptr;
        if (tp != nullptr) tp->step_index = static_cast<int>(summary.steps);

        step(row, ref, state, sink, do_correction, has_angle, tp);

        summary.steps++;
        if (do_correction) summary.corrections++;
        if (summary.time_first == 0.0) summary.time_first = imu_time;
        summary.time_last = imu_time;

        if (want_telemetry)
        {
            observer->onState(state, *tp);
        }

        // Прогресс — не чаще 4 Гц, чтобы не забивать сигналами GUI.
        if (observer != nullptr && (imu_time - last_report_time) >= 0.25)
        {
            last_report_time = imu_time;
            observer->onProgress(imu_time, done_rows, total_rows);
        }

        if (cancel != nullptr && cancel->cancelled())
        {
            imu.close();
            sns.close();
            sink.close();
            summary.error = "cancelled";
            return false;
        }
    }

    imu.close();
    sns.close();
    sink.close();

    if (observer != nullptr) observer->onProgress(summary.time_last, done_rows, total_rows);

    const auto wall_end = std::chrono::high_resolution_clock::now();
    summary.wall_seconds = std::chrono::duration<double>(wall_end - wall_start).count();

    summary.ok = true;
    return true;
}

} // namespace nav