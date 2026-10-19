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
#include "bias_log.h"
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

    void writeGuard(const data_io::SnsGuardRecord &r) override
    {
        if (file_ != nullptr) file_->writeGuard(r);
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
    // Бинарный режим — как в ImuReader, чтобы 0x1A в повреждённых наборах
    // не обрезал подсчёт числа строк.
    std::ifstream f(imu_path, std::ios::in | std::ios::binary);
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

    // Источник входных данных: файлы в options.data_dir либо заданный
    // вызывающим (например, MockDataSource — тесты, позже поток с порта).
    data_io::FileDataSource owned_source(options.data_dir);
    data_io::IDataSource *src = (options.source != nullptr) ? options.source : &owned_source;
    const bool source_is_file = (options.source == nullptr);

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

    const bool has_angle = src->hasAngle();
    summary.has_angle = has_angle;
    if (log)
    {
        *log << "angle.dat/angle.data: "
             << (has_angle ? "found" : "not found (angles will be zero)") << std::endl;
    }

    const std::filesystem::path tools_dir = options.output_dir;

    // Открытие источника (с начала): нужно для чтения StartupNav и первого отсчёта СНС.
    if (!src->open())
    {
        summary.error = src->error().empty() ? "cannot open input data" : src->error();
        if (log) *log << summary.error << std::endl;
        return false;
    }

    double start_lon = 0.0, start_lat = 0.0, start_alt = 0.0;
    double align_time = settings.align_fallback_s;

    if (src->startupNav(start_lon, start_lat, start_alt, align_time))
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

        SnsSample first;
        if (!src->nextSns(first))
        {
            summary.error = "no reference data";
            if (log) *log << summary.error << std::endl;
            return false;
        }

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

    // Перемотка к началу потока ИМУ и выставка по тому же интерфейсу, что и
    // основной цикл (файл или память — без различий в математике).
    if (!src->open())
    {
        summary.error = src->error().empty() ? "cannot open input data" : src->error();
        if (log) *log << summary.error << std::endl;
        return false;
    }

    // Этап 4 плана (R5): эталон курса и оценка его шума по потоку СНС за окно
    // выставки. Нужны только при наличии angle.dat: поток СНС независим от
    // потока ИМУ, а перед основным циклом источник перематывается ещё раз,
    // поэтому чтение здесь ничего не ломает.
    //
    // σ̂ считается по отклонениям от первого отсчёта (нормировка в (−π, π]):
    // покой на окне выставки гарантирует, что в разброс попадает только шум
    // эталона, а не повороты. Он и есть честный P0(курса) R5.
    double sns_heading0 = 0.0;      // курс первого отсчёта СНС, рад
    double sns_heading_sigma = 0.0; // σ̂ шума эталона курса по окну, рад
    int sns_heading_count = 0;
    if (has_angle)
    {
        SnsSample sample;
        double dev_sum = 0.0, dev_sum2 = 0.0;
        while (src->nextSns(sample) && sample.time <= align_time)
        {
            if (sns_heading_count == 0)
            {
                sns_heading0 = sample.heading;
            }
            const double dev = normalize_angle(sample.heading - sns_heading0);
            dev_sum += dev;
            dev_sum2 += dev * dev;
            sns_heading_count++;
        }
        if (sns_heading_count > 1)
        {
            const double n = static_cast<double>(sns_heading_count);
            const double var = fmax((dev_sum2 - dev_sum * dev_sum / n) / (n - 1.0), 0.0);
            sns_heading_sigma = sqrt(var);
        }
    }

    AlignImuProvider align_provider =
        [src](double &Time, double &Wx, double &Wy, double &Wz,
              double &Ax, double &Ay, double &Az) -> bool
    {
        std::vector<double> row;
        while (src->nextImu(row))
        {
            if (!ins::isValidRow(row))
            {
                continue;
            }
            Time = row[0];
            Wx = row[2];
            Wy = row[3];
            Wz = row[4];
            Ax = row[5];
            Ay = row[6];
            Az = row[7];
            return true;
        }
        return false;
    };
    double omega_h_meas = -1.0;  // измеренный |ω_h| за окно выставки, рад/с
    alignFromImuProvider(align_provider, &Yaw_0, &Pitch_0, &Roll_0,
                         &ba_x, &ba_y, &ba_z, start_lat, start_alt, align_time,
                         source_is_file, &omega_h_meas);

    if (log)
    {
        *log << "[Degree] Yaw: " << Yaw_0 * 180.0 / PI
             << ", Pitch: " << Pitch_0 * 180.0 / PI
             << ", Roll: " << Roll_0 * 180.0 / PI << std::endl;
        *log << "ba: " << ba_x << ", " << ba_y << ", " << ba_z << std::endl;
    }

    // Этап 4 плана (R4): годность курсовой выставки. Ω_h — ожидаемая
    // проекция вращения Земли на горизонт; обратное неравенство треугольника
    // даёт ||ω_h| − Ω_h| ≤ |bg_гор|, поэтому отношение
    // ||ω_h| − Ω_h|/Ω_h ≤ |bg_гор|/Ω_h и превышение порога — достаточное
    // основание признать курс по вращению Земли недостоверным. Само
    // отношение занижено: перпендикулярная Земле составляющая смещения в
    // модуль |ω_h| не входит, а она и даёт основную ошибку курса.
    constexpr double GYRO_COMPASS_LIMIT = 0.3;
    const double omega_h_expected = fabs(cos(start_lat * DEG_TO_RAD)) * U_EARTH;
    const double gc_ratio = (omega_h_meas >= 0.0 && omega_h_expected > 0.0)
                                ? fabs(omega_h_meas - omega_h_expected) / omega_h_expected
                                : -1.0;
    const bool gyro_compass_ok = gc_ratio >= 0.0 && gc_ratio <= GYRO_COMPASS_LIMIT;

    // Этап 4 плана (R5): при наличии angle.dat начальный курс берём из первого
    // отсчёта СНС (эталон), а не из gyro-compassing, и P0(курс) = σ_hdg².
    // σ_hdg — шум эталона курса: берём максимум конфигурации и оценки по
    // данным, потому что на наборах шум angle.dat (0.05…1.0°) больше
    // настроенного значения, и P0 пришлось бы занижать.
    const bool heading_from_sns = has_angle && sns_heading_count > 0;
    double sig_heading0 = -1.0;  // <= 0 — формула выставки σ_bg/Ω_h
    if (heading_from_sns)
    {
        Yaw_0 = sns_heading0;
        sig_heading0 = fmax(ins::kalman_cfg.sig_hdg, sns_heading_sigma);
    }

    const Vector ba0 = {ba_x, ba_y, ba_z};

    NavState state = initialAlignment(start_lat * DEG_TO_RAD, start_lon * DEG_TO_RAD,
                                      start_alt, Yaw_0, Pitch_0, Roll_0, ba0,
                                      sig_heading0);

    summary.yaw0_deg = Yaw_0 * 180.0 / PI;
    summary.pitch0_deg = Pitch_0 * 180.0 / PI;
    summary.roll0_deg = Roll_0 * 180.0 / PI;
    summary.ba[0] = ba_x;
    summary.ba[1] = ba_y;
    summary.ba[2] = ba_z;
    summary.heading_from_sns = heading_from_sns;
    summary.gyro_compass_ratio = gc_ratio;
    summary.sig_heading0 = (sig_heading0 > 0.0)
                               ? sig_heading0
                               : sqrt(fmax(at(state.P, 6, 6, ins::KF_STATE), 0.0));
    // Достоверность взятого курса: из эталона СНС — всегда; из
    // gyro-compassing — только если проверка по вращению Земли пройдена
    // (без данных о ней считаем курс недостоверным, а не наоборот).
    summary.heading_reliable = heading_from_sns ? true : gyro_compass_ok;

    if (log)
    {
        *log << "[Rad] Yaw: " << state.att.heading << ", Pitch: " << state.att.pitch
             << ", Roll: " << state.att.roll << std::endl;

        // Этап 4 (R4, R5): источник начального курса и его достоверность.
        *log << "heading source: "
             << (heading_from_sns ? "angle.dat (first SNS sample)" : "gyro-compassing")
             << std::endl;
        *log << "sigma(heading0) = " << summary.sig_heading0 << " rad" << std::endl;
        if (gc_ratio >= 0.0)
        {
            *log << "gyro-compassing check: ||w_h|-Omega_h|/Omega_h = " << gc_ratio
                 << " (lower bound of |bg_hor|/Omega_h, limit " << GYRO_COMPASS_LIMIT
                 << ")" << std::endl;
        }
        if (heading_from_sns && !gyro_compass_ok)
        {
            *log << "note: gyro-compassing would be unreliable, heading taken from angle.dat"
                 << std::endl;
        }
        if (!has_angle)
        {
            *log << "WARNING: angle.dat not found: initial heading from gyro-compassing"
                 << (gyro_compass_ok ? ", earth-rate check passed but heading is unverified"
                                     : ", earth-rate check failed: heading is unreliable")
                 << std::endl;
        }
        *log << "Output: " << tools_dir.string() << std::endl;
    }

    if (observer != nullptr) observer->onAlignment(summary);

    // Основной цикл читает потоки с начала.
    if (!src->open())
    {
        summary.error = src->error().empty() ? "cannot open input data" : src->error();
        if (log) *log << summary.error << std::endl;
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
            src->close();
            return false;
        }
        file_sink = &file_logger;
    }

    ForwardingSink sink(file_sink, observer);
    sink.writeHeader();

    // Журнал смещений (Этап 0 плана по смещениям): пишется только по флагу,
    // поэтому result/reference/errors и test_parity не затрагиваются.
    BiasLogWriter bias_log;
    if (options.write_bias_log)
    {
        const std::string bias_file = (tools_dir / "bias.txt").string();
        if (!bias_log.open(bias_file))
        {
            summary.error = "cannot open " + bias_file;
            src->close();
            sink.close();
            return false;
        }
        if (log) *log << "bias log: " << bias_file << std::endl;
    }

    const bool want_telemetry = observer != nullptr && options.capture_telemetry;
    RunPacer *pacer = options.pacer;
    StepTelemetry tel;

    // Время последней записи журнала — для декадирования и для дедупликации
    // финального кадра (ровная сетка окон 1/10/30/… с в отчёте).
    double bias_last_written = -1.0;
    std::size_t bias_rows = 0;

    std::vector<double> row;
    SnsSample ref;
    SnsSample last_ref;
    SnsSample hold_ref;
    bool has_ref = false;
    bool hold_valid = false;

    const double total_rows =
        (observer != nullptr) ? static_cast<double>(src->imuRowCount()) : 0.0;
    double done_rows = 0.0;
    double last_report_time = -1.0;

    while (src->nextImu(row))
    {
        done_rows += 1.0;

        if (!ins::isValidRow(row))
        {
            // Битые строки imu.dat: раньше отбрасывались молча. Считаем и
            // логируем, чтобы предполётная проверка набора (Этап 0) показывала,
            // что сравнения с истиной могут быть неполными.
            summary.invalid_imu_rows++;
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
                src->close();
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
            if (!src->nextSns(last_ref))
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
        if (summary.steps == 1) summary.time_first = imu_time;
        summary.time_last = imu_time;

        // Журнал смещений (Этап 0). Оценки ba/bg и трим меняются на каждом
        // такте (tilt-апдейт 200 Гц), поэтому «записать при изменении» дал бы
        // ~26 МБ на прогон. Отчёту нужны окна 1/10/30/… с, а динамика смещений
        // на порядки медленнее такта: декадируем до 5 Гц. Финальный кадр
        // дописывается после цикла, чтобы отчёт всегда видел конечное состояние.
        if (bias_log.isOpen() &&
            (summary.steps == 1 || (imu_time - bias_last_written) >= 0.2))
        {
            BiasSample sample;
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
            src->close();
            sink.close();
            summary.error = "cancelled";
            return false;
        }
    }

    // Финальный кадр журнала: декадирование могло пропустить последнее
    // состояние, а отчёт всегда должен видеть конечные оценки и сигмы.
    if (bias_log.isOpen() && summary.steps > 0)
    {
        BiasSample sample;
        sample.time = summary.time_last;
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

    src->close();
    sink.close();
    bias_log.close();

    if (log && bias_rows > 0)
    {
        *log << "bias log: " << bias_rows << " rows" << std::endl;
    }
    if (summary.invalid_imu_rows > 0 && log)
    {
        *log << "WARNING: skipped " << summary.invalid_imu_rows
             << " malformed imu.dat rows" << std::endl;
    }

    if (observer != nullptr) observer->onProgress(summary.time_last, done_rows, total_rows);

    const auto wall_end = std::chrono::high_resolution_clock::now();
    summary.wall_seconds = std::chrono::duration<double>(wall_end - wall_start).count();

    summary.ok = true;
    return true;
}

} // namespace nav