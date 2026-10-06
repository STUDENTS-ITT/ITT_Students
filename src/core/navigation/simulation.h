// simulation.h — Прогон полного цикла БИНС/СНС-комплексирования.
//
// Единая точка входа для консольного imitator и Qt-приложения: математика
// выполняется ровно одна и та же, различаются только приёмник результатов
// (файлы и/или подписка SimulationObserver) и обратная связь.
//
// Последовательность:
//   1. Чтение параметров (settings.ini) и StartupNav.ini.
//   2. Автономная выставка (Median + EMA) — начальные углы ориентации.
//   3. Формирование начального состояния.
//   4. Основной цикл по строкам imu.dat: интегрирование, коррекция tilt,
//      коррекция по СНС, выдача результатов подписчикам.

#pragma once

#include <cstddef>
#include <ostream>
#include <string>
#include <vector>

#include "../data_io/data_reader.h"
#include "../data_io/data_writer.h"
#include "trajectory.h"

namespace nav
{

class RunPacer;

// Параметры прогона.
struct RunOptions
{
    std::string data_dir;       // каталог с imu.dat / gps.dat / angle.dat / StartupNav.ini
    std::string output_dir;     // каталог для result.txt / reference.txt / errors.txt
    std::string settings_file;  // settings.ini (пусто — значения по умолчанию)
    bool write_files = true;    // писать ли файлы результатов
    bool capture_telemetry = false;  // считать ли диагонали P и инновацию z
    RunPacer *pacer = nullptr;  // пауза и темп воспроизведения (nullptr — без ограничений)
    // Темп воспроизведения: > 0 — во столько раз медленнее реального времени,
    // 0 — считать максимально быстро (по умолчанию).
    double speed_factor = 0.0;
};

// Итоги прогона.
struct RunSummary
{
    bool ok = false;
    std::string error;         // текст ошибки, если ok == false
    bool has_angle = false;    // найден ли angle.dat
    double start_lon_deg = 0;
    double start_lat_deg = 0;
    double start_alt = 0;
    double align_time_s = 0;
    double yaw0_deg = 0;       // начальный курс из выставки, град
    double pitch0_deg = 0;
    double roll0_deg = 0;
    double ba[3] = {0, 0, 0};  // смещение акселерометра из выставки, м/с²
    std::size_t steps = 0;         // число выполненных тактов
    std::size_t corrections = 0;   // число коррекций по СНС
    double time_first = 0;
    double time_last = 0;
    double wall_seconds = 0;       // длительность расчёта, с
};

// Обратная связь из цикла счисления. Все методы вызываются из рабочего потока.
class SimulationObserver
{
public:
    virtual ~SimulationObserver() = default;

    // Выставка завершена: начальные углы и смещение акселерометра.
    virtual void onAlignment(const RunSummary &summary) { (void)summary; }

    // Результат на каждом такте (200 Гц).
    virtual void onResult(const data_io::NavResult &result) { (void)result; }

    // Эталон СНС и вектор ошибок x — только на кадрах коррекции (1 Гц).
    virtual void onCorrection(const data_io::NavReference &reference, const Vector &x)
    {
        (void)reference;
        (void)x;
    }

    // Полное состояние и диагностика фильтра на каждом такте.
    // Вызывается только при RunOptions::capture_telemetry.
    virtual void onState(const NavState &state, const StepTelemetry &telemetry)
    {
        (void)state;
        (void)telemetry;
    }

    // Прогресс: t — время такта, с; done/total — доля обработанных строк.
    // total == 0, если число строк неизвестно.
    virtual void onProgress(double t, double done, double total)
    {
        (void)t;
        (void)done;
        (void)total;
    }
};

// Проверка отмены прогона (вызывается из рабочего потока).
class CancellationToken
{
public:
    virtual ~CancellationToken() = default;
    virtual bool cancelled() const { return false; }
};

// Темп прогона: пауза и ограничение скорости «как в реальном времени».
// Реализуется в GUI (QWaitCondition), ядро Qt не знает: вызывается перед
// каждым тактом через RunOptions::pacer, реализация может задержать вызов
// (пауза, медленное воспроизведение) и вернуть управление, когда пора считать.
class RunPacer
{
public:
    virtual ~RunPacer() = default;
    // sim_time_s — время такта по данным imu.dat; speed_factor из
    // RunOptions: > 0 — во столько раз медленнее реального времени,
    // 0 — считать без ограничения темпа.
    virtual void beforeStep(double sim_time_s, double speed_factor)
    {
        (void)sim_time_s;
        (void)speed_factor;
    }
};

// Выполнить прогон. Возвращает false при ошибке, описание — в summary.error.
// Отмена (cancel) также возвращает false и заполняет summary.error = "cancelled".
bool runSimulation(const RunOptions &options, RunSummary &summary,
                   SimulationObserver *observer = nullptr,
                   CancellationToken *cancel = nullptr,
                   std::ostream *log = nullptr);

// Применение устаревшего params.ini (ключи без секций) — обратная совместимость.
// Объявлено в filter_settings.h рядом с applySettings/loadFilterSettings.

// Чтение StartupNav.ini (отдельная функция — используется и в GUI).
bool readStartupNav(const std::string &path, double &lon_deg, double &lat_deg,
                    double &alt, double &time_s);

// Число строк в imu.dat (для оценки времени прогона и прогресса).
std::size_t countImuRows(const std::string &imu_path);

} // namespace nav