// simulation_worker.h — Фоновый прогон ядра ESKF.
//
// Worker — QObject, переносимый в QThread: run() выполняет блокирующий
// nav::runSimulation в потоке расчёта. Наблюдателем ядра выступает сам Worker,
// телеметрия складывается в потокобезопасный буфер, а наружу отдаются только
// редкие сигналы (прогресс ~4 Гц, состояние фильтра 10 Гц, завершение).

#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <QMutex>
#include <QObject>
#include <QString>
#include <QWaitCondition>

#include "core/ins/ins_filter.h"
#include "core/navigation/simulation.h"
#include "core_bridge/telemetry_buffer.h"

namespace gui
{

// Пауза и ограничение темпа воспроизведения для nav::runSimulation.
// Живёт в потоке расчёта, команды приходят из GUI-потока через
// QMetaObject::invokeMethod (очередь событий потока), поэтому состояние
// защищено QMutex.
class RunPacerImpl : public nav::RunPacer
{
public:
    explicit RunPacerImpl(nav::CancellationToken *cancel) : cancel_(cancel) {}

    void setPaused(bool paused);
    bool isPaused() const;

    void setSpeedFactor(double factor);
    double speedFactor() const;

    // Ошибка ожидания (снят мьютекс) — сбросить состояние темпа.
    void resetTiming(double sim_time_s);

    void beforeStep(double sim_time_s, double speed_factor) override;

private:
    void waitWhilePaused();
    void waitForSlot(double sim_time_s);

    nav::CancellationToken *cancel_ = nullptr;

    mutable QMutex mutex_;
    QWaitCondition resumed_;
    bool paused_ = false;
    double speed_ = 0.0;  // 0 — без ограничения темпа
    double last_sim_s_ = 0.0;
    double base_sim_s_ = 0.0;
    qint64 base_wall_ms_ = 0;
};

class SimulationWorker : public QObject, public nav::SimulationObserver,
                         public nav::CancellationToken
{
    Q_OBJECT

public:
    explicit SimulationWorker(QObject *parent = nullptr);
    ~SimulationWorker() override;

    // Параметры прогона. Вызывается до старта (из GUI-потока).
    void configure(const nav::RunOptions &options) { options_ = options; }

    // Запрос отмены — потокобезопасно, можно из GUI-потока.
    void requestStop() { stop_requested_.store(true); }

    // Пауза/продолжение и темп воспроизведения — вызываются из GUI-потока
    // через invokeMethod (очередь событий потока расчёта).
public Q_SLOTS:
    void setPaused(bool paused);
    void setSpeedFactor(double factor);
    bool isPaused() const;

    // Блокирующий прогон. Вызывается в потоке расчёта (слот run()).
    void run();

    // nav::SimulationObserver — вызываются из потока расчёта.
    void onAlignment(const nav::RunSummary &summary) override;
    void onResult(const data_io::NavResult &result) override;
    void onCorrection(const data_io::NavReference &reference, const Vector &x) override;
    void onState(const nav::NavState &state, const nav::StepTelemetry &telemetry) override;
    void onProgress(double t, double done, double total) override;

    // nav::CancellationToken.
    bool cancelled() const override { return stop_requested_.load(); }

    const TelemetryBuffer &buffer() const { return buffer_; }
    TelemetryBuffer &buffer() { return buffer_; }

Q_SIGNALS:
    void alignmentReady(const QString &text);
    void finished(bool ok, const QString &message);
    void logMessage(const QString &text, int level);
    void stateUpdate(double time, int step, const QString &diag);
    void progress(int percent, double time);

private:
    nav::RunOptions options_;
    TelemetryBuffer buffer_;
    std::atomic_bool stop_requested_{false};
    RunPacerImpl pacer_{this};

    // Последние значения, приходящие разными вызовами наблюдателя.
    std::mutex state_mutex_;
    data_io::NavResult last_result_{};
    data_io::NavReference last_reference_{};
    Vector last_errors_ = Vector(ins::KF_STATE, 0.0);
    bool has_reference_ = false;
};

} // namespace gui