// simulation_worker.cpp — Фоновый прогон ядра ESKF.

#include "workers/simulation_worker.h"

#include <algorithm>

#include <QDateTime>
#include <QMutexLocker>

#include "core/utils/constants.h"

namespace gui
{
namespace
{
qint64 wallMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}
} // namespace

// ---------------------------------------------------------------- RunPacerImpl

void RunPacerImpl::setPaused(bool paused)
{
    QMutexLocker lock(&mutex_);
    if (paused_ == paused) return;
    paused_ = paused;
    if (paused_)
    {
        // Пауза не должна «накапливать» долг по темпу: сдвигаем базу.
        base_sim_s_ = last_sim_s_;
        base_wall_ms_ = wallMs();
    }
    else
    {
        base_wall_ms_ = wallMs();
        resumed_.wakeAll();
    }
}

bool RunPacerImpl::isPaused() const
{
    QMutexLocker lock(&mutex_);
    return paused_;
}

void RunPacerImpl::setSpeedFactor(double factor)
{
    QMutexLocker lock(&mutex_);
    if (qFuzzyCompare(speed_ + 1.0, factor + 1.0)) return;
    speed_ = (factor > 0.0) ? factor : 0.0;
    base_sim_s_ = last_sim_s_;
    base_wall_ms_ = wallMs();
}

double RunPacerImpl::speedFactor() const
{
    QMutexLocker lock(&mutex_);
    return speed_;
}

void RunPacerImpl::resetTiming(double sim_time_s)
{
    QMutexLocker lock(&mutex_);
    last_sim_s_ = sim_time_s;
    base_sim_s_ = sim_time_s;
    base_wall_ms_ = wallMs();
}

void RunPacerImpl::waitWhilePaused()
{
    QMutexLocker lock(&mutex_);
    while (paused_ && (cancel_ == nullptr || !cancel_->cancelled()))
    {
        // Ожидание с таймаутом, чтобы отмена не «залипала» в cond_.
        resumed_.wait(&mutex_, 100);
    }
}

void RunPacerImpl::waitForSlot(double sim_time_s)
{
    QMutexLocker lock(&mutex_);
    if (speed_ <= 0.0) return;

    for (;;)
    {
        if (cancel_ != nullptr && cancel_->cancelled()) return;
        if (paused_)
        {
            resumed_.wait(&mutex_, 100);
            base_wall_ms_ = wallMs();  // пауза не входит в темп
            continue;
        }
        const double target_ms = (sim_time_s - base_sim_s_) / speed_ * 1000.0;
        const double left_ms = target_ms - static_cast<double>(wallMs() - base_wall_ms_);
        if (left_ms <= 0.0) return;
        resumed_.wait(&mutex_, static_cast<unsigned long>(std::min(left_ms, 50.0)));
    }
}

void RunPacerImpl::beforeStep(double sim_time_s, double speed_factor)
{
    (void)speed_factor;  // фактический темп берём из setSpeedFactor()

    {
        QMutexLocker lock(&mutex_);
        last_sim_s_ = sim_time_s;
    }

    waitWhilePaused();
    if (cancel_ != nullptr && cancel_->cancelled()) return;

    waitForSlot(sim_time_s);
}

// ------------------------------------------------------------- SimulationWorker

SimulationWorker::SimulationWorker(QObject *parent) : QObject(parent) {}

SimulationWorker::~SimulationWorker() = default;

void SimulationWorker::setPaused(bool paused)
{
    pacer_.setPaused(paused);
    Q_EMIT logMessage(paused ? tr("Пауза") : tr("Продолжение"), 1);
}

void SimulationWorker::setSpeedFactor(double factor)
{
    pacer_.setSpeedFactor(factor);
    Q_EMIT logMessage(factor > 0.0
                          ? tr("Темп воспроизведения: 1/%1 от реального времени").arg(factor)
                          : tr("Темп воспроизведения: максимум"),
                      1);
}

bool SimulationWorker::isPaused() const
{
    return pacer_.isPaused();
}

void SimulationWorker::run()
{
    nav::RunSummary summary;

    // Лог расчёта (angle.dat / StartupNav / выставка) уходит в панель лога GUI.
    // Направляем std::cout в буфер строк, чтобы не терять сообщения ядра.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        has_reference_ = false;
        last_errors_.assign(ins::KF_STATE, 0.0);
    }

    Q_EMIT logMessage(tr("Запуск прогона: %1").arg(QString::fromStdString(options_.data_dir)), 0);
    Q_EMIT logMessage(tr("Запись файлов результатов: %1")
                          .arg(options_.write_files ? tr("да") : tr("нет")), 0);

    nav::RunOptions opts = options_;
    opts.pacer = &pacer_;
    pacer_.resetTiming(summary.time_first);

    const bool ok = nav::runSimulation(opts, summary, this, this, nullptr);

    if (!ok)
    {
        const QString message = QString::fromStdString(
            summary.error.empty() ? std::string("прогон не выполнен") : summary.error);
        Q_EMIT logMessage(message, 2);
        Q_EMIT finished(false, message);
        return;
    }

    const QString message = tr("Прогон завершён: тактов %1, коррекций СНС %2, "
                               "расчёт %3 с")
                                .arg(summary.steps)
                                .arg(summary.corrections)
                                .arg(summary.wall_seconds, 0, 'f', 2);
    Q_EMIT logMessage(message, 0);
    Q_EMIT finished(true, message);
}

void SimulationWorker::onAlignment(const nav::RunSummary &summary)
{
    const QString text =
        tr("Выставка %1 с: курс %2°, тангаж %3°, крен %4°; ba = (%5, %6, %7); "
           "старт %8° с.ш., %9° в.д., alt %10 м")
            .arg(summary.align_time_s, 0, 'f', 2)
            .arg(summary.yaw0_deg, 0, 'f', 4)
            .arg(summary.pitch0_deg, 0, 'f', 4)
            .arg(summary.roll0_deg, 0, 'f', 4)
            .arg(summary.ba[0], 0, 'f', 5)
            .arg(summary.ba[1], 0, 'f', 5)
            .arg(summary.ba[2], 0, 'f', 5)
            .arg(summary.start_lat_deg, 0, 'f', 6)
            .arg(summary.start_lon_deg, 0, 'f', 6)
            .arg(summary.start_alt, 0, 'f', 2);

    Q_EMIT alignmentReady(text);
    Q_EMIT logMessage(text, 0);
}

void SimulationWorker::onResult(const data_io::NavResult &result)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_result_ = result;
}

void SimulationWorker::onCorrection(const data_io::NavReference &reference, const Vector &x)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_reference_ = reference;
    last_errors_ = x;
    has_reference_ = true;
}

void SimulationWorker::onState(const nav::NavState &state, const nav::StepTelemetry &telemetry)
{
    if (!options_.capture_telemetry) return;

    TelemetrySample s;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        s.time = telemetry.time;
        s.result = last_result_;
        s.reference = last_reference_;
        s.has_reference = has_reference_;
        if (has_reference_) s.errors = last_errors_;
    }

    s.innovation = telemetry.innovation;
    s.has_innovation = telemetry.corrected &&
                       telemetry.innovation.size() == static_cast<std::size_t>(ins::KF_MEAS);
    s.nis = telemetry.stats.nis;
    s.nees = telemetry.stats.nees;
    s.nis_dof = telemetry.stats.dof;
    s.ba = telemetry.ba;
    s.bg = telemetry.bg;
    for (int i = 0; i < ins::KF_STATE; i++) s.p_diag[i] = telemetry.P_diag[i];
    s.corrected = telemetry.corrected;
    s.step_index = telemetry.step_index;

    buffer_.push(s);

    // Панель состояния обновляем 10 Гц вместо 200 Гц.
    if (telemetry.step_index % 20 == 0)
    {
        QStringList diag;
        diag << tr("t = %1 с").arg(telemetry.time, 0, 'f', 3);
        diag << tr("такт %1").arg(telemetry.step_index);
        diag << tr("x[0] = %1").arg(state.x[0], 0, 'e', 3);
        diag << tr("P[0] = %1").arg(s.p_diag[0], 0, 'e', 3);
        diag << tr("ba = (%1, %2, %3)")
                    .arg(s.ba[0], 0, 'e', 2)
                    .arg(s.ba[1], 0, 'e', 2)
                    .arg(s.ba[2], 0, 'e', 2);
        Q_EMIT stateUpdate(telemetry.time, telemetry.step_index, diag.join("; "));
    }
}

void SimulationWorker::onProgress(double t, double done, double total)
{
    int percent = -1;
    if (total > 0.0)
    {
        const double frac = done / total;
        percent = static_cast<int>(frac * 100.0);
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
    }
    Q_EMIT progress(percent, t);
}

} // namespace gui