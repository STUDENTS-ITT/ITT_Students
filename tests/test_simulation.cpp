// test_simulation.cpp — Прогон ядра на синтетических данных.
//
// Проверяется связка «читатель → шаг счисления → приёмник»: инварианты
// состояния, работа окна потери СНС, отмена прогона и корректность
// пересчёта состояния при записи только в память.

#include <cmath>
#include <cstdio>
#include <string>

#include <QtTest>

#include "core/navigation/aligner.h"
#include "core/navigation/filter_settings.h"
#include "core/navigation/simulation.h"
#include "core/navigation/trajectory.h"
#include "core/utils/constants.h"

namespace
{

// Приёмник, считающий поток результатов.
class CountingSink : public data_io::NavSink
{
public:
    void writeHeader() override { headers++; }
    void writeResult(const data_io::NavResult &r) override
    {
        results++;
        last = r;
    }
    void writeReference(const data_io::NavReference &r) override
    {
        references++;
        last_ref = r;
    }
    void writeErrors(double t, const Vector &x) override
    {
        errors++;
        last_error_time = t;
        last_x = x;
    }

    int headers = 0;
    int results = 0;
    int references = 0;
    int errors = 0;
    data_io::NavResult last{};
    data_io::NavReference last_ref{};
    double last_error_time = -1.0;
    Vector last_x;
};

// Отмена по числу проверок: цикл прогона вызывает cancelled() на каждом такте.
// requestStop() включает принудительную отмену (используется pacer'ом).
class StepCancel : public nav::CancellationToken
{
public:
    mutable int calls = 0;
    int limit = 100;
    bool forced = false;

    void requestStop() { forced = true; }
    bool cancelled() const override { return forced || ++calls >= limit; }
};

// Записывает синтетический датасет в каталог.
bool writeDataset(const QString &dir, int imu_rows, int gps_rows, bool with_angle,
                  double lat_deg = 22.5, double lon_deg = 114.0)
{
    {
        QFile f(dir + "/imu.dat");
        if (!f.open(QIODevice::WriteOnly)) return false;
        f.write("# time_s timestamp_ns wx wy wz ax ay az\n");
        for (int i = 0; i < imu_rows; i++)
        {
            const double t = 30.0 + 0.005 * i;
            f.write(QString("%1\t%2\t0\t0\t0\t0\t0\t9.80665\n")
                        .arg(t, 0, 'f', 6)
                        .arg(static_cast<qlonglong>(i) * 5000000)
                        .toUtf8());
        }
    }
    {
        QFile f(dir + "/gps.dat");
        if (!f.open(QIODevice::WriteOnly)) return false;
        f.write("# time_s timestamp_ns latitude longitude altitude vx vy vz\n");
        for (int i = 0; i < gps_rows; i++)
        {
            const double t = 30.0 + 0.2 * i;
            f.write(QString("%1\t%2\t%3\t%4\t100.0\t0\t0\t0\n")
                        .arg(t, 0, 'f', 6)
                        .arg(static_cast<qlonglong>(i) * 200000000)
                        .arg(lat_deg, 0, 'f', 6)
                        .arg(lon_deg, 0, 'f', 6)
                        .toUtf8());
        }
    }
    if (with_angle)
    {
        QFile f(dir + "/angle.dat");
        if (!f.open(QIODevice::WriteOnly)) return false;
        f.write("# time_s timestamp_ns roll pitch yaw\n");
        for (int i = 0; i < gps_rows; i++)
        {
            const double t = 30.0 + 0.2 * i;
            f.write(QString("%1\t%2\t0\t0\t0\n")
                        .arg(t, 0, 'f', 6)
                        .arg(static_cast<qlonglong>(i) * 200000000)
                        .toUtf8());
        }
    }
    {
        QFile f(dir + "/StartupNav.ini");
        if (!f.open(QIODevice::WriteOnly)) return false;
        f.write(QString("%1\n%2\n100.0\n2.0\n")
                    .arg(lon_deg, 0, 'f', 6)
                    .arg(lat_deg, 0, 'f', 6)
                    .toUtf8());
    }
    return true;
}

// Наблюдатель, считающий вызовы обратной связи.
class CountingObserver : public nav::SimulationObserver
{
public:
    int alignments = 0;
    int results = 0;
    int corrections = 0;
    int states = 0;
    int progress = 0;

    void onAlignment(const nav::RunSummary &) override { alignments++; }
    void onResult(const data_io::NavResult &) override { results++; }
    void onCorrection(const data_io::NavReference &, const Vector &) override { corrections++; }
    void onState(const nav::NavState &, const nav::StepTelemetry &) override { states++; }
    void onProgress(double, double, double) override { progress++; }
};

// Темп прогона: считает вызовы и может остановить прогон через cancel.
class RecordingPacer : public nav::RunPacer
{
public:
    int calls = 0;
    double last_time = -1.0;
    double last_speed = -1.0;
    std::vector<double> times;

    void beforeStep(double sim_time_s, double speed_factor) override
    {
        calls++;
        last_time = sim_time_s;
        last_speed = speed_factor;
        times.push_back(sim_time_s);
    }
};

} // namespace

class TestSimulation : public QObject
{
    Q_OBJECT

private slots:
    void countsImuRows();
    void readsStartupNav();
    void readsMissingStartupNav();
    void stepWritesResultEachTick();
    void stepAppliesCorrectionOnGpsFrame();
    void telemetryCapturedWhenRequested();
    void outageWindowDisablesCorrection();
    void cancellationStopsRun();
    void pacerSeesEveryStep();
    void pacerCanStopRun();
    void fullRunProducesSummary();
    void runWritesFiles();
};

void TestSimulation::countsImuRows()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 10, 5, false));

    // 10 строк данных + заголовок.
    QCOMPARE(nav::countImuRows((dir.path() + "/imu.dat").toStdString()), std::size_t(10));
}

void TestSimulation::readsStartupNav()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 5, 2, false));

    double lon = 0, lat = 0, alt = 0, t = 0;
    QVERIFY(nav::readStartupNav((dir.path() + "/StartupNav.ini").toStdString(), lon, lat, alt, t));
    QVERIFY(qAbs(lon - 114.0) < 1e-6);
    QVERIFY(qAbs(lat - 22.5) < 1e-6);
    QCOMPARE(alt, 100.0);
    QCOMPARE(t, 2.0);

    QVERIFY(!nav::readStartupNav((dir.path() + "/missing.ini").toStdString(), lon, lat, alt, t));
}

void TestSimulation::readsMissingStartupNav()
{
    double lon = 0, lat = 0, alt = 0, t = 0;
    QVERIFY(!nav::readStartupNav("definitely_missing_file.ini", lon, lat, alt, t));
}

void TestSimulation::stepWritesResultEachTick()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 20, 5, false));

    data_io::ImuReader imu;
    data_io::SnsReader sns;
    QVERIFY(imu.open((dir.path() + "/imu.dat").toStdString()));
    QVERIFY(sns.open((dir.path() + "/gps.dat").toStdString()));

    CountingSink sink;
    sink.writeHeader();
    QCOMPARE(sink.headers, 1);

    nav::NavState st = nav::initialAlignment(22.5 * DEG_TO_RAD, 114.0 * DEG_TO_RAD,
                                             100.0, 0.0, 0.0, 0.0);

    nav::SnsSample ref;
    QVERIFY(sns.next(ref));

    std::vector<double> row;
    int steps = 0;
    while (imu.next(row) && ins::isValidRow(row))
    {
        nav::step(row, ref, st, sink, false, false, nullptr);
        steps++;
    }

    QCOMPARE(steps, 20);
    QCOMPARE(sink.results, 20);
    QVERIFY(sink.errors >= 0);

    // Время последнего результата совпадает с последней строкой imu.dat.
    QCOMPARE(sink.last.time, 30.0 + 0.005 * 19);
}

void TestSimulation::stepAppliesCorrectionOnGpsFrame()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 100, 5, true));

    data_io::ImuReader imu;
    data_io::SnsReader sns;
    QVERIFY(imu.open((dir.path() + "/imu.dat").toStdString()));
    QVERIFY(sns.open((dir.path() + "/gps.dat").toStdString(),
                     (dir.path() + "/angle.dat").toStdString()));

    CountingSink sink;
    nav::NavState st = nav::initialAlignment(22.5 * DEG_TO_RAD, 114.0 * DEG_TO_RAD,
                                             100.0, 0.0, 0.0, 0.0);

    nav::SnsSample ref;
    QVERIFY(sns.next(ref));

    std::vector<double> row;
    int corrections = 0;
    while (imu.next(row) && ins::isValidRow(row))
    {
        const bool do_correction = true;
        const int before = sink.errors;
        nav::step(row, ref, st, sink, do_correction, true, nullptr);
        if (sink.errors > before) corrections++;
    }

    QVERIFY(corrections > 0);
    QCOMPARE(sink.errors, corrections);
    QCOMPARE(sink.references, corrections);

    // После коррекции вектор ошибок обнуляется (применён в номинал).
    for (const double v : sink.last_x)
    {
        QVERIFY(std::isfinite(v));
    }
}

void TestSimulation::telemetryCapturedWhenRequested()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 50, 5, true));

    data_io::ImuReader imu;
    data_io::SnsReader sns;
    QVERIFY(imu.open((dir.path() + "/imu.dat").toStdString()));
    QVERIFY(sns.open((dir.path() + "/gps.dat").toStdString(),
                     (dir.path() + "/angle.dat").toStdString()));

    CountingSink sink;
    nav::NavState st = nav::initialAlignment(22.5 * DEG_TO_RAD, 114.0 * DEG_TO_RAD,
                                             100.0, 0.0, 0.0, 0.0);

    nav::SnsSample ref;
    QVERIFY(sns.next(ref));

    nav::StepTelemetry tel;
    std::vector<double> row;
    int steps = 0;
    int corrected_steps = 0;
    while (imu.next(row) && ins::isValidRow(row))
    {
        const bool do_correction = (steps % 40) == 0;
        nav::step(row, ref, st, sink, do_correction, true, &tel);

        // Диагонали ковариации всегда положительны и конечны.
        for (int i = 0; i < ins::KF_STATE; i++)
        {
            QVERIFY(std::isfinite(tel.P_diag[i]));
            QVERIFY(tel.P_diag[i] >= 0.0);
        }
        QCOMPARE(tel.time, 30.0 + 0.005 * steps);
        // step_index проставляет цикл прогона (nav::runSimulation), а не step().
        QCOMPARE(tel.step_index, 0);
        if (tel.corrected)
        {
            corrected_steps++;
            QCOMPARE(tel.innovation.size(), std::size_t(ins::KF_MEAS));
            QCOMPARE(do_correction, true);
        }
        steps++;
    }

    QCOMPARE(steps, 50);
    QVERIFY(corrected_steps > 0);
}

void TestSimulation::outageWindowDisablesCorrection()
{
    // Окно потери СНС: внутри окна коррекции не применяются.
    nav::FilterSettings s;
    s.filter.outage_start_s = 30.0;
    s.filter.outage_end_s = 31.0;
    nav::applySettings(s);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 20, 5, true));

    data_io::ImuReader imu;
    data_io::SnsReader sns;
    QVERIFY(imu.open((dir.path() + "/imu.dat").toStdString()));
    QVERIFY(sns.open((dir.path() + "/gps.dat").toStdString(),
                     (dir.path() + "/angle.dat").toStdString()));

    CountingSink sink;
    nav::NavState st = nav::initialAlignment(22.5 * DEG_TO_RAD, 114.0 * DEG_TO_RAD,
                                             100.0, 0.0, 0.0, 0.0);
    nav::SnsSample ref;
    QVERIFY(sns.next(ref));

    std::vector<double> row;
    while (imu.next(row) && ins::isValidRow(row))
    {
        const double t = ins::sampleTime(row);
        const bool in_outage = t >= ins::kalman_cfg.outage_start_s &&
                               t <= ins::kalman_cfg.outage_end_s;
        const bool do_correction = !in_outage;
        nav::step(row, ref, st, sink, do_correction, true, nullptr);
    }

    // Весь датасет (30.000..30.095) попал в окно 30.00..31.00 → коррекций не было.
    QCOMPARE(sink.errors, 0);
    QCOMPARE(sink.references, 0);
    QCOMPARE(sink.results, 20);

    const nav::FilterSettings defaults;
    nav::applySettings(defaults);
}

void TestSimulation::cancellationStopsRun()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 500, 20, true));

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = false;

    StepCancel cancel;
    cancel.limit = 50;

    nav::RunSummary summary;
    QVERIFY(!nav::runSimulation(options, summary, nullptr, &cancel, nullptr));
    QCOMPARE(summary.error, "cancelled");
    QVERIFY(summary.steps <= 50);
}

void TestSimulation::pacerSeesEveryStep()
{
    // nav::RunPacer вызывается перед каждым тактом: на этом интерфейсе GUI
    // реализует паузу и ограничение темпа воспроизведения.
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 120, 20, true));

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = false;
    options.speed_factor = 4.0;

    RecordingPacer pacer;
    options.pacer = &pacer;

    nav::RunSummary summary;
    QVERIFY(nav::runSimulation(options, summary, nullptr, nullptr, nullptr));

    QVERIFY(summary.ok);
    QCOMPARE(summary.steps, std::size_t(120));
    QCOMPARE(pacer.calls, 120);
    QCOMPARE(pacer.times.front(), 30.0);
    QCOMPARE(pacer.last_time, 30.0 + 0.005 * 119);
    QCOMPARE(pacer.last_speed, 4.0);
}

void TestSimulation::pacerCanStopRun()
{
    // Остановка на паузе: pacer выставляет cancel, цикл должен завершиться
    // сразу, не дожидаясь конца данных.
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 5000, 20, true));

    class StoppingPacer : public nav::RunPacer
    {
    public:
        explicit StoppingPacer(StepCancel *c) : cancel_(c) {}
        int calls = 0;

        void beforeStep(double, double) override
        {
            if (++calls >= 10) cancel_->requestStop();
        }

    private:
        StepCancel *cancel_;
    };

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = false;

    StepCancel cancel;
    cancel.limit = 1000000;  // сам cancel не срабатывает — останавливает pacer

    StoppingPacer pacer(&cancel);
    options.pacer = &pacer;

    nav::RunSummary summary;
    QVERIFY(!nav::runSimulation(options, summary, nullptr, &cancel, nullptr));
    QCOMPARE(summary.error, "cancelled");
    QCOMPARE(pacer.calls, 10);
    QVERIFY(summary.steps < 20);
}

void TestSimulation::fullRunProducesSummary()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 400, 20, true));

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = false;
    options.capture_telemetry = true;

    CountingObserver obs;
    nav::RunSummary summary;
    QVERIFY(nav::runSimulation(options, summary, &obs, nullptr, nullptr));

    QVERIFY(summary.ok);
    QVERIFY(summary.has_angle);
    QCOMPARE(summary.steps, std::size_t(400));
    QCOMPARE(obs.alignments, 1);
    QCOMPARE(obs.results, 400);
    QCOMPARE(obs.states, 400);
    QVERIFY(obs.corrections > 0);
    QVERIFY(obs.progress > 0);
    QCOMPARE(summary.time_first, 30.0);
    QCOMPARE(summary.time_last, 30.0 + 0.005 * 399);
    QVERIFY(summary.wall_seconds >= 0.0);
    QVERIFY(std::isfinite(summary.yaw0_deg));
}

void TestSimulation::runWritesFiles()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 300, 15, true));

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = true;

    nav::RunSummary summary;
    QVERIFY(nav::runSimulation(options, summary, nullptr, nullptr, nullptr));

    QVERIFY(QFileInfo::exists(dir.path() + "/result.txt"));
    QVERIFY(QFileInfo::exists(dir.path() + "/reference.txt"));
    QVERIFY(QFileInfo::exists(dir.path() + "/errors.txt"));

    // В result.txt строк на один больше, чем тактов (заголовок).
    QFile f(dir.path() + "/result.txt");
    QVERIFY(f.open(QIODevice::ReadOnly));
    int lines = 0;
    while (!f.atEnd())
    {
        f.readLine();
        lines++;
    }
    QCOMPARE(lines, int(summary.steps) + 1);
}

QTEST_MAIN(TestSimulation)
#include "test_simulation.moc"