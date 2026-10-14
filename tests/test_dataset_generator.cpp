// test_dataset_generator.cpp — Синтез тестовых наборов данных (этап 4).
//
// Проверяется генератор dataset::generateDataset: корректность структуры
// сгенерированных файлов, читаемость их штатными ридерами, согласованность
// StartupNav.ini и, главное, «обратное свойство»: чистые (без шума) показания
// ИМУ при прогоне через runSimulation не должны далеко уходить от истины.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <QtTest>

#include "core/data_io/data_reader.h"
#include "core/data_io/data_writer.h"
#include "core/data_io/dataset_generator.h"
#include "core/ins/imu_processor.h"
#include "core/navigation/simulation.h"
#include "core/utils/constants.h"

namespace
{

// Строка результата/истины: 10 колонок (deg, м, м/с).
struct Row10
{
    double time = 0;
    double lon = 0;
    double lat = 0;
    double alt = 0;
    double heading = 0;
    double pitch = 0;
    double roll = 0;
    double vn = 0;
    double vh = 0;
    double ve = 0;
};

// Чтение файла вида truth.txt / result.txt (заголовок пропускается).
bool readRows(const QString &path, QVector<Row10> &rows)
{
    rows.clear();
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return false;

    const QString text = QString::fromUtf8(f.readAll());
    const QStringList lines = text.split('\n');
    for (int i = 1; i < lines.size(); ++i)  // первая строка — заголовок
    {
        const QString line = lines[i].trimmed();
        if (line.isEmpty()) continue;

        std::vector<double> items;
        data_io::parseLine(line.toStdString(), items);
        if (items.size() < 10) continue;

        Row10 r;
        r.time = items[0];
        r.lon = items[1];
        r.lat = items[2];
        r.alt = items[3];
        r.heading = items[4];
        r.pitch = items[5];
        r.roll = items[6];
        r.vn = items[7];
        r.vh = items[8];
        r.ve = items[9];
        rows.push_back(r);
    }
    return true;
}

// Горизонтальное расстояние между двумя точками (приближение плоскости).
double horizontalDistance(double lon1, double lat1, double lon2, double lat2)
{
    const double rh = R_EARTH;
    const double d_n = (lat2 - lat1) * rh;
    const double d_e = (lon2 - lon1) * rh * cos(0.5 * (lat1 + lat2));
    return sqrt(d_n * d_n + d_e * d_e);
}

constexpr double ALIGN_S = 60.0;

// Базовая запись конфигурации: 60 с стоянки + разгон/поворот/торможение.
// Полная длительность 160 с.
dataset::DatasetConfig makeConfig(const QString &out_dir, uint64_t seed)
{
    dataset::DatasetConfig cfg;
    cfg.name = "test_set";
    cfg.out_dir = out_dir.toStdString();
    cfg.imu_hz = 200.0;
    cfg.gps_hz = 10.0;
    cfg.angle_hz = 10.0;
    cfg.lat_deg = 47.641468;
    cfg.lon_deg = -122.1401651;
    cfg.alt_m = 116.975;
    cfg.heading_deg = 5.0;
    cfg.pitch_deg = -0.33;
    cfg.roll_deg = 0.0;
    cfg.align_time_s = ALIGN_S;
    cfg.profile = {
        {ALIGN_S, 0.0, 0.0, 0.0},   // стоянка и выставка
        {20.0, 10.0, 0.0, 0.0},     // разгон до 10 м/с прямо
        {40.0, 10.0, 0.0, 1.5},     // поворот вперед-налево 1.5 град/с
        {20.0, 10.0, 0.0, 0.0},     // прямо
        {20.0, 0.0, 0.0, 0.0},      // торможение до 0
    };
    cfg.seed = seed;
    return cfg;
}

// Обнуление всех шумов: идеальные показания для проверки согласованности модели.
void zeroNoise(dataset::NoiseConfig &n)
{
    n.gyro_sigma = 0.0;
    n.gyro_bias_deg_h = {0.0, 0.0, 0.0};
    n.gyro_bias_rw_deg_h_sqrt_s = 0.0;
    n.accel_sigma = 0.0;
    n.accel_bias_mg = {0.0, 0.0, 0.0};
    n.accel_bias_rw_mg_sqrt_s = 0.0;
    n.gps_pos_h_sigma = 0.0;
    n.gps_pos_v_sigma = 0.0;
    n.gps_vel_sigma = 0.0;
    n.gps_pos_offset = {0.0, 0.0, 0.0};
    n.gps_pos_corr_sigma = 0.0;
    n.angle_sigma_deg = {0.0, 0.0, 0.0};
    n.imu_jitter_s = 0.0;
    n.imu_dropout_p = 0.0;
    n.sns_jitter_s = 0.0;
}

} // namespace

// Число строк данных в файле с заголовком.
static int countDataRows(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return -1;
    int rows = 0;
    while (!f.atEnd())
    {
        f.readLine();
        rows++;
    }
    return rows - 1;  // заголовок
}

class TestDatasetGenerator : public QObject
{
    Q_OBJECT

private slots:
    void generatesFilesAndReport();
    void writesStartupNavIni();
    void readsBackWithReaders();
    void cleanRunTracksTruth();
    void noisyRunIsPlausible();
    void rejectsBadConfig();
};

void TestDatasetGenerator::generatesFilesAndReport()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    dataset::DatasetConfig cfg = makeConfig(dir.path(), 42);
    dataset::DatasetReport report;
    std::string error;
    QVERIFY2(dataset::generateDataset(cfg, report, &error), error.c_str());

    const QStringList files = {
        "imu.dat", "gps.dat", "angle.dat", "truth.txt", "StartupNav.ini"};
    for (const QString &f : files)
    {
        QVERIFY2(QFileInfo::exists(dir.filePath(f)),
                 qPrintable(dir.filePath(f) + " missing"));
    }

    // Частоты: 200 Гц ИМУ, 10 Гц СНС.
    constexpr double duration = ALIGN_S + 20.0 + 40.0 + 20.0 + 20.0;
    const std::size_t expected_imu = static_cast<std::size_t>(duration * 200.0) + 1;
    const std::size_t expected_sns = static_cast<std::size_t>(duration * 10.0);

    QCOMPARE(report.gps_rows, expected_sns);
    QCOMPARE(report.angle_rows, expected_sns);
    QCOMPARE(report.truth_rows, expected_sns);
    // Пропуски (imu_dropout_p) уменьшают число записанных строк.
    QVERIFY(report.imu_rows <= expected_imu);
    QVERIFY(report.imu_rows >= expected_imu * 3 / 4);
    QVERIFY(qAbs(report.duration_s - duration) < 1e-6);
    QVERIFY(report.traveled_m > 700.0);
    QVERIFY(report.traveled_m < 900.0);

    QCOMPARE(countDataRows(dir.filePath("gps.dat")), int(expected_sns));
    QCOMPARE(countDataRows(dir.filePath("angle.dat")), int(expected_sns));
    QCOMPARE(countDataRows(dir.filePath("truth.txt")), int(expected_sns));
    QCOMPARE(countDataRows(dir.filePath("imu.dat")), int(report.imu_rows));

    // Заголовки файлов.
    {
        QFile f(dir.filePath("imu.dat"));
        QVERIFY(f.open(QIODevice::ReadOnly));
        QCOMPARE(QString::fromUtf8(f.readLine()).trimmed(),
                 QString("# time_s\ttimestamp_ns\twx\twy\twz\tax\tay\taz"));
    }
    {
        QFile f(dir.filePath("gps.dat"));
        QVERIFY(f.open(QIODevice::ReadOnly));
        QCOMPARE(QString::fromUtf8(f.readLine()).trimmed(),
                 QString("# time_s\ttimestamp_ns\tlatitude\tlongitude\taltitude\tvx\tvy\tvz"));
    }
    {
        QFile f(dir.filePath("angle.dat"));
        QVERIFY(f.open(QIODevice::ReadOnly));
        QCOMPARE(QString::fromUtf8(f.readLine()).trimmed(),
                 QString("# time_s\ttimestamp_ns\troll\tpitch\tyaw"));
    }
}

void TestDatasetGenerator::writesStartupNavIni()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    dataset::DatasetConfig cfg = makeConfig(dir.path(), 42);
    dataset::DatasetReport report;
    std::string error;
    QVERIFY(dataset::generateDataset(cfg, report, &error));

    double lon = 0, lat = 0, alt = 0, t = 0;
    QVERIFY(nav::readStartupNav(
        dir.filePath("StartupNav.ini").toStdString(), lon, lat, alt, t));
    QVERIFY(fabs(lon - cfg.lon_deg) < 1e-9);
    QVERIFY(fabs(lat - cfg.lat_deg) < 1e-9);
    QCOMPARE(alt, cfg.alt_m);
    QCOMPARE(t, ALIGN_S);
}

void TestDatasetGenerator::readsBackWithReaders()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    dataset::DatasetConfig cfg = makeConfig(dir.path(), 7);
    dataset::DatasetReport report;
    std::string error;
    QVERIFY(dataset::generateDataset(cfg, report, &error));

    // ИМУ: первая строка t=0, время возрастает, строки валидны.
    data_io::ImuReader imu;
    QVERIFY(imu.open(dir.filePath("imu.dat").toStdString()));
    std::vector<double> row;
    QVERIFY(imu.next(row));
    QVERIFY(ins::isValidRow(row));
    const double t0 = ins::sampleTime(row);
    QVERIFY(t0 >= 0.0 && t0 < 0.01);  // дрожание метки времени, не ровно 0

    double prev = -1.0;
    std::size_t rows = 1;
    while (imu.next(row))
    {
        QVERIFY(ins::isValidRow(row));
        const double t = ins::sampleTime(row);
        QVERIFY(t > prev);
        prev = t;
        rows++;
    }
    QCOMPARE(rows, report.imu_rows);
    imu.close();

    // СНС: первый кадр рядом со стартовой точкой, курс — из профиля (5°).
    data_io::SnsReader sns;
    QVERIFY(sns.open(dir.filePath("gps.dat").toStdString(),
                     dir.filePath("angle.dat").toStdString()));
    nav::SnsSample s;
    QVERIFY(sns.next(s));
    QVERIFY(s.time > 0.0 && s.time < 0.3);
    QVERIFY2(horizontalDistance(cfg.lon_deg * DEG_TO_RAD, cfg.lat_deg * DEG_TO_RAD, s.lon, s.lat) < 5.0,
             qPrintable(QString("first gps off by %1 m").arg(horizontalDistance(cfg.lon_deg * DEG_TO_RAD, cfg.lat_deg * DEG_TO_RAD, s.lon, s.lat), 0, 'f', 2)));
    QVERIFY(fabs(s.alt - cfg.alt_m) < 10.0);
    QVERIFY(fabs(s.heading - cfg.heading_deg * DEG_TO_RAD) < 0.05);

    std::size_t frames = 1;
    nav::SnsSample prev_s = s;
    while (sns.next(s))
    {
        QVERIFY(s.time > prev_s.time);
        prev_s = s;
        frames++;
    }
    QCOMPARE(frames, report.gps_rows);
    sns.close();
}

void TestDatasetGenerator::cleanRunTracksTruth()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    dataset::DatasetConfig cfg = makeConfig(dir.path(), 1);
    zeroNoise(cfg.noise);
    dataset::DatasetReport report;
    std::string error;
    QVERIFY(dataset::generateDataset(cfg, report, &error));

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = true;

    nav::RunSummary summary;
    QVERIFY2(nav::runSimulation(options, summary, nullptr, nullptr, nullptr),
             summary.error.c_str());
    QVERIFY(summary.ok);
    QVERIFY(summary.corrections > 100);

    QVector<Row10> truth_rows, result_rows;
    QVERIFY(readRows(dir.filePath("truth.txt"), truth_rows));
    QVERIFY(readRows(dir.filePath("result.txt"), result_rows));
    QVERIFY(!truth_rows.isEmpty());
    QVERIFY(!result_rows.isEmpty());

    // Истина и результат снимаются в разные моменты (10 Гц против 200 Гц):
    // берём строку результата, ближайшую к последней строке истины.
    const Row10 &truth = truth_rows.back();
    const Row10 *best = &result_rows.front();
    double best_dt = 1e300;
    for (const Row10 &r : result_rows)
    {
        const double dt = fabs(r.time - truth.time);
        if (dt < best_dt)
        {
            best_dt = dt;
            best = &r;
        }
    }

    QVERIFY(best_dt < 0.1);
    // Идеальные данные: расхождение лишь на ошибку дискретизации интеграторов.
    QVERIFY(horizontalDistance(truth.lon * DEG_TO_RAD, truth.lat * DEG_TO_RAD,
                               best->lon * DEG_TO_RAD, best->lat * DEG_TO_RAD) < 2.0);
    QVERIFY(fabs(best->alt - truth.alt) < 1.5);
    QVERIFY(fabs(best->heading - truth.heading) < 0.5);
}

void TestDatasetGenerator::noisyRunIsPlausible()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    // Отладочно: фиксированный каталог для разбора расходимости.
    dataset::DatasetConfig cfg = makeConfig(dir.path(), 42);  // шумы по умолчанию
    dataset::DatasetReport report;
    std::string error;
    QVERIFY(dataset::generateDataset(cfg, report, &error));

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = true;

    nav::RunSummary summary;
    QVERIFY2(nav::runSimulation(options, summary, nullptr, nullptr, nullptr),
             summary.error.c_str());
    QVERIFY(summary.ok);
    QVERIFY(summary.corrections > 100);

    QVector<Row10> truth_rows, result_rows;
    QVERIFY(readRows(dir.filePath("truth.txt"), truth_rows));
    QVERIFY(readRows(dir.filePath("result.txt"), result_rows));

    // Сравниваем по предпоследней строке истины.
    const Row10 &truth = truth_rows.size() > 1 ? truth_rows[truth_rows.size() - 2] : truth_rows.back();
    const Row10 *best = &result_rows.front();
    double best_dt = 1e300;
    for (const Row10 &r : result_rows)
    {
        const double dt = fabs(r.time - truth.time);
        if (dt < best_dt)
        {
            best_dt = dt;
            best = &r;
        }
    }

    // С коррекцией по СНС ошибка остаётся в пределах десятков метров,
    // а не растёт, как свободное счисление (километры при смещении ДУС 20°/ч).
    const double d_h = horizontalDistance(truth.lon * DEG_TO_RAD, truth.lat * DEG_TO_RAD,
                                          best->lon * DEG_TO_RAD, best->lat * DEG_TO_RAD);
    const double d_up = fabs(best->alt - truth.alt);
    const double d_hdg = fabs(best->heading - truth.heading);
    QVERIFY2(d_h < 25.0,
             qPrintable(QString("horizontal error %1 m (expect < 25)").arg(d_h, 0, 'f', 2)));
    QVERIFY2(d_up < 15.0,
             qPrintable(QString("vertical error %1 m (expect < 15)").arg(d_up, 0, 'f', 2)));
    QVERIFY2(d_hdg < 5.0,
             qPrintable(QString("heading error %1 deg (expect < 5)").arg(d_hdg, 0, 'f', 2)));
}

void TestDatasetGenerator::rejectsBadConfig()
{
    dataset::DatasetConfig cfg = makeConfig(QDir::tempPath() + "/rejects_bad_config", 42);
    std::string error;

    dataset::DatasetReport report;
    cfg.profile.clear();
    QVERIFY(!dataset::generateDataset(cfg, report, &error));
    QVERIFY(!error.empty());

    cfg = makeConfig(QDir::tempPath() + "/rejects_bad_config", 42);
    cfg.out_dir.clear();
    QVERIFY(!dataset::generateDataset(cfg, report, &error));
    QVERIFY(!error.empty());
}

QTEST_MAIN(TestDatasetGenerator)
#include "test_dataset_generator.moc"