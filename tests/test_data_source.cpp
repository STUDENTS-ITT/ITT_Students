// test_data_source.cpp — Источник входных данных IDataSource (этап 8).
//
// Проверяется, что счисления читает данные через общий интерфейс и что
// файловый (FileDataSource) и «памятный» (MockDataSource) источники дают
// побитово одинаковый результат прогона на одних и тех же данных.

#include <cmath>
#include <string>
#include <vector>

#include <QtTest>

#include "core/data_io/data_source.h"
#include "core/data_io/data_reader.h"
#include "core/navigation/simulation.h"
#include "core/utils/constants.h"

namespace
{

// Записывает синтетический датасет старого формата в каталог
// (imu.dat / gps.dat / angle.dat / StartupNav.ini).
bool writeDataset(const QString &dir, int imu_rows, int gps_rows,
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

QByteArray readAll(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return QByteArray();
    return f.readAll();
}

// Секунды → "ЧЧ:ММ:СС.дробь" (новый формат dataset_1786981733).
QString hms(double t)
{
    const qlonglong whole = static_cast<qlonglong>(t);
    const int h = static_cast<int>(whole / 3600);
    const int m = static_cast<int>((whole % 3600) / 60);
    const double s = t - static_cast<double>(whole / 60 * 60);
    return QString("%1:%2:%3")
        .arg(h, 2, 10, QLatin1Char('0'))
        .arg(m, 2, 10, QLatin1Char('0'))
        .arg(s, 0, 'f', 9);
}

// Записывает синтетический датасет нового формата (.data) в каталог.
bool writeNewDataset(const QString &dir, int imu_rows, int gps_rows,
                     double lat_deg = 22.5, double lon_deg = 114.0)
{
    {
        QFile f(dir + "/imu.data");
        if (!f.open(QIODevice::WriteOnly)) return false;
        f.write("#timestamp [ns]\tw_RS_S_x [rad s^-1]\tw_RS_S_y [rad s^-1]\tw_RS_S_z [rad s^-1]\ta_RS_S_x [m s^-2]\ta_RS_S_y [m s^-2]\ta_RS_S_z [m s^-2]\n");
        for (int i = 0; i < imu_rows; i++)
        {
            const double t = 0.005 * i;
            f.write(QString("%1\t0\t0\t0\t0\t0\t9.80665\n").arg(hms(t)).toUtf8());
        }
    }
    {
        QFile f(dir + "/gps.data");
        if (!f.open(QIODevice::WriteOnly)) return false;
        f.write("#timestamp [ns]\tlatitude [deg]\tlongitude [deg]\taltitude [m]\tv_x [m s^-1]\tv_y [m s^-1]\tv_z [m s^-1]\n");
        for (int i = 0; i < gps_rows; i++)
        {
            const double t = 0.2 * i;
            f.write(QString("%1\t%2\t%3\t100.0\t0\t0\t0\n")
                        .arg(hms(t))
                        .arg(lat_deg, 0, 'f', 6)
                        .arg(lon_deg, 0, 'f', 6)
                        .toUtf8());
        }
    }
    {
        QFile f(dir + "/angle.data");
        if (!f.open(QIODevice::WriteOnly)) return false;
        // Заголовок без '#' — как в реальном наборе line_dataset.
        f.write("timestamp_ns\tx\ty\tz\tqw\tqx\tqy\tqz\tvx\tvy\tvz\troll\tpitch\tyaw\n");
        for (int i = 0; i < gps_rows; i++)
        {
            const double t = 0.2 * i;
            f.write(QString("%1\t0\t0\t0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\n")
                        .arg(hms(t))
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

} // namespace

class TestDataSource : public QObject
{
    Q_OBJECT

private slots:
    void fileSourceReadsStreams();
    void fileSourceReadsNewFormat();
    void mockSourceRewindsAndStreams();
    void mockMatchesFileRun();
    void realDataFormatSanity();
};

void TestDataSource::fileSourceReadsStreams()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeDataset(dir.path(), 400, 20));

    data_io::FileDataSource src(dir.path().toStdString());
    QVERIFY(src.open());
    QVERIFY(src.hasAngle());
    QCOMPARE(src.imuRowCount(), std::size_t(400));

    double lon = 0, lat = 0, alt = 0, t = 0;
    QVERIFY(src.startupNav(lon, lat, alt, t));
    QVERIFY(qAbs(lon - 114.0) < 1e-6);
    QVERIFY(qAbs(lat - 22.5) < 1e-6);
    QCOMPARE(alt, 100.0);
    QCOMPARE(t, 2.0);

    // Полное чтение потока ИМУ, затем перемотка через open().
    std::vector<double> row;
    int imu_rows = 0;
    while (src.nextImu(row)) imu_rows++;
    QCOMPARE(imu_rows, 400);

    QVERIFY(src.open());
    QVERIFY(src.nextImu(row));
    QCOMPARE(row[0], 30.0);

    // Полное чтение потока СНС, затем перемотка.
    nav::SnsSample s;
    int sns_rows = 0;
    while (src.nextSns(s)) sns_rows++;
    QCOMPARE(sns_rows, 20);

    QVERIFY(src.open());
    QVERIFY(src.nextSns(s));
    QCOMPARE(s.time, 30.0);
    QVERIFY(qAbs(s.lat - 22.5 * DEG_TO_RAD) < 1e-9);
    src.close();
}

void TestDataSource::fileSourceReadsNewFormat()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QVERIFY(writeNewDataset(dir.path(), 400, 20));

    // При отсутствии .dat файловая реализация выбирает .data.
    data_io::FileDataSource src(dir.path().toStdString());
    QVERIFY(src.open());
    QVERIFY(src.hasAngle());
    QCOMPARE(src.imuRowCount(), std::size_t(400));

    nav::RunOptions options;
    options.data_dir = dir.path().toStdString();
    options.output_dir = dir.path().toStdString();
    options.write_files = false;

    nav::RunSummary summary;
    QVERIFY(nav::runSimulation(options, summary, nullptr, nullptr, nullptr));
    QVERIFY(summary.ok);
    QVERIFY(summary.has_angle);
    QCOMPARE(summary.steps, std::size_t(400));
    QCOMPARE(summary.time_first, 0.0);
    src.close();
}

// Sanity-прогон на реальном наборе нового формата
// ITT_Students-Katya/imitator_sq/test/line_dataset (если он есть на диске).
void TestDataSource::realDataFormatSanity()
{
    const QString dir = QString::fromUtf8(ITT_LINE_DATASET_DIR);
    if (!QFileInfo::exists(dir + "/imu.data"))
    {
        QSKIP("line_dataset отсутствует — sanity-прогон пропущен");
    }

    data_io::FileDataSource src(dir.toStdString());
    QVERIFY(src.open());
    QVERIFY(src.hasAngle());
    QVERIFY(src.imuRowCount() > 0);

    QTemporaryDir out;
    QVERIFY(out.isValid());
    nav::RunOptions options;
    options.data_dir = dir.toStdString();
    options.output_dir = out.path().toStdString();
    options.write_files = true;

    nav::RunSummary summary;
    QVERIFY(nav::runSimulation(options, summary, nullptr, nullptr, nullptr));
    QVERIFY(summary.ok);
    QVERIFY(summary.steps > 0);
    QVERIFY(QFileInfo::exists(out.path() + "/result.txt"));
    QVERIFY(QFileInfo::exists(out.path() + "/reference.txt"));
    QVERIFY(QFileInfo::exists(out.path() + "/errors.txt"));
    src.close();
}

void TestDataSource::mockSourceRewindsAndStreams()
{
    data_io::MockDataSource mock;
    mock.imu_rows = {{30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665},
                     {30.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665}};
    nav::SnsSample s;
    s.time = 30.0;
    s.lat = 22.5 * DEG_TO_RAD;
    s.lon = 114.0 * DEG_TO_RAD;
    s.alt = 100.0;
    mock.sns_samples.push_back(s);
    mock.has_angle = true;

    QVERIFY(mock.open());
    QVERIFY(!mock.startupNav(s.lon, s.lat, s.alt, s.time));  // StartupNav не задан

    std::vector<double> row;
    QVERIFY(mock.nextImu(row));
    QCOMPARE(row.size(), std::size_t(8));
    QCOMPARE(row[0], 30.0);
    QVERIFY(mock.nextImu(row));
    QCOMPARE(row[0], 30.005);
    QVERIFY(!mock.nextImu(row));

    nav::SnsSample out;
    QVERIFY(mock.nextSns(out));
    QVERIFY(!mock.nextSns(out));

    // open() перематывает оба потока.
    QVERIFY(mock.open());
    QVERIFY(mock.nextImu(row));
    QCOMPARE(row[0], 30.0);
    QVERIFY(mock.nextSns(out));
}

void TestDataSource::mockMatchesFileRun()
{
    QTemporaryDir src_dir;
    QVERIFY(src_dir.isValid());
    QVERIFY(writeDataset(src_dir.path(), 400, 20));

    // Собираем MockDataSource из тех же данных через те же читатели.
    data_io::MockDataSource mock;
    {
        data_io::ImuReader imu;
        QVERIFY(imu.open((src_dir.path() + "/imu.dat").toStdString()));
        std::vector<double> row;
        while (imu.next(row)) mock.imu_rows.push_back(row);
    }
    {
        data_io::SnsReader sns;
        QVERIFY(sns.open((src_dir.path() + "/gps.dat").toStdString(),
                         (src_dir.path() + "/angle.dat").toStdString()));
        nav::SnsSample s;
        while (sns.next(s)) mock.sns_samples.push_back(s);
    }
    mock.has_angle = true;
    mock.has_startup = true;
    mock.startup_lon_deg = 114.0;
    mock.startup_lat_deg = 22.5;
    mock.startup_alt = 100.0;
    mock.startup_time_s = 2.0;

    QTemporaryDir file_out;
    QTemporaryDir mock_out;
    QVERIFY(file_out.isValid());
    QVERIFY(mock_out.isValid());

    nav::RunOptions file_opts;
    file_opts.data_dir = src_dir.path().toStdString();
    file_opts.output_dir = file_out.path().toStdString();
    file_opts.write_files = true;

    nav::RunSummary file_summary;
    QVERIFY(nav::runSimulation(file_opts, file_summary, nullptr, nullptr, nullptr));

    nav::RunOptions mock_opts;
    mock_opts.data_dir = "ignored";
    mock_opts.output_dir = mock_out.path().toStdString();
    mock_opts.write_files = true;
    mock_opts.source = &mock;

    nav::RunSummary mock_summary;
    QVERIFY(nav::runSimulation(mock_opts, mock_summary, nullptr, nullptr, nullptr));

    // Итоги прогона совпадают.
    QVERIFY(file_summary.ok);
    QVERIFY(mock_summary.ok);
    QCOMPARE(mock_summary.has_angle, file_summary.has_angle);
    QCOMPARE(mock_summary.steps, file_summary.steps);
    QCOMPARE(mock_summary.corrections, file_summary.corrections);
    QCOMPARE(mock_summary.time_first, file_summary.time_first);
    QCOMPARE(mock_summary.time_last, file_summary.time_last);
    QCOMPARE(mock_summary.yaw0_deg, file_summary.yaw0_deg);

    // Файлы результатов совпадают побитово.
    QCOMPARE(readAll(mock_out.path() + "/result.txt"),
             readAll(file_out.path() + "/result.txt"));
    QCOMPARE(readAll(mock_out.path() + "/reference.txt"),
             readAll(file_out.path() + "/reference.txt"));
    QCOMPARE(readAll(mock_out.path() + "/errors.txt"),
             readAll(file_out.path() + "/errors.txt"));
}

QTEST_MAIN(TestDataSource)
#include "test_data_source.moc"
