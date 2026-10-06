// test_data_io.cpp — Чтение входных файлов и запись результатов.

#include <cmath>
#include <cstdio>
#include <string>

#include <QtTest>

#include "core/data_io/data_reader.h"
#include "core/data_io/data_writer.h"
#include "core/ins/imu_processor.h"
#include "core/utils/constants.h"

class TestDataIo : public QObject
{
    Q_OBJECT

private slots:
    void parsesNumericRow();
    void splitsTextRow();
    void validatesImuRow();
    void readsImuFile();
    void readsSnsFileAndJoinsAngles();
    void readsSnsWithoutAngles();
    void loggerWritesAllThreeFiles();
};

void TestDataIo::parsesNumericRow()
{
    std::vector<double> items;
    data_io::parseLine("0.18494180\t123456\t0.001\t-0.002\t0.003\t1.0\t2.0\t3.0", items);

    QCOMPARE(items.size(), std::size_t(8));
    QCOMPARE(items[0], 0.18494180);
    QCOMPARE(items[2], 0.001);
    QCOMPARE(items[7], 3.0);

    // Пустая строка — пустой вектор.
    data_io::parseLine("   ", items);
    QVERIFY(items.empty());
}

void TestDataIo::splitsTextRow()
{
    const auto parts = data_io::splitLine("1.5\t-2.5\ttail");
    QCOMPARE(parts.size(), std::size_t(3));
    QCOMPARE(QString::fromStdString(parts[0]), QString("1.5"));
    QCOMPARE(QString::fromStdString(parts[2]), QString("tail"));
}

void TestDataIo::validatesImuRow()
{
    QVERIFY(ins::isValidRow(std::vector<double>(8, 0.0)));
    QVERIFY(ins::isValidRow(std::vector<double>(10, 0.0)));
    QVERIFY(!ins::isValidRow(std::vector<double>(7, 0.0)));
    QVERIFY(!ins::isValidRow(std::vector<double>{}));

    std::vector<double> row = {10.0, 1234.0, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0};
    QCOMPARE(ins::sampleTime(row), 10.0);

    const Vector ba = {0.5, 0.5, 0.5};
    const Vector a = ins::accel(row, ba);
    QCOMPARE(a[0], 0.5);
    QCOMPARE(a[1], 1.5);
    QCOMPARE(a[2], 2.5);

    const Vector bg = {0.01, 0.01, 0.01};
    const Vector g = ins::gyro(row, bg);
    QVERIFY(qAbs(g[0] - 0.09) < 1e-12);
    QVERIFY(qAbs(g[2] - 0.29) < 1e-12);
}

void TestDataIo::readsImuFile()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("imu.dat");

    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("# time_s timestamp_ns wx wy wz ax ay az\n");
    f.write("0.00\t0\t0\t0\t0\t0\t0\t9.8\n");
    f.write("0.01\t1\t0.01\t0\t0\t0\t0\t9.81\n");
    f.close();

    data_io::ImuReader imu;
    QVERIFY(imu.open(path.toStdString()));

    std::vector<double> row;
    QVERIFY(imu.next(row));  // первая строка после заголовка
    QCOMPARE(ins::sampleTime(row), 0.00);

    QVERIFY(imu.next(row));
    QCOMPARE(ins::sampleTime(row), 0.01);

    QVERIFY(!imu.next(row));
    imu.close();
}

void TestDataIo::readsSnsFileAndJoinsAngles()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    const QString gps = dir.filePath("gps.dat");
    const QString angle = dir.filePath("angle.dat");

    QFile fg(gps);
    QVERIFY(fg.open(QIODevice::WriteOnly));
    fg.write("# time_s timestamp_ns latitude longitude altitude vx vy vz\n");
    fg.write("1.0\t0\t22.5\t114.0\t100.0\t1.0\t0.0\t2.0\n");
    fg.write("2.0\t0\t22.6\t114.1\t101.0\t1.1\t0.1\t2.1\n");
    fg.close();

    QFile fa(angle);
    QVERIFY(fa.open(QIODevice::WriteOnly));
    fa.write("# time_s timestamp_ns roll pitch yaw\n");
    fa.write("1.0\t0\t0.01\t0.02\t0.03\n");
    fa.write("2.0\t0\t0.11\t0.12\t0.13\n");
    fa.close();

    data_io::SnsReader sns;
    QVERIFY(sns.open(gps.toStdString(), angle.toStdString()));

    nav::SnsSample s1;
    QVERIFY(sns.next(s1));
    QCOMPARE(s1.time, 1.0);
    // Градусы → радианы.
    QVERIFY(qAbs(s1.lat - 22.5 * DEG_TO_RAD) < 1e-12);
    QVERIFY(qAbs(s1.lon - 114.0 * DEG_TO_RAD) < 1e-12);
    QCOMPARE(s1.alt, 100.0);
    QCOMPARE(s1.vn, 1.0);
    QCOMPARE(s1.ve, 2.0);
    // Углы angle.dat трактуются как радианы.
    QVERIFY(qAbs(s1.heading - 0.03) < 1e-12);
    QVERIFY(qAbs(s1.pitch - 0.02) < 1e-12);
    QVERIFY(qAbs(s1.roll - 0.01) < 1e-12);

    nav::SnsSample s2;
    QVERIFY(sns.next(s2));
    QCOMPARE(s2.time, 2.0);
    QVERIFY(qAbs(s2.heading - 0.13) < 1e-12);

    QVERIFY(!sns.next(s2));
    sns.close();
}

void TestDataIo::readsSnsWithoutAngles()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    const QString gps = dir.filePath("gps.dat");
    QFile fg(gps);
    QVERIFY(fg.open(QIODevice::WriteOnly));
    fg.write("# header\n");
    fg.write("1.0\t0\t22.5\t114.0\t100.0\t0\t0\t0\n");
    fg.close();

    data_io::SnsReader sns;
    QVERIFY(sns.open(gps.toStdString(), ""));

    nav::SnsSample s;
    QVERIFY(sns.next(s));
    QCOMPARE(s.time, 1.0);
    // Без angle.dat углы нулевые.
    QCOMPARE(s.heading, 0.0);
    QCOMPARE(s.pitch, 0.0);
    QCOMPARE(s.roll, 0.0);
    QCOMPARE(s.maxSpeed(), 0.0);
    sns.close();
}

void TestDataIo::loggerWritesAllThreeFiles()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    const QString result = dir.filePath("result.txt");
    const QString reference = dir.filePath("reference.txt");
    const QString errors = dir.filePath("errors.txt");

    data_io::NavLogger log;
    QVERIFY(log.open(result.toStdString(), reference.toStdString(), errors.toStdString()));
    log.writeHeader();

    data_io::NavResult r;
    r.time = 1.5;
    r.lon = 114.0;
    r.lat = 22.5;
    log.writeResult(r);

    data_io::NavReference ref;
    ref.time = 1.5;
    log.writeReference(ref);

    Vector x(15, 0.5);
    log.writeErrors(1.5, x);
    log.close();

    QVERIFY(QFileInfo::exists(result));
    QVERIFY(QFileInfo::exists(reference));
    QVERIFY(QFileInfo::exists(errors));

    QFile f(result);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString text = QString::fromUtf8(f.readAll());
    QVERIFY(text.startsWith("time"));
    QVERIFY(text.contains("1.50000000"));

    QFile e(errors);
    QVERIFY(e.open(QIODevice::ReadOnly));
    const QString etext = QString::fromUtf8(e.readAll());
    QVERIFY(etext.contains("x14"));
    QVERIFY(etext.contains("0.5000"));
}

// Приёмник-память: проверка интерфейса NavSink без файлов.
namespace
{

class MemorySink : public data_io::NavSink
{
public:
    int headers = 0;
    int results = 0;
    int references = 0;
    int errors = 0;

    void writeHeader() override { headers++; }
    void writeResult(const data_io::NavResult &) override { results++; }
    void writeReference(const data_io::NavReference &) override { references++; }
    void writeErrors(double, const Vector &) override { errors++; }
};

} // namespace

QTEST_MAIN(TestDataIo)
#include "test_data_io.moc"