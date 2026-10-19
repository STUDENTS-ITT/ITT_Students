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
    void parsesHmsTimestamp();
    void readsNewFormatImu();
    void readsNewFormatSnsAndJoinsAngles();
    void skipsGarbageRows();
    void loggerWritesAllThreeFiles();
    void loggerWritesConsistency();
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

void TestDataIo::parsesHmsTimestamp()
{
    double t = 0.0;
    QVERIFY(data_io::parseTimestamp("00:00:00.006000128", t));
    QVERIFY(qAbs(t - 0.006000128) < 1e-12);

    QVERIFY(data_io::parseTimestamp("00:01:01.500000000", t));
    QVERIFY(qAbs(t - 61.5) < 1e-12);

    QVERIFY(data_io::parseTimestamp("12:34:05.250", t));
    QVERIFY(qAbs(t - (12.0 * 3600.0 + 34.0 * 60.0 + 5.25)) < 1e-9);

    // Резерв: обычное число с плавающей точкой.
    QVERIFY(data_io::parseTimestamp("123.456", t));
    QCOMPARE(t, 123.456);

    // Мусор — false.
    QVERIFY(!data_io::parseTimestamp("abc", t));
}

void TestDataIo::readsNewFormatImu()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("imu.data");

    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("# timestamp wx wy wz ax ay az\n");
    f.write("00:00:00.000000128\t0\t0\t0\t0\t0\t9.8\n");
    f.write("00:01:01.500000000\t0.01\t0.02\t0.03\t1.0\t2.0\t3.0\n");
    f.close();

    data_io::ImuReader imu;
    QVERIFY(imu.open(path.toStdString()));

    std::vector<double> row;
    QVERIFY(imu.next(row));  // первая строка после заголовка
    QCOMPARE(row.size(), std::size_t(8));
    QVERIFY(qAbs(row[0] - 0.000000128) < 1e-9);
    QCOMPARE(row[1], 0.0);  // timestamp_ns для нового формата — ноль
    QCOMPARE(row[7], 9.8);

    QVERIFY(imu.next(row));
    QCOMPARE(ins::sampleTime(row), 61.5);
    QCOMPARE(row[2], 0.01);
    QCOMPARE(row[4], 0.03);
    QCOMPARE(row[5], 1.0);

    QVERIFY(!imu.next(row));
    imu.close();
}

void TestDataIo::readsNewFormatSnsAndJoinsAngles()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    const QString gps = dir.filePath("gps.data");
    const QString angle = dir.filePath("angle.data");

    QFile fg(gps);
    QVERIFY(fg.open(QIODevice::WriteOnly));
    fg.write("# time lat lon alt vx vy vz\n");
    fg.write("00:00:01.000000000\t22.5\t114.0\t100.0\t1.0\t0.0\t2.0\n");
    fg.write("00:00:02.000000000\t22.6\t114.1\t101.0\t1.1\t0.1\t2.1\n");
    fg.close();

    QFile fa(angle);
    QVERIFY(fa.open(QIODevice::WriteOnly));
    // 14 столбцов: time x y z qw qx qy qz vx vy vz roll pitch yaw.
    fa.write("# time x y z qw qx qy qz vx vy vz roll pitch yaw\n");
    fa.write("00:00:01.000000000\t0\t0\t0\t1\t0\t0\t0\t1.0\t0\t2.0\t0.01\t0.02\t0.03\n");
    fa.write("00:00:02.000000000\t0\t0\t0\t1\t0\t0\t0\t1.1\t0.1\t2.1\t0.11\t0.12\t0.13\n");
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
    QVERIFY(qAbs(s1.vn - 1.0) < 1e-12);
    QCOMPARE(s1.vh, 0.0);
    QVERIFY(qAbs(s1.ve - 2.0) < 1e-12);
    // Углы angle.data (колонки 11-13) трактуются как радианы.
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

// Старый формат: файл с мусорными строками (включая байт 0x1A) должен
// читаться в бинарном режиме, мусор — пропускаться, строки после него — читаться.
void TestDataIo::skipsGarbageRows()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("imu.dat");

    QByteArray bytes;
    bytes += "# time wx wy wz ax ay az\n";
    bytes += "0.0\t0\t1\t2\t3\t4\t5\t6\n";
    // Двоичный мусор с управляющим байтом 0x1A (Ctrl-Z). Длину задаём явно:
    // удобный байт 0x00 внутри строки обрезал бы её по strlen.
    bytes += QByteArray("\xEA\x32\xF5\x36\x1A\x00\xFF garbage here\n", 21);
    bytes += "1.5\t0\t7\t8\t9\t10\t11\t12\n";

    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write(bytes);
    f.close();

    data_io::ImuReader imu;
    QVERIFY(imu.open(path.toStdString()));

    std::vector<double> row;
    QVERIFY(imu.next(row));
    QCOMPARE(row[0], 0.0);

    // После мусорной строки (и байта 0x1A) должна прийти следующая валидная.
    QVERIFY(imu.next(row));
    QCOMPARE(row[0], 1.5);
    QCOMPARE(row[7], 12.0);

    QVERIFY(!imu.next(row));
    imu.close();
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

void TestDataIo::loggerWritesConsistency()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    const QString result = dir.filePath("result.txt");
    const QString reference = dir.filePath("reference.txt");
    const QString errors = dir.filePath("errors.txt");
    const QString consistency = dir.filePath("consistency.txt");

    data_io::ConsistencyRecord rec;
    rec.time = 1.0;
    rec.nis = 9.5;
    rec.dof = 9;

    // По умолчанию согласованность не запрашивается и файл не создаётся:
    // ядро в этом режиме не считает NIS/NEES.
    data_io::NavLogger off;
    QVERIFY(off.open(result.toStdString(), reference.toStdString(), errors.toStdString()));
    QVERIFY(!off.wantsConsistency());
    off.writeConsistency(rec);
    off.close();
    QVERIFY(!QFileInfo::exists(consistency));

    // С флагом файл создаётся лениво — по первой записи.
    data_io::NavLogger on;
    QVERIFY(on.open(result.toStdString(), reference.toStdString(), errors.toStdString()));
    on.setConsistencyEnabled(true);
    QVERIFY(on.wantsConsistency());
    on.writeConsistency(rec);
    rec.time = 2.0;
    rec.nis = 11.25;
    on.writeConsistency(rec);
    on.close();

    QVERIFY(QFileInfo::exists(consistency));
    QFile f(consistency);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString text = QString::fromUtf8(f.readAll());
    QVERIFY(text.contains("time"));
    QVERIFY(text.contains("nis"));
    QVERIFY(text.contains("11.250000"));
    QVERIFY(text.contains("9"));
    // Итоговая строка о числе кадров — признак корректного close().
    QVERIFY(text.contains("# Кадров коррекции 2."));
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