// test_dataset_truth.cpp — проверка разбора <data_dir>/truth.txt.
//
// Тест не запускает подбор: раскладывает в QTemporaryDir синтетические
// файлы истины (корректный, отсутствующий, повреждённый) и проверяет,
// что панель «Подбор» разрешит сравнение с истиной только для корректного.

#include <QtTest>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include "settings/dataset_truth.h"

namespace
{

void writeUtf8(const QString &path, const QByteArray &data)
{
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    QVERIFY2(f.open(QIODevice::WriteOnly | QIODevice::Truncate), qPrintable(path));
    f.write(data);
    f.close();
}

// Строка-заголовок формата reference.txt/truth.txt.
const char *kHeader =
    "time           lon            lat            alt            heading        "
    "pitch          roll           vn             vh             ve\n";

} // namespace

class TestDatasetTruth : public QObject
{
    Q_OBJECT

private slots:
    void okFile();
    void loadReadsRows();
    void loadRejectsCorrupt();
    void missingFile();
    void emptyDirIsMissing();
    void corruptShortLine();
    void corruptNonNumeric();
    void headerOnlyIsCorrupt();
};

void TestDatasetTruth::okFile()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    QByteArray data(kHeader);
    data += "0.10000000     -122.14016513  47.64146792    116.97500000   5.00000000     "
            "0.00000000     0.00000000     0.00000000     0.00000000     0.00000000\n";
    data += "0.20000000     -122.14016513  47.64146792    116.97500000   5.00000000     "
            "0.00000000     0.00000000     0.00000000     0.00000000     0.00000000\n";
    writeUtf8(dir.path() + "/truth.txt", data);

    const gui::TruthCheck check = gui::checkDatasetTruth(dir.path());
    QCOMPARE(check.state, gui::TruthState::Ok);
    QCOMPARE(check.rows, 2);
    QCOMPARE(check.bad_line, 0);
    QVERIFY(gui::truthStatusText(check).contains(QStringLiteral("ок")));
}

void TestDatasetTruth::loadReadsRows()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    QByteArray data(kHeader);
    data += "0.10000000     -122.14016513  47.64146792    116.97500000   5.00000000     "
            "0.00000000     0.00000000     1.50000000     2.50000000     3.50000000\n";
    data += "0.20000000     -122.14000000  47.64100000    100.00000000   6.00000000     "
            "0.10000000     0.20000000     0.00000000     0.00000000     0.00000000\n";
    writeUtf8(dir.path() + "/truth.txt", data);

    QVector<gui::TruthSample> rows;
    gui::TruthCheck check;
    QVERIFY(gui::loadDatasetTruth(dir.path(), &rows, &check));
    QCOMPARE(check.state, gui::TruthState::Ok);
    QCOMPARE(rows.size(), 2);
    QCOMPARE(rows[0].v[0], 0.1);
    QCOMPARE(rows[0].v[2], 47.64146792);
    QCOMPARE(rows[0].v[7], 1.5);
    QCOMPARE(rows[1].v[4], 6.0);
    // Загрузчик и проверка согласованы по числу строк.
    QCOMPARE(gui::checkDatasetTruth(dir.path()).rows, rows.size());
}

void TestDatasetTruth::loadRejectsCorrupt()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    writeUtf8(dir.path() + "/truth.txt", QByteArray(kHeader) + "0.1 0.2\n");

    QVector<gui::TruthSample> rows;
    gui::TruthCheck check;
    QVERIFY(!gui::loadDatasetTruth(dir.path(), &rows, &check));
    QVERIFY(rows.isEmpty());
    QCOMPARE(check.state, gui::TruthState::Corrupt);
}

void TestDatasetTruth::missingFile()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    const gui::TruthCheck check = gui::checkDatasetTruth(dir.path());
    QCOMPARE(check.state, gui::TruthState::Missing);
    QVERIFY(gui::truthStatusText(check).contains(QStringLiteral("не найден")));
}

void TestDatasetTruth::emptyDirIsMissing()
{
    const gui::TruthCheck check = gui::checkDatasetTruth(QString());
    QCOMPARE(check.state, gui::TruthState::Missing);
}

void TestDatasetTruth::corruptShortLine()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    QByteArray data(kHeader);
    data += "0.10000000     -122.14016513  47.64146792    116.97500000\n";   // 4 поля
    writeUtf8(dir.path() + "/truth.txt", data);

    const gui::TruthCheck check = gui::checkDatasetTruth(dir.path());
    QCOMPARE(check.state, gui::TruthState::Corrupt);
    QCOMPARE(check.bad_line, 2);
    QVERIFY(gui::truthStatusText(check).contains(QStringLiteral("повреждён")));
}

void TestDatasetTruth::corruptNonNumeric()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    QByteArray data(kHeader);
    data += "0.10000000     -12?.1399516?  47.64146792    116.97500000   5.00000000     "
            "0.00000000     0.00000000     0.00000000     0.00000000     0.00000000\n";
    writeUtf8(dir.path() + "/truth.txt", data);

    const gui::TruthCheck check = gui::checkDatasetTruth(dir.path());
    QCOMPARE(check.state, gui::TruthState::Corrupt);
    QCOMPARE(check.bad_line, 2);
}

void TestDatasetTruth::headerOnlyIsCorrupt()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    writeUtf8(dir.path() + "/truth.txt", QByteArray(kHeader));

    const gui::TruthCheck check = gui::checkDatasetTruth(dir.path());
    QCOMPARE(check.state, gui::TruthState::Corrupt);
}

QTEST_MAIN(TestDatasetTruth)
#include "test_dataset_truth.moc"
