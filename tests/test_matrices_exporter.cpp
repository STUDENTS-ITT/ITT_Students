// test_matrices_exporter.cpp — Статические матрицы фильтра для экспорта.
//
// Проверяется, что GUI-экспортёр собирает Q/R/P0/H из настроек фильтра и
// параметров датасета, совпадая с формулами ядра, и корректно пишет три
// формата (CSV/Markdown/JSON).

#include <QtTest>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "core/navigation/filter_settings.h"
#include "core/utils/constants.h"
#include "export/matrices_exporter.h"

namespace
{

bool writeFile(const QString &path, const QString &text)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return false;
    return f.write(text.toUtf8()) >= 0;
}

nav::FilterSettings makeSettings()
{
    nav::FilterSettings s;
    s.filter.sig_g = 3.0e-4;
    s.filter.sig_a = 3.2e-3;
    s.filter.sig_bg = 1.17e-5;
    s.filter.sig_ba = 2.0e-5;
    s.filter.sig_pos = 6.2e-8;
    s.filter.sig_h = 5.0;
    s.filter.sig_v = 1.38e-1;
    s.filter.sig_hdg = 1.75e-2;
    return s;
}

} // namespace

class TestMatricesExporter : public QObject
{
    Q_OBJECT

private Q_SLOTS:
    void buildsMatricesFromSettings();
    void formatsContainData();
    void writesThreeFiles();
    void readsDatasetParameters();

private:
    static constexpr double kTol = 1e-30;
};

void TestMatricesExporter::buildsMatricesFromSettings()
{
    const nav::FilterSettings s = makeSettings();
    const double lat_deg = 55.0;
    const double T = 0.005;

    const gui::FilterMatrices m =
        gui::buildFilterMatrices(s, QStringLiteral("test"), lat_deg, T);

    QCOMPARE(m.Q.size(), static_cast<std::size_t>(ins::KF_STATE * ins::KF_STATE));
    QCOMPARE(m.R.size(), static_cast<std::size_t>(ins::KF_MEAS * ins::KF_MEAS));
    QCOMPARE(m.P0.size(), static_cast<std::size_t>(ins::KF_STATE * ins::KF_STATE));
    QCOMPARE(m.H.size(), static_cast<std::size_t>(ins::KF_MEAS * ins::KF_STATE));

    // Q: диагональ по блокам.
    QVERIFY(std::fabs(at(m.Q, 0, 0, ins::KF_STATE)) < kTol);
    QVERIFY(std::fabs(at(m.Q, 3, 3, ins::KF_STATE) - s.filter.sig_a * s.filter.sig_a * T) < kTol);
    QVERIFY(std::fabs(at(m.Q, 6, 6, ins::KF_STATE) - s.filter.sig_g * s.filter.sig_g * T) < kTol);
    QVERIFY(std::fabs(at(m.Q, 9, 9, ins::KF_STATE) - s.filter.sig_ba * s.filter.sig_ba * T) < kTol);
    QVERIFY(std::fabs(at(m.Q, 12, 12, ins::KF_STATE) - s.filter.sig_bg * s.filter.sig_bg * T) < kTol);

    // R: позиция/высота/скорость/курс; тангаж и крен — «игнор».
    QVERIFY(std::fabs(at(m.R, 0, 0, ins::KF_MEAS) - s.filter.sig_pos * s.filter.sig_pos) < kTol);
    QVERIFY(std::fabs(at(m.R, 2, 2, ins::KF_MEAS) - s.filter.sig_h * s.filter.sig_h) < kTol);
    QVERIFY(std::fabs(at(m.R, 3, 3, ins::KF_MEAS) - s.filter.sig_v * s.filter.sig_v) < kTol);
    QVERIFY(std::fabs(at(m.R, 6, 6, ins::KF_MEAS) - s.filter.sig_hdg * s.filter.sig_hdg) < kTol);
    QVERIFY(at(m.R, 7, 7, ins::KF_MEAS) > 1.0e6);
    QVERIFY(at(m.R, 8, 8, ins::KF_MEAS) > 1.0e6);

    // P0: позиция/высота и курс, выведенный из σ_bg по широте.
    const double heading_sigma =
        s.filter.sig_bg / (std::fabs(std::cos(lat_deg * DEG_TO_RAD)) * U_EARTH);
    QVERIFY(std::fabs(at(m.P0, 0, 0, ins::KF_STATE) -
                      (2.0 * s.filter.sig_pos) * (2.0 * s.filter.sig_pos)) < kTol);
    QVERIFY(std::fabs(at(m.P0, 6, 6, ins::KF_STATE) - heading_sigma * heading_sigma) < kTol);
    QVERIFY(std::fabs(at(m.P0, 7, 7, ins::KF_STATE) - DEG_TO_RAD * DEG_TO_RAD) < kTol);

    // H: единицы на первых 9 диагоналях, остальное — нули.
    QVERIFY(std::fabs(at(m.H, 0, 0, ins::KF_STATE) - 1.0) < kTol);
    QVERIFY(std::fabs(at(m.H, 8, 8, ins::KF_STATE) - 1.0) < kTol);
    QVERIFY(std::fabs(at(m.H, 0, 9, ins::KF_STATE)) < kTol);
}

void TestMatricesExporter::formatsContainData()
{
    const gui::FilterMatrices m =
        gui::buildFilterMatrices(makeSettings(), QStringLiteral("setA"), 55.0, 0.005);

    const QString csv = gui::formatMatricesCsv(m);
    QVERIFY(csv.contains(QStringLiteral("dataset=setA")));
    QVERIFY(csv.contains(QStringLiteral("# Q (15x15)")));
    QVERIFY(csv.contains(QStringLiteral("# R (9x9)")));
    QVERIFY(csv.contains(QStringLiteral("# P0 (15x15)")));
    QVERIFY(csv.contains(QStringLiteral("# H (9x15)")));

    const QString md = gui::formatMatricesMarkdown(m);
    QVERIFY(md.contains(QStringLiteral("P0 — начальная ковариация")));

    const QString json = gui::formatMatricesJson(m);
    const QJsonDocument doc = QJsonDocument::fromJson(json.toUtf8());
    QVERIFY(!doc.isNull());
    const QJsonObject root = doc.object();
    QCOMPARE(root.value(QStringLiteral("dataset")).toString(), QStringLiteral("setA"));
    QCOMPARE(root.value(QStringLiteral("Q")).toArray().size(), 15);
}

void TestMatricesExporter::writesThreeFiles()
{
    QTemporaryDir tmp;
    QVERIFY(tmp.isValid());

    const gui::FilterMatrices m =
        gui::buildFilterMatrices(makeSettings(), QStringLiteral("setA"), 55.0, 0.005);

    QString error;
    QVERIFY2(gui::writeFilterMatrices(m, tmp.path(), QStringLiteral("matrices_setA"), &error),
             qPrintable(error));

    for (const QString &ext : {QStringLiteral("csv"), QStringLiteral("md"), QStringLiteral("json")})
    {
        const QString path = QDir(tmp.path()).filePath(QStringLiteral("matrices_setA.") + ext);
        QVERIFY2(QFileInfo::exists(path), qPrintable(path));
    }
}

void TestMatricesExporter::readsDatasetParameters()
{
    QTemporaryDir tmp;
    QVERIFY(tmp.isValid());
    const QString dir = tmp.path();

    QVERIFY(writeFile(QDir(dir).filePath(QStringLiteral("imu.dat")),
                      QStringLiteral("time timestamp_ns wx wy wz ax ay az\n"
                                     "0.000 0 0 0 0 0 0 0\n"
                                     "0.005 0 0 0 0 0 0 0\n")));
    QVERIFY(writeFile(QDir(dir).filePath(QStringLiteral("gps.dat")),
                      QStringLiteral("time timestamp_ns lat lon alt vx vy vz\n"
                                     "0.000 0 0.83 2.13 100 0 0 0\n")));
    QVERIFY(writeFile(QDir(dir).filePath(QStringLiteral("StartupNav.ini")),
                      QStringLiteral("2.1300 ; lon deg\n"
                                     "47.6414679 ; lat deg\n"
                                     "116.975 ; alt m\n"
                                     "120.0 ; align s\n")));

    double lat = 0.0;
    double T = 0.0;
    QVERIFY(gui::readDatasetParameters(dir, &lat, &T));
    QVERIFY(std::fabs(lat - 47.6414679) < 1e-9);
    QVERIFY(std::fabs(T - 0.005) < 1e-12);
}

QTEST_APPLESS_MAIN(TestMatricesExporter)

#include "test_matrices_exporter.moc"
