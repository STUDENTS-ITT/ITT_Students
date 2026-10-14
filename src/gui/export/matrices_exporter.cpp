// matrices_exporter.cpp — Реализация экспорта статических матриц фильтра.

#include "export/matrices_exporter.h"

#include <algorithm>

#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <QSaveFile>
#include <QTextStream>

#include "core/utils/constants.h"
#include "core/data_io/data_source.h"
#include "core/ins/imu_processor.h"
#include "core/navigation/aligner.h"
#include "core/navigation/simulation.h"

namespace gui
{
namespace
{

// Сигма «игнорировать» для тангажа/крена в 9-мерной коррекции по СНС
// (совпадает с SIG_TILT_IGNORE в core/navigation/trajectory.h): углы
// обрабатываются отдельным tilt-контуром по акселерометру, поэтому в
// матрице R курсовой коррекции эти компоненты заведомо «не доверяют» СНС.
constexpr double kSigTiltIgnore = 1e6 * DEG_TO_RAD;

// Компактное представление числа с запасом значащих цифр.
QString num(double v)
{
    return QString::number(v, 'g', 12);
}

// Строка со значениями сигм фильтра (общая для всех форматов).
QString sigmaLine(const ins::KalmanConfig &c)
{
    return QStringLiteral(
               "sigma_g=%1, sigma_a=%2, sigma_bg=%3, sigma_ba=%4, "
               "sigma_pos=%5, sigma_h=%6, sigma_v=%7, sigma_ang=%8")
        .arg(num(c.sig_g), num(c.sig_a), num(c.sig_bg), num(c.sig_ba),
             num(c.sig_pos), num(c.sig_h), num(c.sig_v), num(c.sig_hdg));
}

// --- CSV ---------------------------------------------------------------

QString matrixCsv(const QString &name, const Matrix &m, int rows, int cols)
{
    QString out;
    out += QStringLiteral("# %1 (%2x%3)\n").arg(name).arg(rows).arg(cols);

    out += QStringLiteral("row");
    for (int j = 0; j < cols; ++j)
        out += QStringLiteral(",%1").arg(j);
    out += QLatin1Char('\n');

    for (int i = 0; i < rows; ++i)
    {
        out += QString::number(i);
        for (int j = 0; j < cols; ++j)
            out += QStringLiteral(",%1").arg(num(at(m, i, j, cols)));
        out += QLatin1Char('\n');
    }
    return out;
}

// --- Markdown ----------------------------------------------------------

QString matrixMarkdown(const QString &name, const Matrix &m, int rows, int cols)
{
    QString out;
    out += QStringLiteral("## %1 (%2×%3)\n\n").arg(name).arg(rows).arg(cols);

    out += QStringLiteral("| i\\j |");
    for (int j = 0; j < cols; ++j)
        out += QStringLiteral(" %1 |").arg(j);
    out += QLatin1Char('\n');

    out += QStringLiteral("|---|");
    for (int j = 0; j < cols; ++j)
        out += QStringLiteral("---|");
    out += QLatin1Char('\n');

    for (int i = 0; i < rows; ++i)
    {
        out += QStringLiteral("| **%1** |").arg(i);
        for (int j = 0; j < cols; ++j)
            out += QStringLiteral(" %1 |").arg(num(at(m, i, j, cols)));
        out += QLatin1Char('\n');
    }
    out += QLatin1Char('\n');
    return out;
}

// --- JSON --------------------------------------------------------------

QJsonArray jsonMatrix(const Matrix &m, int rows, int cols)
{
    QJsonArray out;
    for (int i = 0; i < rows; ++i)
    {
        QJsonArray row;
        for (int j = 0; j < cols; ++j)
            row.append(at(m, i, j, cols));
        out.append(row);
    }
    return out;
}

// --- Запись ------------------------------------------------------------

bool writeTextFile(const QString &path, const QString &text, QString *error)
{
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error) *error = QObject::tr("Не удалось открыть %1 для записи").arg(path);
        return false;
    }

    QTextStream s(&f);
    s << text;
    s.flush();
    if (!f.commit())
    {
        if (error) *error = QObject::tr("Не удалось записать %1").arg(path);
        return false;
    }
    return true;
}

} // namespace

FilterMatrices buildFilterMatrices(const nav::FilterSettings &settings,
                                   const QString &dataset, double lat_deg, double T)
{
    FilterMatrices m;
    m.dataset = dataset;
    m.lat_deg = lat_deg;
    m.T = T;
    m.cfg = settings.filter;

    // В 9-мерной коррекции по СНС курс берётся из настроек, а тангаж/крен
    // игнорируются (обрабатываются отдельным tilt-контуром).
    m.sig_hdg = m.cfg.sig_hdg;
    m.sig_pitch = kSigTiltIgnore;
    m.sig_roll = kSigTiltIgnore;

    m.Q = ins::Qj_matrix(m.cfg, m.T);
    m.R = ins::Rj_matrix(m.cfg, m.sig_hdg, m.sig_pitch, m.sig_roll);
    m.P0 = nav::initialCovariance(m.cfg, m.lat_deg * DEG_TO_RAD);
    m.H = H_matrix(ins::KF_MEAS, ins::KF_STATE);
    return m;
}

bool readDatasetParameters(const QString &data_dir, double *lat_deg, double *T)
{
    if (T) *T = IMU_PERIOD;
    if (lat_deg) *lat_deg = 0.0;
    if (data_dir.isEmpty())
        return false;

    bool ok = false;

    const QString ini = QDir(data_dir).filePath(QStringLiteral("StartupNav.ini"));
    double lon = 0, lat = 0, alt = 0, t = 0;
    if (nav::readStartupNav(ini.toStdString(), lon, lat, alt, t))
    {
        if (lat_deg) *lat_deg = lat;
        ok = true;
    }

    // Номинальный шаг T — медиана положительных интервалов между отсчётами
    // ИМУ. В реальных наборах time_s дрожит (jitter), поэтому интервал между
    // двумя первыми строками не показателен; медиана устойчива. Выборку
    // ограничиваем, чтобы не читать огромные файлы целиком.
    data_io::FileDataSource src(data_dir.toStdString());
    if (src.open())
    {
        constexpr int kMaxSamples = 20000;
        std::vector<double> dts;
        dts.reserve(1024);
        std::vector<double> row;
        double prev = 0.0;
        bool has_prev = false;
        int count = 0;
        while (count < kMaxSamples && src.nextImu(row))
        {
            if (!ins::isValidRow(row))
                continue;
            ++count;
            const double ts = ins::sampleTime(row);
            if (has_prev)
            {
                const double dt = ts - prev;
                if (dt > 0.0 && dt < 1.0)
                    dts.push_back(dt);
            }
            prev = ts;
            has_prev = true;
        }
        src.close();

        if (!dts.empty())
        {
            std::sort(dts.begin(), dts.end());
            const std::size_t n = dts.size();
            const double median = (n % 2 == 0) ? 0.5 * (dts[n / 2 - 1] + dts[n / 2]) : dts[n / 2];
            if (T) *T = median;
            ok = true;
        }
    }

    return ok;
}

QString formatMatricesCsv(const FilterMatrices &m)
{
    QString out;
    out += QStringLiteral("# Матрицы фильтра ESKF (15 состояний), статические.\n");
    out += QStringLiteral("# dataset=%1\n").arg(m.dataset);
    out += QStringLiteral("# lat_deg=%1\n").arg(num(m.lat_deg));
    out += QStringLiteral("# T_s=%1\n").arg(num(m.T));
    out += QStringLiteral("# %1\n").arg(sigmaLine(m.cfg));
    out += QStringLiteral("# R: sig_hdg=%1; sig_pitch=sig_roll=%2 рад "
                          "(тангаж/крен в СНС-коррекции игнорируются)\n")
               .arg(num(m.sig_hdg), num(m.sig_pitch));
    out += QLatin1Char('\n');

    out += matrixCsv(QStringLiteral("Q"), m.Q, ins::KF_STATE, ins::KF_STATE);
    out += QLatin1Char('\n');
    out += matrixCsv(QStringLiteral("R"), m.R, ins::KF_MEAS, ins::KF_MEAS);
    out += QLatin1Char('\n');
    out += matrixCsv(QStringLiteral("P0"), m.P0, ins::KF_STATE, ins::KF_STATE);
    out += QLatin1Char('\n');
    out += matrixCsv(QStringLiteral("H"), m.H, ins::KF_MEAS, ins::KF_STATE);
    return out;
}

QString formatMatricesMarkdown(const FilterMatrices &m)
{
    QString out;
    out += QStringLiteral("# Матрицы фильтра ESKF (15 состояний), статические\n\n");
    out += QStringLiteral("- dataset: `%1`\n").arg(m.dataset);
    out += QStringLiteral("- широта начальной точки: `%1°`\n").arg(num(m.lat_deg));
    out += QStringLiteral("- шаг дискретизации T: `%1 с`\n").arg(num(m.T));
    out += QStringLiteral("- %1\n").arg(sigmaLine(m.cfg));
    out += QStringLiteral("- R: `sig_hdg=%1`; `sig_pitch=sig_roll=%2` рад "
                          "(тангаж/крен в СНС-коррекции игнорируются)\n\n")
               .arg(num(m.sig_hdg), num(m.sig_pitch));

    out += matrixMarkdown(QStringLiteral("Q — шум процесса"), m.Q, ins::KF_STATE, ins::KF_STATE);
    out += matrixMarkdown(QStringLiteral("R — шум измерений"), m.R, ins::KF_MEAS, ins::KF_MEAS);
    out += matrixMarkdown(QStringLiteral("P0 — начальная ковариация"), m.P0, ins::KF_STATE, ins::KF_STATE);
    out += matrixMarkdown(QStringLiteral("H — матрица наблюдения"), m.H, ins::KF_MEAS, ins::KF_STATE);
    return out;
}

QString formatMatricesJson(const FilterMatrices &m)
{
    QJsonObject root;
    root.insert(QStringLiteral("dataset"), m.dataset);
    root.insert(QStringLiteral("lat_deg"), m.lat_deg);
    root.insert(QStringLiteral("T_s"), m.T);

    QJsonObject sigma;
    sigma.insert(QStringLiteral("sig_g"), m.cfg.sig_g);
    sigma.insert(QStringLiteral("sig_a"), m.cfg.sig_a);
    sigma.insert(QStringLiteral("sig_bg"), m.cfg.sig_bg);
    sigma.insert(QStringLiteral("sig_ba"), m.cfg.sig_ba);
    sigma.insert(QStringLiteral("sig_pos"), m.cfg.sig_pos);
    sigma.insert(QStringLiteral("sig_h"), m.cfg.sig_h);
    sigma.insert(QStringLiteral("sig_v"), m.cfg.sig_v);
    sigma.insert(QStringLiteral("sig_ang"), m.cfg.sig_hdg);
    root.insert(QStringLiteral("sigma"), sigma);

    QJsonObject r_sigma;
    r_sigma.insert(QStringLiteral("sig_hdg"), m.sig_hdg);
    r_sigma.insert(QStringLiteral("sig_pitch"), m.sig_pitch);
    r_sigma.insert(QStringLiteral("sig_roll"), m.sig_roll);
    root.insert(QStringLiteral("R_sigma"), r_sigma);

    root.insert(QStringLiteral("Q"), jsonMatrix(m.Q, ins::KF_STATE, ins::KF_STATE));
    root.insert(QStringLiteral("R"), jsonMatrix(m.R, ins::KF_MEAS, ins::KF_MEAS));
    root.insert(QStringLiteral("P0"), jsonMatrix(m.P0, ins::KF_STATE, ins::KF_STATE));
    root.insert(QStringLiteral("H"), jsonMatrix(m.H, ins::KF_MEAS, ins::KF_STATE));

    return QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

bool writeFilterMatrices(const FilterMatrices &m, const QString &dir,
                         const QString &base, QString *error)
{
    const QString prefix = QDir(dir).filePath(base);
    if (!writeTextFile(prefix + QStringLiteral(".csv"), formatMatricesCsv(m), error))
        return false;
    if (!writeTextFile(prefix + QStringLiteral(".md"), formatMatricesMarkdown(m), error))
        return false;
    if (!writeTextFile(prefix + QStringLiteral(".json"), formatMatricesJson(m), error))
        return false;
    return true;
}

} // namespace gui
