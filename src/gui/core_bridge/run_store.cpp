// run_store.cpp — SQLite-хранилище истории прогонов.

#include "core_bridge/run_store.h"

#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QSqlRecord>
#include <QSet>
#include <QVariant>
#include <atomic>

namespace gui
{
namespace
{
std::atomic<quint64> g_connection_counter{0};
}

RunStore::RunStore() = default;

RunStore::~RunStore()
{
    close();
}

bool RunStore::isOpen() const
{
    return db_.isValid() && db_.isOpen();
}

bool RunStore::open(const QString &fileName, QString *error)
{
    close();

    file_ = fileName;
    connection_ = QString("itt_runs_%1").arg(g_connection_counter.fetch_add(1));

    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", connection_);
    db.setDatabaseName(fileName);
    if (!db.open())
    {
        if (error) *error = db.lastError().text();
        QSqlDatabase::removeDatabase(connection_);
        connection_.clear();
        return false;
    }

    db_ = db;

    QString schema_error;
    if (!createSchema(&schema_error))
    {
        if (error) *error = schema_error;
        close();
        return false;
    }
    return true;
}

void RunStore::close()
{
    if (db_.isValid())
    {
        if (db_.isOpen()) db_.close();
        db_ = QSqlDatabase();
        QSqlDatabase::removeDatabase(connection_);
    }
    connection_.clear();
}

bool RunStore::createSchema(QString *error)
{
    QSqlQuery q(db_);

    const char *statements[] = {
        "PRAGMA journal_mode = WAL",
        "PRAGMA synchronous = NORMAL",
        "CREATE TABLE IF NOT EXISTS runs ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  name TEXT NOT NULL,"
        "  created_at TEXT NOT NULL,"
        "  data_dir TEXT,"
        "  config_json TEXT,"
        "  settings_ini TEXT,"
        "  steps INTEGER DEFAULT 0,"
        "  corrections INTEGER DEFAULT 0,"
        "  wall_seconds REAL DEFAULT 0,"
        "  has_angle INTEGER DEFAULT 0)",

        "CREATE TABLE IF NOT EXISTS samples ("
        "  run_id INTEGER NOT NULL,"
        "  t REAL, lon REAL, lat REAL, alt REAL,"
        "  heading REAL, pitch REAL, roll REAL,"
        "  vn REAL, vh REAL, ve REAL,"
        "  has_ref INTEGER DEFAULT 0,"
        "  ref_lon REAL, ref_lat REAL, ref_alt REAL,"
        "  ref_heading REAL, ref_pitch REAL, ref_roll REAL,"
        "  ref_vn REAL, ref_vh REAL, ref_ve REAL,"
        "  PRIMARY KEY (run_id, t))",

        "CREATE TABLE IF NOT EXISTS filter_errors ("
        "  run_id INTEGER NOT NULL,"
        "  t REAL,"
        "  x0 REAL, x1 REAL, x2 REAL, x3 REAL, x4 REAL,"
        "  x5 REAL, x6 REAL, x7 REAL, x8 REAL, x9 REAL,"
        "  x10 REAL, x11 REAL, x12 REAL, x13 REAL, x14 REAL,"
        "  nis REAL, nees REAL, nis_dof INTEGER DEFAULT 0,"
        "  PRIMARY KEY (run_id, t))",

        "CREATE TABLE IF NOT EXISTS covariances ("
        "  run_id INTEGER NOT NULL,"
        "  t REAL,"
        "  p0 REAL, p1 REAL, p2 REAL, p3 REAL, p4 REAL,"
        "  p5 REAL, p6 REAL, p7 REAL, p8 REAL, p9 REAL,"
        "  p10 REAL, p11 REAL, p12 REAL, p13 REAL, p14 REAL,"
        "  PRIMARY KEY (run_id, t))",

        "CREATE INDEX IF NOT EXISTS idx_samples_run ON samples(run_id)",
    };

    for (const char *sql : statements)
    {
        if (!q.exec(QString::fromLatin1(sql)))
        {
            if (error) *error = q.lastError().text();
            return false;
        }
    }

    // Миграция базы, созданной до появления settings_ini: CREATE TABLE IF NOT
    // EXISTS для существующей таблицы ничего не делает, поэтому колонку
    // добавляем явным ALTER TABLE по результату PRAGMA table_info.
    bool has_settings_ini = false;
    QSqlQuery info(db_);
    if (info.exec("PRAGMA table_info(runs)"))
    {
        while (info.next())
        {
            if (info.value(1).toString() == QLatin1String("settings_ini"))
            {
                has_settings_ini = true;
                break;
            }
        }
    }
    if (!has_settings_ini)
    {
        QSqlQuery alter(db_);
        if (!alter.exec("ALTER TABLE runs ADD COLUMN settings_ini TEXT"))
        {
            if (error) *error = alter.lastError().text();
            return false;
        }
    }

    // Та же миграция для колонок согласованности filter_errors (NIS/NEES):
    // без них сохранённый прогон терял раздел NIS/NEES, и отчёт по прогону из
    // истории печатал «Нет данных» (включая отчёт self-test, который открывает
    // прогон из истории перед сборкой PDF).
    QSet<QString> error_columns;
    QSqlQuery error_info(db_);
    if (error_info.exec("PRAGMA table_info(filter_errors)"))
    {
        while (error_info.next()) error_columns.insert(error_info.value(1).toString());
    }
    const char *consistency_columns[] = {"nis", "nees", "nis_dof"};
    const char *consistency_sql[] = {
        "ALTER TABLE filter_errors ADD COLUMN nis REAL",
        "ALTER TABLE filter_errors ADD COLUMN nees REAL",
        "ALTER TABLE filter_errors ADD COLUMN nis_dof INTEGER DEFAULT 0",
    };
    for (int i = 0; i < 3; ++i)
    {
        if (error_columns.contains(QLatin1String(consistency_columns[i]))) continue;
        QSqlQuery alter(db_);
        if (!alter.exec(QLatin1String(consistency_sql[i])))
        {
            if (error) *error = alter.lastError().text();
            return false;
        }
    }

    // Та же миграция для эталона СНС в samples: без этих колонок прогон,
    // открытый из истории, терял отклонения в таблице и опору отчёта.
    QSet<QString> sample_columns;
    QSqlQuery sample_info(db_);
    if (sample_info.exec("PRAGMA table_info(samples)"))
    {
        while (sample_info.next()) sample_columns.insert(sample_info.value(1).toString());
    }
    const char *reference_columns[] = {
        "has_ref", "ref_lon", "ref_lat", "ref_alt", "ref_heading",
        "ref_pitch", "ref_roll", "ref_vn", "ref_vh", "ref_ve"};
    const char *reference_sql[] = {
        "ALTER TABLE samples ADD COLUMN has_ref INTEGER DEFAULT 0",
        "ALTER TABLE samples ADD COLUMN ref_lon REAL",
        "ALTER TABLE samples ADD COLUMN ref_lat REAL",
        "ALTER TABLE samples ADD COLUMN ref_alt REAL",
        "ALTER TABLE samples ADD COLUMN ref_heading REAL",
        "ALTER TABLE samples ADD COLUMN ref_pitch REAL",
        "ALTER TABLE samples ADD COLUMN ref_roll REAL",
        "ALTER TABLE samples ADD COLUMN ref_vn REAL",
        "ALTER TABLE samples ADD COLUMN ref_vh REAL",
        "ALTER TABLE samples ADD COLUMN ref_ve REAL",
    };
    for (int i = 0; i < 10; ++i)
    {
        if (sample_columns.contains(QLatin1String(reference_columns[i]))) continue;
        QSqlQuery alter(db_);
        if (!alter.exec(QLatin1String(reference_sql[i])))
        {
            if (error) *error = alter.lastError().text();
            return false;
        }
    }
    return true;
}

qint64 RunStore::beginRun(const QString &name, const nav::RunSummary &summary,
                          const QString &dataDir, const QString &configJson,
                          const QString &settingsIni)
{
    if (!isOpen()) return -1;

    QSqlQuery q(db_);
    q.prepare("INSERT INTO runs (name, created_at, data_dir, config_json, "
              "settings_ini, steps, corrections, wall_seconds, has_angle) "
              "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)");
    q.addBindValue(name);
    q.addBindValue(QDateTime::currentDateTime().toString(Qt::ISODate));
    q.addBindValue(dataDir);
    q.addBindValue(configJson);
    q.addBindValue(settingsIni);
    q.addBindValue(static_cast<qlonglong>(summary.steps));
    q.addBindValue(static_cast<qlonglong>(summary.corrections));
    q.addBindValue(summary.wall_seconds);
    q.addBindValue(summary.has_angle ? 1 : 0);

    if (!q.exec()) return -1;
    return q.lastInsertId().toLongLong();
}

QString RunStore::runSettings(qint64 runId) const
{
    if (!isOpen()) return QString();

    QSqlQuery q(db_);
    if (!q.prepare("SELECT settings_ini FROM runs WHERE id = ?")) return QString();
    q.addBindValue(runId);
    if (!q.exec() || !q.next()) return QString();
    return q.value(0).toString();
}

bool RunStore::appendSamples(qint64 runId, const QVector<SampleRow> &rows)
{
    if (!isOpen() || rows.isEmpty()) return true;

    db_.transaction();
    QSqlQuery q(db_);
    q.prepare("INSERT OR REPLACE INTO samples "
              "(run_id, t, lon, lat, alt, heading, pitch, roll, vn, vh, ve, "
              " has_ref, ref_lon, ref_lat, ref_alt, ref_heading, ref_pitch, "
              " ref_roll, ref_vn, ref_vh, ref_ve) "
              "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

    for (const SampleRow &r : rows)
    {
        q.addBindValue(runId);
        q.addBindValue(r.t);
        q.addBindValue(r.lon);
        q.addBindValue(r.lat);
        q.addBindValue(r.alt);
        q.addBindValue(r.heading);
        q.addBindValue(r.pitch);
        q.addBindValue(r.roll);
        q.addBindValue(r.vn);
        q.addBindValue(r.vh);
        q.addBindValue(r.ve);
        q.addBindValue(r.has_reference ? 1 : 0);
        q.addBindValue(r.ref_lon);
        q.addBindValue(r.ref_lat);
        q.addBindValue(r.ref_alt);
        q.addBindValue(r.ref_heading);
        q.addBindValue(r.ref_pitch);
        q.addBindValue(r.ref_roll);
        q.addBindValue(r.ref_vn);
        q.addBindValue(r.ref_vh);
        q.addBindValue(r.ref_ve);
        if (!q.exec())
        {
            db_.rollback();
            return false;
        }
    }
    return db_.commit();
}

bool RunStore::appendErrors(qint64 runId, const std::vector<double> &times,
                            const std::vector<std::vector<double>> &errors,
                            const std::vector<double> &nis,
                            const std::vector<double> &nees)
{
    if (!isOpen() || times.empty()) return true;

    QStringList cols;
    cols << "run_id" << "t";
    for (int i = 0; i < ins::KF_STATE; i++) cols << QString("x%1").arg(i);
    // Согласованность кадра: NIS/NEES и dof (0 — данных нет, см. loadErrors).
    cols << "nis" << "nees" << "nis_dof";

    db_.transaction();
    QSqlQuery q(db_);
    q.prepare(QString("INSERT OR REPLACE INTO filter_errors (%1) VALUES (%2)")
                  .arg(cols.join(", "),
                       QStringList(cols.size(), "?").join(", ")));

    for (std::size_t k = 0; k < times.size(); k++)
    {
        const std::vector<double> &x = errors[k];
        q.addBindValue(runId);
        q.addBindValue(times[k]);
        for (int i = 0; i < ins::KF_STATE; i++)
        {
            q.addBindValue(static_cast<std::size_t>(i) < x.size() ? x[i] : 0.0);
        }
        const bool has_nis = k < nis.size() && k < nees.size();
        q.addBindValue(has_nis ? nis[k] : 0.0);
        q.addBindValue(has_nis ? nees[k] : 0.0);
        q.addBindValue(has_nis ? ins::KF_MEAS : 0);
        if (!q.exec())
        {
            db_.rollback();
            return false;
        }
    }
    return db_.commit();
}

bool RunStore::appendCovariances(qint64 runId, const std::vector<double> &times,
                                  const std::vector<std::vector<double>> &pDiag)
{
    if (!isOpen() || times.empty()) return true;

    QStringList cols;
    cols << "run_id" << "t";
    for (int i = 0; i < ins::KF_STATE; i++) cols << QString("p%1").arg(i);

    db_.transaction();
    QSqlQuery q(db_);
    q.prepare(QString("INSERT OR REPLACE INTO covariances (%1) VALUES (%2)")
                  .arg(cols.join(", "),
                       QStringList(cols.size(), "?").join(", ")));

    for (std::size_t k = 0; k < times.size(); k++)
    {
        const std::vector<double> &p = pDiag[k];
        q.addBindValue(runId);
        q.addBindValue(times[k]);
        for (int i = 0; i < ins::KF_STATE; i++)
        {
            q.addBindValue(static_cast<std::size_t>(i) < p.size() ? p[i] : 0.0);
        }
        if (!q.exec())
        {
            db_.rollback();
            return false;
        }
    }
    return db_.commit();
}

QVector<RunRecord> RunStore::recentRuns(int limit)
{
    QVector<RunRecord> out;
    if (!isOpen()) return out;

    QSqlQuery q(db_);
    if (!q.exec(QString("SELECT id, name, created_at, data_dir, config_json, steps, "
                        "corrections, wall_seconds, has_angle "
                        "FROM runs ORDER BY id DESC LIMIT %1")
                    .arg(limit)))
    {
        return out;
    }

    while (q.next())
    {
        RunRecord r;
        r.id = q.value(0).toLongLong();
        r.name = q.value(1).toString();
        r.created = QDateTime::fromString(q.value(2).toString(), Qt::ISODate);
        r.data_dir = q.value(3).toString();
        r.config = q.value(4).toString();
        r.steps = q.value(5).toInt();
        r.corrections = q.value(6).toInt();
        r.wall_seconds = q.value(7).toDouble();
        r.has_angle = q.value(8).toInt() != 0;
        out.push_back(r);
    }
    return out;
}

QVector<SampleRow> RunStore::loadSamples(qint64 runId) const
{
    QVector<SampleRow> out;
    if (!isOpen()) return out;

    QSqlQuery q(db_);
    if (!q.prepare("SELECT t, lon, lat, alt, heading, pitch, roll, vn, vh, ve, "
                   "has_ref, ref_lon, ref_lat, ref_alt, ref_heading, ref_pitch, "
                   "ref_roll, ref_vn, ref_vh, ref_ve "
                   "FROM samples WHERE run_id = ? ORDER BY t"))
    {
        return out;
    }
    q.addBindValue(runId);
    if (!q.exec()) return out;

    while (q.next())
    {
        SampleRow r;
        r.t = q.value(0).toDouble();
        r.lon = q.value(1).toDouble();
        r.lat = q.value(2).toDouble();
        r.alt = q.value(3).toDouble();
        r.heading = q.value(4).toDouble();
        r.pitch = q.value(5).toDouble();
        r.roll = q.value(6).toDouble();
        r.vn = q.value(7).toDouble();
        r.vh = q.value(8).toDouble();
        r.ve = q.value(9).toDouble();
        r.has_reference = q.value(10).toInt() != 0;
        r.ref_lon = q.value(11).toDouble();
        r.ref_lat = q.value(12).toDouble();
        r.ref_alt = q.value(13).toDouble();
        r.ref_heading = q.value(14).toDouble();
        r.ref_pitch = q.value(15).toDouble();
        r.ref_roll = q.value(16).toDouble();
        r.ref_vn = q.value(17).toDouble();
        r.ref_vh = q.value(18).toDouble();
        r.ref_ve = q.value(19).toDouble();
        out.push_back(r);
    }
    return out;
}

bool RunStore::loadErrors(qint64 runId, std::vector<double> &times,
                          std::vector<std::vector<double>> &errors,
                          std::vector<double> *nis, std::vector<double> *nees) const
{
    times.clear();
    errors.clear();
    if (nis != nullptr) nis->clear();
    if (nees != nullptr) nees->clear();
    if (!isOpen()) return false;

    QStringList cols;
    for (int i = 0; i < ins::KF_STATE; i++) cols << QString("x%1").arg(i);
    cols << "nis" << "nees" << "nis_dof";

    QSqlQuery q(db_);
    if (!q.prepare(QString("SELECT t, %1 FROM filter_errors WHERE run_id = ? ORDER BY t")
                       .arg(cols.join(", "))))
    {
        return false;
    }
    q.addBindValue(runId);
    if (!q.exec()) return false;

    while (q.next())
    {
        times.push_back(q.value(0).toDouble());
        std::vector<double> x(ins::KF_STATE, 0.0);
        for (int i = 0; i < ins::KF_STATE; i++)
        {
            x[i] = q.value(i + 1).toDouble();
        }
        errors.push_back(x);

        const double row_nis = q.value(ins::KF_STATE + 1).toDouble();
        const double row_nees = q.value(ins::KF_STATE + 2).toDouble();
        const int row_dof = q.value(ins::KF_STATE + 3).toInt();
        if (nis != nullptr) nis->push_back(row_dof > 0 ? row_nis : 0.0);
        if (nees != nullptr) nees->push_back(row_dof > 0 ? row_nees : 0.0);
    }
    return true;
}

bool RunStore::loadCovariances(qint64 runId, std::vector<double> &times,
                                std::vector<std::vector<double>> &pDiag) const
{
    times.clear();
    pDiag.clear();
    if (!isOpen()) return false;

    QStringList cols;
    for (int i = 0; i < ins::KF_STATE; i++) cols << QString("p%1").arg(i);

    QSqlQuery q(db_);
    if (!q.prepare(QString("SELECT t, %1 FROM covariances WHERE run_id = ? ORDER BY t")
                       .arg(cols.join(", "))))
    {
        return false;
    }
    q.addBindValue(runId);
    if (!q.exec()) return false;

    while (q.next())
    {
        times.push_back(q.value(0).toDouble());
        std::vector<double> p(ins::KF_STATE, 0.0);
        for (int i = 0; i < ins::KF_STATE; i++) p[i] = q.value(i + 1).toDouble();
        pDiag.push_back(p);
    }
    return true;
}

bool RunStore::removeRun(qint64 runId)
{
    if (!isOpen()) return false;
    db_.transaction();
    QSqlQuery q(db_);
    for (const char *table : {"samples", "filter_errors", "covariances", "runs"})
    {
        const QString sql = table == QString("runs")
                                ? QString("DELETE FROM runs WHERE id = ?")
                                : QString("DELETE FROM %1 WHERE run_id = ?")
                                      .arg(QString::fromLatin1(table));
        q.prepare(sql);
        q.addBindValue(runId);
        if (!q.exec())
        {
            db_.rollback();
            return false;
        }
    }
    return db_.commit();
}

bool RunStore::clearAll()
{
    if (!isOpen()) return false;

    db_.transaction();
    QSqlQuery q(db_);
    for (const char *table : {"samples", "filter_errors", "covariances", "runs"})
    {
        if (!q.exec(QString("DELETE FROM %1").arg(QString::fromLatin1(table))))
        {
            db_.rollback();
            return false;
        }
    }
    return db_.commit();
}

} // namespace gui