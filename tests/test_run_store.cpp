// test_run_store.cpp — SQLite-хранилище истории прогонов: снимок параметров
// фильтра (settings_ini) и миграция старой схемы без этой колонки.

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QTemporaryDir>
#include <QtTest>
#include <QVariant>
#include <vector>

#include "core/ins/ins_filter.h"
#include "core_bridge/run_store.h"

class TestRunStore : public QObject
{
    Q_OBJECT

private slots:
    void storesSettingsSnapshot();
    void migratesLegacySchema();
    void storesConsistency();
    void migratesLegacyErrorColumns();
};

namespace
{

nav::RunSummary makeSummary()
{
    nav::RunSummary s;
    s.steps = 1234;
    s.corrections = 56;
    s.wall_seconds = 4.5;
    s.has_angle = true;
    return s;
}

} // namespace

void TestRunStore::storesSettingsSnapshot()
{
    gui::RunStore store;
    QVERIFY(store.open(":memory:"));

    const QString ini = QString("[filter]\nsigma_g = 0.5\n[tilt]\nsettle_s = 0.7\n");
    const qint64 id = store.beginRun("run1", makeSummary(), "C:/data",
                                     "{\"conf\":1}", ini);
    QVERIFY(id > 0);

    QCOMPARE(store.runSettings(id), ini);

    const auto runs = store.recentRuns();
    QCOMPARE(runs.size(), 1);
    QCOMPARE(runs[0].id, id);
    QCOMPARE(runs[0].name, QString("run1"));
    QCOMPARE(runs[0].steps, 1234);
    QCOMPARE(runs[0].has_angle, true);

    store.close();
}

void TestRunStore::migratesLegacySchema()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("legacy.db");

    // Старая схема без settings_ini и одна запись в ней.
    {
        QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", "legacy_setup");
        db.setDatabaseName(path);
        QVERIFY(db.open());
        QSqlQuery q(db);
        QVERIFY(q.exec("CREATE TABLE runs ("
                       " id INTEGER PRIMARY KEY AUTOINCREMENT,"
                       " name TEXT NOT NULL,"
                       " created_at TEXT NOT NULL,"
                       " data_dir TEXT,"
                       " config_json TEXT,"
                       " steps INTEGER DEFAULT 0,"
                       " corrections INTEGER DEFAULT 0,"
                       " wall_seconds REAL DEFAULT 0,"
                       " has_angle INTEGER DEFAULT 0)"));
        QVERIFY(q.exec("INSERT INTO runs (name, created_at) VALUES ('old', '2020-01-01')"));
        db.close();
    }
    QSqlDatabase::removeDatabase("legacy_setup");

    gui::RunStore store;
    QVERIFY(store.open(path));

    // Старая запись: колонка добавлена миграцией, снимка настроек нет.
    const auto old_runs = store.recentRuns();
    QCOMPARE(old_runs.size(), 1);
    QVERIFY(store.runSettings(old_runs[0].id).isEmpty());

    // Новая запись уже хранит снимок.
    const QString ini = QString("[filter]\nsigma_a = 1.5\n");
    const qint64 id = store.beginRun("new", makeSummary(), "C:/data", {}, ini);
    QVERIFY(id > 0);
    QCOMPARE(store.runSettings(id), ini);

    store.close();
}

// NIS/NEES сохраняются вместе с векторами ошибок кадров коррекции: иначе
// отчёт по прогону из истории печатал бы «Нет данных» в разделе согласованности.
void TestRunStore::storesConsistency()
{
    gui::RunStore store;
    QVERIFY(store.open(":memory:"));

    const qint64 id = store.beginRun("run2", makeSummary(), "C:/data", QString(), QString());
    QVERIFY(id > 0);

    const std::vector<double> times = {10.0, 11.0, 12.0};
    const std::vector<std::vector<double>> errors = {
        std::vector<double>(ins::KF_STATE, 0.1),
        std::vector<double>(ins::KF_STATE, 0.2),
        std::vector<double>(ins::KF_STATE, 0.3)};
    const std::vector<double> nis = {9.5, 10.5, 11.5};
    const std::vector<double> nees = {8.0, 8.5, 9.0};
    QVERIFY(store.appendErrors(id, times, errors, nis, nees));

    std::vector<double> read_t;
    std::vector<double> read_nis;
    std::vector<double> read_nees;
    std::vector<std::vector<double>> read_errors;
    QVERIFY(store.loadErrors(id, read_t, read_errors, &read_nis, &read_nees));
    QCOMPARE(int(read_t.size()), 3);
    QCOMPARE(int(read_errors.size()), 3);
    QCOMPARE(int(read_nis.size()), 3);
    for (int k = 0; k < 3; k++)
    {
        QCOMPARE(read_t[k], times[k]);
        QCOMPARE(read_nis[k], nis[k]);
        QCOMPARE(read_nees[k], nees[k]);
        QCOMPARE(read_errors[k][0], errors[k][0]);
    }

    // Без рядов согласованности кадр сохраняется с нулями (dof = 0).
    const qint64 id2 = store.beginRun("run3", makeSummary(), "C:/data", QString(), QString());
    QVERIFY(id2 > 0);
    QVERIFY(store.appendErrors(id2, times, errors));
    std::vector<double> t2, nis2;
    std::vector<double> nees2;
    std::vector<std::vector<double>> e2;
    QVERIFY(store.loadErrors(id2, t2, e2, &nis2, &nees2));
    QCOMPARE(int(nis2.size()), 3);
    for (double v : nis2) QCOMPARE(v, 0.0);
    for (double v : nees2) QCOMPARE(v, 0.0);

    store.close();
}

// Старая filter_errors без колонок согласованности: миграция добавляет их, а
// старые кадры читаются как кадры без NIS/NEES (нули), не ломая остальные.
void TestRunStore::migratesLegacyErrorColumns()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("legacy_errors.db");

    {
        QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", "legacy_errors_setup");
        db.setDatabaseName(path);
        QVERIFY(db.open());
        QSqlQuery q(db);
        QVERIFY(q.exec("CREATE TABLE runs ("
                       " id INTEGER PRIMARY KEY AUTOINCREMENT,"
                       " name TEXT NOT NULL,"
                       " created_at TEXT NOT NULL,"
                       " data_dir TEXT,"
                       " config_json TEXT,"
                       " settings_ini TEXT,"
                       " steps INTEGER DEFAULT 0,"
                       " corrections INTEGER DEFAULT 0,"
                       " wall_seconds REAL DEFAULT 0,"
                       " has_angle INTEGER DEFAULT 0)"));
        QVERIFY(q.exec("CREATE TABLE filter_errors ("
                       " run_id INTEGER NOT NULL,"
                       " t REAL,"
                       " x0 REAL, x1 REAL, x2 REAL, x3 REAL, x4 REAL,"
                       " x5 REAL, x6 REAL, x7 REAL, x8 REAL, x9 REAL,"
                       " x10 REAL, x11 REAL, x12 REAL, x13 REAL, x14 REAL,"
                       " PRIMARY KEY (run_id, t))"));
        QVERIFY(q.exec("INSERT INTO runs (name, created_at) VALUES ('old', '2020-01-01')"));
        QVERIFY(q.exec("INSERT INTO filter_errors (run_id, t, x0) VALUES (1, 5.0, 0.25)"));
        db.close();
    }
    QSqlDatabase::removeDatabase("legacy_errors_setup");

    gui::RunStore store;
    QVERIFY(store.open(path));

    // Прогон без ряда согласованности: кадр читается, NIS/NEES нулевые.
    const auto runs = store.recentRuns();
    QCOMPARE(runs.size(), 1);
    std::vector<double> times;
    std::vector<double> nis;
    std::vector<double> nees;
    std::vector<std::vector<double>> errors;
    QVERIFY(store.loadErrors(runs[0].id, times, errors, &nis, &nees));
    QCOMPARE(int(times.size()), 1);
    QCOMPARE(times[0], 5.0);
    QCOMPARE(errors[0][0], 0.25);
    QCOMPARE(int(nis.size()), 1);
    QCOMPARE(nis[0], 0.0);

    // В мигрированную таблицу уже можно писать NIS/NEES.
    const std::vector<double> nis_w = {3.5};
    const std::vector<double> nees_w = {4.5};
    QVERIFY(store.appendErrors(runs[0].id, {6.0}, errors, nis_w, nees_w));
    std::vector<double> times2, nis2, nees2;
    std::vector<std::vector<double>> errors2;
    QVERIFY(store.loadErrors(runs[0].id, times2, errors2, &nis2, &nees2));
    QCOMPARE(int(times2.size()), 2);
    QCOMPARE(nis2[1], 3.5);
    QCOMPARE(nees2[1], 4.5);

    store.close();
}

QTEST_MAIN(TestRunStore)
#include "test_run_store.moc"