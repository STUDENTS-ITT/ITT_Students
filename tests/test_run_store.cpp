// test_run_store.cpp — SQLite-хранилище истории прогонов: снимок параметров
// фильтра (settings_ini) и миграция старой схемы без этой колонки.

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QTemporaryDir>
#include <QtTest>
#include <QVariant>

#include "core_bridge/run_store.h"

class TestRunStore : public QObject
{
    Q_OBJECT

private slots:
    void storesSettingsSnapshot();
    void migratesLegacySchema();
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

QTEST_MAIN(TestRunStore)
#include "test_run_store.moc"