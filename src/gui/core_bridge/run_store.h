// run_store.h — История прогонов в SQLite (QtSql).
//
// Схема:
//   runs(id, name, created_at, data_dir, settings_json, steps, corrections,
//        wall_seconds, has_angle)
//   samples(run_id, t, lon, lat, alt, heading, pitch, roll, vn, vh, ve)
//   filter_errors(run_id, t, x0..x14)
//   covariances(run_id, t, p0..p14)
//
// Сравнение двух прогонов (до/после правки параметров) делается обычными
// SQL-запросами, без повторного запуска Python.

#pragma once

#include <cstddef>
#include <vector>

#include <QDateTime>
#include <QSqlDatabase>
#include <QString>
#include <QVector>

#include "core/navigation/simulation.h"

namespace gui
{

// Строка прогонов для списка в GUI.
struct RunRecord
{
    qint64 id = 0;
    QString name;
    QDateTime created;
    QString data_dir;
    int steps = 0;
    int corrections = 0;
    double wall_seconds = 0.0;
    bool has_angle = false;
    QString config;
};

// Один отсчёт траектории (result.txt).
struct SampleRow
{
    double t = 0;
    double lon = 0, lat = 0, alt = 0;
    double heading = 0, pitch = 0, roll = 0;
    double vn = 0, vh = 0, ve = 0;
};

// Хранилище истории прогонов.
class RunStore
{
public:
    RunStore();
    ~RunStore();

    // Открыть (создать при необходимости) базу. fileName может быть ":memory:".
    bool open(const QString &fileName, QString *error = nullptr);
    void close();
    bool isOpen() const;

    QString databaseFile() const { return file_; }

    // Создать запись о прогоне. Возвращает id или -1 при ошибке.
    qint64 beginRun(const QString &name, const nav::RunSummary &summary,
                    const QString &dataDir, const QString &configJson);

    // Записать серию результатов (обычно пачками по несколько тысяч строк).
    bool appendSamples(qint64 runId, const QVector<SampleRow> &rows);

    // Записать серию векторов ошибок фильтра (кадры коррекции).
    // times[k] — время, errors[k] — вектор из ins::KF_STATE компонент.
    bool appendErrors(qint64 runId, const std::vector<double> &times,
                      const std::vector<std::vector<double>> &errors);

    // Записать диагонали P (прорежено).
    bool appendCovariances(qint64 runId, const std::vector<double> &times,
                           const std::vector<std::vector<double>> &pDiag);

    // Последние прогоны (новые сверху).
    QVector<RunRecord> recentRuns(int limit = 50);

    // Загрузить траекторию прогона.
    QVector<SampleRow> loadSamples(qint64 runId) const;

    // Загрузить ошибки фильтра прогона.
    bool loadErrors(qint64 runId, std::vector<double> &times,
                    std::vector<std::vector<double>> &errors) const;

    // Загрузка диагоналей ковариации прогона (по всем тактам ИМУ).
    bool loadCovariances(qint64 runId, std::vector<double> &times,
                         std::vector<std::vector<double>> &pDiag) const;

    // Удалить прогон.
    bool removeRun(qint64 runId);

    // Очистить историю.
    bool clearAll();

private:
    bool createSchema(QString *error);

    QString file_;
    QSqlDatabase db_;
    QString connection_;
};

} // namespace gui