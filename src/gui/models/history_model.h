// history_model.h — Список сохранённых прогонов (QtSql).
//
// Позволяет открыть результаты предыдущего прогона без повторного расчёта:
// траектория и ошибки фильтра читаются из SQLite.

#pragma once

#include <QAbstractTableModel>
#include <QVector>

#include "core_bridge/run_store.h"
#include "core_bridge/telemetry_buffer.h"

namespace gui
{

class HistoryModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum Column
    {
        ColId = 0,
        ColName,
        ColCreated,
        ColSteps,
        ColCorrections,
        ColWall,
        ColDataDir,
        ColCount
    };

    explicit HistoryModel(QObject *parent = nullptr);

    void setStore(RunStore *store);
    void refresh();
    void clear();

    qint64 currentRunId() const { return current_id_; }
    void setCurrentRunId(qint64 id) { current_id_ = id; }

    const QVector<RunRecord> &records() const { return records_; }

    // Открытый прогон для сравнения (для диалога сравнения).
    bool loadRun(qint64 runId, RunSeries &series) const;

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;

private:
    RunStore *store_ = nullptr;
    QVector<RunRecord> records_;
    qint64 current_id_ = -1;
};

} // namespace gui