// nav_table_model.h — Табличная модель результатов навигации.
//
// Источник — RunSeries (ряды телеметрии текущего прогона) и строки,
// загруженные из SQLite для сохранённых прогонов. Модель хранит данные
// копией, чтобы отрисовка и экспорт не зависели от фонового потока.

#pragma once

#include <QAbstractTableModel>
#include <QVector>

#include "core_bridge/run_store.h"
#include "core_bridge/telemetry_buffer.h"

namespace gui
{

class NavTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum Column
    {
        ColTime = 0,
        ColLon,
        ColLat,
        ColAlt,
        ColHeading,
        ColPitch,
        ColRoll,
        ColVn,
        ColVh,
        ColVe,
        ColLatErr,
        ColLonErr,
        ColAltErr,
        ColHdgErr,
        ColCount
    };

    explicit NavTableModel(QObject *parent = nullptr);

    // Заменить данные результатами текущего прогона.
    void setSeries(const RunSeries &series);
    void setSeries(const RunSeries &series, const RunSeries *reference);
    void clear();

    // Загрузить сохранённый прогон из SQLite.
    void loadFromStore(const RunStore &store, qint64 runId);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;

    // Строки для экспорта в CSV.
    QString toCsv() const;

private:
    struct Record
    {
        double time = 0;
        double lon = 0, lat = 0, alt = 0;
        double heading = 0, pitch = 0, roll = 0;
        double vn = 0, vh = 0, ve = 0;
        bool has_ref = false;
        double ref_lon = 0, ref_lat = 0, ref_alt = 0, ref_heading = 0;
    };

    QVector<Record> records_;
};

} // namespace gui