// params_table_model.h — Редактируемая таблица параметров фильтра.
//
// Показывает FilterSettings / TiltManeuverPolicy из ядра: имя (ключ INI),
// значение, единицы измерения, описание. Правки идут напрямую в
// FilterSettings, из них же собирается settings.ini.

#pragma once

#include <QAbstractTableModel>
#include <QString>
#include <QVector>

#include "core/navigation/filter_settings.h"
#include "core/navigation/maneuver_tilt.h"

namespace gui
{

class ParamsTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum Column
    {
        ColName = 0,
        ColValue,
        ColUnit,
        ColDescription,
        ColCount
    };

    explicit ParamsTableModel(QObject *parent = nullptr);

    void setSettings(const nav::FilterSettings &settings);
    const nav::FilterSettings &settings() const { return settings_; }

    // Применить текущие значения из модели к ядру (ins::kalman_cfg и tiltPolicy).
    void applyToCore();

    // Обработать правку ячейки.
    bool setValueAt(int row, double value);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    bool setData(const QModelIndex &index, const QVariant &value,
                 int role = Qt::EditRole) override;

    // Ключ параметра в формате group/key (для INI).
    QString keyOf(int row) const;

private:
    struct Item
    {
        QString group;   // "filter" или "tilt"
        QString key;     // полный ключ, например "filter/sigma_g"
        double *target = nullptr;   // куда писать значение
        double value = 0;
        QString unit;
        QString description;
    };

    void rebuild();

    QVector<Item> items_;
    nav::FilterSettings settings_{};
};

} // namespace gui