// params_table_model.cpp — Таблица параметров фильтра и tilt-политики.

#include "models/params_table_model.h"

#include <cmath>

namespace gui
{

ParamsTableModel::ParamsTableModel(QObject *parent) : QAbstractTableModel(parent)
{
    rebuild();
}

void ParamsTableModel::setSettings(const nav::FilterSettings &settings)
{
    settings_ = settings;
    rebuild();
}

void ParamsTableModel::rebuild()
{
    beginResetModel();
    items_.clear();

    const std::vector<nav::SettingDescriptor> desc = nav::filterSettingDescriptors();

    // Описания ядра дают ключ/подпись/единицы/диапазон, а указатели на поля
    // берём из структур: тот же список, что использует loadFilterSettings.
    for (const nav::SettingDescriptor &d : desc)
    {
        Item item;
        item.key = QString::fromStdString(d.key);
        item.unit = QString::fromStdString(d.units);
        item.description = QString::fromStdString(d.label);
        items_.push_back(item);
    }

    auto bind = [this](const char *key, double *field) {
        for (Item &item : items_)
        {
            if (item.key == QLatin1String(key))
            {
                item.target = field;
                item.value = *field;
                return;
            }
        }
    };

    // ins::KalmanConfig
    bind("filter/sigma_g", &settings_.filter.sig_g);
    bind("filter/sigma_a", &settings_.filter.sig_a);
    bind("filter/sigma_bg", &settings_.filter.sig_bg);
    bind("filter/sigma_ba", &settings_.filter.sig_ba);
    bind("filter/sigma_pos", &settings_.filter.sig_pos);
    bind("filter/sigma_h", &settings_.filter.sig_h);
    bind("filter/sigma_v", &settings_.filter.sig_v);
    bind("filter/sigma_hdg", &settings_.filter.sig_hdg);
    bind("filter/outage_start_s", &settings_.filter.outage_start_s);
    bind("filter/outage_end_s", &settings_.filter.outage_end_s);

    // TiltManeuverPolicy
    bind("tilt/gyro_off_rad_s", &settings_.tilt.gyro_off_rad_s);
    bind("tilt/settle_s", &settings_.tilt.settle_s);
    bind("tilt/brake_body_accel", &settings_.tilt.brake_body_accel);
    bind("tilt/tau_fast_s", &settings_.tilt.tau_fast_s);
    bind("tilt/tau_slow_s", &settings_.tilt.tau_slow_s);
    bind("tilt/fast_err_rad", &settings_.tilt.fast_err_rad);
    bind("tilt/level_acc_rad", &settings_.tilt.level_acc_rad);
    bind("tilt/bias_gain", &settings_.tilt.bias_gain);
    bind("tilt/bias_max_rad_s", &settings_.tilt.bias_max_rad_s);
    bind("tilt/divergence_deadband_rad", &settings_.tilt.divergence_deadband_rad);

    // run
    bind("run/align_time_override_s", &settings_.align_time_override_s);
    bind("run/align_fallback_s", &settings_.align_fallback_s);

    endResetModel();
}

void ParamsTableModel::applyToCore()
{
    nav::applySettings(settings_);
}

bool ParamsTableModel::setValueAt(int row, double value)
{
    if (row < 0 || row >= items_.size()) return false;
    Item &item = items_[row];
    if (item.target == nullptr) return false;

    *item.target = value;
    item.value = value;
    return true;
}

int ParamsTableModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : items_.size();
}

int ParamsTableModel::columnCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : ColCount;
}

QVariant ParamsTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid()) return QVariant();
    const int row = index.row();
    if (row < 0 || row >= items_.size()) return QVariant();
    const Item &item = items_[row];

    switch (role)
    {
    case Qt::DisplayRole:
    case Qt::EditRole:
        switch (index.column())
        {
        case ColName: return item.key;
        case ColValue: return item.value;
        case ColUnit: return item.unit;
        case ColDescription: return item.description;
        default: return QVariant();
        }

    case Qt::TextAlignmentRole:
        if (index.column() == ColValue) return static_cast<int>(Qt::AlignRight | Qt::AlignVCenter);
        return QVariant();

    case Qt::ToolTipRole:
        if (index.column() == ColValue) return item.key;
        return QVariant();

    default:
        return QVariant();
    }
}

QVariant ParamsTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return QVariant();

    switch (section)
    {
    case ColName: return tr("Ключ");
    case ColValue: return tr("Значение");
    case ColUnit: return tr("Ед.");
    case ColDescription: return tr("Описание");
    default: return QVariant();
    }
}

Qt::ItemFlags ParamsTableModel::flags(const QModelIndex &index) const
{
    if (!index.isValid()) return Qt::NoItemFlags;

    Qt::ItemFlags f = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
    if (index.column() == ColValue && index.row() < items_.size() &&
        items_[index.row()].target != nullptr)
    {
        f |= Qt::ItemIsEditable;
    }
    return f;
}

bool ParamsTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (!index.isValid()) return false;
    if (role != Qt::EditRole && role != Qt::DisplayRole) return false;
    if (index.column() != ColValue) return false;

    bool ok = false;
    const double v = value.toDouble(&ok);
    if (!ok) return false;

    const int row = index.row();
    if (!setValueAt(row, v)) return false;

    Q_EMIT dataChanged(index.sibling(index.row(), ColValue),
                       index.sibling(index.row(), ColDescription));
    return true;
}

QString ParamsTableModel::keyOf(int row) const
{
    if (row < 0 || row >= items_.size()) return QString();
    return items_[row].key;
}

} // namespace gui