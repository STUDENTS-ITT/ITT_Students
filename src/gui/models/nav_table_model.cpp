// nav_table_model.cpp — Табличная модель результатов навигации.

#include "models/nav_table_model.h"

#include <QStringList>

#include "core/utils/constants.h"

namespace gui
{

NavTableModel::NavTableModel(QObject *parent) : QAbstractTableModel(parent) {}

void NavTableModel::clear()
{
    beginResetModel();
    records_.clear();
    endResetModel();
}

void NavTableModel::setSeries(const RunSeries &series)
{
    beginResetModel();
    records_.clear();
    records_.reserve(static_cast<int>(series.rows.size()));

    for (const RunSeries::Row &r : series.rows)
    {
        Record rec;
        rec.time = r.time;
        rec.lon = r.result[RunSeries::Lon];
        rec.lat = r.result[RunSeries::Lat];
        rec.alt = r.result[RunSeries::Alt];
        rec.heading = r.result[RunSeries::Heading];
        rec.pitch = r.result[RunSeries::Pitch];
        rec.roll = r.result[RunSeries::Roll];
        rec.vn = r.result[RunSeries::Vn];
        rec.vh = r.result[RunSeries::Vh];
        rec.ve = r.result[RunSeries::Ve];
        if (r.has_reference)
        {
            rec.has_ref = true;
            rec.ref_lon = r.reference[RunSeries::Lon];
            rec.ref_lat = r.reference[RunSeries::Lat];
            rec.ref_alt = r.reference[RunSeries::Alt];
            rec.ref_heading = r.reference[RunSeries::Heading];
        }
        records_.push_back(rec);
    }
    endResetModel();
}

void NavTableModel::setSeries(const RunSeries &series, const RunSeries *reference)
{
    Q_UNUSED(reference);
    setSeries(series);
}

void NavTableModel::loadFromStore(const RunStore &store, qint64 runId)
{
    const QVector<SampleRow> rows = store.loadSamples(runId);

    beginResetModel();
    records_.clear();
    records_.reserve(rows.size());
    for (const SampleRow &r : rows)
    {
        Record rec;
        rec.time = r.t;
        rec.lon = r.lon;
        rec.lat = r.lat;
        rec.alt = r.alt;
        rec.heading = r.heading;
        rec.pitch = r.pitch;
        rec.roll = r.roll;
        rec.vn = r.vn;
        rec.vh = r.vh;
        rec.ve = r.ve;
        records_.push_back(rec);
    }
    endResetModel();
}

int NavTableModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : records_.size();
}

int NavTableModel::columnCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : ColCount;
}

QVariant NavTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid()) return QVariant();
    const int row = index.row();
    if (row < 0 || row >= records_.size()) return QVariant();

    const Record &r = records_[row];

    if (role == Qt::TextAlignmentRole)
    {
        return static_cast<int>(Qt::AlignRight | Qt::AlignVCenter);
    }
    if (role != Qt::DisplayRole) return QVariant();

    switch (index.column())
    {
    case ColTime: return r.time;
    case ColLon: return r.lon;
    case ColLat: return r.lat;
    case ColAlt: return r.alt;
    case ColHeading: return r.heading;
    case ColPitch: return r.pitch;
    case ColRoll: return r.roll;
    case ColVn: return r.vn;
    case ColVh: return r.vh;
    case ColVe: return r.ve;

    case ColLatErr:
        if (!r.has_ref) return QVariant();
        return (r.lat - r.ref_lat) * R_EARTH * RAD_TO_DEG;
    case ColLonErr:
        if (!r.has_ref) return QVariant();
        return (r.lon - r.ref_lon) * R_EARTH * cos(r.ref_lat * DEG_TO_RAD) * RAD_TO_DEG;
    case ColAltErr:
        if (!r.has_ref) return QVariant();
        return r.alt - r.ref_alt;
    case ColHdgErr:
        if (!r.has_ref) return QVariant();
        return normalize_angle((r.heading - r.ref_heading) * DEG_TO_RAD) * RAD_TO_DEG;
    default: return QVariant();
    }
}

QVariant NavTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return QVariant();

    switch (section)
    {
    case ColTime: return tr("t, с");
    case ColLon: return tr("Лон, °");
    case ColLat: return tr("Широта, °");
    case ColAlt: return tr("Высота, м");
    case ColHeading: return tr("Курс, °");
    case ColPitch: return tr("Тангаж, °");
    case ColRoll: return tr("Крен, °");
    case ColVn: return tr("Vn, м/с");
    case ColVh: return tr("Vh, м/с");
    case ColVe: return tr("Ve, м/с");
    case ColLatErr: return tr("Δшир, м");
    case ColLonErr: return tr("Δлон, м");
    case ColAltErr: return tr("Δh, м");
    case ColHdgErr: return tr("Δкурс, °");
    default: return QVariant();
    }
}

QString NavTableModel::toCsv() const
{
    QString out;
    QTextStream s(&out);

    s << "t_s,lon_deg,lat_deg,alt_m,heading_deg,pitch_deg,roll_deg,"
      << "vn,vh,ve,dlat_m,dlon_m,dalt_m,dhdg_deg\n";

    for (const Record &r : records_)
    {
        s << QString::number(r.time, 'f', 4) << ','
          << QString::number(r.lon, 'f', 9) << ','
          << QString::number(r.lat, 'f', 9) << ','
          << QString::number(r.alt, 'f', 3) << ','
          << QString::number(r.heading, 'f', 6) << ','
          << QString::number(r.pitch, 'f', 6) << ','
          << QString::number(r.roll, 'f', 6) << ','
          << QString::number(r.vn, 'f', 4) << ','
          << QString::number(r.vh, 'f', 4) << ','
          << QString::number(r.ve, 'f', 4);

        if (r.has_ref)
        {
            s << ','
              << QString::number((r.lat - r.ref_lat) * R_EARTH * RAD_TO_DEG, 'f', 3) << ','
              << QString::number((r.lon - r.ref_lon) * R_EARTH *
                                     cos(r.ref_lat * DEG_TO_RAD) * RAD_TO_DEG, 'f', 3) << ','
              << QString::number(r.alt - r.ref_alt, 'f', 3) << ','
              << QString::number(normalize_angle((r.heading - r.ref_heading) * DEG_TO_RAD) *
                                     RAD_TO_DEG, 'f', 6);
        }
        s << '\n';
    }
    return out;
}

} // namespace gui