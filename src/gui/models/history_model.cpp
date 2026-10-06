// history_model.cpp — Таблица истории прогонов.

#include "models/history_model.h"

#include <QLocale>

#include "core/utils/constants.h"

namespace gui
{

HistoryModel::HistoryModel(QObject *parent) : QAbstractTableModel(parent) {}

void HistoryModel::setStore(RunStore *store)
{
    store_ = store;
    refresh();
}

void HistoryModel::clear()
{
    beginResetModel();
    records_.clear();
    current_id_ = -1;
    endResetModel();
}

void HistoryModel::refresh()
{
    beginResetModel();
    records_ = store_ != nullptr ? store_->recentRuns(200) : QVector<RunRecord>();
    endResetModel();
}

bool HistoryModel::loadRun(qint64 runId, RunSeries &series) const
{
    if (store_ == nullptr) return false;

    series.clear();

    std::vector<double> times;
    std::vector<std::vector<double>> errors;
    if (!store_->loadErrors(runId, times, errors)) return false;

    for (std::size_t k = 0; k < times.size(); k++)
    {
        RunSeries::CorrectionRow row;
        row.time = times[k];
        for (int i = 0; i < ins::KF_STATE && i < static_cast<int>(errors[k].size()); i++)
        {
            row.x[i] = errors[k][i];
        }
        series.corrections.push_back(row);
    }

    const QVector<SampleRow> samples = store_->loadSamples(runId);
    series.rows.reserve(static_cast<std::size_t>(samples.size()));

    // Диагонали P лежат в своей таблице и по времени не обязаны совпадать
    // с шагами samples — подставляем их в строки по ближайшему времени.
    std::vector<double> covTimes;
    std::vector<std::vector<double>> covDiag;
    store_->loadCovariances(runId, covTimes, covDiag);
    std::size_t covCursor = 0;

    for (const SampleRow &s : samples)
    {
        RunSeries::Row row;
        row.time = s.t;
        row.result[RunSeries::Time] = s.t;
        row.result[RunSeries::Lon] = s.lon;
        row.result[RunSeries::Lat] = s.lat;
        row.result[RunSeries::Alt] = s.alt;
        row.result[RunSeries::Heading] = s.heading;
        row.result[RunSeries::Pitch] = s.pitch;
        row.result[RunSeries::Roll] = s.roll;
        row.result[RunSeries::Vn] = s.vn;
        row.result[RunSeries::Vh] = s.vh;
        row.result[RunSeries::Ve] = s.ve;

        while (covCursor < covTimes.size() && covTimes[covCursor] < s.t) ++covCursor;
        if (covCursor < covTimes.size() &&
            qAbs(covTimes[covCursor] - s.t) <= 1e-6 * (1.0 + qAbs(s.t)))
        {
            const std::vector<double> &p = covDiag[covCursor];
            for (int i = 0; i < ins::KF_STATE && i < static_cast<int>(p.size()); i++)
            {
                row.p[i] = p[i];
            }
        }

        series.rows.push_back(row);
    }

    return !series.rows.empty() || !series.corrections.empty();
}

int HistoryModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : records_.size();
}

int HistoryModel::columnCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : ColCount;
}

QVariant HistoryModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid()) return QVariant();
    const int row = index.row();
    if (row < 0 || row >= records_.size()) return QVariant();

    const RunRecord &r = records_[row];

    if (role == Qt::TextAlignmentRole)
    {
        return static_cast<int>(Qt::AlignRight | Qt::AlignVCenter);
    }

    if (role != Qt::DisplayRole) return QVariant();

    switch (index.column())
    {
    case ColId: return r.id;
    case ColName: return r.name;
    case ColCreated: return r.created.toString("yyyy-MM-dd hh:mm:ss");
    case ColSteps: return r.steps;
    case ColCorrections: return r.corrections;
    case ColWall: return r.wall_seconds;
    case ColDataDir: return r.data_dir;
    default: return QVariant();
    }
}

QVariant HistoryModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return QVariant();

    switch (section)
    {
    case ColId: return tr("ID");
    case ColName: return tr("Прогон");
    case ColCreated: return tr("Дата");
    case ColSteps: return tr("Такты");
    case ColCorrections: return tr("Коррекции");
    case ColWall: return tr("Счёт, с");
    case ColDataDir: return tr("Каталог данных");
    default: return QVariant();
    }
}

} // namespace gui