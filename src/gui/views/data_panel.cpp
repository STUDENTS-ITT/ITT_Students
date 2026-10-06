// data_panel.cpp — Панель входных данных и таблица результатов.

#include "views/data_panel.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSplitter>
#include <QTableView>
#include <QTreeView>
#include <QVBoxLayout>

namespace gui
{

TimeFilterProxy::TimeFilterProxy(QObject *parent) : QSortFilterProxyModel(parent) {}

void TimeFilterProxy::setTimeWindow(double t_min, double t_max)
{
    if (t_min_ == t_min && t_max_ == t_max) return;
    t_min_ = t_min;
    t_max_ = t_max;
    // Qt 6.12: invalidateFilter() устарел — пересчёт выполняет пара
    // begin/endFilterChange().
    beginFilterChange();
    endFilterChange();
}

bool TimeFilterProxy::filterAcceptsRow(int row, const QModelIndex &parent) const
{
    if (!QSortFilterProxyModel::filterAcceptsRow(row, parent)) return false;
    if (t_min_ < 0.0 && t_max_ < 0.0) return true;

    const QAbstractItemModel *src = sourceModel();
    if (src == nullptr) return true;

    const QModelIndex idx = src->index(row, NavTableModel::ColTime, parent);
    if (!idx.isValid()) return true;

    const double t = src->data(idx, Qt::DisplayRole).toDouble();
    if (t_min_ >= 0.0 && t < t_min_) return false;
    if (t_max_ >= 0.0 && t > t_max_) return false;
    return true;
}

DataPanel::DataPanel(QWidget *parent) : QWidget(parent)
{
    auto *root = new QVBoxLayout(this);

    auto *splitter = new QSplitter(Qt::Horizontal);

    // --- Левая часть: дерево входных файлов ---
    auto *left = new QWidget;
    auto *left_layout = new QVBoxLayout(left);
    left_layout->setContentsMargins(0, 0, 0, 0);

    left_layout->addWidget(new QLabel(tr("Входные данные")));

    tree_ = new InputTreeModel(this);
    tree_view_ = new QTreeView;
    tree_view_->setModel(tree_);
    tree_view_->setRootIsDecorated(true);
    tree_view_->setAlternatingRowColors(true);
    tree_view_->header()->setStretchLastSection(true);
    tree_view_->expandAll();
    left_layout->addWidget(tree_view_, 1);

    data_info_ = new QLabel(tr("—"));
    data_info_->setWordWrap(true);
    left_layout->addWidget(data_info_);

    splitter->addWidget(left);

    // --- Правая часть: таблица результатов ---
    auto *right = new QWidget;
    auto *right_layout = new QVBoxLayout(right);
    right_layout->setContentsMargins(0, 0, 0, 0);

    auto *tools = new QHBoxLayout;

    tools->addWidget(new QLabel(tr("Поиск:")));
    search_ = new QLineEdit;
    search_->setPlaceholderText(tr("координаты, высота, курс…"));
    search_->setClearButtonEnabled(true);
    tools->addWidget(search_, 1);

    t_min_ = new QDoubleSpinBox;
    t_min_->setRange(0.0, 1e6);
    t_min_->setDecimals(1);
    t_min_->setSuffix(tr(" с"));
    t_min_->setSpecialValueText(tr("с"));
    t_min_->setMinimumWidth(90);
    tools->addWidget(new QLabel(tr("с")));
    tools->addWidget(t_min_);

    t_max_ = new QDoubleSpinBox;
    t_max_->setRange(0.0, 1e6);
    t_max_->setDecimals(1);
    t_max_->setSuffix(tr(" с"));
    t_max_->setSpecialValueText(tr("по"));
    t_max_->setMaximumWidth(90);
    tools->addWidget(new QLabel(tr("по")));
    tools->addWidget(t_max_);

    auto *reset_btn = new QPushButton(tr("Сбросить"));
    tools->addWidget(reset_btn);

    right_layout->addLayout(tools);

    proxy_ = new TimeFilterProxy(this);
    table_ = new QTableView;
    table_->setModel(proxy_);
    table_->setSortingEnabled(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setAlternatingRowColors(true);
    table_->verticalHeader()->setVisible(false);
    table_->horizontalHeader()->setStretchLastSection(true);
    right_layout->addWidget(table_, 1);

    row_info_ = new QLabel(tr("нет данных"));
    right_layout->addWidget(row_info_);

    splitter->addWidget(right);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 3);

    root->addWidget(splitter, 1);

    connect(search_, &QLineEdit::textChanged, this, &DataPanel::onFilterChanged);
    connect(t_min_, &QDoubleSpinBox::valueChanged, this, &DataPanel::onFilterChanged);
    connect(t_max_, &QDoubleSpinBox::valueChanged, this, &DataPanel::onFilterChanged);
    connect(reset_btn, &QPushButton::clicked, this, &DataPanel::resetFilter);
}

void DataPanel::setDataDir(const QString &dir)
{
    tree_->setDataDir(dir);
    tree_view_->expandAll();

    const qint64 imu_rows = tree_->imuRowCount();
    QStringList present;
    for (const InputFile &f : tree_->files())
    {
        if (f.exists) present << f.name;
    }
    data_info_->setText(
        tr("Каталог: %1\nИМУ: %2 строк (≈ %3 с)\nНайдено: %4")
            .arg(dir)
            .arg(imu_rows)
            .arg(static_cast<double>(imu_rows) / 200.0, 0, 'f', 1)
            .arg(present.isEmpty() ? tr("ничего") : present.join(", ")));

    Q_EMIT dataDirChanged(dir);
}

void DataPanel::setNavModel(NavTableModel *model)
{
    model_ = model;
    proxy_->setSourceModel(model);
    table_->setModel(proxy_);
    if (model != nullptr && table_->horizontalHeader()->count() != model->columnCount())
    {
        table_->resizeColumnsToContents();
    }
    onFilterChanged();
}

void DataPanel::setTimeWindow(double t_min, double t_max)
{
    if (model_ == nullptr) return;

    t_min_->setValue(t_min);
    t_max_->setValue(t_max);

    // Модель прорежена до 200 Гц — при 10 минутах это 120 000 строк, таблица
    // справится, но для комфортной прокрутки оставляем как есть.
    setRowInfo(tr("строк: %1, интервал %2…%3 с")
                   .arg(model_->rowCount())
                   .arg(t_min, 0, 'f', 1)
                   .arg(t_max, 0, 'f', 1));
}

void DataPanel::clearTimeFilter()
{
    t_min_->setValue(0.0);
    t_max_->setValue(0.0);
    resetFilter();
}

void DataPanel::setRowInfo(const QString &text)
{
    row_info_->setText(text);
}

void DataPanel::resetFilter()
{
    search_->clear();
    t_min_->setValue(0.0);
    t_max_->setValue(0.0);
    onFilterChanged();
}

void DataPanel::onFilterChanged()
{
    if (model_ == nullptr)
    {
        setRowInfo(tr("нет данных"));
        return;
    }

    const QString needle = search_->text().trimmed();

    proxy_->setFilterCaseSensitivity(Qt::CaseInsensitive);
    proxy_->setFilterKeyColumn(-1);
    proxy_->setFilterFixedString(needle);

    // Окно времени: нули означают «без ограничения» (спецзначение спинбокса).
    proxy_->setTimeWindow(t_min_->value() > 0.0 ? t_min_->value() : -1.0,
                          t_max_->value() > 0.0 ? t_max_->value() : -1.0);

    const bool filtered = !needle.isEmpty() || t_min_->value() > 0.0 || t_max_->value() > 0.0;
    setRowInfo(tr("строк: %1 из %2%3")
                   .arg(proxy_->rowCount())
                   .arg(model_->rowCount())
                   .arg(filtered ? tr(", фильтр применён") : QString()));
}

} // namespace gui