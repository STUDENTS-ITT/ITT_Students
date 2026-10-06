// state_panel.cpp — Панель состояния фильтра БИНС.

#include "views/state_panel.h"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QTableWidget>
#include <QVBoxLayout>

#include <cmath>

#include "core/utils/constants.h"

namespace gui
{
namespace
{
void setupTable(QTableWidget *table, const QStringList &headers)
{
    table->setColumnCount(headers.size());
    table->setHorizontalHeaderLabels(headers);
    table->verticalHeader()->setVisible(false);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setFocusPolicy(Qt::NoFocus);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    table->setMaximumHeight(160);
}

QString fmt(double v, int decimals = 4)
{
    return QString::number(v, 'f', decimals);
}
} // namespace

StatePanel::StatePanel(QWidget *parent) : QWidget(parent)
{
    auto *root = new QVBoxLayout(this);
    root->setSpacing(6);

    auto *header = new QWidget;
    auto *h = new QHBoxLayout(header);
    h->setContentsMargins(0, 0, 0, 0);
    time_ = new QLabel(tr("t = —"));
    step_ = new QLabel(tr("такт —"));
    status_ = new QLabel(tr("ожидание"));
    h->addWidget(time_);
    h->addWidget(step_);
    h->addStretch(1);
    h->addWidget(status_);
    root->addWidget(header);

    root->addWidget(new QLabel(tr("Навигационные параметры")));
    nav_table_ = new QTableWidget;
    setupTable(nav_table_, QStringList()
                               << tr("Параметр") << tr("Значение") << tr("Ед."));
    nav_table_->setRowCount(0);
    root->addWidget(nav_table_);

    root->addWidget(new QLabel(tr("Смещения датчиков (после коррекции)")));
    bias_table_ = new QTableWidget;
    setupTable(bias_table_, QStringList()
                               << tr("Датчик") << tr("Компонента") << tr("Оценка"));
    root->addWidget(bias_table_);

    accuracy_ = new QLabel(tr("Точность: —"));
    accuracy_->setWordWrap(true);
    root->addWidget(accuracy_);

    reset();
}

void StatePanel::reset()
{
    time_->setText(tr("t = —"));
    step_->setText(tr("такт —"));
    status_->setText(tr("ожидание"));

    nav_table_->setRowCount(0);
    bias_table_->setRowCount(0);
    accuracy_->setText(tr("Точность: —"));
}

void StatePanel::updateSample(const TelemetrySample &s)
{
    time_->setText(tr("t = %1 с").arg(s.time, 0, 'f', 3));
    step_->setText(tr("такт %1").arg(s.step_index));
    status_->setText(s.corrected ? tr("коррекция СНС") : tr("счисление"));

    // Навигационные параметры: первая строка — координаты и высота.
    struct NavRow
    {
        QString name;
        QString value;
        QString unit;
    };
    const NavRow rows[] = {
        {tr("Широта"), fmt(s.result.lat, 9), QStringLiteral("°")},
        {tr("Долгота"), fmt(s.result.lon, 9), QStringLiteral("°")},
        {tr("Высота"), fmt(s.result.alt, 3), QStringLiteral("м")},
        {tr("Курс"), fmt(s.result.heading, 4), QStringLiteral("°")},
        {tr("Тангаж"), fmt(s.result.pitch, 4), QStringLiteral("°")},
        {tr("Крен"), fmt(s.result.roll, 4), QStringLiteral("°")},
        {tr("Vn"), fmt(s.result.vn, 4), QStringLiteral("м/с")},
        {tr("Vh"), fmt(s.result.vh, 4), QStringLiteral("м/с")},
        {tr("Ve"), fmt(s.result.ve, 4), QStringLiteral("м/с")},
    };

    const int nav_count = static_cast<int>(sizeof(rows) / sizeof(rows[0]));
    if (nav_table_->rowCount() != nav_count)
    {
        nav_table_->setRowCount(nav_count);
    }
    for (int i = 0; i < nav_count; i++)
    {
        nav_table_->setItem(i, 0, new QTableWidgetItem(rows[i].name));
        nav_table_->setItem(i, 1, new QTableWidgetItem(rows[i].value));
        nav_table_->setItem(i, 2, new QTableWidgetItem(rows[i].unit));
    }

    // Смещения датчиков.
    struct BiasRow
    {
        QString sensor;
        QString axis;
        double value;
        QString unit;
    };
    const BiasRow bias[] = {
        {tr("акселерометр"), tr("X"), s.ba[0], QStringLiteral("м/с²")},
        {tr("акселерометр"), tr("Y"), s.ba[1], QStringLiteral("м/с²")},
        {tr("акселерометр"), tr("Z"), s.ba[2], QStringLiteral("м/с²")},
        {tr("гироскоп"), tr("X"), s.bg[0], QStringLiteral("рад/с")},
        {tr("гироскоп"), tr("Y"), s.bg[1], QStringLiteral("рад/с")},
        {tr("гироскоп"), tr("Z"), s.bg[2], QStringLiteral("рад/с")},
    };
    const int bias_count = static_cast<int>(sizeof(bias) / sizeof(bias[0]));
    if (bias_table_->rowCount() != bias_count) bias_table_->setRowCount(bias_count);
    for (int i = 0; i < bias_count; i++)
    {
        bias_table_->setItem(i, 0, new QTableWidgetItem(bias[i].sensor));
        bias_table_->setItem(i, 1, new QTableWidgetItem(bias[i].axis));
        bias_table_->setItem(i, 2,
                             new QTableWidgetItem(QString("%1 %2")
                                                      .arg(fmt(bias[i].value, 6))
                                                      .arg(bias[i].unit)));
    }

    // Оценка точности по диагоналям P: СКО позиции (м) и СКВ углов (град).
    if (s.p_diag.size() >= 3)
    {
        const double sxy_m = std::sqrt(std::max(0.0, s.p_diag[0])) * R_EARTH * RAD_TO_DEG;
        const double sxy_alt = std::sqrt(std::max(0.0, s.p_diag[2]));
        const double shdg = std::sqrt(std::max(0.0, s.p_diag[6])) * RAD_TO_DEG;
        const double spitch = std::sqrt(std::max(0.0, s.p_diag[7])) * RAD_TO_DEG;
        const double sroll = std::sqrt(std::max(0.0, s.p_diag[8])) * RAD_TO_DEG;
        accuracy_->setText(tr("СКО (из P): горизонт %1 м, высота %2 м; курс %3°, "
                              "тангаж %4°, крен %5°")
                               .arg(sxy_m, 0, 'f', 2)
                               .arg(sxy_alt, 0, 'f', 2)
                               .arg(shdg, 0, 'f', 3)
                               .arg(spitch, 0, 'f', 3)
                               .arg(sroll, 0, 'f', 3));
    }
}

} // namespace gui