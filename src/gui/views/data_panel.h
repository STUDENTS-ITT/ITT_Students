// data_panel.h — Панель входных данных и результатов.
//
// Слева — дерево входных файлов (imu.dat / gps.dat / angle.dat /
// StartupNav.ini) с размерами и числом строк. Справа — таблица результатов
// с поиском по значениям и фильтром по интервалу времени.

#pragma once

#include <QSortFilterProxyModel>
#include <QWidget>

#include "core_bridge/input_tree.h"
#include "models/nav_table_model.h"

class QLabel;
class QLineEdit;
class QTableView;
class QDoubleSpinBox;
class QTreeView;

namespace gui
{

// Прокси таблицы результатов: текстовый фильтр по любой ячейке плюс
// ограничение по времени.
class TimeFilterProxy : public QSortFilterProxyModel
{
    Q_OBJECT

public:
    explicit TimeFilterProxy(QObject *parent = nullptr);

    // Окно времени, с. Отрицательное значение — без ограничения.
    void setTimeWindow(double t_min, double t_max);

protected:
    bool filterAcceptsRow(int row, const QModelIndex &parent) const override;

private:
    double t_min_ = -1.0;
    double t_max_ = -1.0;
};

class DataPanel : public QWidget
{
    Q_OBJECT

public:
    explicit DataPanel(QWidget *parent = nullptr);

    void setDataDir(const QString &dir);
    QString dataDir() const { return tree_->dataDir(); }

    void setNavModel(NavTableModel *model);
    NavTableModel *navModel() const { return model_; }

    // Отфильтровать таблицу по интервалу времени, с (отрицательные — без
    // ограничения).
    void setTimeWindow(double t_min, double t_max);
    void clearTimeFilter();

    // Информация о числе строк.
    void setRowInfo(const QString &text);

Q_SIGNALS:
    void dataDirChanged(const QString &dir);

private Q_SLOTS:
    void onFilterChanged();
    void resetFilter();

private:
    InputTreeModel *tree_ = nullptr;
    QTreeView *tree_view_ = nullptr;
    QLabel *data_info_ = nullptr;

    NavTableModel *model_ = nullptr;
    QTableView *table_ = nullptr;
    QLineEdit *search_ = nullptr;
    QDoubleSpinBox *t_min_ = nullptr;
    QDoubleSpinBox *t_max_ = nullptr;
    QLabel *row_info_ = nullptr;
    TimeFilterProxy *proxy_ = nullptr;
};

} // namespace gui