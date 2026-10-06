// params_panel.h — Панель параметров фильтра.
//
// Таблица значений FilterSettings / TiltManeuverPolicy, загрузка и сохранение
// settings.ini, сброс к значениям по умолчанию. Здесь же — история прогонов
// (SQLite) для сравнения результатов.

#pragma once

#include <QWidget>

#include "core/navigation/filter_settings.h"
#include "models/history_model.h"
#include "models/params_table_model.h"
#include "settings/app_settings.h"

class QLineEdit;
class QSortFilterProxyModel;
class QTabWidget;
class QTableView;
class QLabel;

namespace gui
{

class ParamsPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ParamsPanel(QWidget *parent = nullptr);

    void setModel(ParamsTableModel *model);
    void setHistoryModel(HistoryModel *model);

    // Текущие параметры (после правок в таблице).
    nav::FilterSettings currentSettings() const;

    // Загрузить из settings.ini. false — файл не найден или не разобран.
    bool loadSettings(const QString &fileName, QString *error = nullptr);
    // Сохранить в settings.ini.
    bool saveSettings(const QString &fileName, QString *error = nullptr);

public Q_SLOTS:
    void resetToDefaults();
    void applyToCore();

Q_SIGNALS:
    void settingsEdited();
    void runSelected(qint64 run_id);

private:
    QTabWidget *tabs_ = nullptr;
    ParamsTableModel *model_ = nullptr;
    QTableView *params_view_ = nullptr;
    HistoryModel *history_ = nullptr;
    // Сортировка и фильтр по имени прогона живут в прокси-модели: сама
    // HistoryModel остаётся источником данных без логики представления.
    QSortFilterProxyModel *history_proxy_ = nullptr;
    QTableView *history_view_ = nullptr;
    QLineEdit *history_filter_ = nullptr;
    QLabel *hint_ = nullptr;
};

} // namespace gui