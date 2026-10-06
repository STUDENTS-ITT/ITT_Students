// params_panel.cpp — Панель параметров фильтра и истории прогонов.

#include "views/params_panel.h"

#include <QAbstractItemView>
#include <QFile>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QSortFilterProxyModel>
#include <QTabWidget>
#include <QTableView>
#include <QVBoxLayout>

#include <fstream>
#include <string>

namespace gui
{

ParamsPanel::ParamsPanel(QWidget *parent) : QWidget(parent)
{
    auto *root = new QVBoxLayout(this);

    tabs_ = new QTabWidget;

    // --- Параметры фильтра ---
    model_ = new ParamsTableModel(this);
    params_view_ = new QTableView;
    params_view_->setModel(model_);
    params_view_->setAlternatingRowColors(true);
    params_view_->verticalHeader()->setVisible(false);
    params_view_->horizontalHeader()->setStretchLastSection(true);
    params_view_->horizontalHeader()->setSectionResizeMode(ParamsTableModel::ColName,
                                                            QHeaderView::ResizeToContents);
    params_view_->horizontalHeader()->setSectionResizeMode(ParamsTableModel::ColUnit,
                                                            QHeaderView::ResizeToContents);
    params_view_->setColumnWidth(ParamsTableModel::ColDescription, 220);
    connect(params_view_, &QAbstractItemView::doubleClicked, this, &ParamsPanel::applyToCore);
    tabs_->addTab(params_view_, tr("Параметры"));

    // --- История прогонов ---
    history_ = new HistoryModel(this);

    // Прокси-модель даёт сортировку по клику на заголовок и фильтр по имени.
    // HistoryModel при этом ничего не знает про представление: она остаётся
    // источником данных (замена на QSqlTableModel её не затронет).
    history_proxy_ = new QSortFilterProxyModel(this);
    history_proxy_->setSourceModel(history_);
    history_proxy_->setSortCaseSensitivity(Qt::CaseInsensitive);
    history_proxy_->setFilterCaseSensitivity(Qt::CaseInsensitive);
    // Фильтр ищет по всем видимым колонкам: имя, дата, каталог.
    history_proxy_->setFilterKeyColumn(-1);

    auto *history_page = new QWidget;
    auto *history_layout = new QVBoxLayout(history_page);
    history_layout->setContentsMargins(0, 0, 0, 0);

    history_filter_ = new QLineEdit;
    history_filter_->setPlaceholderText(tr("Фильтр по названию или каталогу…"));
    history_filter_->setClearButtonEnabled(true);
    history_layout->addWidget(history_filter_);

    history_view_ = new QTableView;
    history_view_->setModel(history_proxy_);
    history_view_->setAlternatingRowColors(true);
    history_view_->verticalHeader()->setVisible(false);
    history_view_->horizontalHeader()->setStretchLastSection(true);
    history_view_->setSortingEnabled(true);
    history_view_->sortByColumn(HistoryModel::ColCreated, Qt::DescendingOrder);
    history_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
    history_view_->setSelectionMode(QAbstractItemView::SingleSelection);
    history_layout->addWidget(history_view_, 1);

    connect(history_filter_, &QLineEdit::textChanged, this, [this](const QString &text) {
        history_proxy_->setFilterFixedString(text);
    });
    connect(history_view_, &QAbstractItemView::doubleClicked, this, [this](const QModelIndex &idx) {
        if (!idx.isValid()) return;
        // Строка в прокси не совпадает со строкой источника: переводим индекс.
        const QModelIndex source = history_proxy_->mapToSource(idx);
        const int row = source.row();
        if (row < 0 || row >= history_->records().size()) return;
        Q_EMIT runSelected(history_->records()[row].id);
    });
    tabs_->addTab(history_page, tr("История"));

    root->addWidget(tabs_, 1);

    hint_ = new QLabel(tr("Двойной щелчок по значению — применить параметры к фильтру. "
                          "Текущие значения записываются в settings.ini при сохранении."));
    hint_->setWordWrap(true);
    root->addWidget(hint_);

    connect(model_, &QAbstractItemModel::dataChanged, this, [this]() {
        Q_EMIT settingsEdited();
    });
}

void ParamsPanel::setModel(ParamsTableModel *model)
{
    model_ = model;
    params_view_->setModel(model);
}

void ParamsPanel::setHistoryModel(HistoryModel *model)
{
    history_ = model;
    history_proxy_->setSourceModel(model);
}

nav::FilterSettings ParamsPanel::currentSettings() const
{
    return model_->settings();
}

bool ParamsPanel::loadSettings(const QString &fileName, QString *error)
{
    nav::FilterSettings settings;
    const std::string path = fileName.toStdString();

    if (!nav::loadFilterSettings(path, settings))
    {
        if (error) *error = tr("Не удалось прочитать %1").arg(fileName);
        return false;
    }

    model_->setSettings(settings);
    Q_EMIT settingsEdited();
    return true;
}

bool ParamsPanel::saveSettings(const QString &fileName, QString *error)
{
    const nav::FilterSettings settings = currentSettings();
    const std::string text = nav::dumpFilterSettings(settings);

    std::ofstream out(fileName.toStdString(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        if (error) *error = tr("Не удалось записать %1").arg(fileName);
        return false;
    }
    out << text;
    out.close();
    return true;
}

void ParamsPanel::resetToDefaults()
{
    model_->setSettings(nav::FilterSettings{});
    Q_EMIT settingsEdited();
}

void ParamsPanel::applyToCore()
{
    model_->applyToCore();
    Q_EMIT settingsEdited();
}

} // namespace gui