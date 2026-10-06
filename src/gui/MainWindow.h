// MainWindow.h — Главное окно imitator_gui.
//
// Собирает все панели: управление прогоном, состояние фильтра, графики
// (собственный QPainter), таблицу результатов, параметры и лог. Прогон ядра
// выполняется в отдельном QThread (SimulationWorker), приём телеметрии —
// через потокобезопасный буфер с отрисовкой 25 раз в секунду.

#pragma once

#include <QMainWindow>
#include <QScopedPointer>

#include "core_bridge/run_store.h"
#include "core_bridge/telemetry_buffer.h"
#include "export/pdf_report.h"
#include "settings/app_settings.h"

class QAction;
class QTabWidget;
class QTimer;
class QThread;

namespace gui
{

class ControlPanel;
class DataPanel;
class HistoryModel;
class LogPanel;
class NavTableModel;
class ParamsPanel;
class ParamsTableModel;
class PlotView;
struct PlotSeries;
class SimulationWorker;
class StatePanel;
class TimeSeriesPlot;
class TrajectoryPlot;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // Настройки передаются из main(), где разобрана командная строка.
    // autostart — запустить расчёт сразу после открытия окна.
    explicit MainWindow(const AppSettings &settings, bool autostart = false,
                        QWidget *parent = nullptr);
    ~MainWindow() override;

    // Текст панели лога (для отчёта --self-test и диагностики).
    QString logText() const;

    // Прогон, сохранённый в текущем сеансе (-1 — ещё не сохранялся).
    qint64 currentRunId() const { return current_run_id_; }

    // Открыть прогон из истории SQLite. Возвращает число загруженных строк
    // (0 — прогон не найден или база закрыта).
    int openRun(qint64 runId);

    // Сборка сводного PDF-отчёта в файл без диалога (используется self-test).
    bool exportReportTo(const QString &path, QString *error = nullptr);

    // Второй прогон для сравнения с текущим (план 2.4): его ряды добавляются
    // на графики пунктиром, а по запросу — их разность с текущим прогоном.
    // Возвращает число загруженных строк (0 — прогон не найден).
    int openCompareRun(qint64 runId);
    bool hasComparison() const
    {
        return compare_run_id_ >= 0 && !series_b_.rows.empty();
    }
    qint64 compareRunId() const { return compare_run_id_; }

protected:
    void closeEvent(QCloseEvent *event) override;

private Q_SLOTS:
    // Управление прогоном.
    void onStart();
    void onStop();
    void onPauseToggled(bool paused);
    void onSpeedFactorChanged(double factor);
    void onWorkerFinished(bool ok, const QString &message);

    // Приём данных из рабочего потока.
    void onWorkerProgress(int percent, double time);
    void onWorkerLog(const QString &text, int level);
    void onWorkerState(double time, int step, const QString &diag);
    void onAlignment();

    // Таймер отрисовки: забирает телеметрию и обновляет графики.
    void onRefresh();

    // Параметры и настройки.
    void onSettingsChanged();
    void onLoadSettingsFile();
    void onSaveSettingsFile();
    void onResetParams();
    void onApplyParams();

    // Инструменты.
    void onExportPlot();
    void onExportSvg();
    void onExportPdf();
    void onExportReport();
    void onExportCsv();
    void onSaveLog();
    void onToggleTheme(bool dark);
    void onResetZoom();
    void onSelectPlot(int index);

    // История прогонов.
    void onRefreshHistory();
    void onOpenRun(qint64 runId);
    void onCompareSelectRun();
    void onCompareDiffToggled(bool on);
    void onCompareClear();

Q_SIGNALS:
    // Прогон завершён (успешно, с ошибкой или по команде остановки).
    // Используется headless-режимом --self-test.
    void runFinished(bool ok, const QString &message);

private:
    void buildUi();
    void buildActions();
    void buildPlots();
    void connectSignals();

    // Построение рядов графиков из накопленной телеметрии.
    void rebuildPlotSeries(int tab);

    // Добавление рядов второго прогона (пунктир) и, если включено, разности
    // «текущий минус второй» на том же графике.
    void appendCompareSeries(QVector<PlotSeries> &out, int tab);

    // Текущий график (вкладка) для экспорта.
    PlotView *currentPlot() const;

    // Блоки графиков для сводного PDF-отчёта: собираются из тех же данных,
    // что уже выведены на экран (включая сравнение и метки СНС).
    QVector<PdfReport::PlotBlock> buildReportBlocks() const;

    // Обновление таблицы результатов (с троттлингом: таблица на 100+ тысяч
    // строк не должна перезаполняться 25 раз в секунду).
    void refreshNavModel(bool force);

    // Перевод географических координат в метры от точки старта.
    void updateTrajectory();

    void applyTheme(bool dark);
    void saveSettings();
    void showAbout();

    // --- Данные ---
    AppSettings settings_;
    RunSeries series_;
    RunSeries series_b_;      // второй прогон для сравнения
    RunStore store_;
    bool store_open_ = false;

    qint64 current_run_id_ = -1;
    qint64 compare_run_id_ = -1;
    bool compare_diff_ = false;   // показывать разность прогонов
    double start_lat_ = 0;
    double start_lon_ = 0;
    bool start_valid_ = false;

    bool running_ = false;
    bool stopping_ = false;
    bool autostart_ = false;

    // --- Виджеты ---
    QScopedPointer<ControlPanel> control_;
    QScopedPointer<StatePanel> state_;
    QScopedPointer<DataPanel> data_;
    QScopedPointer<ParamsPanel> params_;
    QScopedPointer<LogPanel> log_;
    QScopedPointer<NavTableModel> nav_model_;
    QScopedPointer<ParamsTableModel> params_model_;
    QScopedPointer<HistoryModel> history_model_;

    QTabWidget *plot_tabs_ = nullptr;
    TrajectoryPlot *plot_trajectory_ = nullptr;
    TimeSeriesPlot *plot_position_ = nullptr;
    TimeSeriesPlot *plot_attitude_ = nullptr;
    TimeSeriesPlot *plot_velocity_ = nullptr;
    TimeSeriesPlot *plot_errors_ = nullptr;
    TimeSeriesPlot *plot_innovation_ = nullptr;
    TimeSeriesPlot *plot_covariance_ = nullptr;
    TimeSeriesPlot *plot_consistency_ = nullptr;

    // Ряды, которые ещё можно включить/выключить в легенде.
    QAction *theme_action_ = nullptr;
    QAction *export_report_action_ = nullptr;
    QAction *start_action_ = nullptr;
    QAction *stop_action_ = nullptr;
    QAction *compare_diff_action_ = nullptr;
    QAction *compare_clear_action_ = nullptr;

    QTimer *refresh_timer_ = nullptr;
    QThread *thread_ = nullptr;
    SimulationWorker *worker_ = nullptr;

    qint64 last_table_update_ = 0;   // мс, последнее обновление таблицы
    qint64 last_state_update_ = 0;   // мс, последнее обновление панели состояния
    qint64 last_trajectory_n_ = 0;   // число точек на последней отрисовке траектории

    bool plots_dirty_ = true;
};

} // namespace gui