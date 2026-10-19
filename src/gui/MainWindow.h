// MainWindow.h — Главное окно imitator_gui.
//
// Собирает все панели: управление прогоном, состояние фильтра, графики
// (собственный QPainter), таблицу результатов, параметры и лог. Прогон ядра
// выполняется в отдельном QThread (SimulationWorker), приём телеметрии —
// через потокобезопасный буфер с отрисовкой 25 раз в секунду.

#pragma once

#include <QMainWindow>
#include <QPointF>
#include <QScopedPointer>

#include "core_bridge/run_store.h"
#include "core_bridge/telemetry_buffer.h"
#include "core/navigation/bias_report.h"
#include "export/pdf_report.h"
#include "settings/app_settings.h"

class QAction;
class QLabel;
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
class TuningPanel;

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

    // Самопроверка (--self-test): сколько рядов у графиков смещений ИМУ
    // (оценка, ±2σ и истина по каждой из 6 компонент). 0 — оценок смещений у
    // текущего прогона нет (например, прогон открыт из истории SQLite).
    int biasSeriesCount();

    // Экспорт статических матриц фильтра (Q/R/P0/H) в каталог без диалога:
    // три файла matrices_<датасет>.{csv,md,json}. Используется режимом
    // --export-matrices и self-test. false — ошибка (текст в error).
    bool exportMatricesTo(const QString &dir, QString *error = nullptr);

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
    void onGenerateDataset();
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
    void onSaveSettingsAs();
    void onResetParams();
    void onApplyParams();

    // Инструменты.
    void onExportPlot();
    void onExportSvg();
    void onExportPdf();
    void onExportReport();
    void onExportCsv();
    void onExportAllPlots();
    void onExportSeriesCsv();
    void onExportAnimation();
    void onExportMatrices();
    void onSaveLog();
    void onToggleTheme(bool dark);
    void onToggleCache(bool on);
    void onUpdateCacheStatus();
    void onResetZoom();
    void onSelectPlot(int index);

    // История прогонов.
    void onRefreshHistory();
    void onOpenRun(qint64 runId);
    void onCompareSelectRun();
    void onCompareDiffToggled(bool on);
    void onCompareClear();

    // Подбор коэффициентов (вкладка «Подбор»).
    void onApplyTuningPreset(const QString &path);
    void onTuningLog(const QString &text, int level);
    void onTuningFinished(bool ok);
    void onTuningReport(const QString &config_path);

Q_SIGNALS:
    // Прогон завершён (успешно, с ошибкой или по команде остановки).
    // Используется headless-режимом --self-test.
    void runFinished(bool ok, const QString &message);

private:
    // Вкладки графиков: заданы именованно, потому что их номера используются
    // и в rebuildPlotSeries(), и в appendCompareSeries(), а число вкладок
    // менялось (этап 6 плана добавил «Смещения ИМУ»).
    enum PlotTab
    {
        TabTrajectory = 0,
        TabCoordinates,
        TabAngles,
        TabVelocities,
        TabErrors,
        TabInnovation,
        TabCovariance,
        TabConsistency,
        TabBiases
    };

    // Группа компонент вектора ошибок: у каждой группы свои единицы, поэтому
    // каждая выводится отдельным графиком (одна ось — одна единица измерения).
    enum ErrorGroup
    {
        ErrPosition = 0,  // север/восток/высота, м
        ErrVelocity,      // Vn/Vh/Ve, м/с
        ErrAttitude,      // курс/тангаж/крен, рад
        ErrAccelBias,     // ba, м/с²
        ErrGyroBias,      // bg, рад/с
        ErrorGroupCount
    };

    void buildUi();
    void buildActions();
    void buildPlots();
    void connectSignals();

    // Построение рядов графиков из накопленной телеметрии.
    void rebuildPlotSeries(int tab);

    // Наземная истина набора: читает <data_dir>/truth.txt при смене каталога.
    void ensureTruthLoaded();

    // Истинные смещения ИМУ набора: читает <data_dir>/params.json при смене
    // каталога (нужны вкладке «Смещения ИМУ» и блокам отчёта).
    void ensureBiasTruthLoaded();

    // Ряды временных графиков (оценка + истина, плюс второй прогон). Общие для
    // вкладок и сводного отчёта, чтобы экран и PDF строились одинаково.
    QVector<PlotSeries> quantitySeries(int column) const;  // Lat..Ve
    // Компоненты вектора ошибок одной группы (ErrorGroup). lat_deg — широта
    // кадров коррекции: в группе «координаты» широта/долгота переводятся в
    // метры (север/восток), поэтому ось не смешивает радианы и метры.
    QVector<PlotSeries> errorSeries(int group, const QVector<double> &lat_deg) const;
    QVector<PlotSeries> innovationSeries() const;          // z СНС
    QVector<PlotSeries> covarianceSeries() const;          // diag P
    QVector<PlotSeries> consistencySeries() const;         // NIS/NEES
    QVector<PlotSeries> biasSeries(int index) const;       // ba/bg ИМУ (0..5)
    QVector<double> correctionTimes() const;               // моменты коррекции, с

    // Добавление ряда второго прогона (пунктир) для одной величины.
    void appendCompareField(QVector<PlotSeries> &out, int column) const;

    // Добавление рядов второго прогона (пунктир) и, если включено, разности
    // «текущий минус второй» на том же графике (диагностические вкладки).
    // group используется только для вкладки «Ошибки x».
    void appendCompareSeries(QVector<PlotSeries> &out, PlotTab tab, int group = -1) const;

    // Текущий график (вкладка) для экспорта.
    PlotView *currentPlot() const;

    // Все графики во вкладках, в порядке вкладок (для пакетного экспорта
    // и счётчиков кэша).
    QList<PlotView *> allPlots() const;

    // Диагностика кэша: суммарные hits/misses по всем графикам.
    void cacheCounters(quint64 *hits, quint64 *misses) const;

    // Блоки графиков для сводного PDF-отчёта: собираются из тех же данных,
    // что уже выведены на экран (включая сравнение и метки СНС).
    QVector<PdfReport::PlotBlock> buildReportBlocks() const;

    // Обновление таблицы результатов (с троттлингом: таблица на 100+ тысяч
    // строк не должна перезаполняться 25 раз в секунду).
    void refreshNavModel(bool force);

    // Перевод географических координат в метры от точки старта.
    void updateTrajectory();

    // Точки траектории (БИНС) и истины в метрах от точки старта — общий код
    // для updateTrajectory() и экспорта анимации.
    void buildTrajectoryPoints(QVector<QPointF> *nav, QVector<QPointF> *truth = nullptr);

    void applyTheme(bool dark);
    void saveSettings();
    void showAbout();

    // Предупреждение, если применённый пресет подобран на другом наборе
    // (секция [meta] не совпадает с каталогом данных). Не блокирует.
    void warnIfPresetDatasetMismatch(const QString &preset_path);

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

    // Наземная истина набора (truth.txt): кэш для наложения на графики.
    // Перечитывается при смене каталога данных (truth_dir_).
    QVector<TruthSample> truth_;
    QString truth_dir_;

    // Истинные смещения ИМУ набора (params.json): кэш для вкладки «Смещения
    // ИМУ» и блоков отчёта. Перечитывается при смене каталога (bias_truth_dir_).
    nav::BiasTruth bias_truth_;
    QString bias_truth_dir_;

    bool running_ = false;
    bool stopping_ = false;
    bool autostart_ = false;

    // --- Виджеты ---
    QScopedPointer<ControlPanel> control_;
    QScopedPointer<StatePanel> state_;
    QScopedPointer<DataPanel> data_;
    QScopedPointer<ParamsPanel> params_;
    QScopedPointer<LogPanel> log_;
    QScopedPointer<TuningPanel> tuning_;
    QScopedPointer<NavTableModel> nav_model_;
    QScopedPointer<ParamsTableModel> params_model_;
    QScopedPointer<HistoryModel> history_model_;

    QTabWidget *plot_tabs_ = nullptr;
    TrajectoryPlot *plot_trajectory_ = nullptr;
    // Раздельные временные графики по величинам: у каждой свой масштаб и
    // легенда (оценка БИНС и истина одной и той же величины).
    TimeSeriesPlot *plot_lat_ = nullptr;
    TimeSeriesPlot *plot_lon_ = nullptr;
    TimeSeriesPlot *plot_alt_ = nullptr;
    TimeSeriesPlot *plot_heading_ = nullptr;
    TimeSeriesPlot *plot_pitch_ = nullptr;
    TimeSeriesPlot *plot_roll_ = nullptr;
    TimeSeriesPlot *plot_vn_ = nullptr;
    TimeSeriesPlot *plot_ve_ = nullptr;
    TimeSeriesPlot *plot_vh_ = nullptr;
    // Вкладка «Ошибки x» — пять графиков по единицам измерения: координаты (м),
    // скорости (м/с), углы (рад), смещения акселя (м/с²) и гироскопа (рад/с).
    // Одна ось — одна единица (раньше все 15 компонент лежали на одной оси).
    TimeSeriesPlot *plot_err_pos_ = nullptr;
    TimeSeriesPlot *plot_err_vel_ = nullptr;
    TimeSeriesPlot *plot_err_att_ = nullptr;
    TimeSeriesPlot *plot_err_ba_ = nullptr;
    TimeSeriesPlot *plot_err_bg_ = nullptr;
    TimeSeriesPlot *plot_innovation_ = nullptr;
    TimeSeriesPlot *plot_covariance_ = nullptr;
    TimeSeriesPlot *plot_consistency_ = nullptr;
    // Вкладка «Смещения ИМУ»: оценка фильтра, ±2σ(P) и истина params.json.
    TimeSeriesPlot *plot_bias_ba_x_ = nullptr;
    TimeSeriesPlot *plot_bias_ba_y_ = nullptr;
    TimeSeriesPlot *plot_bias_ba_z_ = nullptr;
    TimeSeriesPlot *plot_bias_bg_x_ = nullptr;
    TimeSeriesPlot *plot_bias_bg_y_ = nullptr;
    TimeSeriesPlot *plot_bias_bg_z_ = nullptr;
    // Отдельный «холст» для сборки графиков в PDF-отчёте: не привязан к
    // вкладкам, поэтому в отчёт попадают сами ряды блоков, а не активная вкладка.
    TimeSeriesPlot *report_canvas_ = nullptr;

    // Ряды, которые ещё можно включить/выключить в легенде.
    QAction *theme_action_ = nullptr;
    QAction *export_report_action_ = nullptr;
    QAction *cache_action_ = nullptr;
    QAction *start_action_ = nullptr;
    QAction *stop_action_ = nullptr;
    QAction *compare_diff_action_ = nullptr;
    QAction *compare_clear_action_ = nullptr;

    QTimer *refresh_timer_ = nullptr;
    QTimer *cache_timer_ = nullptr;
    QLabel *cache_status_label_ = nullptr;
    QThread *thread_ = nullptr;
    SimulationWorker *worker_ = nullptr;

    qint64 last_table_update_ = 0;   // мс, последнее обновление таблицы
    qint64 last_state_update_ = 0;   // мс, последнее обновление панели состояния
    qint64 last_trajectory_n_ = 0;   // число точек на последней отрисовке траектории

    bool plots_dirty_ = true;
};

} // namespace gui