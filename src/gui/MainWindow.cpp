// MainWindow.cpp — Главное окно imitator_gui.

#include "MainWindow.h"

#include <algorithm>
#include <cmath>

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QInputDialog>
#include <QMenuBar>
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
#include <QThread>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>

#include "core/utils/constants.h"
#include "export/pdf_report.h"
#include "export/plot_exporter.h"
#include "models/history_model.h"
#include "models/nav_table_model.h"
#include "models/params_table_model.h"
#include "views/control_panel.h"
#include "views/data_panel.h"
#include "views/log_panel.h"
#include "views/params_panel.h"
#include "views/plots/time_series_plot.h"
#include "views/plots/trajectory_plot.h"
#include "views/state_panel.h"
#include "workers/simulation_worker.h"

namespace gui
{
namespace
{
// Ограничение числа точек на ряд в графике: при 10 минутах прогона это 120 000
// точек на ряд — рисовать незачем, достаточно ~2000.
constexpr int kMaxPlotPoints = 2000;

// Интервал отрисовки во время прогона, мс (25 Гц).
constexpr int kRefreshMs = 40;

// Троттлинг обновления таблицы результатов, мс.
constexpr qint64 kTableUpdateMs = 500;

// Тротлинг обновления панели состояния, мс.
constexpr qint64 kStateUpdateMs = 100;

// Палитра для рядов без явно заданного цвета (15 компонент ошибок и P).
const QVector<QColor> &seriesPalette()
{
    static const QVector<QColor> palette = {
        QColor(31, 119, 180),  QColor(255, 127, 14),  QColor(44, 160, 44),
        QColor(214, 39, 40),   QColor(148, 103, 189), QColor(140, 86, 75),
        QColor(227, 119, 194), QColor(127, 127, 127), QColor(188, 189, 34),
        QColor(23, 190, 207),  QColor(174, 199, 232), QColor(255, 187, 120),
        QColor(152, 223, 138), QColor(255, 152, 150), QColor(197, 176, 213)};
    return palette;
}

// Собрать ряд «время — значение» с прореживанием до kMaxPlotPoints точек.
// Значения берутся по индексам, поэтому время и данные всегда согласованы.
PlotSeries makeSeries(const QString &name, const QVector<double> &times,
                      const QVector<double> &values, int color_index,
                      bool markers = false, bool dashed = false)
{
    PlotSeries s;
    s.name = name;
    s.markers = markers;
    s.dashed = dashed;

    if (color_index >= 0 && color_index < seriesPalette().size())
    {
        s.color = seriesPalette()[color_index];
    }
    else
    {
        s.color = seriesPalette().at(color_index % seriesPalette().size());
    }

    const int n = std::min(times.size(), values.size());
    if (n <= 0) return s;

    const int step = std::max(1, (n + kMaxPlotPoints - 1) / kMaxPlotPoints);
    s.points.reserve(n / step + 2);
    for (int i = 0; i < n; i += step)
    {
        s.points.push_back(QPointF(times[i], values[i]));
    }
    if (((n - 1) % step) != 0)
    {
        s.points.push_back(QPointF(times[n - 1], values[n - 1]));
    }
    return s;
}

// Времена отсчётов накопленного ряда (для графиков).
QVector<double> rowTimes(const std::vector<RunSeries::Row> &rows)
{
    QVector<double> out(static_cast<int>(rows.size()));
    for (int i = 0; i < out.size(); i++) out[i] = rows[i].time;
    return out;
}

// Индекс отсчёта с ближайшим по времени значением (-1 — ряд пуст).
// Отсчёты идут по возрастанию времени, поэтому достаточно одного прохода.
int nearestTimeIndex(const std::vector<RunSeries::Row> &rows, double t)
{
    if (rows.empty()) return -1;
    int best = 0;
    double best_dt = std::abs(rows[0].time - t);
    for (std::size_t i = 1; i < rows.size(); i++)
    {
        const double dt = std::abs(rows[i].time - t);
        if (dt < best_dt)
        {
            best_dt = dt;
            best = static_cast<int>(i);
        }
    }
    return best;
}

// Собрать вектор из элементов std::vector<double> ядра.
QVector<double> toQVector(const Vector &v)
{
    QVector<double> out;
    out.reserve(static_cast<int>(v.size()));
    for (std::size_t i = 0; i < v.size(); i++) out.push_back(v[i]);
    return out;
}

// Метка компоненты вектора ошибок x.
QString errorLabel(int i)
{
    static const char *names[ins::KF_STATE] = {
        "x0 широта", "x1 долгота", "x2 высота", "x3 Vn", "x4 Vh", "x5 Ve",
        "x6 курс", "x7 тангаж", "x8 крен", "x9 ba_x", "x10 ba_y", "x11 ba_z",
        "x12 bg_x", "x13 bg_y", "x14 bg_z"};
    if (i >= 0 && i < ins::KF_STATE) return QString::fromLatin1(names[i]);
    return QString("x%1").arg(i);
}

// Метка диагонали ковариации.
QString covLabel(int i)
{
    static const char *names[ins::KF_STATE] = {
        "P00", "P11", "P22", "P33", "P44", "P55", "P66", "P77", "P88",
        "P99", "P1010", "P1111", "P1212", "P1313", "P1414"};
    if (i >= 0 && i < ins::KF_STATE) return QString::fromLatin1(names[i]);
    return QString("P%1").arg(i);
}

// Метка компоненты инновации.
QString innovationLabel(int i)
{
    static const char *names[ins::KF_MEAS] = {
        "Δширота", "Δдолгота", "Δвысота", "ΔVn", "ΔVh", "ΔVe",
        "Δкурс", "Δтангаж", "Δкрен"};
    if (i >= 0 && i < ins::KF_MEAS) return QString::fromLatin1(names[i]);
    return QString("z%1").arg(i);
}
} // namespace

// ---------------------------------------------------------------- MainWindow

MainWindow::MainWindow(const AppSettings &settings, bool autostart, QWidget *parent)
    : QMainWindow(parent), settings_(settings), autostart_(autostart)
{
    setWindowTitle(tr("imitator_gui — БИНС/СНС-комплексирование (ESKF)"));
    setDockOptions(QMainWindow::AnimatedDocks);

    buildPlots();
    buildUi();
    buildActions();
    connectSignals();

    // Модели.
    // Владение через QScopedPointer: parent не назначаем, иначе Qt удалит объект
    // повторно вместе с MainWindow.
    nav_model_.reset(new NavTableModel);
    params_model_.reset(new ParamsTableModel);
    history_model_.reset(new HistoryModel);

    data_->setNavModel(nav_model_.data());
    params_->setModel(params_model_.data());

    // Настройки: параметры фильтра из settings.ini.
    nav::FilterSettings filter_settings;
    if (!settings_.settings_file.isEmpty())
    {
        nav::loadFilterSettings(settings_.settings_file.toStdString(), filter_settings);
    }
    params_model_->setSettings(filter_settings);
    params_model_->applyToCore();

    control_->setSettings(settings_);
    data_->setDataDir(settings_.data_dir);

    if (!settings_.database_file.isEmpty())
    {
        QString err;
        store_open_ = store_.open(settings_.database_file, &err);
        if (store_open_)
        {
            history_model_->setStore(&store_);
            params_->setHistoryModel(history_model_.data());
        }
        else
        {
            log_->append(tr("База истории недоступна (%1): %2")
                             .arg(settings_.database_file, err),
                         2);
        }
    }

    applyTheme(settings_.dark_theme);
    settings_.restoreWindowGeometry(this);

    refresh_timer_ = new QTimer(this);
    connect(refresh_timer_, &QTimer::timeout, this, &MainWindow::onRefresh);
    refresh_timer_->start(kRefreshMs);

    statusBar()->showMessage(tr("Готово"));
    log_->append(tr("imitator_gui: ядро ESKF подключено, Qt %1")
                     .arg(QString::fromLatin1(qVersion())));
    log_->append(tr("Каталог данных: %1").arg(settings_.data_dir));
    log_->append(tr("settings.ini: %1").arg(settings_.settings_file));

    // Начальный пустой график, чтобы окно не было серым до первого прогона.
    rebuildPlotSeries(0);

    if (autostart_)
    {
        QTimer::singleShot(0, this, &MainWindow::onStart);
    }
}

MainWindow::~MainWindow()
{
    if (thread_ != nullptr && thread_->isRunning())
    {
        if (worker_ != nullptr) worker_->requestStop();
        thread_->quit();
        thread_->wait(5000);
    }
}

QString MainWindow::logText() const
{
    return log_ ? log_->text() : QString();
}

void MainWindow::buildUi()
{
    auto *central = new QWidget;
    auto *central_layout = new QVBoxLayout(central);
    central_layout->setContentsMargins(4, 4, 4, 4);

    auto *splitter = new QSplitter(Qt::Horizontal);

    // --- Левая колонка: управление и состояние ---
    control_.reset(new ControlPanel);
    state_.reset(new StatePanel);

    auto *left = new QWidget;
    auto *left_layout = new QVBoxLayout(left);
    left_layout->setContentsMargins(0, 0, 0, 0);
    left_layout->addWidget(control_.data());
    left_layout->addWidget(state_.data(), 1);

    splitter->addWidget(left);

    // --- Правая часть: вкладки графиков, данных, параметров ---
    auto *right = new QTabWidget;

    plot_tabs_ = new QTabWidget;
    plot_tabs_->addTab(plot_trajectory_, tr("Траектория"));
    plot_tabs_->addTab(plot_position_, tr("Координаты"));
    plot_tabs_->addTab(plot_attitude_, tr("Углы"));
    plot_tabs_->addTab(plot_velocity_, tr("Скорости"));
    plot_tabs_->addTab(plot_errors_, tr("Ошибки x"));
    plot_tabs_->addTab(plot_innovation_, tr("Инновации СНС"));
    plot_tabs_->addTab(plot_covariance_, tr("Ковариация P"));
    plot_tabs_->addTab(plot_consistency_, tr("Согласованность"));
    right->addTab(plot_tabs_, tr("Графики"));

    data_.reset(new DataPanel);
    right->addTab(data_.data(), tr("Данные"));

    params_.reset(new ParamsPanel);
    right->addTab(params_.data(), tr("Параметры"));

    log_.reset(new LogPanel);
    right->addTab(log_.data(), tr("Лог"));

    splitter->addWidget(right);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes(QList<int>() << 340 << 1000);

    central_layout->addWidget(splitter, 1);
    setCentralWidget(central);
    resize(1400, 860);
}

void MainWindow::buildActions()
{
    // --- Меню «Прогон» ---
    QMenu *run_menu = menuBar()->addMenu(tr("&Прогон"));

    start_action_ = new QAction(tr("&Запустить"), this);
    start_action_->setShortcut(QKeySequence(Qt::Key_F5));
    run_menu->addAction(start_action_);

    stop_action_ = new QAction(tr("&Остановить"), this);
    stop_action_->setShortcut(QKeySequence(Qt::SHIFT | Qt::Key_F5));
    run_menu->addAction(stop_action_);

    run_menu->addSeparator();
    QAction *load_settings = new QAction(tr("Загрузить settings.ini…"), this);
    run_menu->addAction(load_settings);
    QAction *save_settings = new QAction(tr("Сохранить settings.ini"), this);
    run_menu->addAction(save_settings);
    run_menu->addSeparator();
    QAction *reset_params = new QAction(tr("Сбросить параметры к умолчаниям"), this);
    run_menu->addAction(reset_params);
    QAction *apply_params = new QAction(tr("Применить параметры к фильтру"), this);
    run_menu->addAction(apply_params);

    connect(load_settings, &QAction::triggered, this, &MainWindow::onLoadSettingsFile);
    connect(save_settings, &QAction::triggered, this, &MainWindow::onSaveSettingsFile);
    connect(reset_params, &QAction::triggered, this, &MainWindow::onResetParams);
    connect(apply_params, &QAction::triggered, this, &MainWindow::onApplyParams);

    // --- Меню «Сравнение» (план 2.4) ---
    QMenu *compare_menu = menuBar()->addMenu(tr("&Сравнение"));

    QAction *compare_select = new QAction(tr("Второй прогон из истории…"), this);
    compare_menu->addAction(compare_select);
    connect(compare_select, &QAction::triggered, this, &MainWindow::onCompareSelectRun);

    compare_diff_action_ = new QAction(tr("Показывать разность (текущий − второй)"), this);
    compare_diff_action_->setCheckable(true);
    compare_diff_action_->setEnabled(false);
    compare_menu->addAction(compare_diff_action_);
    connect(compare_diff_action_, &QAction::toggled, this, &MainWindow::onCompareDiffToggled);

    compare_clear_action_ = new QAction(tr("Убрать сравнение"), this);
    compare_clear_action_->setEnabled(false);
    compare_menu->addAction(compare_clear_action_);
    connect(compare_clear_action_, &QAction::triggered, this, &MainWindow::onCompareClear);

    // --- Меню «Вид» ---
    QMenu *view_menu = menuBar()->addMenu(tr("&Вид"));

    QAction *reset_zoom = new QAction(tr("Сбросить масштаб графика"), this);
    view_menu->addAction(reset_zoom);
    connect(reset_zoom, &QAction::triggered, this, &MainWindow::onResetZoom);

    theme_action_ = new QAction(tr("Тёмная тема"), this);
    theme_action_->setCheckable(true);
    theme_action_->setChecked(settings_.dark_theme);
    view_menu->addAction(theme_action_);
    connect(theme_action_, &QAction::toggled, this, &MainWindow::onToggleTheme);

    // --- Меню «Экспорт» ---
    QMenu *export_menu = menuBar()->addMenu(tr("&Экспорт"));

    QAction *export_png = new QAction(tr("График в изображение (PNG/JPG)…"), this);
    export_menu->addAction(export_png);
    QAction *export_svg = new QAction(tr("График в SVG…"), this);
    export_menu->addAction(export_svg);
    QAction *export_pdf = new QAction(tr("График в PDF…"), this);
    export_menu->addAction(export_pdf);
    QAction *export_csv = new QAction(tr("Таблица результатов в CSV…"), this);
    export_menu->addAction(export_csv);
    QAction *save_log = new QAction(tr("Лог в файл…"), this);
    export_menu->addAction(save_log);

    connect(export_png, &QAction::triggered, this, &MainWindow::onExportPlot);
    connect(export_svg, &QAction::triggered, this, &MainWindow::onExportSvg);
    connect(export_pdf, &QAction::triggered, this, &MainWindow::onExportPdf);
    connect(export_csv, &QAction::triggered, this, &MainWindow::onExportCsv);

    export_menu->addSeparator();
    export_report_action_ = new QAction(tr("Сводный отчёт в PDF…"), this);
    export_menu->addAction(export_report_action_);
    connect(export_report_action_, &QAction::triggered, this, &MainWindow::onExportReport);
    connect(save_log, &QAction::triggered, this, &MainWindow::onSaveLog);

    // --- Меню «Справка» ---
    QMenu *help_menu = menuBar()->addMenu(tr("&Справка"));
    QAction *about = new QAction(tr("О программе"), this);
    help_menu->addAction(about);
    connect(about, &QAction::triggered, this, &MainWindow::showAbout);
}

void MainWindow::buildPlots()
{
    plot_trajectory_ = new TrajectoryPlot;
    plot_position_ = new TimeSeriesPlot;
    plot_position_->setTitle(tr("Координаты и высота"));
    plot_position_->setAxisLabels(tr("Время, с"), tr("°, м"));
    plot_position_->setMarginLeft(70);

    plot_attitude_ = new TimeSeriesPlot;
    plot_attitude_->setTitle(tr("Углы ориентации"));
    plot_attitude_->setAxisLabels(tr("Время, с"), tr("°"));
    plot_attitude_->setMarginLeft(70);

    plot_velocity_ = new TimeSeriesPlot;
    plot_velocity_->setTitle(tr("Скорости"));
    plot_velocity_->setAxisLabels(tr("Время, с"), tr("м/с"));
    plot_velocity_->setMarginLeft(70);

    plot_errors_ = new TimeSeriesPlot;
    plot_errors_->setTitle(tr("Вектор ошибок x (кадры коррекции СНС)"));
    plot_errors_->setAxisLabels(tr("Время, с"), tr("ошибка"));
    plot_errors_->setMarginLeft(78);
    plot_errors_->setLegendVisible(true);

    plot_innovation_ = new TimeSeriesPlot;
    plot_innovation_->setTitle(tr("Инновации измерений СНС"));
    plot_innovation_->setAxisLabels(tr("Время, с"), tr("измерение − прогноз"));
    plot_innovation_->setMarginLeft(78);

    plot_covariance_ = new TimeSeriesPlot;
    plot_covariance_->setTitle(tr("Диагонали ковариации P"));
    plot_covariance_->setAxisLabels(tr("Время, с"), tr("Pᵢᵢ"));
    plot_covariance_->setMarginLeft(78);
    // Логарифмический масштаб не делаем (наружу лог тоже не умеем), поэтому
    // порог отсекает нули: показываем только значимые компоненты.
    plot_covariance_->setVisibilityThreshold(1e-12);

    plot_consistency_ = new TimeSeriesPlot;
    plot_consistency_->setTitle(tr("Согласованность фильтра (NIS / NEES)"));
    plot_consistency_->setAxisLabels(tr("Время, с"), tr("статистика"));
    plot_consistency_->setMarginLeft(78);
    plot_consistency_->setLegendVisible(true);
}

void MainWindow::connectSignals()
{
    connect(control_.data(), &ControlPanel::startRequested, this, &MainWindow::onStart);
    connect(control_.data(), &ControlPanel::stopRequested, this, &MainWindow::onStop);
    connect(control_.data(), &ControlPanel::pauseToggled, this, &MainWindow::onPauseToggled);
    connect(control_.data(), &ControlPanel::speedFactorChanged, this,
            &MainWindow::onSpeedFactorChanged);
    connect(control_.data(), &ControlPanel::settingsChanged, this, &MainWindow::onSettingsChanged);
    connect(start_action_, &QAction::triggered, this, &MainWindow::onStart);
    connect(stop_action_, &QAction::triggered, this, &MainWindow::onStop);

    connect(data_.data(), &DataPanel::dataDirChanged, this, [this](const QString &dir) {
        settings_.data_dir = dir;
        control_->setSettings(settings_);
    });

    connect(params_.data(), &ParamsPanel::settingsEdited, this, &MainWindow::onSettingsChanged);
    connect(params_.data(), &ParamsPanel::runSelected, this, &MainWindow::onOpenRun);

    connect(plot_tabs_, &QTabWidget::currentChanged, this, &MainWindow::onSelectPlot);
}

// ------------------------------------------------------------ Управление

void MainWindow::onStart()
{
    if (running_) return;

    // Файлы настроек и результатов должны существовать до прогона.
    settings_ = control_->settings();
    saveSettings();

    nav::RunOptions options = control_->runOptions();
    options.speed_factor = control_->speedFactor();
    if (options.data_dir.empty())
    {
        QMessageBox::warning(this, tr("Прогон"), tr("Укажите каталог данных."));
        return;
    }
    if (options.write_files)
    {
        QDir out(QString::fromStdString(options.output_dir));
        if (!out.exists() && !QDir().mkpath(out.absolutePath()))
        {
            QMessageBox::warning(this, tr("Прогон"),
                                 tr("Не удалось создать каталог результатов:\n%1")
                                     .arg(QString::fromStdString(options.output_dir)));
            return;
        }
    }

    // Новая серия данных.
    series_.clear();
    plots_dirty_ = true;
    start_valid_ = false;
    current_run_id_ = -1;
    nav_model_->clear();
    plot_trajectory_->clearTrajectory();
    plot_trajectory_->clearReference();
    plot_errors_->clearSyncMarkers();
    state_->reset();

    // Параметры фильтра — из таблицы (правки применены к ядру).
    params_model_->applyToCore();

    if (thread_ != nullptr)
    {
        thread_->quit();
        thread_->wait(5000);
        thread_->deleteLater();
        thread_ = nullptr;
        worker_ = nullptr;
    }

    thread_ = new QThread(this);
    worker_ = new SimulationWorker;
    worker_->moveToThread(thread_);
    worker_->configure(options);

    connect(thread_, &QThread::started, worker_, &SimulationWorker::run);
    connect(thread_, &QThread::finished, worker_, &QObject::deleteLater);

    connect(worker_, &SimulationWorker::progress, this, &MainWindow::onWorkerProgress,
            Qt::QueuedConnection);
    connect(worker_, &SimulationWorker::logMessage, this, &MainWindow::onWorkerLog,
            Qt::QueuedConnection);
    connect(worker_, &SimulationWorker::stateUpdate, this, &MainWindow::onWorkerState,
            Qt::QueuedConnection);
    connect(worker_, &SimulationWorker::alignmentReady, this, &MainWindow::onAlignment,
            Qt::QueuedConnection);
    connect(worker_, &SimulationWorker::finished, this, &MainWindow::onWorkerFinished,
            Qt::QueuedConnection);

    running_ = true;
    stopping_ = false;
    control_->setRunning(true);
    start_action_->setEnabled(false);
    stop_action_->setEnabled(true);
    statusBar()->showMessage(tr("Прогон выполняется…"));

    // Темп воспроизведения применяется сразу, пауза — по кнопке.
    if (options.speed_factor > 0.0)
    {
        QMetaObject::invokeMethod(worker_, "setSpeedFactor", Qt::QueuedConnection,
                                  Q_ARG(double, options.speed_factor));
    }

    thread_->start();
}

void MainWindow::onPauseToggled(bool paused)
{
    if (worker_ == nullptr || !running_) return;

    QMetaObject::invokeMethod(worker_, "setPaused", Qt::QueuedConnection,
                              Q_ARG(bool, paused));
    statusBar()->showMessage(paused ? tr("Пауза") : tr("Продолжение"), 4000);
}

void MainWindow::onSpeedFactorChanged(double factor)
{
    if (worker_ == nullptr || !running_) return;
    QMetaObject::invokeMethod(worker_, "setSpeedFactor", Qt::QueuedConnection,
                              Q_ARG(double, factor));
}

void MainWindow::onStop()
{
    if (!running_ || worker_ == nullptr) return;

    stopping_ = true;
    worker_->requestStop();
    stop_action_->setEnabled(false);
    log_->append(tr("Запрошена остановка прогона…"), 1);
    statusBar()->showMessage(tr("Остановка…"));
}

void MainWindow::onWorkerFinished(bool ok, const QString &message)
{
    // Догружаем остаток буфера.
    onRefresh();
    refreshNavModel(true);

    // Результат остановки нужно запомнить до сброса флага.
    const bool was_stopped = stopping_;
    running_ = false;
    stopping_ = false;
    control_->setRunning(false);
    start_action_->setEnabled(true);
    stop_action_->setEnabled(false);

    if (thread_ != nullptr)
    {
        thread_->quit();
        thread_->wait(5000);
    }

    statusBar()->showMessage(was_stopped ? tr("Прогон остановлен")
                                      : (ok ? tr("Прогон завершён")
                                            : tr("Прогон не выполнен")));

    // Прерванный пользователем прогон тоже сохраняем — данные до момента
    // остановки полезны для сравнения с эталоном.
    if ((ok || was_stopped) && settings_.save_to_history && store_open_ &&
        !series_.rows.empty())
    {
        // Запись результатов в SQLite для последующего сравнения.
        nav::RunSummary summary;
        summary.steps = series_.rows.size();
        summary.has_angle = !series_.rows.empty() && series_.rows.front().has_reference;

        QVector<SampleRow> samples;
        samples.reserve(static_cast<int>(series_.rows.size()));
        for (const RunSeries::Row &r : series_.rows)
        {
            SampleRow s;
            s.t = r.time;
            s.lon = r.result[RunSeries::Lon];
            s.lat = r.result[RunSeries::Lat];
            s.alt = r.result[RunSeries::Alt];
            s.heading = r.result[RunSeries::Heading];
            s.pitch = r.result[RunSeries::Pitch];
            s.roll = r.result[RunSeries::Roll];
            s.vn = r.result[RunSeries::Vn];
            s.vh = r.result[RunSeries::Vh];
            s.ve = r.result[RunSeries::Ve];
            samples.push_back(s);
        }

        const QString name = QString("%1  %2")
                                 .arg(QDateTime::currentDateTime().toString("dd.MM.yyyy HH:mm:ss"),
                                      settings_.data_dir);
        current_run_id_ = store_.beginRun(name, summary, settings_.data_dir, QString());
        if (current_run_id_ > 0)
        {
            store_.appendSamples(current_run_id_, samples);

            std::vector<double> times;
            std::vector<std::vector<double>> errors;
            times.reserve(series_.corrections.size());
            errors.reserve(series_.corrections.size());
            for (const RunSeries::CorrectionRow &c : series_.corrections)
            {
                times.push_back(c.time);
                std::vector<double> x(ins::KF_STATE, 0.0);
                for (int i = 0; i < ins::KF_STATE; i++) x[i] = c.x[i];
                errors.push_back(x);
            }
            store_.appendErrors(current_run_id_, times, errors);

            // Диагонали ковариации по всем тактам ИМУ: нужны для графика
            // P и для оценки качества фильтра после загрузки из истории.
            if (!series_.rows.empty())
            {
                std::vector<double> pt;
                std::vector<std::vector<double>> pd;
                pt.reserve(series_.rows.size());
                pd.reserve(series_.rows.size());
                for (const RunSeries::Row &r : series_.rows)
                {
                    pt.push_back(r.time);
                    std::vector<double> p(ins::KF_STATE, 0.0);
                    for (int i = 0; i < ins::KF_STATE; i++) p[i] = r.p[i];
                    pd.push_back(p);
                }
                store_.appendCovariances(current_run_id_, pt, pd);
            }

            history_model_->refresh();
            log_->append(tr("Прогон сохранён в историю: id = %1").arg(current_run_id_));
        }
        else
        {
            log_->append(tr("Не удалось сохранить прогон в историю"), 2);
        }
    }

    if (was_stopped || !ok)
    {
        log_->append(message, ok ? 1 : 2);
    }

    Q_EMIT runFinished(ok && !was_stopped, message);
}

// ------------------------------------------------------------ Приём данных

void MainWindow::onWorkerProgress(int percent, double time)
{
    control_->setProgress(percent, time);
}

void MainWindow::onWorkerLog(const QString &text, int level)
{
    log_->append(text, level);
}

void MainWindow::onWorkerState(double time, int, const QString &diag)
{
    statusBar()->showMessage(diag);
}

void MainWindow::onAlignment()
{
    // Текст выставки уже попал в лог через logMessage — здесь только данные
    // для пересчёта местных координат.
    const QString ini = QDir(settings_.data_dir).filePath("StartupNav.ini");
    double lon = 0, lat = 0, alt = 0, t = 0;
    if (nav::readStartupNav(ini.toStdString(), lon, lat, alt, t))
    {
        start_lat_ = lat;
        start_lon_ = lon;
        start_valid_ = true;
    }
    else
    {
        start_valid_ = false;
    }
}

void MainWindow::onRefresh()
{
    if (worker_ == nullptr) return;

    const std::vector<TelemetrySample> samples = worker_->buffer().drain();
    if (samples.empty()) return;

    for (const TelemetrySample &s : samples)
    {
        RunSeries::Row row;
        row.time = s.time;
        row.corrected = s.corrected;
        row.step_index = s.step_index;

        row.result[RunSeries::Time] = s.result.time;
        row.result[RunSeries::Lon] = s.result.lon;
        row.result[RunSeries::Lat] = s.result.lat;
        row.result[RunSeries::Alt] = s.result.alt;
        row.result[RunSeries::Heading] = s.result.heading;
        row.result[RunSeries::Pitch] = s.result.pitch;
        row.result[RunSeries::Roll] = s.result.roll;
        row.result[RunSeries::Vn] = s.result.vn;
        row.result[RunSeries::Vh] = s.result.vh;
        row.result[RunSeries::Ve] = s.result.ve;

        if (s.has_reference)
        {
            row.has_reference = true;
            row.reference[RunSeries::Time] = s.reference.time;
            row.reference[RunSeries::Lon] = s.reference.lon;
            row.reference[RunSeries::Lat] = s.reference.lat;
            row.reference[RunSeries::Alt] = s.reference.alt;
            row.reference[RunSeries::Heading] = s.reference.heading;
            row.reference[RunSeries::Pitch] = s.reference.pitch;
            row.reference[RunSeries::Roll] = s.reference.roll;
            row.reference[RunSeries::Vn] = s.reference.vn;
            row.reference[RunSeries::Vh] = s.reference.vh;
            row.reference[RunSeries::Ve] = s.reference.ve;
        }

        if (s.corrected && s.has_innovation)
        {
            row.has_innovation = true;
            for (int i = 0; i < ins::KF_MEAS && i < s.innovation.size(); i++)
            {
                row.innovation[i] = s.innovation[i];
            }

            // Кадр коррекции: сохраняем инновацию и вектор ошибок x до
            // применения коррекции — это отдельный (разреженный) ряд.
            RunSeries::CorrectionRow c;
            c.time = s.time;
            for (int i = 0; i < ins::KF_MEAS; i++) c.innovation[i] = row.innovation[i];
            for (int i = 0; i < ins::KF_STATE && i < s.errors.size(); i++)
            {
                c.x[i] = s.errors[i];
            }
            c.nis = s.nis;
            c.nees = s.nees;
            series_.corrections.push_back(c);

            // Согласованность фильтра — тот же разреженный ряд, но без
            // привязки к конкретным компонентам инновации.
            if (s.nis_dof > 0)
            {
                RunSeries::ConsistencyRow k;
                k.time = s.time;
                k.nis = s.nis;
                k.nees = s.nees;
                k.dof = s.nis_dof;
                series_.consistency.push_back(k);
            }
        }

        for (int i = 0; i < ins::KF_STATE && i < s.p_diag.size(); i++)
        {
            row.p[i] = s.p_diag[i];
        }

        series_.rows.push_back(row);
    }

    plots_dirty_ = true;

    // Таблица результатов и траектория — с троттлингом, панель состояния —
    // 10 Гц. Графики перерисовываются для видимой вкладки.
    refreshNavModel(false);
    updateTrajectory();

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (now - last_state_update_ >= kStateUpdateMs)
    {
        last_state_update_ = now;
        state_->updateSample(samples.back());
    }

    if (plots_dirty_) rebuildPlotSeries(plot_tabs_->currentIndex());
}

void MainWindow::refreshNavModel(bool force)
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (!force && now - last_table_update_ < kTableUpdateMs) return;

    last_table_update_ = now;
    nav_model_->setSeries(series_);
    data_->setRowInfo(tr("строк: %1").arg(series_.rows.size()));
}

// ------------------------------------------------------------ Графики

void MainWindow::rebuildPlotSeries(int tab)
{
    const int n = static_cast<int>(series_.rows.size());

    // Времена для рядов по всем тактам.
    QVector<double> times;
    times.reserve(n);
    for (const RunSeries::Row &r : series_.rows) times.push_back(r.time);

    QVector<PlotSeries> out;

    switch (tab)
    {
    case 0:
        // Траектория рисуется через updateTrajectory().
        plots_dirty_ = false;
        return;

    case 1:  // Координаты и высота
    {
        if (n == 0) return;
        QVector<double> lat(n), lon(n), alt(n);
        for (int i = 0; i < n; i++)
        {
            const RunSeries::Row &r = series_.rows[i];
            lat[i] = r.result[RunSeries::Lat];
            lon[i] = r.result[RunSeries::Lon];
            alt[i] = r.result[RunSeries::Alt];
        }
        // Широта/долгота в градусах (1e-5), высота в метрах — по одной оси
        // все три ряда читаются плохо, поэтому координаты переводим в
        // метры от точки старта, а высоту оставляем как есть.
        if (start_valid_)
        {
            const double lat0 = start_lat_ * DEG_TO_RAD;
            const double lon0 = start_lon_ * DEG_TO_RAD;
            QVector<double> north(n), east(n);
            for (int i = 0; i < n; i++)
            {
                north[i] = (lat[i] * DEG_TO_RAD - lat0) * R_EARTH;
                east[i] = (lon[i] * DEG_TO_RAD - lon0) * R_EARTH * std::cos(lat0);
            }
            out.push_back(makeSeries(tr("Север, м"), times, north, 0));
            out.push_back(makeSeries(tr("Восток, м"), times, east, 1));
        }
        else
        {
            out.push_back(makeSeries(tr("Широта, °"), times, lat, 0));
            out.push_back(makeSeries(tr("Долгота, °"), times, lon, 1));
        }
        out.push_back(makeSeries(tr("Высота, м"), times, alt, 2));
        break;
    }

    case 2:  // Углы
    {
        if (n == 0) return;
        QVector<double> hdg(n), pitch(n), roll(n);
        for (int i = 0; i < n; i++)
        {
            const RunSeries::Row &r = series_.rows[i];
            hdg[i] = r.result[RunSeries::Heading];
            pitch[i] = r.result[RunSeries::Pitch];
            roll[i] = r.result[RunSeries::Roll];
        }
        out.push_back(makeSeries(tr("Курс, °"), times, hdg, 0));
        out.push_back(makeSeries(tr("Тангаж, °"), times, pitch, 1));
        out.push_back(makeSeries(tr("Крен, °"), times, roll, 2));
        break;
    }

    case 3:  // Скорости
    {
        if (n == 0) return;
        QVector<double> vn(n), vh(n), ve(n);
        for (int i = 0; i < n; i++)
        {
            const RunSeries::Row &r = series_.rows[i];
            vn[i] = r.result[RunSeries::Vn];
            vh[i] = r.result[RunSeries::Vh];
            ve[i] = r.result[RunSeries::Ve];
        }
        out.push_back(makeSeries(tr("Vn, м/с"), times, vn, 0));
        out.push_back(makeSeries(tr("Vh, м/с"), times, vh, 1));
        out.push_back(makeSeries(tr("Ve, м/с"), times, ve, 2));
        break;
    }

    case 4:  // Вектор ошибок x (только кадры коррекции СНС)
    {
        const int m = static_cast<int>(series_.corrections.size());
        if (m == 0) return;

        QVector<double> ct(m);
        for (int k = 0; k < m; k++) ct[k] = series_.corrections[k].time;

        for (int i = 0; i < ins::KF_STATE; i++)
        {
            QVector<double> v(m);
            for (int k = 0; k < m; k++) v[k] = series_.corrections[k].x[i];
            out.push_back(makeSeries(errorLabel(i), ct, v, i, true));
        }
        plot_errors_->setSyncMarkers(ct);
        plot_errors_->setVisibilityThreshold(1e-18);
        appendCompareSeries(out, tab);
        plot_errors_->setSeries(out);
        plots_dirty_ = false;
        return;
    }

    case 5:  // Инновации измерений СНС
    {
        const int m = static_cast<int>(series_.corrections.size());
        if (m == 0) return;

        QVector<double> ct(m);
        for (int k = 0; k < m; k++) ct[k] = series_.corrections[k].time;

        int color = 0;
        for (int i = 0; i < ins::KF_MEAS; i++)
        {
            QVector<double> v(m);
            bool nonzero = false;
            for (int k = 0; k < m; k++)
            {
                v[k] = series_.corrections[k].innovation[i];
                if (std::abs(v[k]) > 0.0) nonzero = true;
            }
            // Нулевые компоненты (например, курс без angle.dat) не рисуем.
            if (!nonzero) continue;
            out.push_back(makeSeries(innovationLabel(i), ct, v, color++, true));
        }
        plot_innovation_->setSyncMarkers(ct);
        plot_innovation_->setVisibilityThreshold(1e-18);
        appendCompareSeries(out, tab);
        plot_innovation_->setSeries(out);
        plots_dirty_ = false;
        return;
    }

    case 6:  // Диагонали ковариации P
    {
        if (n == 0) return;
        for (int i = 0; i < ins::KF_STATE; i++)
        {
            QVector<double> v(n);
            for (int k = 0; k < n; k++) v[k] = series_.rows[k].p[i];
            out.push_back(makeSeries(covLabel(i), times, v, i));
        }
        break;
    }

    case 7:  // NIS / NEES на кадрах коррекции СНС
    {
        const int m = static_cast<int>(series_.consistency.size());
        if (m == 0) return;

        QVector<double> ct(m);
        QVector<double> nis(m);
        QVector<double> nees(m);
        int dof = 0;
        for (int k = 0; k < m; k++)
        {
            ct[k] = series_.consistency[k].time;
            nis[k] = series_.consistency[k].nis;
            nees[k] = series_.consistency[k].nees;
            dof = series_.consistency[k].dof;
        }
        if (dof <= 0) return;

        // Опорная линия — ожидаемое значение хи-квадрат: в согласованном
        // фильфе NIS и NEES тяготеют к числу степеней свободы.
        QVector<double> expect(m, static_cast<double>(dof));
        out.push_back(makeSeries(tr("ожидание (%1)").arg(dof), ct, expect, 6, true));
        out.push_back(makeSeries(tr("NIS"), ct, nis, 0, true));
        out.push_back(makeSeries(tr("NEES"), ct, nees, 1, true));

        plot_consistency_->setSyncMarkers(ct);
        plot_consistency_->setVisibilityThreshold(1e-12);
        appendCompareSeries(out, tab);
        plot_consistency_->setSeries(out);
        plots_dirty_ = false;
        return;
    }

    default:
        return;
    }

    PlotView *plot = nullptr;
    switch (tab)
    {
    case 1: plot = plot_position_; break;
    case 2: plot = plot_attitude_; break;
    case 3: plot = plot_velocity_; break;
    case 6: plot = plot_covariance_; break;
    case 7: plot = plot_consistency_; break;
    default: break;
    }
    appendCompareSeries(out, tab);
    if (plot != nullptr) plot->setSeries(out);

    plots_dirty_ = false;
}

// Блоки графиков для сводного PDF-отчёта (план 2.9).
//
// Ряды берутся прямо из виджетов вкладок, поэтому в отчёт попадает ровно то,
// что нарисовано на экране: с учётом второго прогона, разности и меток СНС.
// Траектория уходит отдельным блоком со своим виджетом.
QVector<PdfReport::PlotBlock> MainWindow::buildReportBlocks() const
{
    QVector<PdfReport::PlotBlock> blocks;

    PdfReport::PlotBlock traj;
    traj.caption = tr("Траектория");
    traj.widget = plot_trajectory_;
    blocks.push_back(traj);

    const TimeSeriesPlot *tabs[] = {plot_position_, plot_attitude_, plot_velocity_,
                                     plot_errors_, plot_innovation_, plot_covariance_,
                                     plot_consistency_};
    const QString captions[] = {tr("Координаты и высота"), tr("Углы"), tr("Скорости"),
                                tr("Ошибки x"), tr("Инновации СНС"), tr("Диагонали P"),
                                tr("Согласованность (NIS/NEES)")};

    for (int i = 0; i < 7; i++)
    {
        if (tabs[i] == nullptr || tabs[i]->seriesCount() == 0) continue;
        PdfReport::PlotBlock block;
        block.caption = captions[i];
        block.series = tabs[i]->reportSeries();
        block.sync_markers = tabs[i]->syncMarkers();
        blocks.push_back(block);
    }
    return blocks;
}

// Сравнение двух прогонов (план 2.4).
//
// Ряды второго прогона добавляются пунктиром в те же графики, что и основной
// прогон. Поскольку прогоны могут иметь разные отметки времени, разность
// считается по ближайшему по времени отсчёту второго прогона: оба считались на
// одной и той же записи ИМУ, поэтому временные сетки практически совпадают.
void MainWindow::appendCompareSeries(QVector<PlotSeries> &out, int tab)
{
    if (!hasComparison()) return;

    const RunSeries &B = series_b_;
    const RunSeries &A = series_;
    const QString suffix = tr(" [прогон %1]").arg(compare_run_id_);

    // Индексы полей, по которым строится сравнение, — те же, что у основного
    // графика: координаты/высота, углы, скорости, компоненты x, диагонали P,
    // NIS/NEES.
    QVector<QPair<QString, RunSeries::ResultField>> fields;
    switch (tab)
    {
    case 1:
        fields.append({tr("Север, м"), RunSeries::Lat});
        fields.append({tr("Восток, м"), RunSeries::Lon});
        fields.append({tr("Высота, м"), RunSeries::Alt});
        break;
    case 2:
        fields.append({tr("Курс, °"), RunSeries::Heading});
        fields.append({tr("Тангаж, °"), RunSeries::Pitch});
        fields.append({tr("Крен, °"), RunSeries::Roll});
        break;
    case 3:
        fields.append({tr("Vn, м/с"), RunSeries::Vn});
        fields.append({tr("Vh, м/с"), RunSeries::Vh});
        fields.append({tr("Ve, м/с"), RunSeries::Ve});
        break;
    default:
        break;
    }

    // Сравнение по тем же временам, что и у основного ряда.
    const int n = static_cast<int>(A.rows.size());

    auto rowValue = [](const RunSeries::Row &r, RunSeries::ResultField f) {
        return r.result[f];
    };

    if (!fields.isEmpty())
    {
        // Широта/долгота сравниваем в метрах от общей точки отсчёта — иначе
        // разность в градусах читается неоднозначно.
        const double lat0 = start_lat_ * DEG_TO_RAD;
        const double lon0 = start_lon_ * DEG_TO_RAD;

        for (int k = 0; k < fields.size(); k++)
        {
            const RunSeries::ResultField f = fields[k].second;

            QVector<double> bt(static_cast<int>(B.rows.size()));
            for (int i = 0; i < bt.size(); i++) bt[i] = rowValue(B.rows[i], f);

            QVector<double> at(n);
            for (int i = 0; i < n; i++) at[i] = rowValue(A.rows[i], f);

            // Перевод координат в метры — как на основном графике.
            if (start_valid_ && (f == RunSeries::Lat || f == RunSeries::Lon))
            {
                for (int i = 0; i < at.size(); i++)
                {
                    at[i] = (at[i] * DEG_TO_RAD - (f == RunSeries::Lat ? lat0 : lon0)) *
                            R_EARTH * (f == RunSeries::Lon ? std::cos(lat0) : 1.0);
                }
                for (int i = 0; i < bt.size(); i++)
                {
                    bt[i] = (bt[i] * DEG_TO_RAD - (f == RunSeries::Lat ? lat0 : lon0)) *
                            R_EARTH * (f == RunSeries::Lon ? std::cos(lat0) : 1.0);
                }
            }

            out.push_back(makeSeries(fields[k].first + suffix, rowTimes(A.rows), bt, k, false, true));

            if (compare_diff_)
            {
                // Разность по ближайшему времени второго прогона.
                QVector<double> diff;
                diff.reserve(n);
                for (int i = 0; i < n; i++)
                {
                    const double t = A.rows[i].time;
                    int j = nearestTimeIndex(B.rows, t);
                    if (j < 0) continue;
                    diff.push_back(at[i] - bt[j]);
                }
                if (!diff.isEmpty())
                {
                    out.push_back(makeSeries(tr("Δ %1").arg(fields[k].first),
                                             rowTimes(A.rows), diff, k + 7, false, true));
                }
            }
        }
        return;
    }

    // Разреженные ряды (ошибки, инновации, NIS/NEES) — сравниваем покомпонентно.
    if (tab == 6)
    {
        for (int i = 0; i < ins::KF_STATE; i++)
        {
            QVector<double> bv(static_cast<int>(B.rows.size()));
            for (int k = 0; k < bv.size(); k++) bv[k] = B.rows[k].p[i];
            out.push_back(makeSeries(covLabel(i) + suffix, rowTimes(B.rows), bv, i, false, true));
        }
        return;
    }

    QVector<double> ct(static_cast<int>(B.corrections.size()));
    for (int i = 0; i < ct.size(); i++) ct[i] = B.corrections[i].time;
    if (ct.isEmpty()) return;

    // Компоненты вектора ошибок x (табы 4, 5).
    if (tab == 4 || tab == 5)
    {
        for (int i = 0; i < ins::KF_STATE; i++)
        {
            QVector<double> bv(ct.size());
            for (int k = 0; k < bv.size(); k++) bv[k] = B.corrections[k].x[i];
            out.push_back(makeSeries(errorLabel(i) + suffix, ct, bv, i, true, true));
        }
        return;
    }

    // Согласованность NIS/NEES (таб 7) — из своего разреженного ряда.
    if (tab == 7 && !B.consistency.empty())
    {
        QVector<double> bt(static_cast<int>(B.consistency.size()));
        QVector<double> bn(bt.size()), be(bt.size());
        for (int i = 0; i < bt.size(); i++)
        {
            bt[i] = B.consistency[i].time;
            bn[i] = B.consistency[i].nis;
            be[i] = B.consistency[i].nees;
        }
        out.push_back(makeSeries(tr("NIS") + suffix, bt, bn, 0, true, true));
        out.push_back(makeSeries(tr("NEES") + suffix, bt, be, 1, true, true));
        return;
    }
}

void MainWindow::updateTrajectory()
{
    if (series_.rows.empty()) return;

    // Точка отсчёта — из StartupNav.ini (или первый результат, если INI нет).
    if (!start_valid_)
    {
        start_lat_ = series_.rows.front().result[RunSeries::Lat];
        start_lon_ = series_.rows.front().result[RunSeries::Lon];
        start_valid_ = true;
    }

    const double lat0 = start_lat_ * DEG_TO_RAD;
    const double lon0 = start_lon_ * DEG_TO_RAD;

    QVector<QPointF> nav;
    QVector<QPointF> ref;

    // Шаг прореживания: на экране всё равно не видно больше ~2000 точек.
    const std::size_t total = series_.rows.size();
    const std::size_t step = std::max<std::size_t>(1, total / kMaxPlotPoints);

    for (std::size_t i = 0; i < total; i += step)
    {
        const RunSeries::Row &r = series_.rows[i];
        const double d_n = (r.result[RunSeries::Lat] * DEG_TO_RAD - lat0) * R_EARTH;
        const double d_e = (r.result[RunSeries::Lon] * DEG_TO_RAD - lon0) * R_EARTH *
                           std::cos(lat0);
        nav.push_back(QPointF(d_e, d_n));

        if (r.has_reference)
        {
            const double rn = (r.reference[RunSeries::Lat] * DEG_TO_RAD - lat0) * R_EARTH;
            const double re = (r.reference[RunSeries::Lon] * DEG_TO_RAD - lon0) * R_EARTH *
                              std::cos(lat0);
            ref.push_back(QPointF(re, rn));
        }
    }

    // Последняя точка траектории всегда рисуется — конец прогона важен.
    if (step > 1 && ((total - 1) % step) != 0)
    {
        const RunSeries::Row &r = series_.rows.back();
        const double d_n = (r.result[RunSeries::Lat] * DEG_TO_RAD - lat0) * R_EARTH;
        const double d_e = (r.result[RunSeries::Lon] * DEG_TO_RAD - lon0) * R_EARTH *
                           std::cos(lat0);
        nav.push_back(QPointF(d_e, d_n));
    }

    plot_trajectory_->setNavPoints(nav);
    plot_trajectory_->setReferencePoints(ref);
    plot_trajectory_->setShowReference(!ref.isEmpty());
}

void MainWindow::onSelectPlot(int index)
{
    Q_UNUSED(index);
    plots_dirty_ = true;
    rebuildPlotSeries(plot_tabs_->currentIndex());
    if (plot_tabs_->currentWidget() != nullptr) plot_tabs_->currentWidget()->update();
}

PlotView *MainWindow::currentPlot() const
{
    if (plot_tabs_ == nullptr) return nullptr;
    return qobject_cast<PlotView *>(plot_tabs_->currentWidget());
}

void MainWindow::onResetZoom()
{
    PlotView *plot = currentPlot();
    if (plot == nullptr) return;

    TrajectoryPlot *traj = qobject_cast<TrajectoryPlot *>(plot);
    if (traj != nullptr)
    {
        traj->resetView();
        return;
    }
    plot->resetRanges();
}

// ------------------------------------------------------------ Параметры

void MainWindow::onSettingsChanged()
{
    settings_ = control_->settings();
    saveSettings();
}

void MainWindow::onLoadSettingsFile()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Загрузить параметры"), settings_.settings_file,
        tr("INI-файлы (*.ini);;Все файлы (*)"));
    if (path.isEmpty()) return;

    QString error;
    if (!params_->loadSettings(path, &error))
    {
        QMessageBox::warning(this, tr("Параметры"), error);
        return;
    }

    settings_.settings_file = path;
    params_model_->applyToCore();
    control_->setSettings(settings_);
    saveSettings();
    log_->append(tr("Параметры загружены из %1").arg(path));
}

void MainWindow::onSaveSettingsFile()
{
    if (settings_.settings_file.isEmpty())
    {
        onLoadSettingsFile();
        return;
    }

    QString error;
    if (!params_->saveSettings(settings_.settings_file, &error))
    {
        QMessageBox::warning(this, tr("Параметры"), error);
        return;
    }
    log_->append(tr("Параметры сохранены в %1").arg(settings_.settings_file));
}

void MainWindow::onResetParams()
{
    if (QMessageBox::question(this, tr("Параметры"),
                              tr("Сбросить все параметры к значениям по умолчанию?")) !=
        QMessageBox::Yes)
    {
        return;
    }
    params_->resetToDefaults();
    params_model_->applyToCore();
    log_->append(tr("Параметры сброшены к умолчаниям"));
}

void MainWindow::onApplyParams()
{
    params_model_->applyToCore();
    log_->append(tr("Параметры применены к фильтру"));
}

// ------------------------------------------------------------ Экспорт

void MainWindow::onExportPlot()
{
    PlotView *plot = currentPlot();
    if (plot == nullptr) return;

    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить график"), settings_.output_dir + "/plot.png",
        tr("Изображения (*.png *.jpg *.bmp)"));
    if (path.isEmpty()) return;

    QString error;
    if (!PlotExporter::exportPlotImage(plot, path, settings_.export_width,
                                      settings_.export_height, &error))
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }
    log_->append(tr("График сохранён: %1").arg(path));
}

void MainWindow::onExportSvg()
{
    PlotView *plot = currentPlot();
    if (plot == nullptr) return;

    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить график в SVG"), settings_.output_dir + "/plot.svg",
        tr("SVG (*.svg)"));
    if (path.isEmpty()) return;

    QString error;
    if (!PlotExporter::exportPlotSvg(plot, path, settings_.export_width,
                                     settings_.export_height, &error))
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }
    log_->append(tr("График сохранён: %1").arg(path));
}

void MainWindow::onExportPdf()
{
    PlotView *plot = currentPlot();
    if (plot == nullptr) return;

    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить график в PDF"), settings_.output_dir + "/plot.pdf",
        tr("PDF (*.pdf)"));
    if (path.isEmpty()) return;

    QString error;
    if (!PlotExporter::exportPlotPdf(plot, path, settings_.export_width,
                                    settings_.export_height, &error))
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }
    log_->append(tr("График сохранён: %1").arg(path));
}

// Сборка сводного отчёта по текущему прогону в указанный файл. Отдельный
// метод без диалога — его использует self-test.
bool MainWindow::exportReportTo(const QString &path, QString *error)
{
    if (series_.rows.empty())
    {
        if (error != nullptr)
            *error = tr("Нет данных: сначала выполните прогон или откройте его из истории");
        return false;
    }

    PdfReport::Options options;
    options.data_dir = settings_.data_dir;
    options.title = hasComparison()
                        ? tr("Отчёт по прогону (сравнение с прогоном %1)").arg(compare_run_id_)
                        : tr("Отчёт по прогону");

    // «Холст» для блоков: текущий график вкладки. Его ряды на время сборки
    // отчёта подменяются содержимым блоков и возвращаются обратно.
    PlotView *canvas = currentPlot();
    if (canvas == nullptr) canvas = plot_trajectory_;

    return PdfReport::build(canvas, series_, buildReportBlocks(), options, path, error);
}

// Сводный отчёт по прогону в PDF: таблицы метрик + графики (план 2.9).
void MainWindow::onExportReport()
{
    if (series_.rows.empty())
    {
        QMessageBox::information(this, tr("Отчёт"),
                                 tr("Нет данных: сначала выполните прогон "
                                    "или откройте его из истории"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить отчёт"), settings_.output_dir + "/report.pdf",
        tr("PDF (*.pdf)"));
    if (path.isEmpty()) return;

    QString error;
    if (!exportReportTo(path, &error))
    {
        QMessageBox::warning(this, tr("Отчёт"), error);
        return;
    }
    log_->append(tr("Отчёт сохранён: %1").arg(path));
}

void MainWindow::onExportCsv()
{
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить таблицу результатов"), settings_.output_dir + "/results.csv",
        tr("CSV (*.csv)"));
    if (path.isEmpty()) return;

    QString error;
    if (!PlotExporter::exportTableCsv(nav_model_.data(), path, &error))
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }
    log_->append(tr("Таблица сохранена: %1").arg(path));
}

void MainWindow::onSaveLog()
{
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить лог"), settings_.output_dir + "/imitator_gui.log",
        tr("Текст (*.txt *.log)"));
    if (path.isEmpty()) return;
    log_->saveToFile(path);
    log_->append(tr("Лог сохранён: %1").arg(path));
}

// ------------------------------------------------------------ Вид и тема

void MainWindow::onToggleTheme(bool dark)
{
    applyTheme(dark);
    settings_.dark_theme = dark;
    saveSettings();
}

void MainWindow::applyTheme(bool dark)
{
    QPalette p = qobject_cast<QApplication *>(QApplication::instance())->palette();
    if (dark)
    {
        p.setColor(QPalette::Window, QColor(37, 39, 43));
        p.setColor(QPalette::WindowText, QColor(220, 223, 228));
        p.setColor(QPalette::Base, QColor(28, 30, 34));
        p.setColor(QPalette::AlternateBase, QColor(43, 46, 51));
        p.setColor(QPalette::Text, QColor(220, 223, 228));
        p.setColor(QPalette::Button, QColor(50, 53, 58));
        p.setColor(QPalette::ButtonText, QColor(220, 223, 228));
        p.setColor(QPalette::Highlight, QColor(64, 110, 170));
        p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
        p.setColor(QPalette::ToolTipBase, QColor(60, 63, 68));
        p.setColor(QPalette::ToolTipText, QColor(230, 233, 238));
    }
    else
    {
        p = QApplication::style()->standardPalette();
    }
    QApplication::setPalette(p);

    plot_trajectory_->setTheme(dark);
    plot_position_->setTheme(dark);
    plot_attitude_->setTheme(dark);
    plot_velocity_->setTheme(dark);
    plot_errors_->setTheme(dark);
    plot_innovation_->setTheme(dark);
    plot_covariance_->setTheme(dark);
    plot_consistency_->setTheme(dark);
}

// ------------------------------------------------------------ История

void MainWindow::onRefreshHistory()
{
    if (!store_open_) return;
    history_model_->refresh();
}

void MainWindow::onOpenRun(qint64 runId)
{
    openRun(runId);
}

int MainWindow::openRun(qint64 runId)
{
    if (!store_open_) return 0;

    RunSeries loaded;
    if (!history_model_->loadRun(runId, loaded) || loaded.rows.empty())
    {
        log_->append(tr("Прогон %1 не содержит результатов").arg(runId), 2);
        return 0;
    }

    series_ = loaded;
    plots_dirty_ = true;
    current_run_id_ = runId;
    nav_model_->setSeries(series_);
    updateTrajectory();
    rebuildPlotSeries(plot_tabs_->currentIndex());

    log_->append(tr("Открыт сохранённый прогон %1: %2 точек")
                     .arg(runId)
                     .arg(series_.rows.size()));
    return static_cast<int>(series_.rows.size());
}

int MainWindow::openCompareRun(qint64 runId)
{
    if (!store_open_ || runId < 0) return 0;

    RunSeries loaded;
    if (!history_model_->loadRun(runId, loaded) || loaded.rows.empty())
    {
        log_->append(tr("Прогон %1 не содержит результатов").arg(runId), 2);
        return 0;
    }

    series_b_ = loaded;
    compare_run_id_ = runId;
    compare_diff_action_->setEnabled(true);
    compare_clear_action_->setEnabled(true);

    plots_dirty_ = true;
    rebuildPlotSeries(plot_tabs_->currentIndex());
    if (plot_tabs_->currentWidget() != nullptr) plot_tabs_->currentWidget()->update();

    log_->append(tr("Второй прогон для сравнения: %1 (%2 точек)")
                     .arg(runId)
                     .arg(series_b_.rows.size()));
    return static_cast<int>(series_b_.rows.size());
}

void MainWindow::onCompareSelectRun()
{
    if (!store_open_ || history_model_ == nullptr) return;

    const QVector<RunRecord> &records = history_model_->records();
    if (records.isEmpty())
    {
        log_->append(tr("История прогонов пуста"), 2);
        return;
    }

    QStringList items;
    QVector<qint64> ids;
    for (const RunRecord &r : records)
    {
        items.append(tr("%1: %2 (%3 точек, %4)")
                         .arg(r.id)
                         .arg(r.name.isEmpty() ? tr("без имени") : r.name)
                         .arg(r.steps)
                         .arg(r.created.toString(Qt::ISODate)));
        ids.push_back(r.id);
    }

    bool ok = false;
    const QString chosen = QInputDialog::getItem(
        this, tr("Второй прогон для сравнения"), tr("Прогон:"), items, 0, false, &ok);
    if (!ok) return;

    const int index = items.indexOf(chosen);
    if (index < 0 || index >= ids.size()) return;
    openCompareRun(ids[index]);
}

void MainWindow::onCompareDiffToggled(bool on)
{
    compare_diff_ = on;
    plots_dirty_ = true;
    rebuildPlotSeries(plot_tabs_->currentIndex());
    if (plot_tabs_->currentWidget() != nullptr) plot_tabs_->currentWidget()->update();
}

void MainWindow::onCompareClear()
{
    series_b_.clear();
    compare_run_id_ = -1;
    compare_diff_ = false;
    compare_diff_action_->setChecked(false);
    compare_diff_action_->setEnabled(false);
    compare_clear_action_->setEnabled(false);

    plots_dirty_ = true;
    rebuildPlotSeries(plot_tabs_->currentIndex());
    if (plot_tabs_->currentWidget() != nullptr) plot_tabs_->currentWidget()->update();

    log_->append(tr("Сравнение прогонов отключено"));
}

// ------------------------------------------------------------ Прочее

void MainWindow::saveSettings()
{
    settings_.saveWindowGeometry(this);
}

void MainWindow::showAbout()
{
    QMessageBox::about(
        this, tr("О программе"),
        tr("<h3>imitator_gui</h3>"
           "<p>Qt-обвязка над консольным БИНС/СНС-комплексом (ESKF, 15 состояний).</p>"
           "<p>Математика выполняется в Qt-независимом ядре <code>esfcore</code> — "
           "она совпадает с консольным <code>imitator</code> побитово.</p>"
           "<p>Графики построены собственным кодом на QPainter, без QtCharts.</p>"
           "<p>Qt %1</p>")
            .arg(QString::fromLatin1(qVersion())));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (running_ && worker_ != nullptr)
    {
        const auto answer = QMessageBox::question(
            this, tr("Выход"), tr("Прогон ещё выполняется. Прервать и выйти?"),
            QMessageBox::Yes | QMessageBox::No);
        if (answer != QMessageBox::Yes)
        {
            event->ignore();
            return;
        }
        worker_->requestStop();
    }

    // Дождаться потока расчёта, иначе обращения к destroyed QObject.
    if (thread_ != nullptr && thread_->isRunning())
    {
        thread_->quit();
        thread_->wait(5000);
    }

    settings_ = control_->settings();
    settings_.dark_theme = theme_action_->isChecked();
    saveSettings();
    settings_.save();

    event->accept();
}

} // namespace gui