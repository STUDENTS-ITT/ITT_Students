// MainWindow.cpp — Главное окно imitator_gui.

#include "MainWindow.h"

#include <algorithm>
#include <cmath>

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMenuBar>
#include <QMessageBox>
#include <QLabel>
#include <QLineEdit>
#include <QProgressDialog>
#include <QPushButton>
#include <QRegularExpression>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
#include <QThread>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QtConcurrent>

#include "core/utils/constants.h"
#include "core/data_io/dataset_generator.h"
#include "core/navigation/simulation.h"
#include "export/pdf_report.h"
#include "export/tuning_report.h"
#include "export/plot_exporter.h"
#include "export/animation_exporter.h"
#include "export/matrices_exporter.h"
#include "models/history_model.h"
#include "models/nav_table_model.h"
#include "models/params_table_model.h"
#include "settings/dataset_truth.h"
#include "settings/preset_meta.h"
#include "views/control_panel.h"
#include "views/data_panel.h"
#include "views/log_panel.h"
#include "views/params_panel.h"
#include "views/plots/time_series_plot.h"
#include "views/plots/trajectory_plot.h"
#include "views/state_panel.h"
#include "views/tuning_panel.h"
#include "workers/simulation_worker.h"

namespace gui
{
namespace
{
// Ограничение числа точек на ряд в графике: при 10 минутах прогона это 120 000
// точек на ряд — рисовать незачем, достаточно ~2000.
constexpr int kMaxPlotPoints = 2000;

// Ограничение для PDF-отчёта: график блока занимает ~700×300 px, поэтому 2000
// точек на ряд избыточны и во много раз раздувают векторный файл.
constexpr int kMaxReportPoints = 600;

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

// Прореживание ряда до n точек: равномерная выборка с сохранением обеих
// крайних точек. Применяется и к экранным графикам (kMaxPlotPoints), и к
// блокам PDF-отчёта (kMaxReportPoints).
PlotSeries decimateSeries(const PlotSeries &in, int n)
{
    const int total = in.points.size();
    if (n < 2 || total <= n) return in;

    PlotSeries out = in;
    out.points.clear();
    out.points.reserve(n);
    for (int i = 0; i < n; i++)
    {
        const int idx = static_cast<int>(static_cast<qint64>(i) * (total - 1) / (n - 1));
        out.points.push_back(in.points[idx]);
    }
    return out;
}

// Дополнительное прореживание ряда для PDF (см. kMaxReportPoints).
PlotSeries decimateForReport(const PlotSeries &in)
{
    return decimateSeries(in, kMaxReportPoints);
}

QVector<PlotSeries> decimateForReport(const QVector<PlotSeries> &in)
{
    QVector<PlotSeries> out;
    out.reserve(in.size());
    for (const PlotSeries &s : in) out.push_back(decimateForReport(s));
    return out;
}

// Прореживание рядов для экрана (kMaxPlotPoints): ряд смещений ИМУ приходит по
// каждому такту прогона (десятки тысяч точек), для отрисовки это избыточно.
QVector<PlotSeries> decimateForPlot(const QVector<PlotSeries> &in)
{
    QVector<PlotSeries> out;
    out.reserve(in.size());
    for (const PlotSeries &s : in) out.push_back(decimateSeries(s, kMaxPlotPoints));
    return out;
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
//
// Метки берутся из строк в UTF-8 (исходники компилируются с /utf-8), поэтому
// fromUtf8, а не fromLatin1: иначе кириллица превращалась бы в мусор.
QString errorLabel(int i)
{
    static const char *names[ins::KF_STATE] = {
        "x0 широта", "x1 долгота", "x2 высота", "x3 Vn", "x4 Vh", "x5 Ve",
        "x6 курс", "x7 тангаж", "x8 крен", "x9 ba_x", "x10 ba_y", "x11 ba_z",
        "x12 bg_x", "x13 bg_y", "x14 bg_z"};
    if (i >= 0 && i < ins::KF_STATE) return QString::fromUtf8(names[i]);
    return QString("x%1").arg(i);
}

// Метка диагонали ковариации.
QString covLabel(int i)
{
    static const char *names[ins::KF_STATE] = {
        "P00", "P11", "P22", "P33", "P44", "P55", "P66", "P77", "P88",
        "P99", "P1010", "P1111", "P1212", "P1313", "P1414"};
    if (i >= 0 && i < ins::KF_STATE) return QString::fromUtf8(names[i]);
    return QString("P%1").arg(i);
}

// Метка компоненты инновации.
QString innovationLabel(int i)
{
    static const char *names[ins::KF_MEAS] = {
        "Δширота", "Δдолгота", "Δвысота", "ΔVn", "ΔVh", "ΔVe",
        "Δкурс", "Δтангаж", "Δкрен"};
    if (i >= 0 && i < ins::KF_MEAS) return QString::fromUtf8(names[i]);
    return QString("z%1").arg(i);
}

// Группы компонент вектора ошибок (MainWindow::ErrorGroup): у каждой группы
// свои единицы, поэтому каждая выводится своим графиком. Индекс группы в этом
// файле — обычное число (enum класса недоступен свободным функциям).
int errorGroupFirst(int group)
{
    return 3 * group;
}

// Подпись компоненты j (0..2) группы group в единицах группы: широта и долгота
// переводятся в метры (север/восток), как в таблице отклонений PDF-отчёта.
QString errorGroupLabel(int group, int j)
{
    const int i = errorGroupFirst(group) + j;
    if (group != 0) return errorLabel(i);
    static const char *names[3] = {"широта (север)", "долгота (восток)", "высота"};
    return QString("%1 %2").arg(errorLabel(i), QString::fromUtf8(names[j]));
}

// Значение компоненты j группы group в единицах группы. Широта/долгота в
// состоянии фильтра — радианы: север = x0·R, восток = x1·R·cos φ.
double errorGroupValue(int group, int j, double x, double lat_deg)
{
    if (group != 0) return x;
    if (j == 0) return x * R_EARTH;
    if (j == 1) return x * R_EARTH * std::cos(lat_deg * DEG_TO_RAD);
    return x;
}

// Широта строк результата (град) в моменты кадров коррекции. Оба ряда идут по
// возрастанию времени, поэтому достаточно одного прохода двумя указателями
// (nearestTimeIndex — линейный поиск, в цикле по кадрам он был бы O(n·m)).
QVector<double> latAtCorrections(const std::vector<RunSeries::Row> &rows,
                                 const QVector<double> &ct)
{
    QVector<double> out(ct.size(), 0.0);
    if (rows.empty()) return out;

    int j = 0;
    const int n = static_cast<int>(rows.size());
    for (int k = 0; k < ct.size(); k++)
    {
        while (j + 1 < n && rows[j + 1].time <= ct[k]) j++;
        out[k] = rows[j].result[RunSeries::Lat];
    }
    return out;
}

// Цвет истинной величины на графиках (оценка идёт из общей палитры).
const QColor kTruthColor(0, 150, 60);

// Название величины по колонке результата/истины (1..9). Нумерация колонок в
// RunSeries::ResultField и в TruthSample.v одна и та же, поэтому индекс общий.
QString quantityTitle(int column)
{
    switch (column)
    {
    case RunSeries::Lon: return QObject::tr("Долгота");
    case RunSeries::Lat: return QObject::tr("Широта");
    case RunSeries::Alt: return QObject::tr("Высота");
    case RunSeries::Heading: return QObject::tr("Курс");
    case RunSeries::Pitch: return QObject::tr("Тангаж");
    case RunSeries::Roll: return QObject::tr("Крен");
    case RunSeries::Vn: return QObject::tr("Vn");
    case RunSeries::Ve: return QObject::tr("Ve");
    case RunSeries::Vh: return QObject::tr("Vh");
    default: return QObject::tr("Величина");
    }
}

// Единица измерения величины — подпись оси Y.
QString quantityUnit(int column)
{
    switch (column)
    {
    case RunSeries::Alt: return QObject::tr("м");
    case RunSeries::Vn:
    case RunSeries::Ve:
    case RunSeries::Vh: return QObject::tr("м/с");
    default: return QObject::tr("°");
    }
}

// Страница вкладки с несколькими графиками: у каждой величины свой масштаб и
// легенда, поэтому они раскладываются вертикально, а не рисуются на одном поле.
QWidget *stackPlots(const QVector<PlotView *> &plots)
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);
    for (PlotView *p : plots) layout->addWidget(p, 1);
    return page;
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

    // Счётчики кэша кадров графиков — постоянная метка справа в строке
    // состояния, обновляется раз в полсекунды.
    cache_status_label_ = new QLabel(this);
    statusBar()->addPermanentWidget(cache_status_label_);
    cache_timer_ = new QTimer(this);
    connect(cache_timer_, &QTimer::timeout, this, &MainWindow::onUpdateCacheStatus);
    cache_timer_->start(500);
    onUpdateCacheStatus();
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
    // Координаты, углы и скорости — каждая величина отдельным графиком
    // (с истиной для сравнения), поэтому на вкладке стопка из трёх графиков.
    plot_tabs_->addTab(stackPlots({plot_lat_, plot_lon_, plot_alt_}), tr("Координаты"));
    plot_tabs_->addTab(stackPlots({plot_heading_, plot_pitch_, plot_roll_}), tr("Углы"));
    plot_tabs_->addTab(stackPlots({plot_vn_, plot_ve_, plot_vh_}), tr("Скорости"));
    // Вектор ошибок x — стопка графиков по единицам измерения: на одной оси
    // теперь метры, м/с, рад, м/с² и рад/с не смешиваются.
    plot_tabs_->addTab(stackPlots({plot_err_pos_, plot_err_vel_, plot_err_att_,
                                   plot_err_ba_, plot_err_bg_}),
                       tr("Ошибки x"));
    plot_tabs_->addTab(plot_innovation_, tr("Инновации СНС"));
    plot_tabs_->addTab(plot_covariance_, tr("Ковариация P"));
    plot_tabs_->addTab(plot_consistency_, tr("Согласованность"));
    // Смещения ИМУ (этап 6 плана): оценка фильтра, полоса ±2σ(P) и истина
    // набора из params.json — по графику на компоненту.
    plot_tabs_->addTab(stackPlots({plot_bias_ba_x_, plot_bias_ba_y_, plot_bias_ba_z_,
                                   plot_bias_bg_x_, plot_bias_bg_y_, plot_bias_bg_z_}),
                       tr("Смещения ИМУ"));
    right->addTab(plot_tabs_, tr("Графики"));

    data_.reset(new DataPanel);
    right->addTab(data_.data(), tr("Данные"));

    params_.reset(new ParamsPanel);
    right->addTab(params_.data(), tr("Параметры"));

    tuning_.reset(new TuningPanel);
    tuning_->setDataDir(settings_.data_dir);
    right->addTab(tuning_.data(), tr("Подбор"));

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
    QAction *save_settings_as = new QAction(tr("Сохранить settings.ini как…"), this);
    run_menu->addAction(save_settings_as);
    run_menu->addSeparator();
    QAction *reset_params = new QAction(tr("Сбросить параметры к умолчаниям"), this);
    run_menu->addAction(reset_params);
    QAction *apply_params = new QAction(tr("Применить параметры к фильтру"), this);
    run_menu->addAction(apply_params);

    run_menu->addSeparator();
    QAction *gen_dataset = new QAction(tr("Сгенерировать тестовые данные..."), this);
    run_menu->addAction(gen_dataset);

    connect(load_settings, &QAction::triggered, this, &MainWindow::onLoadSettingsFile);
    connect(save_settings, &QAction::triggered, this, &MainWindow::onSaveSettingsFile);
    connect(save_settings_as, &QAction::triggered, this, &MainWindow::onSaveSettingsAs);
    connect(reset_params, &QAction::triggered, this, &MainWindow::onResetParams);
    connect(apply_params, &QAction::triggered, this, &MainWindow::onApplyParams);
    connect(gen_dataset, &QAction::triggered, this, &MainWindow::onGenerateDataset);

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

    cache_action_ = new QAction(tr("Кэш кадров графиков"), this);
    cache_action_->setCheckable(true);
    cache_action_->setChecked(true);
    view_menu->addAction(cache_action_);
    connect(cache_action_, &QAction::toggled, this, &MainWindow::onToggleCache);

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
    QAction *export_series_csv = new QAction(tr("Данные активного графика в CSV…"), this);
    export_menu->addAction(export_series_csv);
    QAction *export_matrices = new QAction(tr("Матрицы фильтра (Q/R/P0/H)…"), this);
    export_menu->addAction(export_matrices);
    QAction *export_all = new QAction(tr("Все графики…"), this);
    export_menu->addAction(export_all);
    QAction *export_animation = new QAction(tr("Анимация…"), this);
    export_menu->addAction(export_animation);
    QAction *save_log = new QAction(tr("Лог в файл…"), this);
    export_menu->addAction(save_log);

    connect(export_png, &QAction::triggered, this, &MainWindow::onExportPlot);
    connect(export_svg, &QAction::triggered, this, &MainWindow::onExportSvg);
    connect(export_pdf, &QAction::triggered, this, &MainWindow::onExportPdf);
    connect(export_csv, &QAction::triggered, this, &MainWindow::onExportCsv);
    connect(export_series_csv, &QAction::triggered, this, &MainWindow::onExportSeriesCsv);
    connect(export_matrices, &QAction::triggered, this, &MainWindow::onExportMatrices);
    connect(export_all, &QAction::triggered, this, &MainWindow::onExportAllPlots);
    connect(export_animation, &QAction::triggered, this, &MainWindow::onExportAnimation);

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

    // Временные графики величин: у каждой свой виджет, чтобы на графике была
    // ровно одна величина (оценка) и её истина — без наложения осей.
    const auto add_time_plot = [this](TimeSeriesPlot *&plot, int column) {
        plot = new TimeSeriesPlot;
        plot->setTitle(quantityTitle(column));
        plot->setAxisLabels(tr("Время, с"), quantityUnit(column));
        plot->setMarginLeft(70);
    };

    add_time_plot(plot_lat_, RunSeries::Lat);
    add_time_plot(plot_lon_, RunSeries::Lon);
    add_time_plot(plot_alt_, RunSeries::Alt);
    add_time_plot(plot_heading_, RunSeries::Heading);
    add_time_plot(plot_pitch_, RunSeries::Pitch);
    add_time_plot(plot_roll_, RunSeries::Roll);
    add_time_plot(plot_vn_, RunSeries::Vn);
    add_time_plot(plot_ve_, RunSeries::Ve);
    add_time_plot(plot_vh_, RunSeries::Vh);

    // Вектор ошибок x — по графику на группу единиц измерения: координаты (м),
    // скорости (м/с), углы (рад), смещения акселя (м/с²), смещения гироскопа
    // (рад/с). Раньше все 15 компонент лежали на одной оси «ошибка».
    const auto add_error_plot = [](TimeSeriesPlot *&plot, const QString &title,
                                   const QString &unit) {
        plot = new TimeSeriesPlot;
        plot->setTitle(title);
        plot->setAxisLabels(QObject::tr("Время, с"), unit);
        plot->setMarginLeft(88);
        plot->setVisibilityThreshold(1e-18);
        plot->setLegendVisible(true);
    };
    add_error_plot(plot_err_pos_, tr("Вектор ошибок x: координаты"), tr("м"));
    add_error_plot(plot_err_vel_, tr("Вектор ошибок x: скорости"), tr("м/с"));
    add_error_plot(plot_err_att_, tr("Вектор ошибок x: углы"), tr("рад"));
    add_error_plot(plot_err_ba_, tr("Вектор ошибок x: смещение акселя"), tr("м/с²"));
    add_error_plot(plot_err_bg_, tr("Вектор ошибок x: смещение гироскопа"), tr("рад/с"));

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

    // Смещения ИМУ: одна компонента на график — оценка фильтра, полоса ±2σ(P)
    // и истина набора (params.json) в инженерных единицах (мг и °/ч) — тех же,
    // что в отчёте по смещениям и в params.json.
    const auto add_bias_plot = [](TimeSeriesPlot *&plot, int index) {
        plot = new TimeSeriesPlot;
        plot->setTitle(PdfReport::biasCaption(index));
        plot->setAxisLabels(QObject::tr("Время, с"), PdfReport::biasUnit(index));
        plot->setMarginLeft(88);
        plot->setLegendVisible(true);
    };
    add_bias_plot(plot_bias_ba_x_, 0);
    add_bias_plot(plot_bias_ba_y_, 1);
    add_bias_plot(plot_bias_ba_z_, 2);
    add_bias_plot(plot_bias_bg_x_, 3);
    add_bias_plot(plot_bias_bg_y_, 4);
    add_bias_plot(plot_bias_bg_z_, 5);

    // Холст отчёта: виджет вне вкладок, нужен только для отрисовки блоков
    // графиков в PDF. Родитель — окно, чтобы он жил и удалялся вместе с ним.
    // В layout он не входит, поэтому его обязательно прячем: иначе пустой
    // график (сетка 0…1) рисуется поверх окна в левом верхнем углу.
    report_canvas_ = new TimeSeriesPlot(this);
    report_canvas_->hide();
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
        tuning_->setDataDir(dir);
    });

    connect(params_.data(), &ParamsPanel::settingsEdited, this, &MainWindow::onSettingsChanged);
    connect(params_.data(), &ParamsPanel::runSelected, this, &MainWindow::onOpenRun);

    connect(tuning_.data(), &TuningPanel::logMessage, this, &MainWindow::onTuningLog);
    connect(tuning_.data(), &TuningPanel::applyPresetRequested, this,
            &MainWindow::onApplyTuningPreset);
    connect(tuning_.data(), &TuningPanel::tuningFinished, this, &MainWindow::onTuningFinished);
    connect(tuning_.data(), &TuningPanel::tuningReportRequested, this,
            &MainWindow::onTuningReport);

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
    plot_trajectory_->clearTruth();
    for (TimeSeriesPlot *plot : {plot_err_pos_, plot_err_vel_, plot_err_att_, plot_err_ba_,
                                 plot_err_bg_})
        plot->clearSyncMarkers();
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
            // Эталон СНС сохраняем вместе с отсчётом: при открытии прогона из
            // истории он нужен для колонки отклонений в таблице и экспорта
            // ряда, а также как опора отчёта, если истины в наборе нет.
            s.has_reference = r.has_reference;
            s.ref_lon = r.reference[RunSeries::Lon];
            s.ref_lat = r.reference[RunSeries::Lat];
            s.ref_alt = r.reference[RunSeries::Alt];
            s.ref_heading = r.reference[RunSeries::Heading];
            s.ref_pitch = r.reference[RunSeries::Pitch];
            s.ref_roll = r.reference[RunSeries::Roll];
            s.ref_vn = r.reference[RunSeries::Vn];
            s.ref_vh = r.reference[RunSeries::Vh];
            s.ref_ve = r.reference[RunSeries::Ve];
            samples.push_back(s);
        }

        const QString name = QString("%1  %2")
                                 .arg(QDateTime::currentDateTime().toString("dd.MM.yyyy HH:mm:ss"),
                                      settings_.data_dir);
        // Снимок параметров фильтра: применённые в этом прогоне значения
        // (таблица) сериализуются в INI и восстанавливаются при открытии.
        const QString settings_ini =
            QString::fromStdString(nav::dumpFilterSettings(params_->currentSettings()));
        current_run_id_ =
            store_.beginRun(name, summary, settings_.data_dir, QString(), settings_ini);
        if (current_run_id_ > 0)
        {
            store_.appendSamples(current_run_id_, samples);

            std::vector<double> times;
            std::vector<std::vector<double>> errors;
            times.reserve(series_.corrections.size());
            errors.reserve(series_.corrections.size());
            // Ряды согласованности — те же кадры коррекции: сохраняем, чтобы
            // отчёт по прогону из истории не терял раздел NIS/NEES.
            std::vector<double> nis;
            std::vector<double> nees;
            nis.reserve(series_.corrections.size());
            nees.reserve(series_.corrections.size());
            for (const RunSeries::CorrectionRow &c : series_.corrections)
            {
                times.push_back(c.time);
                std::vector<double> x(ins::KF_STATE, 0.0);
                for (int i = 0; i < ins::KF_STATE; i++) x[i] = c.x[i];
                errors.push_back(x);
                nis.push_back(c.nis);
                nees.push_back(c.nees);
            }
            store_.appendErrors(current_run_id_, times, errors, nis, nees);

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

        // Оценки смещений ИМУ (СИ) — для вкладки «Смещения ИМУ»: фильтр держит
        // их в NavState::ba/bg, а не в векторе ошибок x[9..14].
        for (int i = 0; i < 3 && i < s.ba.size(); i++) row.ba[i] = s.ba[i];
        for (int i = 0; i < 3 && i < s.bg.size(); i++) row.bg[i] = s.bg[i];
        series_.has_bias = true;

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
    ensureTruthLoaded();
    ensureBiasTruthLoaded();
    plots_dirty_ = false;

    switch (tab)
    {
    case TabTrajectory:
        // Траектория рисуется через updateTrajectory().
        return;

    case TabCoordinates:  // Координаты: широта, долгота, высота
        if (series_.rows.empty()) return;
        plot_lat_->setSeries(quantitySeries(RunSeries::Lat));
        plot_lon_->setSeries(quantitySeries(RunSeries::Lon));
        plot_alt_->setSeries(quantitySeries(RunSeries::Alt));
        return;

    case TabAngles:  // Углы: курс, тангаж, крен
        if (series_.rows.empty()) return;
        plot_heading_->setSeries(quantitySeries(RunSeries::Heading));
        plot_pitch_->setSeries(quantitySeries(RunSeries::Pitch));
        plot_roll_->setSeries(quantitySeries(RunSeries::Roll));
        return;

    case TabVelocities:  // Скорости: vn, ve, vh
        if (series_.rows.empty()) return;
        plot_vn_->setSeries(quantitySeries(RunSeries::Vn));
        plot_ve_->setSeries(quantitySeries(RunSeries::Ve));
        plot_vh_->setSeries(quantitySeries(RunSeries::Vh));
        return;

    case TabErrors:  // Вектор ошибок x (кадры коррекции) — по группе единиц
    {
        if (series_.corrections.empty()) return;
        TimeSeriesPlot *plots[ErrorGroupCount] = {plot_err_pos_, plot_err_vel_, plot_err_att_,
                                                 plot_err_ba_, plot_err_bg_};
        // Широта кадров нужна группе «координаты» (перевод рад → м): считается
        // один раз на перерисовку, а не в каждой группе.
        const QVector<double> ct = correctionTimes();
        const QVector<double> lat = latAtCorrections(series_.rows, ct);
        for (int g = 0; g < ErrorGroupCount; g++)
        {
            const QVector<PlotSeries> out = errorSeries(g, lat);
            if (out.isEmpty()) continue;
            plots[g]->setSyncMarkers(ct);
            plots[g]->setSeries(out);
        }
        return;
    }

    case TabInnovation:  // Инновации измерений СНС
    {
        const QVector<PlotSeries> out = innovationSeries();
        if (out.isEmpty()) return;
        plot_innovation_->setSyncMarkers(correctionTimes());
        plot_innovation_->setVisibilityThreshold(1e-18);
        plot_innovation_->setSeries(out);
        return;
    }

    case TabCovariance:  // Диагонали ковариации P
    {
        const QVector<PlotSeries> out = covarianceSeries();
        if (out.isEmpty()) return;
        plot_covariance_->setSeries(out);
        return;
    }

    case TabConsistency:  // NIS / NEES на кадрах коррекции СНС
    {
        const QVector<PlotSeries> out = consistencySeries();
        if (out.isEmpty()) return;
        plot_consistency_->setSyncMarkers(correctionTimes());
        plot_consistency_->setVisibilityThreshold(1e-12);
        plot_consistency_->setSeries(out);
        return;
    }

    case TabBiases:  // Оценки смещений ИМУ: оценка, ±2σ(P), истина
    {
        if (!series_.has_bias) return;
        TimeSeriesPlot *plots[6] = {plot_bias_ba_x_, plot_bias_ba_y_, plot_bias_ba_z_,
                                    plot_bias_bg_x_, plot_bias_bg_y_, plot_bias_bg_z_};
        for (int i = 0; i < 6; i++)
        {
            const QVector<PlotSeries> out = biasSeries(i);
            if (out.isEmpty()) continue;
            plots[i]->setSeries(decimateForPlot(out));
        }
        return;
    }

    default:
        return;
    }
}

// ------------------------------------------------- Ряды временных графиков
//
// Ряды строятся из накопленной телеметрии и общих для экрана и PDF функций:
// один и тот же ряд уходит и в виджет вкладки, и в блок отчёта. Поэтому отчёт
// не зависит от того, какая вкладка открыта (в self-test активна «Траектория»).

void MainWindow::ensureTruthLoaded()
{
    if (truth_dir_ == settings_.data_dir) return;
    truth_dir_ = settings_.data_dir;
    truth_.clear();
    loadDatasetTruth(settings_.data_dir, &truth_);
}

// Истинные смещения ИМУ набора (params.json, раздел noise_si) для вкладки
// «Смещения ИМУ» и блоков отчёта. Файла может не быть (реальные записи,
// наборы без паспорта) — тогда истина просто не рисуется.
void MainWindow::ensureBiasTruthLoaded()
{
    if (bias_truth_dir_ == settings_.data_dir) return;
    bias_truth_dir_ = settings_.data_dir;
    bias_truth_ = nav::BiasTruth();
    if (settings_.data_dir.isEmpty()) return;

    const QString path = QDir(settings_.data_dir).filePath(QStringLiteral("params.json"));
    nav::loadBiasTruth(path.toStdString(), bias_truth_, nullptr);
}

QVector<double> MainWindow::correctionTimes() const
{
    QVector<double> ct(static_cast<int>(series_.corrections.size()));
    for (int i = 0; i < ct.size(); i++) ct[i] = series_.corrections[i].time;
    return ct;
}

// Ряд одной величины: оценка БИНС и истина (truth.txt) в одних единицах.
// Колонки результата и строки истины пронумерованы одинаково (Lat..Ve).
QVector<PlotSeries> MainWindow::quantitySeries(int column) const
{
    QVector<PlotSeries> out;
    if (series_.rows.empty()) return out;

    const QVector<double> times = rowTimes(series_.rows);
    QVector<double> est(times.size());
    for (int i = 0; i < times.size(); i++) est[i] = series_.rows[i].result[column];
    out.push_back(makeSeries(quantityTitle(column), times, est, 0));

    if (!truth_.isEmpty())
    {
        QVector<double> tt;
        QVector<double> tv;
        tt.reserve(truth_.size());
        tv.reserve(truth_.size());
        for (const TruthSample &s : truth_)
        {
            tt.push_back(s.v[0]);
            tv.push_back(s.v[column]);
        }
        PlotSeries truth = makeSeries(tr("Истина"), tt, tv, 3);
        truth.color = kTruthColor;
        truth.dashed = true;
        out.push_back(truth);
    }

    appendCompareField(out, column);
    return out;
}

// Вектор ошибок x на кадрах коррекции СНС — компоненты одной группы единиц
// измерения (ErrorGroup). В группе «координаты» широта и долгота переводятся
// в метры (север/восток) по широте кадра: иначе на оси смешивались бы рад и м.
QVector<PlotSeries> MainWindow::errorSeries(int group, const QVector<double> &lat_deg) const
{
    QVector<PlotSeries> out;
    const int m = static_cast<int>(series_.corrections.size());
    if (m == 0) return out;
    if (group < 0 || group >= ErrorGroupCount) return out;

    const QVector<double> ct = correctionTimes();
    const int first = errorGroupFirst(group);
    for (int j = 0; j < 3; j++)
    {
        const int i = first + j;
        QVector<double> v(m);
        for (int k = 0; k < m; k++)
        {
            const double lat = (k < lat_deg.size()) ? lat_deg[k] : 0.0;
            v[k] = errorGroupValue(group, j, series_.corrections[k].x[i], lat);
        }
        out.push_back(makeSeries(errorGroupLabel(group, j), ct, v, j, true));
    }
    appendCompareSeries(out, TabErrors, group);
    return out;
}

int MainWindow::biasSeriesCount()
{
    ensureBiasTruthLoaded();
    int total = 0;
    for (int i = 0; i < 6; i++) total += biasSeries(i).size();
    return total;
}

// Смещения ИМУ на каждом такте: оценка фильтра, полоса ±2σ(P) и истина набора
// (та же сборка ряда, что уходит блоками в PDF-отчёт).
QVector<PlotSeries> MainWindow::biasSeries(int index) const
{
    // У прогонов, открытых из истории SQLite, оценок смещений нет: в таблице
    // прогонов хранятся только результат, диагонали P и NIS/NEES. Вкладка и
    // блоки отчёта для таких прогонов остаются пустыми (has_bias).
    if (!series_.has_bias) return QVector<PlotSeries>();
    return PdfReport::biasSeries(series_, bias_truth_, index);
}

// Инновации измерений СНС (нулевые компоненты не рисуем).
QVector<PlotSeries> MainWindow::innovationSeries() const
{
    QVector<PlotSeries> out;
    const int m = static_cast<int>(series_.corrections.size());
    if (m == 0) return out;

    const QVector<double> ct = correctionTimes();
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
    appendCompareSeries(out, TabInnovation);
    return out;
}

// Диагонали ковариации P по всем тактам.
QVector<PlotSeries> MainWindow::covarianceSeries() const
{
    QVector<PlotSeries> out;
    if (series_.rows.empty()) return out;

    const QVector<double> times = rowTimes(series_.rows);
    for (int i = 0; i < ins::KF_STATE; i++)
    {
        QVector<double> v(times.size());
        for (int k = 0; k < times.size(); k++) v[k] = series_.rows[k].p[i];
        out.push_back(makeSeries(covLabel(i), times, v, i));
    }
    return out;
}

// NIS/NEES на кадрах коррекции СНС с опорной линией ожидания.
QVector<PlotSeries> MainWindow::consistencySeries() const
{
    QVector<PlotSeries> out;
    const int m = static_cast<int>(series_.consistency.size());
    if (m == 0) return out;

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
    if (dof <= 0) return out;

    // Опорная линия — ожидаемое значение хи-квадрат: в согласованном фильтре
    // NIS и NEES тяготеют к числу степеней свободы.
    const QVector<double> expect(m, static_cast<double>(dof));
    out.push_back(makeSeries(tr("ожидание (%1)").arg(dof), ct, expect, 6, true));
    out.push_back(makeSeries(tr("NIS"), ct, nis, 0, true));
    out.push_back(makeSeries(tr("NEES"), ct, nees, 1, true));
    appendCompareSeries(out, TabConsistency);
    return out;
}

// Блоки графиков для сводного PDF-отчёта (план 2.9).
//
// Ряды строятся теми же функциями, что и вкладки, поэтому отчёт содержит все
// графики независимо от открытой вкладки: каждая величина отдельно, с истиной
// для сравнения, плюс согласованность фильтра (NIS / NEES).
QVector<PdfReport::PlotBlock> MainWindow::buildReportBlocks() const
{
    QVector<PdfReport::PlotBlock> blocks;

    // Траектория — отдельным виджетом: у него свои оси (восток/север, м) и
    // разметка (оценка БИНС, точки СНС, истина).
    PdfReport::PlotBlock traj;
    traj.caption = tr("Траектория (восток/север, м)");
    traj.widget = plot_trajectory_;
    blocks.push_back(traj);

    // Временной ряд одной величины: оценка и истина на одном поле. После
    // графика величины добавляется её ошибочный ряд — знаковое отклонение от
    // опоры в метрах/градусах/м·с⁻¹ (единицы таблицы отклонений).
    const auto add_quantity = [this, &blocks](int column) {
        const QVector<PlotSeries> s = quantitySeries(column);
        if (!s.isEmpty())
        {
            PdfReport::PlotBlock b;
            b.caption = quantityTitle(column);
            b.series = decimateForReport(s);
            b.x_label = tr("Время, с");
            b.y_label = quantityUnit(column);
            blocks.push_back(b);
        }

        // Окно выставки для графика ошибок не режем: на графике должна быть
        // видна сходимость фильтра, в отличие от таблицы отклонений.
        const QVector<PlotSeries> e = PdfReport::errorSeries(series_, truth_, column);
        if (e.isEmpty()) return;
        PdfReport::PlotBlock eb;
        eb.caption = tr("Ошибка: %1").arg(quantityTitle(column));
        eb.series = decimateForReport(e);
        eb.x_label = tr("Время, с");
        eb.y_label = PdfReport::errorUnit(column);
        blocks.push_back(eb);
    };

    add_quantity(RunSeries::Lat);
    add_quantity(RunSeries::Lon);
    add_quantity(RunSeries::Alt);
    add_quantity(RunSeries::Heading);
    add_quantity(RunSeries::Pitch);
    add_quantity(RunSeries::Roll);
    add_quantity(RunSeries::Vn);
    add_quantity(RunSeries::Ve);
    add_quantity(RunSeries::Vh);

    const auto add_diagnostic = [this, &blocks](const QString &caption, const QString &unit,
                                                const QVector<PlotSeries> &s,
                                                const QVector<double> &markers) {
        if (s.isEmpty()) return;
        PdfReport::PlotBlock b;
        b.caption = caption;
        b.series = decimateForReport(s);
        b.sync_markers = markers;
        b.x_label = tr("Время, с");
        b.y_label = unit;
        blocks.push_back(b);
    };

    add_diagnostic(tr("Согласованность фильтра (NIS / NEES)"), tr("статистика"),
                   consistencySeries(), correctionTimes());

    // Смещения ИМУ (этап 6 плана): оценка фильтра, полоса ±2σ(P) и истина набора
    // из params.json — по блоку на компоненту, в инженерных единицах (мг, °/ч).
    for (int i = 0; i < 6; i++)
    {
        const QVector<PlotSeries> s = biasSeries(i);
        if (s.isEmpty()) continue;
        PdfReport::PlotBlock b;
        b.caption = PdfReport::biasCaption(i);
        b.series = decimateForReport(s);
        b.x_label = tr("Время, с");
        b.y_label = PdfReport::biasUnit(i);
        blocks.push_back(b);
    }

    return blocks;
}

// Сравнение двух прогонов (план 2.4).
//
// Ряды второго прогона добавляются пунктиром в те же графики, что и основной
// прогон. Поскольку прогоны могут иметь разные отметки времени, разность
// считается по ближайшему по времени отсчёту второго прогона: оба считались на
// одной и той же записи ИМУ, поэтому временные сетки практически совпадают.

// Сравнение для одной величины (Lat..Ve): ряд второго прогона и, если включено,
// разность «текущий минус второй» на том же графике.
void MainWindow::appendCompareField(QVector<PlotSeries> &out, int column) const
{
    if (!hasComparison()) return;

    const RunSeries &B = series_b_;
    const QVector<double> at = rowTimes(series_.rows);
    const QVector<double> bt = rowTimes(B.rows);
    const QString suffix = tr(" [прогон %1]").arg(compare_run_id_);
    const QString label = quantityTitle(column);

    QVector<double> bv(bt.size());
    for (int i = 0; i < bt.size(); i++) bv[i] = B.rows[i].result[column];
    out.push_back(makeSeries(label + suffix, bt, bv, 1, false, true));

    if (!compare_diff_) return;

    QVector<double> av(at.size());
    for (int i = 0; i < at.size(); i++) av[i] = series_.rows[i].result[column];

    QVector<double> diff;
    diff.reserve(at.size());
    for (int i = 0; i < at.size(); i++)
    {
        const int j = nearestTimeIndex(B.rows, at[i]);
        if (j < 0) continue;
        diff.push_back(av[i] - B.rows[j].result[column]);
    }
    if (!diff.isEmpty())
        out.push_back(makeSeries(tr("Δ %1").arg(label), at, diff, 6, false, true));
}

void MainWindow::appendCompareSeries(QVector<PlotSeries> &out, PlotTab tab, int group) const
{
    if (!hasComparison()) return;

    const RunSeries &B = series_b_;
    const QString suffix = tr(" [прогон %1]").arg(compare_run_id_);

    // Диагонали P — покомпонентно, по временам второго прогона.
    if (tab == TabCovariance)
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

    // Компоненты вектора ошибок x той же группы, что и на экране: в группе
    // «координаты» — те же метры (север/восток), что у основного прогона.
    if (tab == TabErrors)
    {
        QVector<double> b_lat;
        if (group == ErrPosition) b_lat = latAtCorrections(B.rows, ct);
        const int first = errorGroupFirst(group);
        for (int j = 0; j < 3; j++)
        {
            const int i = first + j;
            QVector<double> bv(ct.size());
            for (int k = 0; k < bv.size(); k++)
            {
                const double lat = (k < b_lat.size()) ? b_lat[k] : 0.0;
                bv[k] = errorGroupValue(group, j, B.corrections[k].x[i], lat);
            }
            out.push_back(makeSeries(errorGroupLabel(group, j) + suffix, ct, bv, j, true, true));
        }
        return;
    }

    // Инновации СНС второго прогона (нулевые компоненты, как и на экране,
    // пропускаются: без angle.dat курс не измеряется).
    if (tab == TabInnovation)
    {
        int color = 0;
        for (int i = 0; i < ins::KF_MEAS; i++)
        {
            QVector<double> bv(ct.size());
            bool nonzero = false;
            for (int k = 0; k < bv.size(); k++)
            {
                bv[k] = B.corrections[k].innovation[i];
                if (std::abs(bv[k]) > 0.0) nonzero = true;
            }
            if (!nonzero) continue;
            out.push_back(
                makeSeries(innovationLabel(i) + suffix, ct, bv, color++, true, true));
        }
        return;
    }

    // Согласованность NIS/NEES второго прогона — из своего разреженного ряда.
    if (tab == TabConsistency && !B.consistency.empty())
    {
        QVector<double> bt(static_cast<int>(B.consistency.size()));
        QVector<double> bn(bt.size());
        QVector<double> be(bt.size());
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

void MainWindow::buildTrajectoryPoints(QVector<QPointF> *nav, QVector<QPointF> *truth)
{
    nav->clear();
    if (truth != nullptr) truth->clear();
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

    // Шаг прореживания: на экране всё равно не видно больше ~2000 точек.
    const std::size_t total = series_.rows.size();
    const std::size_t step = std::max<std::size_t>(1, total / kMaxPlotPoints);

    for (std::size_t i = 0; i < total; i += step)
    {
        const RunSeries::Row &r = series_.rows[i];
        const double d_n = (r.result[RunSeries::Lat] * DEG_TO_RAD - lat0) * R_EARTH;
        const double d_e = (r.result[RunSeries::Lon] * DEG_TO_RAD - lon0) * R_EARTH *
                           std::cos(lat0);
        nav->push_back(QPointF(d_e, d_n));
    }

    // Последняя точка траектории всегда рисуется — конец прогона важен.
    if (step > 1 && ((total - 1) % step) != 0)
    {
        const RunSeries::Row &r = series_.rows.back();
        const double d_n = (r.result[RunSeries::Lat] * DEG_TO_RAD - lat0) * R_EARTH;
        const double d_e = (r.result[RunSeries::Lon] * DEG_TO_RAD - lon0) * R_EARTH *
                           std::cos(lat0);
        nav->push_back(QPointF(d_e, d_n));
    }

    // Истинная траектория (truth.txt) — в тех же местных координатах. Колонки
    // истины совпадают с result, поэтому индексы RunSeries::Lat/Lon подходят.
    if (truth != nullptr && !truth_.isEmpty())
    {
        const std::size_t ts = truth_.size();
        const std::size_t tstep = std::max<std::size_t>(1, ts / kMaxPlotPoints);
        const auto add_truth = [&](const TruthSample &s) {
            const double tn = (s.v[RunSeries::Lat] * DEG_TO_RAD - lat0) * R_EARTH;
            const double te = (s.v[RunSeries::Lon] * DEG_TO_RAD - lon0) * R_EARTH * std::cos(lat0);
            truth->push_back(QPointF(te, tn));
        };
        for (std::size_t i = 0; i < ts; i += tstep) add_truth(truth_[i]);
        if (tstep > 1 && ((ts - 1) % tstep) != 0) add_truth(truth_.back());
    }
}

void MainWindow::updateTrajectory()
{
    if (series_.rows.empty()) return;

    ensureTruthLoaded();

    QVector<QPointF> nav;
    QVector<QPointF> truth;
    buildTrajectoryPoints(&nav, &truth);

    plot_trajectory_->setNavPoints(nav);
    plot_trajectory_->setTruthPoints(truth);
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

    QWidget *page = plot_tabs_->currentWidget();
    if (page == nullptr) return nullptr;
    if (auto *plot = qobject_cast<PlotView *>(page)) return plot;

    // Вкладка-стопка (координаты/углы/скорости): активен первый график.
    const auto nested = page->findChildren<PlotView *>();
    return nested.isEmpty() ? nullptr : nested.first();
}

QList<PlotView *> MainWindow::allPlots() const
{
    QList<PlotView *> plots;
    if (plot_tabs_ == nullptr) return plots;
    for (int i = 0; i < plot_tabs_->count(); ++i)
    {
        QWidget *page = plot_tabs_->widget(i);
        if (auto *plot = qobject_cast<PlotView *>(page))
        {
            plots.push_back(plot);
            continue;
        }
        // Стопка графиков на вкладке — собираем вложенные в порядке укладки.
        const auto nested = page->findChildren<PlotView *>();
        for (PlotView *plot : nested) plots.push_back(plot);
    }
    return plots;
}

void MainWindow::cacheCounters(quint64 *hits, quint64 *misses) const
{
    quint64 h = 0, m = 0;
    const auto plots = allPlots();
    for (const PlotView *plot : plots)
    {
        h += plot->cacheHits();
        m += plot->cacheMisses();
    }
    if (hits) *hits = h;
    if (misses) *misses = m;
}

void MainWindow::onToggleCache(bool on)
{
    const auto plots = allPlots();
    for (PlotView *plot : plots) plot->setCacheEnabled(on);
    onUpdateCacheStatus();
}

void MainWindow::onUpdateCacheStatus()
{
    if (cache_status_label_ == nullptr) return;

    quint64 hits = 0, misses = 0;
    cacheCounters(&hits, &misses);

    quint64 total = hits + misses;
    const int percent = total > 0 ? static_cast<int>((100 * hits) / total) : 0;
    const QString state = cache_action_ != nullptr && cache_action_->isChecked()
                              ? tr("вкл")
                              : tr("выкл");
    cache_status_label_->setText(tr("кэш %1: %2/%3 (%4%)")
                                     .arg(state)
                                     .arg(hits)
                                     .arg(misses)
                                     .arg(percent));
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

// Предупреждение, если пресет подобран на другом наборе: [meta].dataset не
// совпадает с именем текущего каталога данных. Пресет применяется в любом
// случае — это подсказка, а не запрет; старый пресет без [meta] пропускаем.
void MainWindow::warnIfPresetDatasetMismatch(const QString &preset_path)
{
    const PresetMeta meta = readPresetMeta(preset_path);
    QString warning;
    if (presetMatchesDataset(meta, control_->settings().data_dir, &warning) ||
        warning.isEmpty())
    {
        return;
    }
    log_->append(warning, 1);  // уровень 1 даёт префикс «[ВНИМАНИЕ]»
    statusBar()->showMessage(warning, 10000);
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
    warnIfPresetDatasetMismatch(path);
}

void MainWindow::onSaveSettingsFile()
{
    if (settings_.settings_file.isEmpty())
    {
        onSaveSettingsAs();
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

void MainWindow::onSaveSettingsAs()
{
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить параметры как"),
        settings_.settings_file.isEmpty() ? gui::AppSettings::defaultSettingsFile()
                                          : settings_.settings_file,
        tr("INI-файлы (*.ini);;Все файлы (*)"));
    if (path.isEmpty()) return;

    QString error;
    if (!params_->saveSettings(path, &error))
    {
        QMessageBox::warning(this, tr("Параметры"), error);
        return;
    }

    settings_.settings_file = path;
    control_->setSettings(settings_);
    saveSettings();
    log_->append(tr("Параметры сохранены в %1").arg(path));
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

// ------------------------------------------------------------ Подбор

void MainWindow::onApplyTuningPreset(const QString &path)
{
    QString error;
    if (!params_->loadSettings(path, &error))
    {
        QMessageBox::warning(this, tr("Подбор"), error);
        return;
    }

    // Таблица параметров обновлена; применяем к ядру и сохраняем как
    // settings.ini, чтобы следующий прогон шёл с подобранными коэффициентами.
    params_->applyToCore();
    saveSettings();
    log_->append(tr("Подобранный пресет применён: %1").arg(path));
    warnIfPresetDatasetMismatch(path);
}

void MainWindow::onTuningLog(const QString &text, int level)
{
    log_->append(text, level);
}

void MainWindow::onTuningFinished(bool ok)
{
    statusBar()->showMessage(ok ? tr("Подбор завершён")
                                : tr("Подбор завершился с ошибкой"),
                             ok ? 10000 : 0);
}

// «Отчёт о подборе»: приложение само собирает PDF из артефактов тюнера
// (best_params.txt, eval.json) по выбранному конфигу.
void MainWindow::onTuningReport(const QString &config_path)
{
    // Подпапка и имя файла — из набора (data_dir) в конфиге:
    // reports/<набор>/tuning_<набор>.pdf.
    QString dataset = TuningReport::datasetFor(config_path);
    if (dataset.isEmpty())
    {
        dataset = QFileInfo(config_path).completeBaseName();
        dataset.remove(QRegularExpression(QStringLiteral("^tuner_config\\.?")));
    }
    if (dataset.isEmpty()) dataset = QStringLiteral("dataset");
    const QString suggested =
        QDir(QDir(AppSettings::defaultReportsDir()).filePath(dataset))
            .filePath(QStringLiteral("tuning_%1.pdf").arg(dataset));

    const QString path = QFileDialog::getSaveFileName(
        this, tr("Отчёт о подборе"), suggested, tr("PDF (*.pdf)"));
    if (path.isEmpty()) return;

    QString error;
    if (!TuningReport::build(config_path, path, &error))
    {
        QMessageBox::warning(this, tr("Отчёт о подборе"), error);
        return;
    }
    log_->append(tr("Отчёт о подборе сохранён: %1").arg(path));
    statusBar()->showMessage(tr("Отчёт о подборе сохранён"), 8000);
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

    // Имя набора — последний компонент каталога данных (как в matrices_<набор>).
    if (!settings_.data_dir.isEmpty())
        options.dataset = QDir(settings_.data_dir).dirName();
    options.preset = settings_.settings_file;

    // Паспорт набора: сверка секции [meta] пресета с выбранным каталогом
    // данных. Старый пресет без [meta] и обычный settings.ini дают «нет».
    if (!settings_.settings_file.isEmpty())
    {
        const PresetMeta meta = readPresetMeta(settings_.settings_file);
        const QString current = QDir(settings_.data_dir).dirName();
        const QString tuned = meta.dataset.trimmed();
        if (!meta.present || tuned.isEmpty())
            options.preset_note = tr("нет (пресет без секции [meta])");
        else if (!current.isEmpty() && tuned == current)
            options.preset_note = tr("совпадает");
        else
            options.preset_note = tr("пресет подобран на наборе «%1»").arg(tuned);
    }

    // Наземная истина набора: если truth.txt корректен, ошибки прогона
    // считаются по нему (как в отчёте о подборе), иначе — по эталону СНС.
    // Тот же кэш используется для наложения истины на графики.
    ensureTruthLoaded();
    options.truth = truth_;

    // Окно анализа — время выставки: как в simulation.cpp, сначала StartupNav.ini
    // набора, при его отсутствии — align_fallback_s настроек; явное
    // переопределение (align_time_override_s) имеет приоритет. Предвыставочные
    // отсчёты в статистику не берутся.
    {
        const nav::FilterSettings cur = params_->currentSettings();
        double align_time_s = cur.align_fallback_s;
        if (!settings_.data_dir.isEmpty())
        {
            const QString startup =
                QDir(settings_.data_dir).filePath(QStringLiteral("StartupNav.ini"));
            double lon = 0.0, lat = 0.0, alt = 0.0, t = 0.0;
            if (nav::readStartupNav(startup.toStdString(), lon, lat, alt, t) && t > 0.0)
                align_time_s = t;
        }
        if (cur.align_time_override_s > 0.0) align_time_s = cur.align_time_override_s;
        options.align_time_s = align_time_s;
    }

    // Снимок параметров: у сохранённого прогона берём их из истории, иначе —
    // текущие значения таблицы «Параметры».
    QString filter_ini;
    if (current_run_id_ > 0 && store_open_)
        filter_ini = store_.runSettings(current_run_id_);
    if (filter_ini.isEmpty())
        filter_ini = QString::fromStdString(nav::dumpFilterSettings(params_->currentSettings()));
    options.settings_ini = filter_ini;

    // «Холст» для блоков — отдельный график вне вкладок: так в отчёт попадают
    // сами ряды блоков, а не то, что открыто на экране (активная вкладка
    // «Траектория» рисовала бы вместо временных рядов траекторию).
    PlotView *canvas = report_canvas_;
    if (canvas == nullptr) canvas = plot_err_pos_;

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

    // Отчёты складываем в reports/<набор>/: run_<набор>.pdf.
    QString dataset = settings_.data_dir.isEmpty()
                          ? QStringLiteral("dataset")
                          : QDir(settings_.data_dir).dirName();
    const QString suggested =
        QDir(QDir(AppSettings::defaultReportsDir()).filePath(dataset))
            .filePath(QStringLiteral("run_%1.pdf").arg(dataset));

    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить отчёт"), suggested, tr("PDF (*.pdf)"));
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

void MainWindow::onExportSeriesCsv()
{
    if (series_.rows.empty())
    {
        QMessageBox::information(this, tr("Экспорт"),
                                 tr("Нет данных: сначала выполните прогон "
                                    "или откройте его из истории"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(
        this, tr("Сохранить данные активного графика"), settings_.output_dir + "/series.csv",
        tr("CSV (*.csv)"));
    if (path.isEmpty()) return;

    QString error;
    if (!PlotExporter::exportSeriesCsv(series_, path, &error))
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }
    log_->append(tr("Ряд сохранён: %1").arg(path));
}

// Экспорт статических матриц фильтра (Q/R/P0/H) для текущего датасета:
// σ берутся из таблицы параметров, широта — из StartupNav.ini, шаг T —
// медиана интервалов между отсчётами ИМУ. Работает для любого набора данных.
void MainWindow::onExportMatrices()
{
    const QString start_dir =
        settings_.output_dir.isEmpty() ? settings_.data_dir : settings_.output_dir;
    const QString dir = QFileDialog::getExistingDirectory(
        this, tr("Каталог для экспорта матриц"), start_dir);
    if (dir.isEmpty())
        return;

    QString error;
    if (!exportMatricesTo(dir, &error))
    {
        QMessageBox::warning(this, tr("Экспорт матриц"), error);
        return;
    }

    log_->append(tr("Матрицы фильтра (Q/R/P0/H) сохранены в %1").arg(dir));
    statusBar()->showMessage(tr("Матрицы фильтра (Q/R/P0/H) экспортированы"), 8000);
}

bool MainWindow::exportMatricesTo(const QString &dir, QString *error)
{
    if (dir.isEmpty())
    {
        if (error) *error = tr("Каталог для экспорта не задан");
        return false;
    }

    // Настройки фильтра — как в таблице «Параметры».
    const nav::FilterSettings filter_settings = params_model_->settings();

    // Имя датасета, широта начальной точки и шаг дискретизации.
    QString dataset = tr("dataset");
    if (!settings_.data_dir.isEmpty())
    {
        const QString dir_name = QDir(settings_.data_dir).dirName();
        if (!dir_name.isEmpty())
            dataset = dir_name;
    }

    double lat_deg = 0.0;
    double T = IMU_PERIOD;
    readDatasetParameters(settings_.data_dir, &lat_deg, &T);

    // Имя файлов: недопустимые в имени символы заменяем подчёркиванием.
    QString base = QStringLiteral("matrices_") + dataset;
    base.replace(QRegularExpression(QStringLiteral("[^0-9A-Za-zА-Яа-я_-]+")),
                 QStringLiteral("_"));

    const FilterMatrices matrices =
        buildFilterMatrices(filter_settings, dataset, lat_deg, T);
    return writeFilterMatrices(matrices, dir, base, error);
}

void MainWindow::onExportAllPlots()
{
    const QList<PlotView *> plots = allPlots();
    if (plots.isEmpty())
    {
        QMessageBox::information(this, tr("Экспорт"), tr("Нет графиков"));
        return;
    }

    const QString dir = QFileDialog::getExistingDirectory(
        this, tr("Каталог для экспорта всех графиков"), settings_.output_dir);
    if (dir.isEmpty()) return;

    bool ok = false;
    const QString format = QInputDialog::getItem(
        this, tr("Формат экспорта"), tr("Формат:"),
        QStringList{tr("PNG"), tr("JPG"), tr("SVG"), tr("PDF")}, 0, false, &ok);
    if (!ok) return;

    QList<PlotExportItem> items;
    items.reserve(plots.size());
    for (PlotView *plot : plots)
    {
        const QString name = plot->title().isEmpty() ? tr("график") : plot->title();
        items.push_back({name, plot});
    }

    QProgressDialog progress(tr("Экспорт графиков…"), tr("Отмена"), 0, items.size(), this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.setValue(0);

    QString error;
    int exported = 0;
    const bool all_ok = PlotExporter::exportAllPlots(
        items, dir, format.toLower(),
        settings_.export_width, settings_.export_height,
        [&progress, &items](int done, int total) {
            progress.setValue(done);
            if (done < total) progress.setLabelText(items[done].name);
            return !progress.wasCanceled();
        },
        &exported, &error);
    progress.setValue(items.size());

    if (!all_ok)
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }
    if (exported == 0)
    {
        QMessageBox::information(this, tr("Экспорт"), tr("Экспорт прерван"));
        return;
    }
    log_->append(tr("Экспортировано графиков: %1 → %2").arg(exported).arg(dir));
}

// Экспорт анимации активной вкладки: GIF или последовательность PNG.
void MainWindow::onExportAnimation()
{
    PlotView *plot = currentPlot();
    if (plot == nullptr || series_.rows.empty())
    {
        QMessageBox::information(this, tr("Экспорт анимации"),
                                 tr("Нет данных: сначала выполните прогон или откройте его из истории"));
        return;
    }

    const double t0 = series_.timeMin();
    const double t1 = series_.timeMax();

    // --- Параметры ---
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Экспорт анимации"));
    QFormLayout *form = new QFormLayout(&dlg);

    QComboBox *format_box = new QComboBox(&dlg);
    format_box->addItem(tr("GIF-файл"), QStringLiteral("gif"));
    format_box->addItem(tr("PNG-кадры (в каталог)"), QStringLiteral("png"));

    QSpinBox *fps_box = new QSpinBox(&dlg);
    fps_box->setRange(1, 60);
    fps_box->setValue(10);

    QSpinBox *frames_box = new QSpinBox(&dlg);
    frames_box->setRange(2, 500);
    frames_box->setValue(60);

    QSpinBox *width_box = new QSpinBox(&dlg);
    width_box->setRange(64, 4096);
    width_box->setValue(settings_.export_width);

    QSpinBox *height_box = new QSpinBox(&dlg);
    height_box->setRange(64, 4096);
    height_box->setValue(settings_.export_height);

    QComboBox *window_box = new QComboBox(&dlg);
    window_box->addItem(tr("Нарастающее (вся траектория)"), 0);
    window_box->addItem(tr("Скользящее окно"), 1);

    QSpinBox *percent_box = new QSpinBox(&dlg);
    percent_box->setRange(5, 90);
    percent_box->setValue(20);
    percent_box->setSuffix(tr(" %"));
    connect(window_box, &QComboBox::currentIndexChanged, this,
            [percent_box](int index) { percent_box->setEnabled(index == 1); });
    percent_box->setEnabled(false);

    form->addRow(tr("Формат:"), format_box);
    form->addRow(tr("Кадров в секунду:"), fps_box);
    form->addRow(tr("Число кадров:"), frames_box);
    form->addRow(tr("Ширина, px:"), width_box);
    form->addRow(tr("Высота, px:"), height_box);
    form->addRow(tr("Окно времени:"), window_box);
    form->addRow(tr("Ширина окна:"), percent_box);

    QDialogButtonBox *buttons =
        new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    buttons->button(QDialogButtonBox::Ok)->setText(tr("Экспортировать"));
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    form->addRow(buttons);

    if (dlg.exec() != QDialog::Accepted) return;

    const QString format = format_box->currentData().toString();
    const int fps = fps_box->value();
    const int frames = frames_box->value();
    const int width = width_box->value();
    const int height = height_box->value();
    const bool sliding = window_box->currentData().toInt() == 1;
    const double window_span = (t1 - t0) * percent_box->value() / 100.0;

    QString gif_file;
    QString png_dir;
    if (format == QLatin1String("gif"))
    {
        gif_file = QFileDialog::getSaveFileName(
            this, tr("Сохранить анимацию"), settings_.output_dir + "/animation.gif",
            tr("GIF-анимация (*.gif)"));
        if (gif_file.isEmpty()) return;
    }
    else
    {
        png_dir = QFileDialog::getExistingDirectory(
            this, tr("Каталог для PNG-кадров"), settings_.output_dir);
        if (png_dir.isEmpty()) return;
    }

    // Траектория: полные точки для фиксации масштаба и растущая часть по кадрам.
    const bool is_trajectory = (plot == plot_trajectory_);
    QVector<QPointF> anim_nav;
    if (is_trajectory)
    {
        buildTrajectoryPoints(&anim_nav);
        plot_trajectory_->setNavPoints(anim_nav);
        plot_trajectory_->resetView();
    }

    QProgressDialog progress(tr("Построение кадров…"), tr("Отмена"), 0, frames, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);

    QString error;
    QVector<QImage> img_frames = AnimationExporter::renderFrames(
        plot, width, height, frames,
        [&](int frame, int total) {
            progress.setLabelText(tr("Кадр %1 из %2").arg(frame + 1).arg(total));
            if (is_trajectory)
            {
                const int count =
                    std::clamp(qRound(anim_nav.size() * double(frame + 1) / total), 0,
                               static_cast<int>(anim_nav.size()));
                plot_trajectory_->setNavPoints(anim_nav.mid(0, count));
            }
            else
            {
                double x0 = t0;
                double x1 = t1;
                if (sliding)
                {
                    const double step_step = frames > 1
                                                 ? std::max(0.0, (t1 - t0) - window_span) /
                                                       (frames - 1)
                                                 : 0.0;
                    x0 = t0 + step_step * frame;
                    x1 = std::min(t1, x0 + window_span);
                }
                else
                {
                    x1 = t0 + (t1 - t0) * (frame + 1) / total;
                }
                plot->setXRange(x0, x1);
            }
        },
        [&](int frame, int total) {
            progress.setValue(frame);
            return !progress.wasCanceled();
        },
        &error);

    // Восстановить вид графика.
    if (is_trajectory)
    {
        updateTrajectory();
        plot_trajectory_->resetView();
    }
    else
    {
        rebuildPlotSeries(plot_tabs_->currentIndex());
        plot->resetRanges();
    }

    if (!error.isEmpty())
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }
    if (img_frames.isEmpty())
    {
        QMessageBox::information(this, tr("Экспорт анимации"), tr("Нет кадров"));
        return;
    }
    if (img_frames.size() < frames)
    {
        QMessageBox::information(this, tr("Экспорт анимации"), tr("Экспорт прерван"));
        return;
    }

    bool ok = false;
    if (format == QLatin1String("gif"))
        ok = AnimationExporter::writeGif(img_frames, gif_file, fps, &error);
    else
        ok = AnimationExporter::writePngSequence(img_frames, png_dir, QStringLiteral("frame"),
                                                 &error);
    if (!ok)
    {
        QMessageBox::warning(this, tr("Экспорт"), error);
        return;
    }

    if (format == QLatin1String("gif"))
        log_->append(tr("Анимация сохранена: %1 (%2 кадров, %3 FPS)")
                         .arg(gif_file)
                         .arg(img_frames.size())
                         .arg(fps));
    else
        log_->append(tr("Сохранены кадры: %1 (%2 шт.)").arg(png_dir).arg(img_frames.size()));
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

    const auto plots = allPlots();
    for (PlotView *plot : plots) plot->setTheme(dark);
    if (report_canvas_ != nullptr) report_canvas_->setTheme(dark);
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

    // Снимок параметров фильтра, с которым сделан прогон: предложить применить.
    // В headless self-test модальный диалог повесил бы расчёт, поэтому диалог
    // показываем только в интерактивном режиме.
    const QString settings_ini = store_.runSettings(runId);
    if (!autostart_ && !settings_ini.isEmpty())
    {
        QMessageBox box(this);
        box.setWindowTitle(tr("Параметры прогона"));
        box.setIcon(QMessageBox::Question);
        box.setText(tr("Прогон сохранён с набором параметров фильтра.\n"
                       "Применить их к таблице параметров и к фильтру?"));
        box.setDetailedText(settings_ini);
        QPushButton *apply = box.addButton(tr("Применить"), QMessageBox::AcceptRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();

        if (box.clickedButton() == apply)
        {
            QString error;
            if (!params_->loadSettingsText(settings_ini, &error))
            {
                QMessageBox::warning(this, tr("Параметры"), error);
            }
            else
            {
                params_->applyToCore();
                log_->append(tr("Применены параметры из прогона %1").arg(runId));
            }
        }
    }

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

void MainWindow::onGenerateDataset()
{
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Сгенерировать тестовые данные"));
    dlg.setMinimumWidth(420);

    QFormLayout form(&dlg);

    QLineEdit *name_edit = new QLineEdit("square_test");
    form.addRow(tr("Имя набора"), name_edit);

    QLineEdit *out_edit = new QLineEdit(settings_.data_dir);
    QPushButton *browse = new QPushButton(tr("Обзор..."));
    QHBoxLayout *out_h = new QHBoxLayout;
    out_h->addWidget(out_edit);
    out_h->addWidget(browse);
    form.addRow(tr("Каталог вывода"), out_h);

    QDoubleSpinBox *imu_hz = new QDoubleSpinBox;
    imu_hz->setRange(1.0, 1000.0);
    imu_hz->setValue(200.0);
    form.addRow(tr("Частота ИМУ, Гц"), imu_hz);

    QDoubleSpinBox *gps_hz = new QDoubleSpinBox;
    gps_hz->setRange(0.1, 100.0);
    gps_hz->setValue(10.0);
    form.addRow(tr("Частота СНС, Гц"), gps_hz);

    QDoubleSpinBox *angle_hz = new QDoubleSpinBox;
    angle_hz->setRange(0.1, 100.0);
    angle_hz->setValue(10.0);
    form.addRow(tr("Частота углов, Гц"), angle_hz);

    QDoubleSpinBox *dur1 = new QDoubleSpinBox;
    dur1->setRange(1.0, 1e6);
    dur1->setValue(120.0);
    form.addRow(tr("Выставка (покой), с"), dur1);

    QDialogButtonBox btns(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    form.addRow(&btns);

    connect(browse, &QPushButton::clicked, this, [this, out_edit]() {
        const QString dir = QFileDialog::getExistingDirectory(
            this, tr("Выбрать каталог"), out_edit->text());
        if (!dir.isEmpty()) out_edit->setText(dir);
    });
    connect(&btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(&btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return;

    const QString out_dir = out_edit->text();
    if (out_dir.trimmed().isEmpty())
    {
        QMessageBox::warning(this, tr("Генерация"), tr("Не задан каталог вывода."));
        return;
    }

    // Предупреждение перед перезаписью уже существующего набора.
    if (QFileInfo::exists(QDir(out_dir).filePath(QStringLiteral("imu.dat"))))
    {
        const auto answer = QMessageBox::question(
            this, tr("Генерация"),
            tr("В каталоге %1 уже есть imu.dat. Перезаписать данные?").arg(out_dir),
            QMessageBox::Yes | QMessageBox::No);
        if (answer != QMessageBox::Yes) return;
    }

    dataset::DatasetConfig cfg;
    cfg.name = name_edit->text().toStdString();
    cfg.out_dir = out_dir.toStdString();
    cfg.imu_hz = imu_hz->value();
    cfg.gps_hz = gps_hz->value();
    cfg.angle_hz = angle_hz->value();
    // Длительность выставки пишется в StartupNav.ini и задаёт участок покоя
    // (первый сегмент профиля) — он обязателен для работы выравнивателя.
    cfg.align_time_s = dur1->value();
    cfg.profile.clear();
    cfg.profile.push_back({cfg.align_time_s, 0.0, 0.0, 0.0});
    cfg.profile.push_back({20.0, 10.0, 0.0, 0.0});
    cfg.profile.push_back({40.0, 10.0, 0.0, 1.5});
    cfg.profile.push_back({20.0, 10.0, 0.0, 0.0});
    cfg.profile.push_back({20.0, 0.0, 0.0, 0.0});

    log_->append(tr("Генерация тестовых данных в %1 ...").arg(out_dir));

    struct GenResult
    {
        bool ok = false;
        dataset::DatasetReport report;
        std::string error;
    };

    // Генератор не даёт обратного вызова прогресса, поэтому показываем
    // неопределённый прогресс, пока расчёт идёт в фоновом потоке.
    QProgressDialog pd(tr("Генерация тестовых данных..."), QString(), 0, 0, this);
    pd.setWindowTitle(tr("Генерация"));
    pd.setWindowModality(Qt::WindowModal);
    pd.setMinimumDuration(0);

    QFutureWatcher<GenResult> watcher;
    connect(&watcher, &QFutureWatcher<GenResult>::finished, &pd, &QProgressDialog::accept);
    watcher.setFuture(QtConcurrent::run([cfg]() {
        GenResult r;
        r.ok = dataset::generateDataset(cfg, r.report, &r.error);
        return r;
    }));
    pd.exec();

    const GenResult result = watcher.result();
    if (result.ok)
    {
        log_->append(tr("Готово: IMU=%1, GPS=%2, ANGLE=%3, TRUTH=%4; каталог %5")
                         .arg(result.report.imu_rows)
                         .arg(result.report.gps_rows)
                         .arg(result.report.angle_rows)
                         .arg(result.report.truth_rows)
                         .arg(out_dir));
        QMessageBox::information(this, tr("Готово"),
                                 tr("Сгенерировано: IMU=%1, GPS=%2, ANGLE=%3\nКаталог: %4")
                                     .arg(result.report.imu_rows)
                                     .arg(result.report.gps_rows)
                                     .arg(result.report.angle_rows)
                                     .arg(out_dir));
        data_->setDataDir(out_dir);
    }
    else
    {
        log_->append(tr("Ошибка генерации: %1").arg(QString::fromStdString(result.error)), 2);
        QMessageBox::critical(this, tr("Ошибка"), QString::fromStdString(result.error));
    }
}

} // namespace gui