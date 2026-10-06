// pdf_report.cpp — Сводный отчёт по прогону в PDF (пункт 2.9 плана).

#include "export/pdf_report.h"

#include <algorithm>
#include <cmath>

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QMarginsF>
#include <QPageSize>
#include <QPainter>
#include <QPdfWriter>

#include "views/plots/plot_view.h"
#include "views/plots/time_series_plot.h"

namespace gui
{
namespace
{
constexpr int kPageWidthPt = 595;   // A4
constexpr int kPageHeightPt = 842;
constexpr int kMarginPt = 40;

// Виджет-«холст» отчёта: в него кладутся ряды очередного блока перед
// отрисовкой; содержимое возвращается обратно сразу после renderTo.
PlotView *g_plot = nullptr;

// Названия компонент вектора ошибок — те же, что в легенде графика.
const char *componentName(int i)
{
    static const char *names[ins::KF_STATE] = {
        "x0 шир", "x1 долг", "x2 выс", "x3 Vn", "x4 Vh", "x5 Ve",
        "x6 курс", "x7 тангаж", "x8 крен", "x9 ba_x", "x10 ba_y", "x11 ba_z",
        "x12 bg_x", "x13 bg_y", "x14 bg_z"};
    if (i >= 0 && i < ins::KF_STATE) return names[i];
    return nullptr;
}

// Среднеквадратичное отклонение и максимум модуля по компоненте ошибки.
struct ComponentStats
{
    double rms = 0.0;
    double peak = 0.0;
    int count = 0;
};

ComponentStats statsFor(const RunSeries &series, int component)
{
    ComponentStats st;
    if (series.corrections.empty()) return st;

    double sum_sq = 0.0;
    for (const RunSeries::CorrectionRow &c : series.corrections)
    {
        const double v = c.x[component];
        sum_sq += v * v;
        st.peak = std::max(st.peak, std::abs(v));
        ++st.count;
    }
    if (st.count > 0) st.rms = std::sqrt(sum_sq / st.count);
    return st;
}

// Среднее NIS/NEES и их отношение к числу степеней свободы: в согласованном
// фильтре оба средних тяготеть к dof.
struct ConsistencyStats
{
    bool valid = false;
    int count = 0;
    int dof = 0;
    double nis_mean = 0.0;
    double nees_mean = 0.0;
};

ConsistencyStats consistencyFor(const RunSeries &series)
{
    ConsistencyStats st;
    if (series.consistency.empty()) return st;

    double nis_sum = 0.0;
    double nees_sum = 0.0;
    for (const RunSeries::ConsistencyRow &r : series.consistency)
    {
        nis_sum += r.nis;
        nees_sum += r.nees;
        st.dof = r.dof;
        ++st.count;
    }
    if (st.count > 0)
    {
        st.nis_mean = nis_sum / st.count;
        st.nees_mean = nees_sum / st.count;
        st.valid = st.dof > 0;
    }
    return st;
}

// Постраничная отрисовка текста: возвращает Y-координату нижней границы
// нарисованного блока.
class PdfCursor
{
public:
    explicit PdfCursor(QPainter &painter, QPdfWriter &writer)
        : p_(painter), w_(writer)
    {
    }

    QPainter &painter() { return p_; }
    QPdfWriter &writer() { return w_; }
    double y() const { return y_; }

    void ensure(double height)
    {
        if (y_ + height > kPageHeightPt - kMarginPt) newPage();
    }

    void advance(double dy) { y_ += dy; }

    void newPage()
    {
        w_.newPage();
        y_ = kMarginPt;
    }

private:
    QPainter &p_;
    QPdfWriter &w_;
    double y_ = kMarginPt;
};

void drawHeading(QPdfWriter &writer, QPainter &p, PdfCursor &cursor, const QString &text,
                 int point_size, double space_before)
{
    cursor.ensure(point_size * 3);
    cursor.advance(space_before);

    QFont f = p.font();
    f.setPointSizeF(point_size);
    f.setBold(true);
    p.setFont(f);
    const QFontMetrics fm(f);

    cursor.ensure(fm.height() + 4);
    p.setPen(Qt::black);
    p.drawText(QRectF(kMarginPt, cursor.y(), kPageWidthPt - 2 * kMarginPt, fm.height()),
               Qt::AlignLeft | Qt::AlignVCenter, text);
    cursor.advance(fm.height() + 4);
    p.setFont(QFont());
}

// Строка таблицы: слева подпись, справа значение (выровнено по разрядам).
void drawTableRow(QPdfWriter &writer, QPainter &p, PdfCursor &cursor, const QString &label,
                  const QString &value, double label_width = 240.0)
{
    QFont f = p.font();
    f.setPointSizeF(9.0);
    p.setFont(f);
    const QFontMetrics fm(f);

    cursor.ensure(fm.height() + 2);
    p.setPen(Qt::black);
    const double width = kPageWidthPt - 2 * kMarginPt;
    p.drawText(QRectF(kMarginPt, cursor.y(), label_width, fm.height()),
               Qt::AlignLeft | Qt::AlignVCenter, label);
    p.drawText(QRectF(kMarginPt + label_width, cursor.y(), width - label_width, fm.height()),
               Qt::AlignRight | Qt::AlignVCenter, value);
    cursor.advance(fm.height() + 2);
    p.setFont(QFont());
    Q_UNUSED(writer);
}

// График в рамке на текущей странице: содержимое рисуется тем же renderTo,
// что и на экране, но в прямоугольник страницы (другой размер и DPI).
void drawPlotBlock(QPdfWriter &writer, QPainter &p, PdfCursor &cursor,
                   const PdfReport::PlotBlock &block, double height_pt = 230.0)
{
    // Либо готовый виджет со своими данными (траектория), либо переданный в
    // build() «холст», в который кладётся ряд блока.
    PlotView *plot = block.widget != nullptr ? block.widget : g_plot;
    if (plot == nullptr) return;
    if (block.widget == nullptr && block.series.isEmpty()) return;

    cursor.ensure(height_pt + 30);
    drawHeading(writer, p, cursor, block.caption, 11.0, 10.0);

    const double width = kPageWidthPt - 2 * kMarginPt;
    const QRectF box(kMarginPt, cursor.y(), width, height_pt);

    QFontMetrics fm(p.font());
    const int legend_rows = fm.height() + 6;

// Ряды блока попадают на «холст», после чего тот же виджет рисуется
    // в прямоугольник страницы. Состояние виджета восстанавливается после
    // блока — иначе график на экране сменился бы на содержимое отчёта.
    const bool swap_series = (block.widget == nullptr);
    const QVector<PlotSeries> saved_series = swap_series ? plot->reportSeries()
                                                         : QVector<PlotSeries>();
    TimeSeriesPlot *swap_plot = swap_series ? qobject_cast<TimeSeriesPlot *>(plot) : nullptr;
    const QVector<double> saved_markers =
        (swap_plot != nullptr) ? swap_plot->syncMarkers() : QVector<double>();
    if (swap_plot != nullptr)
    {
        if (block.sync_markers.isEmpty())
            swap_plot->clearSyncMarkers();
        else
            swap_plot->setSyncMarkers(block.sync_markers);
    }
    if (swap_series) plot->setReportSeries(block.series);

    p.save();
    p.setClipRect(box);
    plot->renderTo(p, QSize(static_cast<int>(width), static_cast<int>(height_pt)));
    p.restore();

    if (swap_series) plot->setReportSeries(saved_series);
    // Метки синхронизации тоже относятся к состоянию виджета: после экспорта
    // их нужно вернуть, иначе на экране остались бы метки чужого блока.
    if (swap_plot != nullptr)
    {
        if (saved_markers.isEmpty())
            swap_plot->clearSyncMarkers();
        else
            swap_plot->setSyncMarkers(saved_markers);
    }

    cursor.advance(height_pt + legend_rows);
}

} // namespace

QString PdfReport::summarize(const RunSeries &series)
{
    if (series.rows.empty()) return QObject::tr("Нет данных");

    QStringList lines;
    lines << QObject::tr("Отсчётов: %1").arg(series.rows.size());
    lines << QObject::tr("Кадров коррекции: %1").arg(series.corrections.size());

    const double t0 = series.rows.front().time;
    const double t1 = series.rows.back().time;
    lines << QObject::tr("Длительность: %1 с").arg(t1 - t0, 0, 'f', 2);

    const ConsistencyStats cs = consistencyFor(series);
    if (cs.valid)
    {
        lines << QObject::tr("NIS (среднее): %1 при dof = %2")
                     .arg(cs.nis_mean, 0, 'f', 3).arg(cs.dof);
        lines << QObject::tr("NEES (среднее): %1 при dof = %2")
                     .arg(cs.nees_mean, 0, 'f', 3).arg(cs.dof);
    }

    for (int i = 0; i < ins::KF_STATE; ++i)
    {
        const ComponentStats st = statsFor(series, i);
        if (st.count == 0) continue;
        lines << QStringLiteral("%1: RMS %2, max %3")
                     .arg(QString::fromUtf8(componentName(i)))
                     .arg(st.rms, 0, 'g', 4)
                     .arg(st.peak, 0, 'g', 4);
    }
    return lines.join(QLatin1Char('\n'));
}

bool PdfReport::build(PlotView *plot, const RunSeries &series,
                      const QVector<PlotBlock> &blocks, const Options &options,
                      const QString &fileName, QString *error)
{
    if (series.rows.empty())
    {
        if (error) *error = QObject::tr("Нет данных для отчёта");
        return false;
    }
    if (fileName.isEmpty())
    {
        if (error) *error = QObject::tr("Файл не указан");
        return false;
    }

    const QDir dir = QFileInfo(fileName).absoluteDir();
    if (!dir.exists() && !QDir().mkpath(dir.absolutePath()))
    {
        if (error) *error = QObject::tr("Не удалось создать каталог для %1").arg(fileName);
        return false;
    }

    QPdfWriter writer(fileName);
    writer.setResolution(96);
    writer.setPageSize(QPageSize(QPageSize::A4));
    writer.setPageMargins(QMarginsF(kMarginPt, kMarginPt, kMarginPt, kMarginPt));
    writer.setTitle(options.title.isEmpty() ? QObject::tr("Отчёт по прогону БИНС")
                                            : options.title);

    QPainter p(&writer);
    if (!p.isActive())
    {
        if (error) *error = QObject::tr("Не удалось открыть %1 для записи").arg(fileName);
        return false;
    }
    p.setRenderHint(QPainter::Antialiasing, true);

    PdfCursor cursor(p, writer);

    // --- Титул ---
    drawHeading(writer, p, cursor,
                options.title.isEmpty() ? QObject::tr("Отчёт по прогону БИНС")
                                        : options.title,
                16.0, 0.0);

    drawTableRow(writer, p, cursor, QObject::tr("Создан"),
                 QDateTime::currentDateTime().toString(Qt::ISODate));
    if (!options.data_dir.isEmpty())
    {
        drawTableRow(writer, p, cursor, QObject::tr("Каталог данных"), options.data_dir);
    }
    drawTableRow(writer, p, cursor, QObject::tr("Отсчётов"), QString::number(series.rows.size()));
    drawTableRow(writer, p, cursor, QObject::tr("Кадров коррекции"),
                 QString::number(series.corrections.size()));
    drawTableRow(writer, p, cursor, QObject::tr("Длительность, с"),
                 QString::number(series.rows.back().time - series.rows.front().time, 'f', 2));

    // --- Согласованность фильтра ---
    const ConsistencyStats cs = consistencyFor(series);
    drawHeading(writer, p, cursor, QObject::tr("Согласованность фильтра"), 13.0, 14.0);
    if (cs.valid)
    {
        drawTableRow(writer, p, cursor, QObject::tr("Число степеней свободы"),
                     QString::number(cs.dof));
        drawTableRow(writer, p, cursor, QObject::tr("NIS, среднее"),
                     QString::number(cs.nis_mean, 'f', 3));
        drawTableRow(writer, p, cursor, QObject::tr("NEES, среднее"),
                     QString::number(cs.nees_mean, 'f', 3));
        drawTableRow(writer, p, cursor, QObject::tr("NIS / dof"),
                     QString::number(cs.nis_mean / cs.dof, 'f', 3));
        drawTableRow(writer, p, cursor, QObject::tr("NEES / dof"),
                     QString::number(cs.nees_mean / cs.dof, 'f', 3));
    }
    else
    {
        drawTableRow(writer, p, cursor, QObject::tr("Нет данных"), QStringLiteral("—"));
    }

    // --- Ошибки по компонентам ---
    drawHeading(writer, p, cursor, QObject::tr("Ошибки фильтра (кадры коррекции)"),
                13.0, 14.0);
    drawTableRow(writer, p, cursor, QObject::tr("Компонента"),
                 QObject::tr("RMS"), 200.0);
    drawTableRow(writer, p, cursor, QStringLiteral(""),
                 QObject::tr("max |x|"), 200.0);
    for (int i = 0; i < ins::KF_STATE; ++i)
    {
        const ComponentStats st = statsFor(series, i);
        if (st.count == 0) continue;
        const QString name = QString::fromUtf8(componentName(i));
        drawTableRow(writer, p, cursor, name,
                     QStringLiteral("%1      %2")
                         .arg(st.rms, 0, 'g', 5)
                         .arg(st.peak, 0, 'g', 5),
                     200.0);
    }

    // --- Таблица NIS/NEES (выборка) ---
    if (!series.consistency.empty() && options.max_consistency_rows > 0)
    {
        drawHeading(writer, p, cursor, QObject::tr("Согласованность по времени"),
                    13.0, 14.0);
        const int total = static_cast<int>(series.consistency.size());
        const int limit = std::min(total, options.max_consistency_rows);
        const int step = std::max(1, total / limit);

        drawTableRow(writer, p, cursor, QObject::tr("t, с"), QObject::tr("NIS"), 160.0);
        drawTableRow(writer, p, cursor, QStringLiteral(""), QObject::tr("NEES"), 160.0);
        int printed = 0;
        for (int i = 0; i < total && printed < limit; i += step, ++printed)
        {
            const RunSeries::ConsistencyRow &r = series.consistency[i];
            drawTableRow(writer, p, cursor,
                         QString::number(r.time, 'f', 2),
                         QStringLiteral("%1      %2")
                             .arg(r.nis, 0, 'f', 3)
                             .arg(r.nees, 0, 'f', 3),
                         160.0);
        }
    }

    // --- Графики ---
    if (options.include_plots && plot != nullptr && !blocks.isEmpty())
    {
        g_plot = plot;
        cursor.newPage();
        drawHeading(writer, p, cursor, QObject::tr("Графики"), 13.0, 0.0);
        for (const PlotBlock &block : blocks)
        {
            drawPlotBlock(writer, p, cursor, block, 230.0);
        }
        g_plot = nullptr;
    }

    p.end();
    return true;
}

} // namespace gui