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
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QPdfWriter>

#include "core/utils/constants.h"
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

// Метрики прогона относительно наземной истины. Определения те же, что и в
// тюнере (tools/kalman_tuner/tune_kalman.py): планарная ошибка позиции,
// горизонтальная скорость и модуль ошибки курса; свёртка — P95 и среднее.
struct TruthStats
{
    int count = 0;
    double pos_p95 = 0.0;
    double pos_mean = 0.0;
    double vel_p95 = 0.0;
    double vel_mean = 0.0;
    double hdg_p95 = 0.0;
    double hdg_mean = 0.0;
};

TruthStats truthStats(const RunSeries &series, const QVector<TruthSample> &truth,
                      double align_time_s)
{
    TruthStats st;
    if (series.rows.empty() || truth.isEmpty()) return st;

    QVector<double> pos;
    QVector<double> vel;
    QVector<double> hdg;
    pos.reserve(truth.size());
    vel.reserve(truth.size());
    hdg.reserve(truth.size());

    // Для каждого отсчёта истины берём ближайшую по времени строку результата
    // (ряды отсортированы, окно двигается вперёд). Отсчёты до конца выставки
    // пропускаем — это окно анализа, см. PdfReport::Options::align_time_s.
    std::size_t j = 0;
    for (const TruthSample &tr : truth)
    {
        const double t = tr.v[0];
        if (t < align_time_s) continue;
        while (j + 1 < series.rows.size() && series.rows[j + 1].time <= t) ++j;
        std::size_t best = j;
        if (j + 1 < series.rows.size() &&
            std::abs(series.rows[j + 1].time - t) < std::abs(series.rows[j].time - t))
        {
            best = j + 1;
        }
        const RunSeries::Row &r = series.rows[best];

        const double lat_ref = tr.v[2];
        const double d_n = (r.result[RunSeries::Lat] - tr.v[2]) * 111000.0;
        const double d_e = (r.result[RunSeries::Lon] - tr.v[1]) * 111000.0 *
                           std::cos(lat_ref * DEG_TO_RAD);
        pos.push_back(std::hypot(d_n, d_e));

        vel.push_back(std::hypot(r.result[RunSeries::Vn] - tr.v[7],
                                 r.result[RunSeries::Ve] - tr.v[9]));

        double dh = std::fmod(r.result[RunSeries::Heading] - tr.v[4] + 180.0, 360.0);
        if (dh < 0.0) dh += 360.0;
        hdg.push_back(std::abs(dh - 180.0));
    }

    auto percentile95 = [](QVector<double> &v) {
        if (v.isEmpty()) return 0.0;
        std::sort(v.begin(), v.end());
        const double idx = 0.95 * (v.size() - 1);
        const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
        const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
        const double frac = idx - static_cast<double>(lo);
        return v[lo] * (1.0 - frac) + v[hi] * frac;
    };
    auto mean = [](const QVector<double> &v) {
        if (v.isEmpty()) return 0.0;
        double s = 0.0;
        for (double x : v) s += x;
        return s / static_cast<double>(v.size());
    };

    st.count = pos.size();
    st.pos_mean = mean(pos);
    st.vel_mean = mean(vel);
    st.hdg_mean = mean(hdg);
    st.pos_p95 = percentile95(pos);
    st.vel_p95 = percentile95(vel);
    st.hdg_p95 = percentile95(hdg);
    return st;
}

// Названия величин в таблице отклонений — те же, что в легендах графиков.
const char *quantityName(int column)
{
    switch (column)
    {
    case RunSeries::Lon: return "Долгота";
    case RunSeries::Lat: return "Широта";
    case RunSeries::Alt: return "Высота";
    case RunSeries::Heading: return "Курс";
    case RunSeries::Pitch: return "Тангаж";
    case RunSeries::Roll: return "Крен";
    case RunSeries::Vn: return "Vn";
    case RunSeries::Ve: return "Ve";
    case RunSeries::Vh: return "Vh";
    default: return "Величина";
    }
}

// Модуль отклонения одной величины от опоры в единицах таблицы: широта и
// долгота переводятся в метры, курс берётся по кратчайшей дуге, остальные
// величины — в своих единицах (метры, м/с).
double deviationOf(const RunSeries::Row &row, const double *ref, int column)
{
    switch (column)
    {
    case RunSeries::Lat:
        return std::abs(row.result[RunSeries::Lat] - ref[RunSeries::Lat]) * 111000.0;
    case RunSeries::Lon:
        return std::abs(row.result[RunSeries::Lon] - ref[RunSeries::Lon]) * 111000.0 *
               std::cos(ref[RunSeries::Lat] * DEG_TO_RAD);
    case RunSeries::Heading:
    {
        double dh = std::fmod(row.result[RunSeries::Heading] - ref[RunSeries::Heading] + 180.0,
                              360.0);
        if (dh < 0.0) dh += 360.0;
        return std::abs(dh - 180.0);
    }
    default:
        return std::abs(row.result[column] - ref[column]);
    }
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

// Таблица «величина × порог»: слева название величины, затем максимум модуля
// отклонения и по колонке на порог — число точек, где отклонение его превысило.
// Шапка повторяется, если таблица не помещается на страницу целиком.
void drawDeviationGroup(QPdfWriter &writer, QPainter &p, PdfCursor &cursor,
                        const PdfReport::DeviationGroup &group)
{
    QFont table_font;
    table_font.setPointSizeF(8.5);
    const QFontMetrics fm(table_font);
    const double row_h = fm.height() + 2.0;

    const double total = kPageWidthPt - 2 * kMarginPt;
    const double label_w = 110.0;
    const double peak_w = 68.0;
    const int n = group.thresholds.size();
    const double col_w = n > 0 ? (total - label_w - peak_w) / n : 0.0;
    const double bottom = kPageHeightPt - kMarginPt;

    const auto draw_head = [&]() {
        // Заголовок группы и шапка таблицы должны попасть на одну страницу.
        cursor.ensure(3 * 11.0 + row_h + 8.0);
        drawHeading(writer, p, cursor,
                    QStringLiteral("%1, %2").arg(group.caption, group.unit), 11.0, 10.0);

        p.setFont(table_font);
        p.setPen(Qt::black);
        double x = kMarginPt;
        p.drawText(QRectF(x, cursor.y(), label_w, row_h),
                   Qt::AlignLeft | Qt::AlignVCenter, QObject::tr("Величина"));
        x += label_w;
        p.drawText(QRectF(x, cursor.y(), peak_w, row_h),
                   Qt::AlignRight | Qt::AlignVCenter, QObject::tr("max |Δ|"));
        x += peak_w;
        for (int t = 0; t < n; ++t)
        {
            p.drawText(QRectF(x, cursor.y(), col_w, row_h), Qt::AlignRight | Qt::AlignVCenter,
                       QStringLiteral("> %1").arg(QString::number(group.thresholds[t], 'g', 3)));
            x += col_w;
        }
        cursor.advance(row_h);
        // Линейка под шапкой отделяет её от строк таблицы.
        p.drawLine(QPointF(kMarginPt, cursor.y()), QPointF(kMarginPt + total, cursor.y()));
        cursor.advance(3.0);
    };

    draw_head();

    for (const PdfReport::DeviationRow &row : group.rows)
    {
        if (cursor.y() + row_h > bottom)
        {
            cursor.newPage();
            draw_head();
        }
        p.setFont(table_font);
        p.setPen(Qt::black);
        double x = kMarginPt;
        p.drawText(QRectF(x, cursor.y(), label_w, row_h),
                   Qt::AlignLeft | Qt::AlignVCenter, row.name);
        x += label_w;
        p.drawText(QRectF(x, cursor.y(), peak_w, row_h), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(row.peak, 'g', 4));
        x += peak_w;
        for (int t = 0; t < row.counts.size(); ++t)
        {
            p.drawText(QRectF(x, cursor.y(), col_w, row_h), Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(row.counts[t]));
            x += col_w;
        }
        cursor.advance(row_h);
    }
    p.setFont(QFont());
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
    const PlotView::RangeState saved_ranges =
        swap_series ? plot->rangeState() : PlotView::RangeState();
    const QString saved_x_label = swap_series ? plot->xLabel() : QString();
    const QString saved_y_label = swap_series ? plot->yLabel() : QString();
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
    if (swap_series)
    {
        plot->setReportSeries(block.series);
        // Подписи осей принадлежат блоку: у величин это «Время, с» и единица.
        if (!block.x_label.isEmpty() || !block.y_label.isEmpty())
            plot->setAxisLabels(block.x_label, block.y_label);
        // Диапазоны — по рядам блока: иначе Y-шкала осталась бы от холста,
        // а она у каждого блока своя (общий только диапазон времени).
        plot->fitRangesToSeries();
    }

    // renderTo рисует в собственных координатах, начиная с (0, 0): сдвигаем
    // painter в левый верхний угол блока и обрезаем по размеру блока, иначе
    // график рисовался бы от края страницы и обрезался чужим прямоугольником.
    p.save();
    p.translate(box.topLeft());
    p.setClipRect(QRectF(0, 0, width, height_pt));
    plot->renderTo(p, QSize(static_cast<int>(width), static_cast<int>(height_pt)));
    p.restore();

    if (swap_series)
    {
        plot->setReportSeries(saved_series);
        // Возврат масштаба и подписей холста: сборка отчёта не должна менять
        // состояние графика на экране.
        plot->setAxisLabels(saved_x_label, saved_y_label);
        plot->setRangeState(saved_ranges);
    }
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

QVector<PdfReport::DeviationGroup> PdfReport::deviations(const RunSeries &series,
                                                        const QVector<TruthSample> &truth,
                                                        double align_time_s)
{
    QVector<DeviationGroup> groups;
    if (series.rows.empty()) return groups;

    // Точки сравнения: строка результата и опорная строка той же раскладки, что
    // result (time, lon, lat, alt, heading, pitch, roll, vn, vh, ve).
    QVector<int> row_idx;
    QVector<const double *> refs;

    if (!truth.isEmpty())
    {
        // Как в truthStats: каждому отсчёту истины — ближайшая по времени строка.
        std::size_t j = 0;
        for (const TruthSample &tr : truth)
        {
            const double t = tr.v[0];
            // Окно анализа: точки до конца выставки в таблицу не берём.
            if (t < align_time_s) continue;
            while (j + 1 < series.rows.size() && series.rows[j + 1].time <= t) ++j;
            std::size_t best = j;
            if (j + 1 < series.rows.size() &&
                std::abs(series.rows[j + 1].time - t) < std::abs(series.rows[j].time - t))
            {
                best = j + 1;
            }
            row_idx.push_back(static_cast<int>(best));
            refs.push_back(tr.v);
        }
    }
    else
    {
        // Наборы без истины: опора — эталон СНС (кадры коррекции).
        for (std::size_t i = 0; i < series.rows.size(); ++i)
        {
            if (!series.rows[i].has_reference) continue;
            if (series.rows[i].time < align_time_s) continue;
            row_idx.push_back(static_cast<int>(i));
            refs.push_back(series.rows[i].reference);
        }
    }
    if (row_idx.isEmpty()) return groups;

    // Пороги по группам: координаты — метры, углы — градусы, скорости — м/с.
    struct GroupSpec
    {
        const char *caption;
        const char *unit;
        int columns[3];
        double thresholds[7];
        int threshold_count;
    };
    static const GroupSpec specs[] = {
        {"Координаты", "м",
         {RunSeries::Lat, RunSeries::Lon, RunSeries::Alt},
         {0.1, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0}, 7},
        {"Углы", "°",
         {RunSeries::Heading, RunSeries::Pitch, RunSeries::Roll},
         {0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0}, 7},
        {"Скорости", "м/с",
         {RunSeries::Vn, RunSeries::Ve, RunSeries::Vh},
         {0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 0.0}, 6},
    };

    for (const GroupSpec &spec : specs)
    {
        DeviationGroup group;
        group.caption = QString::fromUtf8(spec.caption);
        group.unit = QString::fromUtf8(spec.unit);
        group.samples = static_cast<int>(row_idx.size());
        for (int t = 0; t < spec.threshold_count; ++t)
            group.thresholds.push_back(spec.thresholds[t]);

        for (int column : spec.columns)
        {
            DeviationRow row;
            row.name = QString::fromUtf8(quantityName(column));
            row.counts.fill(0, group.thresholds.size());
            for (int k = 0; k < row_idx.size(); ++k)
            {
                const double dev = deviationOf(series.rows[row_idx[k]], refs[k], column);
                row.peak = std::max(row.peak, dev);
                for (int t = 0; t < group.thresholds.size(); ++t)
                {
                    if (dev > group.thresholds[t]) ++row.counts[t];
                }
            }
            group.rows.push_back(row);
        }
        groups.push_back(group);
    }
    return groups;
}

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

    // Отклонения по величинам: та же таблица, что печатается в отчёте. В
    // summarize() истина не передаётся, поэтому опора — эталон СНС.
    for (const DeviationGroup &group : deviations(series, QVector<TruthSample>()))
    {
        if (group.samples == 0) continue;
        QStringList thresholds;
        for (double t : group.thresholds) thresholds << QString::number(t, 'g', 3);
        lines << QStringLiteral("%1 (%2), точек %3, пороги %4")
                     .arg(group.caption, group.unit)
                     .arg(group.samples)
                     .arg(thresholds.join(QLatin1Char(' ')));
        for (const DeviationRow &row : group.rows)
        {
            QStringList counts;
            for (int c : row.counts) counts << QString::number(c);
            lines << QStringLiteral("%1: max |Δ| %2, выше порогов %3")
                         .arg(row.name)
                         .arg(row.peak, 0, 'g', 4)
                         .arg(counts.join(QLatin1Char(' ')));
        }
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
    // Разрешение 72 dpi: единица координат painter — 1 pt, поэтому поля и
    // геометрия страницы ниже заданы в пунктах (A4 = 595×842).
    writer.setResolution(72);
    writer.setPageSize(QPageSize(QPageSize::A4));
    // Поля не отдаём QPdfWriter (по умолчанию у setPageMargins единица —
    // миллиметры, и 40 «пунктов» превратились бы в 40 мм), отступы
    // рисуем сами через kMarginPt.
    writer.setPageMargins(QMarginsF(0, 0, 0, 0), QPageLayout::Point);
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
    if (!options.dataset.isEmpty())
    {
        drawTableRow(writer, p, cursor, QObject::tr("Набор"), options.dataset);
    }
    if (!options.preset.isEmpty())
    {
        drawTableRow(writer, p, cursor, QObject::tr("Пресет"), options.preset);
    }
    if (!options.preset_note.isEmpty())
    {
        drawTableRow(writer, p, cursor, QObject::tr("Паспорт набора"), options.preset_note);
    }
    drawTableRow(writer, p, cursor, QObject::tr("Отсчётов"), QString::number(series.rows.size()));
    drawTableRow(writer, p, cursor, QObject::tr("Кадров коррекции"),
                 QString::number(series.corrections.size()));
    drawTableRow(writer, p, cursor, QObject::tr("Длительность, с"),
                 QString::number(series.rows.back().time - series.rows.front().time, 'f', 2));
    // Файл, по которому считается ошибка: наземная истина набора либо эталон
    // СНС (как в отчёте о подборе).
    drawTableRow(writer, p, cursor, QObject::tr("Ошибка считается по"),
                 options.truth.isEmpty() ? QObject::tr("reference.txt (эталон СНС)")
                                         : QObject::tr("truth.txt (наземная истина)"));
    // Окно анализа: во время выставки фильтр ещё не сошёлся, поэтому её ошибки
    // в статистику не берутся (см. PdfReport::Options::align_time_s).
    if (options.align_time_s > 0.0)
    {
        drawTableRow(writer, p, cursor, QObject::tr("Окно анализа, с"),
                     QObject::tr("t ≥ %1 (после выставки)")
                         .arg(options.align_time_s, 0, 'f', 2));
    }

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

    // --- Ошибка относительно истины ---
    // Считается только когда в наборе есть корректный truth.txt; определения
    // метрик совпадают с тюнером, чтобы числа «подбор/прогон» были сопоставимы.
    if (!options.truth.isEmpty())
    {
        drawHeading(writer, p, cursor, QObject::tr("Ошибка относительно истины"),
                    13.0, 14.0);
        const TruthStats ts = truthStats(series, options.truth, options.align_time_s);
        if (ts.count == 0)
        {
            drawTableRow(writer, p, cursor, QObject::tr("Нет совпадений по времени"),
                         QStringLiteral("—"));
        }
        else
        {
            drawTableRow(writer, p, cursor, QObject::tr("Позиция, м (P95 / среднее)"),
                         QStringLiteral("%1      %2")
                             .arg(ts.pos_p95, 0, 'g', 4)
                             .arg(ts.pos_mean, 0, 'g', 4),
                         320.0);
            drawTableRow(writer, p, cursor, QObject::tr("Скорость, м/с (P95 / среднее)"),
                         QStringLiteral("%1      %2")
                             .arg(ts.vel_p95, 0, 'g', 4)
                             .arg(ts.vel_mean, 0, 'g', 4),
                         320.0);
            drawTableRow(writer, p, cursor, QObject::tr("Курс, ° (P95 / среднее)"),
                         QStringLiteral("%1      %2")
                             .arg(ts.hdg_p95, 0, 'g', 4)
                             .arg(ts.hdg_mean, 0, 'g', 4),
                         320.0);
            drawTableRow(writer, p, cursor, QObject::tr("Сопоставлено точек"),
                         QString::number(ts.count), 320.0);
        }
    }

    // --- Отклонения по величинам ---
    // Для каждой величины: максимум модуля отклонения и число точек, где
    // отклонение превысило порог. Опора та же, что у ошибки выше: наземная
    // истина, а при её отсутствии — эталон СНС (кадры коррекции).
    const QVector<DeviationGroup> dev_groups =
        deviations(series, options.truth, options.align_time_s);
    if (!dev_groups.isEmpty() && dev_groups.front().samples > 0)
    {
        drawHeading(writer, p, cursor,
                    options.truth.isEmpty()
                        ? QObject::tr("Отклонения от эталона СНС по величинам")
                        : QObject::tr("Отклонения от истины по величинам"),
                    13.0, 14.0);
        drawTableRow(writer, p, cursor, QObject::tr("Точек сравнения"),
                     QString::number(dev_groups.front().samples), 320.0);
        for (const DeviationGroup &group : dev_groups)
        {
            if (group.samples > 0) drawDeviationGroup(writer, p, cursor, group);
        }
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

    // --- Параметры фильтра ---
    // Печатается снимок применённых параметров (dumpFilterSettings): секции
    // ([filter], [tilt], [run]) — подзаголовками, ключи — строками таблицы.
    if (!options.settings_ini.isEmpty())
    {
        drawHeading(writer, p, cursor, QObject::tr("Параметры фильтра"), 13.0, 14.0);
        const QStringList ini_lines = options.settings_ini.split(QLatin1Char('\n'));
        for (const QString &raw : ini_lines)
        {
            const QString line = raw.trimmed();
            if (line.isEmpty()) continue;
            if (line.startsWith(QLatin1Char(';')) || line.startsWith(QLatin1Char('#')))
                continue;
            if (line.startsWith(QLatin1Char('[')) && line.endsWith(QLatin1Char(']')))
            {
                drawHeading(writer, p, cursor, line, 11.0, 8.0);
                continue;
            }
            const int eq = line.indexOf(QLatin1Char('='));
            if (eq < 0) continue;
            drawTableRow(writer, p, cursor, line.left(eq).trimmed(),
                         line.mid(eq + 1).trimmed(), 280.0);
        }
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