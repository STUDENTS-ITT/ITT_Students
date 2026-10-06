// time_series_plot.cpp — График величин от времени.

#include "views/plots/time_series_plot.h"

#include <algorithm>
#include <cmath>

#include <QPainter>

namespace gui
{

TimeSeriesPlot::TimeSeriesPlot(QWidget *parent) : PlotView(parent)
{
    setAxisLabels(tr("Время, с"), QString());
    setMarginLeft(70);

    // Палитра по умолчанию: различимые на белом и тёмном фоне цвета.
    palette_ = {
        QColor(31, 119, 180),  QColor(255, 127, 14),  QColor(44, 160, 44),
        QColor(214, 39, 40),   QColor(148, 103, 189), QColor(140, 86, 75),
        QColor(227, 119, 194), QColor(127, 127, 127), QColor(188, 189, 34),
        QColor(23, 190, 207),  QColor(174, 199, 232), QColor(255, 187, 120),
        QColor(152, 223, 138), QColor(255, 152, 150), QColor(197, 176, 213),
    };
}

void TimeSeriesPlot::setPalette(const QVector<QColor> &colors)
{
    if (!colors.isEmpty()) palette_ = colors;
    invalidateCache();
    update();
}

void TimeSeriesPlot::addSeries(const QString &name, const QVector<double> &times,
                               const QVector<double> &values, const QColor &color,
                               bool markers, bool dashed)
{
    PlotSeries s;
    s.name = name;
    s.color = color.isValid() ? color : palette_.at(seriesCount() % palette_.size());
    s.markers = markers;
    s.dashed = dashed;
    s.points.reserve(std::min(times.size(), values.size()));
    for (int i = 0; i < times.size() && i < values.size(); i++)
    {
        s.points.push_back(QPointF(times[i], values[i]));
    }

    QVector<PlotSeries> all = series();
    all.push_back(s);
    visible_.push_back(true);
    setSeries(all);
}

void TimeSeriesPlot::setVisibilityThreshold(double abs_value)
{
    threshold_ = abs_value;
    invalidateCache();
    update();
}

void TimeSeriesPlot::setSeriesVisible(const QString &name, bool visible)
{
    for (int i = 0; i < series().size(); i++)
    {
        if (series()[i].name == name)
        {
            if (i < visible_.size()) visible_[i] = visible;
            invalidateCache();
            return;
        }
    }
    update();
}

bool TimeSeriesPlot::isSeriesVisible(const QString &name) const
{
    for (int i = 0; i < series().size(); i++)
    {
        if (series()[i].name == name)
        {
            return i >= visible_.size() ? true : visible_[i];
        }
    }
    return false;
}

bool TimeSeriesPlot::hasVisibleSeries() const
{
    for (int i = 0; i < series().size(); i++)
    {
        const bool vis = i >= visible_.size() ? true : visible_[i];
        if (!vis) continue;
        if (series()[i].points.isEmpty()) continue;
        if (threshold_ > 0.0)
        {
            double max_abs = 0.0;
            for (const QPointF &p : series()[i].points)
            {
                if (!std::isfinite(p.y())) continue;
                max_abs = std::max(max_abs, std::abs(p.y()));
            }
            if (max_abs < threshold_) continue;
        }
        return true;
    }
    return false;
}

void TimeSeriesPlot::setSyncMarkers(const QVector<double> &times)
{
    sync_times_ = times;
    invalidateCache();
    update();
}

void TimeSeriesPlot::clearSyncMarkers()
{
    sync_times_.clear();
    invalidateCache();
    update();
}

void TimeSeriesPlot::drawEnvelope(QPainter &painter, const PlotSeries &s,
                                  const QRectF &rect, int columns)
{
    // Столько точек на один пиксель, что рисовать их по одной бессмысленно:
    // сворачиваем ряд в пиксельные столбцы и рисуем огибающую min/max
    // (плюс ломаную по середине столбца, чтобы был виден средний уровень).
    const double span_x = (xMax() > xMin()) ? (xMax() - xMin()) : 1.0;
    const double span_y = (yMax() > yMin()) ? (yMax() - yMin()) : 1.0;
    const double x0 = mapToScreen(s.points.first().x(), 0.0, rect).x();

    QVector<double> col_min(columns, 0.0);
    QVector<double> col_max(columns, 0.0);
    QVector<double> col_x(columns, 0.0);
    QVector<int> col_cnt(columns, 0);

    for (const QPointF &p : s.points)
    {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y())) continue;
        int c = static_cast<int>(std::floor(mapToScreen(p.x(), 0.0, rect).x() - x0));
        c = std::min(std::max(c, 0), columns - 1);
        if (col_cnt[c] == 0)
        {
            col_min[c] = col_max[c] = p.y();
        }
        else
        {
            col_min[c] = std::min(col_min[c], p.y());
            col_max[c] = std::max(col_max[c], p.y());
        }
        col_x[c] = p.x();
        ++col_cnt[c];
    }

    const auto toScreenX = [this, rect, span_x](double x) {
        return rect.left() + (x - xMin()) / span_x * rect.width();
    };
    const auto toScreenY = [this, rect, span_y](double y) {
        return rect.bottom() - (y - yMin()) / span_y * rect.height();
    };

    painter.save();
    painter.setBrush(Qt::NoBrush);

    // Огибающая: вертикальные отрезки min..max в каждом непустом столбце.
    QPen band_pen(s.color, 1.0);
    if (s.dashed) band_pen.setStyle(Qt::DashLine);
    painter.setPen(band_pen);
    QPolygonF mid;
    mid.reserve(columns);
    for (int c = 0; c < columns; ++c)
    {
        if (col_cnt[c] == 0) continue;
        const double px = toScreenX(col_x[c]);
        const double y_hi = toScreenY(col_max[c]);
        const double y_lo = toScreenY(col_min[c]);
        if (std::abs(y_hi - y_lo) > 0.5) painter.drawLine(QPointF(px, y_hi), QPointF(px, y_lo));
        mid.push_back(QPointF(px, toScreenY(0.5 * (col_max[c] + col_min[c]))));
    }

    if (mid.size() > 1)
    {
        QPen line_pen(s.color, s.width);
        if (s.dashed) line_pen.setStyle(Qt::DashLine);
        painter.setPen(line_pen);
        painter.drawPolyline(mid);
    }
    painter.restore();
}

void TimeSeriesPlot::drawSeries(QPainter &painter, const PlotSeries &s,
                                const QRectF &rect)
{
    if (s.points.isEmpty()) return;

    painter.save();
    QPen pen(s.color, s.width);
    if (s.dashed) pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);

    if (s.points.size() == 1)
    {
        const QPointF p = mapToScreen(s.points[0].x(), s.points[0].y(), rect);
        painter.setBrush(s.color);
        painter.drawEllipse(p, 2.5, 2.5);
        painter.restore();
        return;
    }

    // Число столбцов = ширина ряда на экране в пикселях.
    const double px0 = mapToScreen(s.points.first().x(), 0.0, rect).x();
    const double px1 = mapToScreen(s.points.last().x(), 0.0, rect).x();
    const int columns = static_cast<int>(std::abs(px1 - px0)) + 1;

    if (columns > 1 && s.points.size() > 2 * columns)
    {
        painter.restore();
        drawEnvelope(painter, s, rect, columns);
        return;
    }

    QPolygonF poly;
    poly.reserve(s.points.size());
    for (const QPointF &p : s.points)
    {
        poly.push_back(mapToScreen(p.x(), p.y(), rect));
    }
    painter.drawPolyline(poly);

    if (s.markers)
    {
        // Разреженный ряд (СНС 10 Гц, ошибки фильтра): рисуем маркеры с шагом,
        // чтобы не строить тысячи точек.
        painter.setBrush(s.color);
        painter.setPen(Qt::NoPen);
        const int step = std::max(1, static_cast<int>(s.points.size()) / 400);
        for (int k = 0; k < s.points.size(); k += step)
        {
            painter.drawEllipse(poly[k], 1.8, 1.8);
        }
    }

    painter.restore();
}

void TimeSeriesPlot::paintContent(QPainter &painter, const QRectF &rect)
{
    // Вертикальные линии моментов коррекции СНС — на фоне кривых.
    if (!sync_times_.isEmpty())
    {
        painter.save();
        painter.setPen(QPen(QColor(200, 60, 60, 90), 1.0, Qt::DotLine));
        for (double t : sync_times_)
        {
            const double x = mapToScreen(t, 0.0, rect).x();
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
        }
        painter.restore();
    }

    const QVector<PlotSeries> &all = series();

    for (int i = 0; i < all.size(); i++)
    {
        const PlotSeries &s = all[i];
        if (i < visible_.size() && !visible_[i]) continue;
        if (s.points.isEmpty()) continue;

        if (threshold_ > 0.0)
        {
            double max_abs = 0.0;
            for (const QPointF &p : s.points)
            {
                if (!std::isfinite(p.y())) continue;
                max_abs = std::max(max_abs, std::abs(p.y()));
            }
            if (max_abs < threshold_) continue;
        }

        drawSeries(painter, s, rect);
    }
}

} // namespace gui