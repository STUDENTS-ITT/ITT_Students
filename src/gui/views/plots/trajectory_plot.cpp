// trajectory_plot.cpp — Траектория в местных координатах (QPainter).

#include "views/plots/trajectory_plot.h"

#include <algorithm>
#include <cmath>

#include <QPainter>

namespace gui
{
namespace
{
const QColor kNavColor(31, 119, 180);
const QColor kRefColor(214, 39, 40);
} // namespace

TrajectoryPlot::TrajectoryPlot(QWidget *parent) : PlotView(parent)
{
    setTitle(tr("Траектория (местные координаты)"));
    setAxisLabels(QString(), tr("север, м"));
    setLegendVisible(false);
    setShowEndMarker(false);
    setMarginLeft(66);
    span_ = 2000.0;
}

void TrajectoryPlot::clearTrajectory()
{
    nav_.clear();
    invalidateCache();
    update();
}

void TrajectoryPlot::setNavPoints(const QVector<QPointF> &points)
{
    nav_ = points;
    invalidateCache();
    update();
}

void TrajectoryPlot::appendNavPoint(double east_m, double north_m)
{
    nav_.push_back(QPointF(east_m, north_m));
    invalidateCache();
    update();
}

void TrajectoryPlot::setReferencePoints(const QVector<QPointF> &points)
{
    ref_ = points;
    invalidateCache();
    update();
}

void TrajectoryPlot::clearReference()
{
    ref_.clear();
    invalidateCache();
    update();
}

void TrajectoryPlot::setShowReference(bool on)
{
    show_reference_ = on;
    invalidateCache();
    update();
}

void TrajectoryPlot::resetView()
{
    auto_view_ = true;
    invalidateCache();
    recomputeView();
    emit viewRangesChanged(xMin(), xMax(), yMin(), yMax());
    update();
}

void TrajectoryPlot::recomputeView()
{
    // Пользовательский зум/панорамирование не трогаем до явного сброса вида.
    if (!auto_view_) return;

    // Одинаковый масштаб по обеим осям (метры на пиксель), чтобы не искажать
    // направления и курс.
    double min_e = 0, max_e = 0, min_n = 0, max_n = 0;
    bool first = true;

    const auto consider = [&](const QVector<QPointF> &pts) {
        for (const QPointF &p : pts)
        {
            if (first)
            {
                min_e = max_e = p.x();
                min_n = max_n = p.y();
                first = false;
            }
            else
            {
                min_e = std::min(min_e, p.x());
                max_e = std::max(max_e, p.x());
                min_n = std::min(min_n, p.y());
                max_n = std::max(max_n, p.y());
            }
        }
    };

    consider(nav_);
    if (show_reference_) consider(ref_);

    if (first)
    {
        setXRange(-span_, span_);
        setYRange(-span_, span_);
        return;
    }

    const double span_e = std::max(1.0, max_e - min_e);
    const double span_n = std::max(1.0, max_n - min_n);
    const double span = std::max(span_e, span_n) * 1.12;

    center_e_ = auto_center_ ? 0.5 * (min_e + max_e) : 0.0;
    center_n_ = auto_center_ ? 0.5 * (min_n + max_n) : 0.0;

    setXRange(center_e_ - span, center_e_ + span);
    setYRange(center_n_ - span, center_n_ + span);
    span_ = span;
}

void TrajectoryPlot::paintContent(QPainter &painter, const QRectF &rect)
{
    // Данные добавляются по ходу прогона, поэтому охват считаем каждый раз.
    recomputeView();

    painter.save();

    if (show_reference_ && !ref_.isEmpty())
    {
        painter.setPen(QPen(kRefColor, 1.0));
        painter.setBrush(kRefColor);
        for (const QPointF &p : ref_)
        {
            const QPointF s = mapToScreen(p.x(), p.y(), rect);
            painter.drawEllipse(s, 2.2, 2.2);
        }
    }

    if (!nav_.isEmpty())
    {
        QPolygonF poly;
        poly.reserve(nav_.size());
        for (const QPointF &p : nav_)
        {
            poly.push_back(mapToScreen(p.x(), p.y(), rect));
        }

        painter.setPen(QPen(kNavColor, 1.8));
        painter.setBrush(Qt::NoBrush);
        painter.drawPolyline(poly);

        // Старт — залитый маркер, конец — кольцо.
        painter.setPen(QPen(kNavColor, 1.4));
        painter.setBrush(kNavColor);
        painter.drawEllipse(poly.first(), 3.0, 3.0);

        if (poly.size() > 1)
        {
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(poly.last(), 5.0, 5.0);
        }
    }

    painter.restore();
}

} // namespace gui