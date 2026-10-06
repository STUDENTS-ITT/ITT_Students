// plot_view.cpp — Базовая отрисовка графиков на QPainter.

#include "views/plots/plot_view.h"

#include <algorithm>
#include <cmath>

#include <QFileInfo>
#include <QFontMetrics>
#include <QMarginsF>
#include <QMouseEvent>
#include <QPageSize>
#include <QPainter>
#include <QPdfWriter>
#include <QPixmap>
#include <QStringList>
#include <QSvgGenerator>
#include <QWheelEvent>

namespace gui
{
namespace
{
// Красивые деления оси: 1, 2, 5 × 10^k.
double niceStep(double range, int target)
{
    if (range <= 0.0) return 1.0;
    const double raw = range / std::max(1, target);
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double norm = raw / mag;
    double step;
    if (norm <= 1.0) step = 1.0;
    else if (norm <= 2.0) step = 2.0;
    else if (norm <= 5.0) step = 5.0;
    else step = 10.0;
    return step * mag;
}

QString formatTick(double v, double step)
{
    const int decimals = step >= 1.0 ? 0 : (step >= 0.1 ? 1 : (step >= 0.01 ? 2 : 3));
    return QString::number(v, 'f', decimals);
}
} // namespace

PlotView::PlotView(QWidget *parent) : QWidget(parent)
{
    setAutoFillBackground(false);
    setMinimumSize(240, 160);
    setMouseTracking(true);
    setTheme(false);
    last_mouse_pos_ = QPoint(-1, -1);
}

void PlotView::invalidateCache()
{
    ++content_version_;
}

void PlotView::setCacheEnabled(bool on)
{
    if (cache_enabled_ == on) return;
    cache_enabled_ = on;
    invalidateCache();
    if (!cache_enabled_)
    {
        // Выключенный кэш не должен держать в памяти картинку прежнего кадра.
        cache_ = QPixmap();
        cache_size_ = QSize();
    }
    update();
}

bool PlotView::isCacheValid(const QSize &size) const
{
    return cache_enabled_ && !cache_.isNull() && cache_size_ == size &&
           cache_version_ == content_version_ && cache_x0_ == xmin_ &&
           cache_x1_ == xmax_ && cache_y0_ == ymin_ && cache_y1_ == ymax_;
}

void PlotView::rebuildCache(const QSize &size)
{
    cache_ = QPixmap(size);
    cache_.fill(background_);

    // Курсор в кэш не попадает: он меняется при движении мыши и не должен
    // приводить к перерисовке содержимого.
    building_cache_ = true;
    {
        QPainter painter(&cache_);
        renderTo(painter, size);
    }
    building_cache_ = false;

    cache_size_ = size;
    cache_version_ = content_version_;
    cache_x0_ = xmin_;
    cache_x1_ = xmax_;
    cache_y0_ = ymin_;
    cache_y1_ = ymax_;
    ++cache_misses_;
}

void PlotView::setTheme(bool dark)
{
    dark_ = dark;
    invalidateCache();
    if (dark_)
    {
        background_ = QColor(24, 26, 30);
        grid_color_ = QColor(58, 62, 70);
        text_color_ = QColor(200, 205, 212);
        axis_color_ = QColor(120, 128, 140);
    }
    else
    {
        background_ = QColor(255, 255, 255);
        grid_color_ = QColor(226, 228, 232);
        text_color_ = QColor(50, 54, 60);
        axis_color_ = QColor(130, 136, 145);
    }
    update();
}

void PlotView::setBackground(const QColor &bg)
{
    background_ = bg;
    invalidateCache();
    update();
}

void PlotView::setGridColor(const QColor &c)
{
    grid_color_ = c;
    invalidateCache();
    update();
}

void PlotView::setAxisLabels(const QString &x, const QString &y)
{
    x_label_ = x;
    y_label_ = y;
    invalidateCache();
    update();
}

void PlotView::setTitle(const QString &title)
{
    title_ = title;
    invalidateCache();
    update();
}

void PlotView::setSeries(QVector<PlotSeries> series)
{
    series_ = std::move(series);
    invalidateCache();
    if (auto_scale_) updateAutoRange();
    emit viewRangesChanged(xmin_, xmax_, ymin_, ymax_);
    update();
}

void PlotView::setReportSeries(const QVector<PlotSeries> &series)
{
    series_ = series;
    invalidateCache();
    // Диапазоны не трогаем: в отчёте их выставляет сам размер страницы,
    // а экранный масштаб пользователя должен остаться прежним.
}

void PlotView::clear()
{
    series_.clear();
    cursor_ = PlotCursor();
    invalidateCache();
    update();
}

void PlotView::setXRange(double xmin, double xmax)
{
    auto_x_ = false;
    xmin_ = xmin;
    xmax_ = (xmax > xmin) ? xmax : xmin + 1.0;
    invalidateCache();
    update();
}

void PlotView::setYRange(double ymin, double ymax)
{
    auto_y_ = false;
    ymin_ = ymin;
    ymax_ = (ymax > ymin) ? ymax : ymin + 1.0;
    invalidateCache();
    update();
}

void PlotView::resetRanges()
{
    auto_x_ = true;
    auto_y_ = true;
    updateAutoRange();
    emit viewRangesChanged(xmin_, xmax_, ymin_, ymax_);
    update();
}

void PlotView::setLegendVisible(bool on)
{
    legend_visible_ = on;
    invalidateCache();
    update();
}

void PlotView::updateAutoRange()
{
    double lo_x = 0, hi_x = 0, lo_y = 0, hi_y = 0;
    bool first = true;

    for (const PlotSeries &s : series_)
    {
        for (const QPointF &p : s.points)
        {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y())) continue;
            if (first)
            {
                lo_x = hi_x = p.x();
                lo_y = hi_y = p.y();
                first = false;
            }
            else
            {
                lo_x = std::min(lo_x, p.x());
                hi_x = std::max(hi_x, p.x());
                lo_y = std::min(lo_y, p.y());
                hi_y = std::max(hi_y, p.y());
            }
        }
    }

    if (first) return;  // данных нет — оставляем прежний диапазон

    if (auto_x_)
    {
        if (hi_x - lo_x < 1e-12)
        {
            lo_x -= 0.5;
            hi_x += 0.5;
        }
        xmin_ = lo_x;
        xmax_ = hi_x;
    }

    if (auto_y_)
    {
        const double span = hi_y - lo_y;
        if (span < 1e-12)
        {
            const double d = std::max(1e-6, std::abs(hi_y) * 0.01);
            lo_y -= d;
            hi_y += d;
        }
        else
        {
            const double pad = span * 0.06;
            lo_y -= pad;
            hi_y += pad;
        }
        ymin_ = lo_y;
        ymax_ = hi_y;
    }
}

void PlotView::niceRange(double &lo, double &hi)
{
    const double span = hi - lo;
    if (span <= 0.0) return;
    const double step = niceStep(span, 5);
    lo = std::floor(lo / step) * step;
    hi = std::ceil(hi / step) * step;
}

QRectF PlotView::plotRectFor(const QSize &size) const
{
    const double w = std::max(10.0, static_cast<double>(size.width()) - margin_left_ - margin_right_);
    const double h = std::max(10.0, static_cast<double>(size.height()) - margin_top_ - margin_bottom_);
    return QRectF(margin_left_, margin_top_, w, h);
}

QPointF PlotView::mapToScreen(double x, double y, const QRectF &rect) const
{
    const double sx = (xmax_ > xmin_) ? (x - xmin_) / (xmax_ - xmin_) : 0.0;
    const double sy = (ymax_ > ymin_) ? (y - ymin_) / (ymax_ - ymin_) : 0.0;
    return QPointF(rect.left() + sx * rect.width(), rect.bottom() - sy * rect.height());
}

QPointF PlotView::mapToData(const QPointF &screen, const QRectF &rect) const
{
    const double sx = (rect.width() > 0) ? (screen.x() - rect.left()) / rect.width() : 0.0;
    const double sy = (rect.height() > 0) ? (rect.bottom() - screen.y()) / rect.height() : 0.0;
    return QPointF(xmin_ + sx * (xmax_ - xmin_), ymin_ + sy * (ymax_ - ymin_));
}

void PlotView::drawGrid(QPainter &painter, const QRectF &rect)
{
    painter.save();
    painter.setPen(QPen(grid_color_, 1.0));
    painter.setBrush(Qt::NoBrush);

    const double step_x = niceStep((xmax_ - xmin_), 6);
    const double step_y = niceStep((ymax_ - ymin_), 5);

    if (std::isfinite(step_x) && step_x > 0)
    {
        for (double v = std::ceil(xmin_ / step_x) * step_x; v <= xmax_ + 1e-12; v += step_x)
        {
            const double x = mapToScreen(v, ymin_, rect).x();
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
        }
    }
    if (std::isfinite(step_y) && step_y > 0)
    {
        for (double v = std::ceil(ymin_ / step_y) * step_y; v <= ymax_ + 1e-12; v += step_y)
        {
            const double y = mapToScreen(xmin_, v, rect).y();
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
        }
    }

    // Рамка области построения.
    painter.setPen(QPen(axis_color_, 1.0));
    painter.drawRect(rect);
    painter.restore();
}

void PlotView::drawAxisLabels(QPainter &painter, const QRectF &rect)
{
    painter.save();
    painter.setPen(text_color_);
    QFont f = painter.font();
    f.setPointSizeF(std::max(7.0, painter.font().pointSizeF() - 1.0));
    painter.setFont(f);
    const QFontMetrics fm(f);

    const double step_x = niceStep((xmax_ - xmin_), 6);
    const double step_y = niceStep((ymax_ - ymin_), 5);

    if (std::isfinite(step_x) && step_x > 0)
    {
        for (double v = std::ceil(xmin_ / step_x) * step_x; v <= xmax_ + 1e-12; v += step_x)
        {
            const double x = mapToScreen(v, 0.0, rect).x();
            const QString text = formatTick(v, step_x);
            painter.drawText(QPointF(x - fm.horizontalAdvance(text) / 2.0,
                                     rect.bottom() + fm.ascent() + 4),
                             text);
        }
    }

    if (std::isfinite(step_y) && step_y > 0)
    {
        for (double v = std::ceil(ymin_ / step_y) * step_y; v <= ymax_ + 1e-12; v += step_y)
        {
            const double y = mapToScreen(0.0, v, rect).y();
            painter.drawText(QRectF(0, y - fm.height() / 2.0, margin_left_ - 6, fm.height()),
                             Qt::AlignRight | Qt::AlignVCenter, formatTick(v, step_y));
        }
    }

    if (!title_.isEmpty())
    {
        QFont tf = painter.font();
        tf.setBold(true);
        painter.setFont(tf);
        painter.drawText(QRectF(rect.left(), 2, rect.width(), margin_top_ - 4),
                         Qt::AlignCenter, title_);
    }

    if (!x_label_.isEmpty())
    {
        painter.drawText(QRectF(rect.left(), rect.bottom() + 16, rect.width(), 14),
                         Qt::AlignCenter, x_label_);
    }
    if (!y_label_.isEmpty())
    {
        painter.save();
        painter.translate(12, rect.center().y());
        painter.rotate(-90);
        painter.drawText(QRectF(-rect.height() / 2, -8, rect.height(), 14),
                         Qt::AlignCenter, y_label_);
        painter.restore();
    }
    painter.restore();
}

void PlotView::drawLegend(QPainter &painter, const QRectF &rect)
{
    if (!legend_visible_ || series_.size() < 2) return;

    painter.save();
    QFont f = painter.font();
    f.setPointSizeF(std::max(7.0, painter.font().pointSizeF() - 1.0));
    painter.setFont(f);
    const QFontMetrics fm(f);

    int width = 0;
    for (const PlotSeries &s : series_)
        width = std::max(width, fm.horizontalAdvance(s.name) + 26);

    const int height = fm.height() + 6;
    const QRectF box(rect.right() - width - 8, rect.top() + 6, width + 8,
                     height * series_.size() + 4);

    painter.setPen(QPen(axis_color_, 1.0));
    painter.setBrush(QColor(background_.red(), background_.green(), background_.blue(),
                            dark_ ? 220 : 245));
    painter.drawRoundedRect(box, 3, 3);

    double y = box.top() + 4;
    for (const PlotSeries &s : series_)
    {
        painter.setPen(QPen(s.color, s.width));
        painter.drawLine(QPointF(box.left() + 4, y + height / 2 - 3),
                         QPointF(box.left() + 20, y + height / 2 - 3));
        painter.setPen(text_color_);
        painter.drawText(QPointF(box.left() + 24, y + height - 4), s.name);
        y += height;
    }
    painter.restore();
}

void PlotView::drawEndMarker(QPainter &painter, const QRectF &rect)
{
    if (!show_end_marker_) return;

    painter.save();
    for (const PlotSeries &s : series_)
    {
        if (s.points.isEmpty()) continue;
        const QPointF last = s.points.last();
        if (!std::isfinite(last.x()) || !std::isfinite(last.y())) continue;
        const QPointF p = mapToScreen(last.x(), last.y(), rect);
        painter.setPen(QPen(s.color, 1.4));
        painter.setBrush(s.color);
        painter.drawEllipse(p, 3.0, 3.0);
    }
    painter.restore();
}

void PlotView::drawCursor(QPainter &painter, const QRectF &rect)
{
    if (!cursor_.valid) return;

    const QPoint pos = cursor_.pos;
    if (!rect.contains(QPointF(pos))) return;

    painter.save();
    painter.setPen(QPen(QColor(120, 130, 150, 160), 1.0, Qt::DashLine));
    painter.drawLine(QPointF(rect.left(), pos.y()), QPointF(rect.right(), pos.y()));
    painter.drawLine(QPointF(pos.x(), rect.top()), QPointF(pos.x(), rect.bottom()));

    if (!cursor_.text.isEmpty())
    {
        QFont f = painter.font();
        f.setPointSizeF(std::max(7.0, painter.font().pointSizeF() - 1.0));
        painter.setFont(f);
        const QFontMetrics fm(f);
        const QRectF box(pos.x() + 8, pos.y() - fm.height() - 8,
                         fm.horizontalAdvance(cursor_.text) + 8, fm.height() + 4);
        painter.setPen(QPen(axis_color_, 1.0));
        painter.setBrush(QColor(background_.red(), background_.green(), background_.blue(),
                                dark_ ? 230 : 250));
        painter.drawRoundedRect(box, 3, 3);
        painter.setPen(text_color_);
        painter.drawText(box.adjusted(4, 2, -4, -2), Qt::AlignLeft | Qt::AlignVCenter,
                         cursor_.text);
    }
    painter.restore();
}

void PlotView::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    const QSize size = this->size();

    if (cache_enabled_ && isCacheValid(size))
    {
        painter.drawPixmap(0, 0, cache_);
        ++cache_hits_;
        // Курсор рисуется поверх кэша, поэтому движение мыши не вызывает
        // перерисовку самих кривых.
        if (cursor_.valid)
        {
            painter.save();
            drawCursor(painter, plotRectFor(size));
            painter.restore();
        }
        return;
    }

    if (!cache_enabled_)
    {
        // Кэш выключен: содержимое рисуется прямо на виджет, без промежуточного
        // QPixmap, поэтому счётчики попаданий/промахов не растут.
        renderTo(painter, size);
        return;
    }

    rebuildCache(size);
    painter.drawPixmap(0, 0, cache_);
    // Курсор в кэш не попал (см. rebuildCache), поэтому на первом кадре
    // его нужно дорисовать поверх только что построенной картинки.
    if (cursor_.valid)
    {
        painter.save();
        drawCursor(painter, plotRectFor(size));
        painter.restore();
    }
}

void PlotView::renderTo(QPainter &painter, const QSize &size)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    painter.fillRect(QRect(QPoint(0, 0), size), background_);

    const QRectF rect = plotRectFor(size);

    drawGrid(painter, rect);
    drawAxisLabels(painter, rect);

    painter.save();
    painter.setClipRect(rect);
    paintContent(painter, rect);
    painter.restore();

    drawEndMarker(painter, rect);
    drawLegend(painter, rect);
    if (!building_cache_) drawCursor(painter, rect);

    painter.restore();
}

int PlotView::nearestPointIndex(const QString &name, double x) const
{
    int best = -1;
    double best_dist = 0;
    for (const PlotSeries &s : series_)
    {
        if (s.name != name) continue;
        for (int i = 0; i < s.points.size(); i++)
        {
            const double d = std::abs(s.points[i].x() - x);
            if (best < 0 || d < best_dist)
            {
                best = i;
                best_dist = d;
            }
        }
        break;
    }
    return best;
}

PlotCursor PlotView::cursorAt(const QPoint &pos) const
{
    PlotCursor c;
    c.pos = pos;

    const QRectF rect = plotRectFor(size());
    if (!rect.contains(QPointF(pos)) || series_.isEmpty()) return c;

    const QPointF data = mapToData(QPointF(pos), rect);

    QStringList parts;
    double best_dist = 1e12;
    QString best_name;
    double best_y = 0;

    for (const PlotSeries &s : series_)
    {
        const int idx = nearestPointIndex(s.name, data.x());
        if (idx < 0) continue;
        const QPointF p = s.points[idx];
        const double d = std::abs(p.x() - data.x());
        if (d < best_dist)
        {
            best_dist = d;
            best_name = s.name;
            best_y = p.y();
        }
    }

    // Ближайшая по X точка должна быть в пределах ~1% ширины диапазона.
    const double tolerance = (xmax_ - xmin_) * 0.01 + 1e-9;
    if (best_name.isEmpty() || best_dist > tolerance) return c;

    c.valid = true;
    c.text = tr("%1: t = %2 с, y = %3").arg(best_name).arg(data.x(), 0, 'f', 2).arg(best_y, 0, 'f', 6);
    return c;
}

void PlotView::mouseMoveEvent(QMouseEvent *event)
{
    if (panning_)
    {
        // Панорамирование: сдвигаем диапазон так, чтобы точка под курсором
        // следовала за мышью.
        const QRectF rect = plotRectFor(size());
        const double dx = event->pos().x() - pan_anchor_.x();
        const double dy = event->pos().y() - pan_anchor_.y();

        const double span_x = pan_xmax_ - pan_xmin_;
        const double span_y = pan_ymax_ - pan_ymin_;
        const double shift_x = (rect.width() > 0.0) ? dx * span_x / rect.width() : 0.0;
        const double shift_y = (rect.height() > 0.0) ? dy * span_y / rect.height() : 0.0;

        auto_x_ = false;
        auto_y_ = false;
        xmin_ = pan_xmin_ - shift_x;
        xmax_ = pan_xmax_ - shift_x;
        ymin_ = pan_ymin_ + shift_y;
        ymax_ = pan_ymax_ + shift_y;

        cursor_ = PlotCursor();
        onUserViewChanged();
        emit viewRangesChanged(xmin_, xmax_, ymin_, ymax_);
        update();
        event->accept();
        return;
    }

    cursor_ = cursorAt(event->pos());
    last_mouse_pos_ = event->pos();
    update();
}

void PlotView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton &&
        plotRectFor(size()).contains(QPointF(event->pos())))
    {
        panning_ = true;
        pan_anchor_ = event->pos();
        pan_xmin_ = xmin_;
        pan_xmax_ = xmax_;
        pan_ymin_ = ymin_;
        pan_ymax_ = ymax_;
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void PlotView::mouseReleaseEvent(QMouseEvent *event)
{
    if (panning_ && event->button() == Qt::LeftButton)
    {
        panning_ = false;
        unsetCursor();
        update();
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void PlotView::wheelEvent(QWheelEvent *event)
{
    const QPoint pos = event->position().toPoint();
    if (!plotRectFor(size()).contains(QPointF(pos)))
    {
        QWidget::wheelEvent(event);
        return;
    }

    const double steps = event->angleDelta().y() / 120.0;
    if (steps == 0.0) return;

    // Прокрутка вверх (steps > 0) приближает: диапазон сжимается.
    const double factor = std::pow(0.8, steps);
    const QPointF anchor = mapToData(QPointF(pos), plotRectFor(size()));

    // Ctrl — только время (ось X), Shift — только величина (ось Y),
    // без модификаторов — обе оси.
    const bool zoom_x = !event->modifiers().testFlag(Qt::ShiftModifier);
    const bool zoom_y = !event->modifiers().testFlag(Qt::ControlModifier);

    if (zoom_x)
    {
        auto_x_ = false;
        xmin_ = anchor.x() + (xmin_ - anchor.x()) * factor;
        xmax_ = anchor.x() + (xmax_ - anchor.x()) * factor;
        if (xmax_ - xmin_ < 1e-12) xmax_ = xmin_ + 1e-12;
    }
    if (zoom_y)
    {
        auto_y_ = false;
        ymin_ = anchor.y() + (ymin_ - anchor.y()) * factor;
        ymax_ = anchor.y() + (ymax_ - anchor.y()) * factor;
        if (ymax_ - ymin_ < 1e-12) ymax_ = ymin_ + 1e-12;
    }

    cursor_ = PlotCursor();
    onUserViewChanged();
    emit viewRangesChanged(xmin_, xmax_, ymin_, ymax_);
    update();
    event->accept();
}

void PlotView::leaveEvent(QEvent *event)
{
    cursor_ = PlotCursor();
    QWidget::leaveEvent(event);
}

void PlotView::mouseDoubleClickEvent(QMouseEvent *event)
{
    // Двойной клик — сбросить масштаб к авто.
    resetRanges();
    QWidget::mouseDoubleClickEvent(event);
}

bool PlotView::saveImage(const QString &fileName, const QSize &size)
{
    const QSize target = size.isValid() && !size.isEmpty() ? size : this->size();
    if (target.width() < 8 || target.height() < 8) return false;

    QPixmap pm(target);
    pm.fill(background_);

    QPainter painter(&pm);
    renderTo(painter, target);
    painter.end();

    return pm.save(fileName, nullptr, 100);
}

bool PlotView::saveSvg(const QString &fileName, const QSize &size)
{
    const QSize target = size.isValid() && !size.isEmpty() ? size : this->size();
    if (target.width() < 8 || target.height() < 8) return false;

    QSvgGenerator generator;
    generator.setFileName(fileName);
    generator.setSize(target);
    generator.setViewBox(QRect(0, 0, target.width(), target.height()));
    generator.setTitle(title_.isEmpty() ? QStringLiteral("plot") : title_);

    QPainter painter(&generator);
    if (!painter.isActive()) return false;
    renderTo(painter, target);
    painter.end();
    return true;
}

bool PlotView::savePdf(const QString &fileName, const QSize &size)
{
    const QSize target = size.isValid() && !size.isEmpty() ? size : this->size();
    if (target.width() < 8 || target.height() < 8) return false;

    QPdfWriter writer(fileName);
    writer.setResolution(96);
    const QSize page(target * 96 / 72);  // страница того же соотношения
    writer.setPageSize(QPageSize(page, QPageSize::Point));
    writer.setPageMargins(QMarginsF(0, 0, 0, 0));

    QPainter painter(&writer);
    if (!painter.isActive()) return false;
    renderTo(painter, page);
    painter.end();
    return true;
}

} // namespace gui