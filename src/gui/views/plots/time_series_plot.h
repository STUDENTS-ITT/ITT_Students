// time_series_plot.h — График величин от времени (X = время, Y = значение).
//
// Используется для траектории (широта/долгота/высота), углов, скоростей,
// ошибок фильтра (x0..x14), инноваций СНС и диагоналей ковариации.
// Это основной инструмент анализа: один ряд данных, произвольное число
// настраиваемых кривых, легенда, экспорт.

#pragma once

#include "views/plots/plot_view.h"

namespace gui
{

class TimeSeriesPlot : public PlotView
{
    Q_OBJECT

public:
    explicit TimeSeriesPlot(QWidget *parent = nullptr);

    // Ряд «время — значение» из массивов.
    void addSeries(const QString &name, const QVector<double> &times,
                   const QVector<double> &values, const QColor &color,
                   bool markers = false, bool dashed = false);

    // Переопределить палитру рядов (например, 15 ошибок фильтра).
    void setPalette(const QVector<QColor> &colors);

    // Порог отображения: ряды с максимумом |y| меньше порога скрываются
    // (чтобы шумные компоненты не мешали).
    void setVisibilityThreshold(double abs_value);
    double visibilityThreshold() const { return threshold_; }

    // Скрыть/показать конкретные ряды.
    void setSeriesVisible(const QString &name, bool visible);
    bool isSeriesVisible(const QString &name) const;

    // Есть ли ряды для отрисовки.
    bool hasVisibleSeries() const;

    // Точки синхронизации с другой кривой (например, моменты коррекции СНС):
    // вертикальные линии.
    void setSyncMarkers(const QVector<double> &times);
    void clearSyncMarkers();
    const QVector<double> &syncMarkers() const { return sync_times_; }

protected:
    void paintContent(QPainter &painter, const QRectF &rect) override;

private:
    // Один ряд: ломаная либо огибающая min/max, если точек больше пикселей.
    void drawSeries(QPainter &painter, const PlotSeries &s, const QRectF &rect);
    void drawEnvelope(QPainter &painter, const PlotSeries &s, const QRectF &rect,
                      int columns);

    QVector<QColor> palette_;
    QVector<bool> visible_;
    QVector<double> sync_times_;
    double threshold_ = 0.0;
};

} // namespace gui