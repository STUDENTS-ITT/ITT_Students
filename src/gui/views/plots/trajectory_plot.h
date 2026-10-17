// trajectory_plot.h — График траектории в местных координатах.
//
// Топографическая проекция (равноугольная): восточное и северное смещение в
// метрах от точки старта. Так расстояния и курсы читаются корректно, в отличие
// от графика «широта/долгота в градусах». Траектория БИНС рисуется сплошной
// линией, истина — пунктиром.

#pragma once

#include "views/plots/plot_view.h"

namespace gui
{

class TrajectoryPlot : public PlotView
{
    Q_OBJECT

public:
    explicit TrajectoryPlot(QWidget *parent = nullptr);

    // Траектория в метрах от старта: восток (x) и север (y).
    void clearTrajectory();
    void setNavPoints(const QVector<QPointF> &points);
    void appendNavPoint(double east_m, double north_m);

    // Истинная траектория (truth.txt) в тех же местных координатах:
    // выводится пунктиром для сравнения с оценкой БИНС.
    void setTruthPoints(const QVector<QPointF> &points_east_north);
    void clearTruth();
    bool hasTruth() const { return !truth_.isEmpty(); }

    void setAutoCenter(bool on) { auto_center_ = on; }

    // Сбросить масштаб к текущему охвату данных.
    void resetView();
    // true, пока вид не изменён пользователем (зум/панорамирование).
    bool isAutoView() const { return auto_view_; }

    // Отрисовка в рамке отчёта: перед выводом осей пересчитывает диапазон по
    // данным (renderTo базового класса рисует сетку и подписи раньше, чем
    // paintContent() успевает обновить вид).
    void renderTo(QPainter &painter, const QSize &size) override;

protected:
    void paintContent(QPainter &painter, const QRectF &rect) override;
    void onUserViewChanged() override { auto_view_ = false; }

private:
    void recomputeView();
    // Легенда источников траектории (оценка / истина).
    void drawTrajectoryLegend(QPainter &painter, const QRectF &rect) const;

    QVector<QPointF> nav_;    // траектория БИНС, м от старта
    QVector<QPointF> truth_;  // истинная траектория, м от старта
    bool auto_center_ = false;
    bool auto_view_ = true;  // масштаб подбирается по данным
    double span_ = 2000.0;  // охват по большей оси, м
    double center_e_ = 0.0; // центр вида, м
    double center_n_ = 0.0;
};

} // namespace gui