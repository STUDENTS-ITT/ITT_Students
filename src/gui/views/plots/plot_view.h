// plot_view.h — Базовый виджет графика, отрисованный на QPainter.
//
// Общие для всех графиков вещи: рамка, сетка, подписи осей, авто-масштаб
// с «прилипанием» к круглым значениям, легенда, экспорт в изображение.
// Никаких QtCharts/QtGraphs — только QPainter.

#pragma once

#include <QColor>
#include <QFont>
#include <QPointF>
#include <QPixmap>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>

namespace gui
{

// Один ряд графика: имя, цвет, значения (индексы соответствуют x).
struct PlotSeries
{
    QString name;
    QColor color;
    QVector<QPointF> points;
    bool dashed = false;
    bool markers = false;  // точки (для разреженных рядов вроде СНС)
    double width = 1.6;
};

// Курсор перекрестия с подсказкой значений.
struct PlotCursor
{
    bool valid = false;
    QPoint pos;
    QString text;
};

class PlotView : public QWidget
{
    Q_OBJECT

public:
    explicit PlotView(QWidget *parent = nullptr);

    // Оси.
    void setAxisLabels(const QString &x, const QString &y);
    void setTitle(const QString &title);
    QString title() const { return title_; }

    // Ряды.
    void setSeries(QVector<PlotSeries> series);
    void clear();

    // Ряды для отчёта в PDF: снимок текущего содержимого и его подстановка
    // без пересчёта диапазона (в отчёте масштаб задаёт сам виджет отрисовки).
    QVector<PlotSeries> reportSeries() const { return series_; }
    void setReportSeries(const QVector<PlotSeries> &series);
    int seriesCount() const { return series_.size(); }

    // Масштаб.
    void setAutoScale(bool on) { auto_scale_ = on; }
    void setXRange(double xmin, double xmax);
    void setYRange(double ymin, double ymax);
    void resetRanges();

    // Оформление.
    void setBackground(const QColor &bg);
    void setGridColor(const QColor &c);
    void setTheme(bool dark);
    void setLegendVisible(bool on);
    void setMarginLeft(int px) { margin_left_ = px; invalidateCache(); }
    void setShowEndMarker(bool on) { show_end_marker_ = on; invalidateCache(); }

    // Отрисовка в QPixmap (для экспорта PNG/PDF и сохранения в файл).
    virtual void renderTo(QPainter &painter, const QSize &size);
    // Сохранить в файл (png/jpg/bmp) с учётом размера виджета или заданного.
    bool saveImage(const QString &fileName, const QSize &size = QSize());
    // Сохранить в PDF через QPdfWriter (модуль PrintSupport).
    bool savePdf(const QString &fileName, const QSize &size = QSize());
    // Сохранить векторно в SVG через QSvgGenerator (модуль Svg).
    bool saveSvg(const QString &fileName, const QSize &size = QSize());

    // Кэш кадра (пункт 2.3 плана): содержимое рисуется один раз и
    // переиспользуется при перерисовке без изменения данных и масштаба.
    // Курсор в кэш не входит и рисуется поверх.
    void setCacheEnabled(bool on);
    bool isCacheEnabled() const { return cache_enabled_; }
    // Счётчики для диагностики: сколько раз содержимое бралось из кэша,
    // сколько раз перерисовывалось.
    quint64 cacheHits() const { return cache_hits_; }
    quint64 cacheMisses() const { return cache_misses_; }

    // Текущий диапазон времени (для оси X графиков времени).
    double xMin() const { return xmin_; }
    double xMax() const { return xmax_; }
    double yMin() const { return ymin_; }
    double yMax() const { return ymax_; }
    // true, пока масштаб по оси подобран автоматически.
    bool isAutoX() const { return auto_x_; }
    bool isAutoY() const { return auto_y_; }

    // Точка под курсором (в пикселях виджета) — для показа подсказки.
    PlotCursor cursorAt(const QPoint &pos) const;

    // Область построения внутри виджета указанного размера (без полей).
    QRectF plotArea(const QSize &size) const { return plotRectFor(size); }

    QSize sizeHint() const override { return QSize(420, 260); }

signals:
    // Изменился диапазон осей (авто-подбор, зум, панорамирование, сброс).
    void viewRangesChanged(double xmin, double xmax, double ymin, double ymax);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void leaveEvent(QEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

    // Вызывается, когда масштаб изменил пользователь (зум/панорамирование).
    // Наследники с собственным авто-подбором масштаба отключают его здесь.
    virtual void onUserViewChanged() {}

    // Пометить кэш кадра устаревшим. Вызывается из всех методов, меняющих
    // содержимое или оформление (в том числе в наследниках).
    void invalidateCache();

    // Реализуется наследниками: содержимое внутри области построения.
    virtual void paintContent(QPainter &painter, const QRectF &plotRect) = 0;

    // Область построения внутри виджета указанного размера.
    QRectF plotRectFor(const QSize &size) const;

    // Экранные координаты из данных.
    QPointF mapToScreen(double x, double y, const QRectF &plotRect) const;
    QPointF mapToData(const QPointF &screen, const QRectF &plotRect) const;

    // Пересчитать авто-масштаб по всем рядам.
    void updateAutoRange();
    void niceRange(double &lo, double &hi);

    const QVector<PlotSeries> &series() const { return series_; }

    // Индекс ближайшей точки по X (для подсказки).
    int nearestPointIndex(const QString &name, double x) const;

private:
    void drawGrid(QPainter &painter, const QRectF &rect);
    void drawLegend(QPainter &painter, const QRectF &rect);
    void drawAxisLabels(QPainter &painter, const QRectF &rect);
    void drawEndMarker(QPainter &painter, const QRectF &rect);
    void drawCursor(QPainter &painter, const QRectF &rect);

    // Кэш кадра: содержимое + масштаб, при которых он построен.
    bool isCacheValid(const QSize &size) const;
    void rebuildCache(const QSize &size);

    QVector<PlotSeries> series_;

    double xmin_ = 0, xmax_ = 1;
    double ymin_ = 0, ymax_ = 1;
    bool auto_scale_ = true;
    bool auto_x_ = true;
    bool auto_y_ = true;

    QString title_;
    QString x_label_;
    QString y_label_;

    QColor background_;
    QColor grid_color_;
    QColor text_color_;
    QColor axis_color_;

    bool legend_visible_ = true;
    bool show_end_marker_ = true;
    bool dark_ = false;

    int margin_left_ = 64;
    int margin_right_ = 16;
    int margin_top_ = 20;
    int margin_bottom_ = 34;

    QPoint last_mouse_pos_;
    PlotCursor cursor_;

    // Кэш кадра (пункт 2.3 плана). Перерисовка без изменения содержимого
    // и масштаба просто копирует пиксели, вместо повторной отрисовки
    // сотен тысяч точек.
    QPixmap cache_;
    QSize cache_size_;
    int cache_version_ = -1;
    double cache_x0_ = 0.0, cache_x1_ = 0.0;
    double cache_y0_ = 0.0, cache_y1_ = 0.0;
    quint64 cache_hits_ = 0;
    quint64 cache_misses_ = 0;
    bool cache_enabled_ = true;
    int content_version_ = 0;
    // В режиме построения кэша курсор не рисуется — он идёт поверх кэша.
    bool building_cache_ = false;

    // Панорамирование мышью: диапазон на момент нажатия и текущая позиция.
    bool panning_ = false;
    QPoint pan_anchor_;
    double pan_xmin_ = 0.0, pan_xmax_ = 1.0;
    double pan_ymin_ = 0.0, pan_ymax_ = 1.0;
};

} // namespace gui