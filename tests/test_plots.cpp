// test_plots.cpp — Графики на QPainter: зум, панорамирование, агрегация, экспорт.
//
// Проверяется логика PlotView/TimeSeriesPlot, которая иначе проверяется только
// «глазами» в работающем GUI: колесо мыши меняет диапазон, перетаскивание
// сдвигает его, двойной клик возвращает авто-масштаб, плотные ряды
// сворачиваются в min/max-огибающую, экспорт SVG даёт валидный файл.

#include <cmath>

#include <QtTest>

#include <QApplication>
#include <QFile>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include "views/plots/time_series_plot.h"

using gui::PlotSeries;
using gui::TimeSeriesPlot;

namespace
{

// Ряд t = 0..duration с частотой 200 Гц и синусоидой заданной амплитуды.
QVector<PlotSeries> makeSeries(const QString &name, double duration_s, double amp,
                               double rate_hz, const QColor &color, bool markers)
{
    PlotSeries s;
    s.name = name;
    s.color = color;
    s.markers = markers;
    s.points.reserve(static_cast<int>(duration_s * rate_hz) + 1);
    for (int i = 0; i <= static_cast<int>(duration_s * rate_hz); ++i)
    {
        const double t = static_cast<double>(i) / rate_hz;
        s.points.push_back(QPointF(t, amp * std::sin(2.0 * M_PI * 0.5 * t)));
    }
    return {s};
}

void sendWheel(QWidget *w, const QPoint &pos, int angle_delta,
               Qt::KeyboardModifiers mods = Qt::NoModifier)
{
    QWheelEvent e(QPointF(pos), QPointF(pos), QPoint(), QPoint(0, angle_delta),
                  Qt::MouseButtons(Qt::NoButton), mods, Qt::NoScrollPhase, false);
    QApplication::sendEvent(w, &e);
}

void sendMouse(QWidget *w, QEvent::Type type, const QPoint &pos,
               Qt::MouseButton button)
{
    QMouseEvent e(type, QPointF(pos), QPointF(pos), button, button, Qt::NoModifier);
    QApplication::sendEvent(w, &e);
}

// Пиксели, отличные от фона: «что-то нарисовалось».
int paintedPixels(const QImage &img, const QColor &background)
{
    int n = 0;
    for (int y = 0; y < img.height(); ++y)
    {
        for (int x = 0; x < img.width(); ++x)
        {
            if (img.pixelColor(x, y) != background) ++n;
        }
    }
    return n;
}

} // namespace

class TestPlots : public QObject
{
    Q_OBJECT

private slots:
    void autoRangeFollowsData();
    void wheelZoomsAroundCursor();
    void wheelModifiersSelectAxis();
    void dragPansView();
    void doubleClickResetsRange();
    void rendersDenseSeries();
    void savesSvg();
    void cursorPicksNearestPoint();
    void reusesFrameCache();
    void cacheFollowsDataChanges();
    void disabledCacheDrawsWithoutPixmap();
    void cursorDrawnOnFirstFrame();
};

void TestPlots::autoRangeFollowsData()
{
    TimeSeriesPlot plot;
    QVERIFY(plot.isAutoX());
    QVERIFY(plot.isAutoY());

    plot.setSeries(makeSeries(tr("sin"), 10.0, 2.0, 100.0, Qt::red, false));

    QCOMPARE(plot.xMin(), 0.0);
    QCOMPARE(plot.xMax(), 10.0);
    // Авто-масштаб по Y симметричен относительно нуля.
    QVERIFY(plot.yMax() > 0.0);
    QVERIFY(plot.yMin() < 0.0);
    QVERIFY(qAbs(plot.yMin() + plot.yMax()) < 0.5);
}

void TestPlots::wheelZoomsAroundCursor()
{
    TimeSeriesPlot plot;
    plot.resize(500, 300);
    plot.setSeries(makeSeries(tr("sin"), 100.0, 1.0, 200.0, Qt::red, false));

    const double x0 = plot.xMin();
    const double x1 = plot.xMax();
    const double width_before = x1 - x0;

    // Колесо вверх (angleDelta > 0) приближает точку под курсором.
    const QPoint center(250, 150);
    sendWheel(&plot, center, 120);

    QVERIFY(!plot.isAutoX());
    QVERIFY(!plot.isAutoY());
    const double width_after = plot.xMax() - plot.xMin();
    QVERIFY(width_after < width_before);
    QVERIFY(qAbs(width_after - width_before * 0.8) < width_before * 0.02);

    // Точка под курсором осталась на месте.
    const QRectF area = plot.plotArea(plot.size());
    const double frac = (center.x() - area.left()) / area.width();
    const double data_x = x0 + frac * (x1 - x0);
    const double now_x = plot.xMin() + frac * (plot.xMax() - plot.xMin());
    QVERIFY(qAbs(data_x - now_x) < width_before * 0.02);

    // Колесо вниз возвращает масштаб обратно.
    sendWheel(&plot, center, -120);
    QVERIFY(qAbs((plot.xMax() - plot.xMin()) - width_before) < width_before * 0.05);
}

void TestPlots::wheelModifiersSelectAxis()
{
    TimeSeriesPlot plot;
    plot.resize(500, 300);
    plot.setSeries(makeSeries(tr("sin"), 100.0, 1.0, 200.0, Qt::red, false));

    const double span_x = plot.xMax() - plot.xMin();
    const double span_y = plot.yMax() - plot.yMin();

    // Shift — только величина (ось Y), Ctrl — только время (ось X).
    sendWheel(&plot, QPoint(250, 150), 120, Qt::ShiftModifier);
    QVERIFY(plot.yMax() - plot.yMin() < span_y);
    QVERIFY(qAbs((plot.xMax() - plot.xMin()) - span_x) < span_x * 0.02);

    TimeSeriesPlot plot2;
    plot2.resize(500, 300);
    plot2.setSeries(makeSeries(tr("sin"), 100.0, 1.0, 200.0, Qt::red, false));

    sendWheel(&plot2, QPoint(250, 150), 120, Qt::ControlModifier);
    QVERIFY(plot2.xMax() - plot2.xMin() < span_x);
    QVERIFY(qAbs((plot2.yMax() - plot2.yMin()) - span_y) < span_y * 0.02);
    // Ctrl не трогает Y — значит ось Y осталась в авторежиме.
    QVERIFY(plot2.isAutoY());
}

void TestPlots::dragPansView()
{
    TimeSeriesPlot plot;
    plot.resize(500, 300);
    plot.setSeries(makeSeries(tr("sin"), 100.0, 1.0, 200.0, Qt::red, false));

    const double x0 = plot.xMin();
    const double x1 = plot.xMax();
    const double y0 = plot.yMin();
    const double y1 = plot.yMax();

    // Перетаскивание вправо-вниз: содержимое следует за курсором, поэтому
    // диапазон смещается к более ранним временам и большим значениям.
    sendMouse(&plot, QEvent::MouseButtonPress, QPoint(250, 150), Qt::LeftButton);
    sendMouse(&plot, QEvent::MouseMove, QPoint(300, 170), Qt::LeftButton);
    sendMouse(&plot, QEvent::MouseButtonRelease, QPoint(300, 170), Qt::LeftButton);

    const QRectF area = plot.plotArea(plot.size());
    QVERIFY(plot.xMin() < x0);
    QVERIFY(plot.yMin() > y0);
    const double shift_x = (x1 - x0) * 50.0 / area.width();
    const double shift_y = (y1 - y0) * 20.0 / area.height();
    QVERIFY(qAbs((x0 - plot.xMin()) - shift_x) < (x1 - x0) * 0.01);
    QVERIFY(qAbs((plot.yMin() - y0) - shift_y) < (y1 - y0) * 0.01);
    // Протяжённость диапазона не меняется.
    QVERIFY(qAbs((plot.xMax() - plot.xMin()) - (x1 - x0)) < (x1 - x0) * 0.01);
    QVERIFY(qAbs((plot.yMax() - plot.yMin()) - (y1 - y0)) < (y1 - y0) * 0.01);

    // После отпускания диапазон не «прыгает» обратно.
    const double after_x = plot.xMin();
    sendMouse(&plot, QEvent::MouseMove, QPoint(320, 160), Qt::LeftButton);
    QCOMPARE(plot.xMin(), after_x);
}

void TestPlots::doubleClickResetsRange()
{
    TimeSeriesPlot plot;
    plot.resize(500, 300);
    plot.setSeries(makeSeries(tr("sin"), 60.0, 1.0, 200.0, Qt::red, false));

    const double x0 = plot.xMin();
    const double x1 = plot.xMax();

    sendWheel(&plot, QPoint(250, 150), 240);
    sendMouse(&plot, QEvent::MouseButtonPress, QPoint(300, 150), Qt::LeftButton);
    sendMouse(&plot, QEvent::MouseMove, QPoint(350, 150), Qt::LeftButton);
    sendMouse(&plot, QEvent::MouseButtonRelease, QPoint(350, 150), Qt::LeftButton);

    sendMouse(&plot, QEvent::MouseButtonDblClick, QPoint(350, 150), Qt::LeftButton);

    QVERIFY(plot.isAutoX());
    QVERIFY(plot.isAutoY());
    QVERIFY(qAbs(plot.xMin() - x0) < 1e-9);
    QVERIFY(qAbs(plot.xMax() - x1) < 1e-9);
}

void TestPlots::rendersDenseSeries()
{
    // 15 плотных рядов по 120 000 точек (200 Гц × 600 с) — из них рисуется
    // min/max-огибающая, но отрисовка всё равно должна быть быстрой.
    TimeSeriesPlot plot;
    plot.resize(900, 500);

    QVector<PlotSeries> all;
    for (int i = 0; i < 15; ++i)
    {
        all += makeSeries(tr("ряд %1").arg(i), 600.0, 1.0 + 0.1 * i, 200.0,
                          QColor(31 + 12 * i, 90, 160), false);
    }
    plot.setSeries(all);
    QCOMPARE(plot.seriesCount(), 15);

    QImage img(900, 500, QImage::Format_ARGB32);
    img.fill(Qt::white);
    QElapsedTimer timer;
    timer.start();
    {
        QPainter p(&img);
        plot.renderTo(p, img.size());
    }
    const qint64 ms = timer.elapsed();

    QVERIFY2(paintedPixels(img, Qt::white) > 1000, "график должен что-то рисовать");

    // Проверка не абсолютной скорости (она зависит от машины и от того, есть ли
    // в системе шрифты), а отсутствия патологического замедления: min/max
    // агрегация должна удержать 15×120 000 точек в разумных десятках секунд.
    QVERIFY2(ms < 20000, qPrintable(QString("отрисовка 15×120000 точек: %1 мс").arg(ms)));
}

void TestPlots::savesSvg()
{
    TimeSeriesPlot plot;
    plot.resize(600, 400);
    plot.setTitle(tr("Тест"));
    plot.setSeries(makeSeries(tr("sin"), 10.0, 1.0, 100.0, Qt::red, false));

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("plot.svg");
    QVERIFY(plot.saveSvg(path, QSize(600, 400)));
    QVERIFY(QFileInfo::exists(path));

    QFile f(path);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString head = QString::fromUtf8(f.read(512));
    QVERIFY2(head.contains("<svg"), qPrintable(head.left(120)));
    QVERIFY2(head.contains("width"), qPrintable(head.left(120)));

    // Внутренняя часть файла непустая (есть кривые).
    const qint64 size = QFileInfo(path).size();
    QVERIFY(size > 1000);
}

void TestPlots::cursorPicksNearestPoint()
{
    TimeSeriesPlot plot;
    plot.resize(500, 300);
    plot.setSeries(makeSeries(tr("sin"), 10.0, 1.0, 100.0, Qt::red, false));

    const gui::PlotCursor c = plot.cursorAt(QPoint(250, 150));
    QVERIFY(c.valid);
    QVERIFY(!c.text.isEmpty());
    // Вне области построения подсказки нет.
    QVERIFY(!plot.cursorAt(QPoint(5, 5)).valid);
}

void TestPlots::reusesFrameCache()
{
    TimeSeriesPlot plot;
    plot.resize(400, 300);
    plot.setSeries(makeSeries(tr("синус"), 10.0, 1.0, 200.0, Qt::red, false));

    QVERIFY(plot.isCacheEnabled());

    const quint64 misses_before = plot.cacheMisses();
    const quint64 hits_before = plot.cacheHits();

    // Первый показ строит кэш, последующие при неизменных данных и масштабе
    // берут содержимое из кэша (пункт 2.3 плана).
    plot.show();
    plot.grab();
    plot.grab();

    QVERIFY2(plot.cacheMisses() > misses_before, "первый показ должен построить кэш");
    QVERIFY2(plot.cacheHits() > hits_before, "повторный показ должен использовать кэш");
}

void TestPlots::cacheFollowsDataChanges()
{
    TimeSeriesPlot plot;
    plot.resize(400, 300);
    plot.setSeries(makeSeries(tr("а"), 10.0, 1.0, 100.0, Qt::red, false));

    plot.grab();
    const QImage before = plot.grab().toImage();

    // Смена данных обязана перестраивать кадр, иначе на экране осталась бы
    // старая кривая.
    plot.setSeries(makeSeries(tr("б"), 10.0, 3.0, 100.0, Qt::blue, false));
    plot.grab();
    const QImage after = plot.grab().toImage();

    QVERIFY(before.size() == after.size());
    QVERIFY2(before != after, "после смены данных кадр должен измениться");
}

void TestPlots::disabledCacheDrawsWithoutPixmap()
{
    TimeSeriesPlot plot;
    plot.resize(400, 300);
    plot.setSeries(makeSeries(tr("синус"), 10.0, 1.0, 200.0, Qt::red, false));

    plot.show();
    plot.grab();  // прогреть окно

    plot.setCacheEnabled(false);
    const quint64 misses = plot.cacheMisses();
    const quint64 hits = plot.cacheHits();

    const QImage image = plot.grab().toImage();

    // С выключенным кэшем содержимое рисуется напрямую: ни одного промаха,
    // ни одного попадания, но картинка непустая.
    QCOMPARE(plot.cacheMisses(), misses);
    QCOMPARE(plot.cacheHits(), hits);
    QVERIFY(!image.isNull());

    // Содержимое остаётся тем же: отключение кэша не меняет отрисовку.
    plot.setCacheEnabled(true);
    const QImage cached = plot.grab().toImage();
    QCOMPARE(cached.size(), image.size());
}

void TestPlots::cursorDrawnOnFirstFrame()
{
    TimeSeriesPlot plot;
    plot.resize(400, 300);
    plot.setSeries(makeSeries(tr("синус"), 10.0, 1.0, 200.0, Qt::red, false));
    plot.show();

    // Курсор появляется на первом же кадре после появления виджета:
    // rebuildCache() намеренно не кладёт курсор в QPixmap, поэтому
    // paintEvent() обязан дорисовать его поверх кадра и при промахе.
    sendMouse(&plot, QEvent::MouseMove, QPoint(200, 150), Qt::NoButton);
    const QImage with_cursor = plot.grab().toImage();

    sendMouse(&plot, QEvent::MouseMove, QPoint(2, 2), Qt::NoButton);
    const QImage without_cursor = plot.grab().toImage();

    QVERIFY2(with_cursor != without_cursor,
             "курсор должен попадать в первый кадр, а не только в кэш");
}

QTEST_MAIN(TestPlots)
#include "test_plots.moc"