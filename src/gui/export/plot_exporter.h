// plot_exporter.h — Экспорт графиков и данных.
//
// PNG/JPEG через QPixmap, PDF/PS через QPdfWriter (QtPrintSupport), CSV —
// текстом из табличной модели.

#pragma once

#include <QString>

class QWidget;

namespace gui
{

class PlotView;
class NavTableModel;
class RunSeries;

class PlotExporter
{
public:
    // Экспорт виджета-графика в изображение (png, jpg, bmp).
    static bool exportPlotImage(PlotView *plot, const QString &fileName,
                                int width = 0, int height = 0, QString *error = nullptr);

    // Экспорт виджета-графика в PDF.
    static bool exportPlotPdf(PlotView *plot, const QString &fileName,
                              int width = 0, int height = 0, QString *error = nullptr);

    // Экспорт виджета-графика в векторный SVG (QSvgGenerator).
    static bool exportPlotSvg(PlotView *plot, const QString &fileName,
                              int width = 0, int height = 0, QString *error = nullptr);

    // Экспорт таблицы результатов в CSV.
    static bool exportTableCsv(NavTableModel *model, const QString &fileName,
                               QString *error = nullptr);

    // Экспорт всего ряда (траектория + эталоны) в текстовый файл.
    static bool exportSeriesCsv(const RunSeries &series, const QString &fileName,
                                QString *error = nullptr);
};

} // namespace gui