// plot_exporter.h — Экспорт графиков и данных.
//
// PNG/JPEG через QPixmap, PDF/PS через QPdfWriter (QtPrintSupport), CSV —
// текстом из табличной модели.

#pragma once

#include <QString>
#include <QList>

#include <functional>

class QWidget;

namespace gui
{

class PlotView;
class NavTableModel;
class RunSeries;

// Один график для пакетного экспорта: имя (название вкладки, без пути) и сам
// виджет. Имя используется в имени файла.
struct PlotExportItem
{
    QString name;
    PlotView *plot;
};

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

    // Пакетный экспорт всех графиков в один каталог и один формат
    // (format: png/jpg/bmp/svg/pdf). Имена файлов — имена вкладок с заменой
    // недопустимых символов. progress вызывается перед каждым графиком и
    // возвращает true, чтобы продолжить, false — прервать (не считается ошибкой).
    // in exported собирается число удачно записанных графиков.
    static bool exportAllPlots(const QList<PlotExportItem> &items,
                               const QString &dir, const QString &format,
                               int width = 0, int height = 0,
                               const std::function<bool(int done, int total)> &progress = nullptr,
                               int *exported = nullptr, QString *error = nullptr);
};

} // namespace gui