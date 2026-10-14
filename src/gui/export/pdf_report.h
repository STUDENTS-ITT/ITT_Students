// pdf_report.h — Сводный отчёт по прогону в PDF (пункт 2.9 плана).
//
// В отчёте: титул с параметрами прогона, статистика ошибок фильтра по
// компонентам x[0..14], сводка NIS/NEES и несколько графиков, отрисованных
// теми же виджетами QPainter, что и в GUI.

#pragma once

#include <QString>
#include <QVector>

#include "core_bridge/run_store.h"
#include "core_bridge/telemetry_buffer.h"
#include "settings/dataset_truth.h"
#include "views/plots/plot_view.h"

namespace gui
{

class PdfReport
{
public:
    // Ряд графика в отчёте. Собирается на стороне окна из тех же данных, что
    // уходят в легенду, поэтому отчёт печатает ровно то, что видно на экране.
    struct PlotBlock
    {
        QString caption;
        // Подписи осей для блока во временных координатах. У каждого блока в
        // отчёте они свои (время/единица либо восток/север), поэтому «холст»
        // переключает их на время отрисовки блока.
        QString x_label;
        QString y_label;
        // Готовый виджет с собственными данными (траектория): рисуется как есть.
        PlotView *widget = nullptr;
        // Либо ряд во временных координатах: кладётся в переданный в build()
        // «холст», чтобы не заводить виджет на каждый блок.
        QVector<PlotSeries> series;
        QVector<double> sync_markers;  // моменты коррекции СНС (вертикали)
    };

    struct Options
    {
        QString title;
        QString data_dir;
        // Шапка: имя набора данных и файл настроек/пресета прогона.
        QString dataset;
        QString preset;
        // Сверка имён «набор ↔ пресет»: статус паспорта набора из секции
        // [meta] пресета (совпадает / подобран на другом наборе / нет).
        QString preset_note;
        // Снимок параметров фильтра (текст INI, dumpFilterSettings): печатается
        // отдельным разделом, чтобы в отчёте были видны применённые значения.
        QString settings_ini;
        // Наземная истина набора (truth.txt), если корректна: по ней в отчёте
        // считается ошибка прогона. Пусто — сравнение с эталоном СНС.
        QVector<TruthSample> truth;
        bool include_plots = true;
        // Ограничение на число строк в таблице NIS/NEES: полный набор за
        // 600 с — это тысячи строк, в отчёте оставляем выборку.
        int max_consistency_rows = 200;
    };

    // Собрать отчёт по текущему прогону. plot — виджет, который используется
    // как «холст» для отрисовки блоков (его ряды перезаписываются).
    static bool build(PlotView *plot, const RunSeries &series,
                      const QVector<PlotBlock> &blocks, const Options &options,
                      const QString &fileName, QString *error = nullptr);

    // Метки и значения, из которых строится отчёт (для тестов и отладки).
    static QString summarize(const RunSeries &series);
};

} // namespace gui