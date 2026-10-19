// test_pdf_report — Сводный PDF-отчёт по прогону (план 2.9).
//
// Проверяется, что отчёт действительно собирается в PDF-файл, содержит
// ожидаемые страницы/байты и что текстовая сводка считается по данным.
// Отчёт собирается напрямую из исходников Qt-слоя, как test_plots.

#include <QtTest>

#include <cmath>

#include <QTemporaryDir>

#include "core/utils/constants.h"
#include "core_bridge/telemetry_buffer.h"
#include "export/pdf_report.h"
#include "views/plots/time_series_plot.h"

using namespace gui;

namespace
{
// Ряд из точек (x — время, y — значение): достаточно, чтобы отчёт был непустым.
PlotSeries makeSeries(const QString &name, const QVector<double> &t, const QVector<double> &v)
{
    PlotSeries s;
    s.name = name;
    s.color = Qt::red;
    s.points.reserve(t.size());
    for (int i = 0; i < t.size(); ++i) s.points.push_back(QPointF(t[i], v[i]));
    return s;
}

RunSeries makeRun()
{
    RunSeries run;
    for (int i = 0; i <= 100; i++)
    {
        RunSeries::Row r{};
        r.time = 0.1 * i;
        for (int k = 0; k < RunSeries::Ve; k++) r.result[k] = 0.01 * k * i;
        r.has_reference = true;
        r.reference[RunSeries::Lon] = 0.001 * i;
        r.reference[RunSeries::Lat] = 0.002 * i;
        run.rows.push_back(r);
    }
    for (int i = 0; i <= 10; i++)
    {
        RunSeries::CorrectionRow c{};
        c.time = 1.0 * i;
        c.x[0] = 0.001 * i;
        c.x[3] = 0.05 * i;
        c.x[4] = -0.02 * i;
        run.corrections.push_back(c);

        RunSeries::ConsistencyRow k{};
        k.time = 1.0 * i;
        k.dof = ins::KF_MEAS;
        k.nis = 9.0 + 0.1 * i;
        k.nees = 8.5 + 0.1 * i;
        run.consistency.push_back(k);
    }
    return run;
}
} // namespace

class TestPdfReport : public QObject
{
    Q_OBJECT

private slots:
    void summaryCountsRows();
    void summaryReportsDofAndErrors();
    void deviationsCountPeaksAndThresholds();
    void deviationsRespectAlignWindow();
    void errorSeriesMatchesDeviations();
    void errorSeriesRespectsAlignWindow();
    void errorSeriesFallsBackToReference();
    void emptyRunIsRejected();
    void buildsPdfFile();
    void buildsPdfWithTruth();
    void blockAxisLabelsAreRestored();
    void plotsDoNotChangeCanvasState();
    void biasSeriesCarriesEstimateSigmaAndTruth();
};

void TestPdfReport::summaryCountsRows()
{
    const RunSeries run = makeRun();
    const QString text = PdfReport::summarize(run);
    QVERIFY2(text.contains(QString::number(run.rows.size())),
             qPrintable(text));
    QVERIFY(text.contains(QString::fromUtf8("Отсчётов")));
}

void TestPdfReport::summaryReportsDofAndErrors()
{
    const RunSeries run = makeRun();
    const QString text = PdfReport::summarize(run);

    QVERIFY2(text.contains(QString::number(ins::KF_MEAS)), qPrintable(text));
    QVERIFY2(text.contains(QString::fromUtf8("NIS")), qPrintable(text));
    QVERIFY2(text.contains(QString::fromUtf8("NEES")), qPrintable(text));
    // RMS по вектору ошибок присутствует для непустых компонент.
    QVERIFY2(text.contains(QString::fromUtf8("RMS")), qPrintable(text));
}

// Таблица отклонений: пороги считаются по модулю отклонения от опоры, а
// максимум — по всем точкам сравнения.
void TestPdfReport::deviationsCountPeaksAndThresholds()
{
    RunSeries run;
    for (int i = 0; i < 10; i++)
    {
        RunSeries::Row r{};
        r.time = 0.1 * i;
        r.result[RunSeries::Lat] = 47.0;
        r.result[RunSeries::Lon] = -122.0;
        r.result[RunSeries::Alt] = 100.0;
        r.result[RunSeries::Heading] = 10.0;
        r.result[RunSeries::Vn] = 1.0;
        run.rows.push_back(r);
    }

    // Пять отсчётов истины: по высоте отклонения 0 / 0.2 / 0.6 / 1.2 / 6.0 м,
    // по широте — 1e-5 град (1.11 м), по курсу — 359° (по кругу 1°).
    const double alt_dev[5] = {0.0, 0.2, 0.6, 1.2, 6.0};
    QVector<TruthSample> truth;
    for (int k = 0; k < 5; k++)
    {
        TruthSample tr;
        tr.v[0] = run.rows[2 * k].time;
        tr.v[1] = -122.0;
        tr.v[2] = 47.0 - 1e-5;
        tr.v[3] = 100.0 - alt_dev[k];
        tr.v[4] = 10.0 - 359.0;
        truth.push_back(tr);
    }

    const QVector<PdfReport::DeviationGroup> groups = PdfReport::deviations(run, truth);
    QCOMPARE(int(groups.size()), 3);
    QCOMPARE(groups[0].samples, 5);
    QCOMPARE(groups[0].caption, QString::fromUtf8("Координаты"));
    QCOMPARE(groups[0].unit, QString::fromUtf8("м"));

    const auto rowNamed = [](const PdfReport::DeviationGroup &g, const QString &name) {
        for (const PdfReport::DeviationRow &r : g.rows)
            if (r.name == name) return r;
        return PdfReport::DeviationRow();
    };

    const PdfReport::DeviationRow alt = rowNamed(groups[0], QString::fromUtf8("Высота"));
    QCOMPARE(int(alt.counts.size()), 7);
    QCOMPARE(alt.counts[0], 4);  // > 0.1 м: 0.2, 0.6, 1.2, 6.0
    QCOMPARE(alt.counts[1], 3);  // > 0.5 м
    QCOMPARE(alt.counts[2], 2);  // > 1 м
    QCOMPARE(alt.counts[3], 1);  // > 1.5 м
    QCOMPARE(alt.counts[6], 1);  // > 5 м
    QVERIFY2(std::abs(alt.peak - 6.0) < 1e-9, qPrintable(QString::number(alt.peak)));

    const PdfReport::DeviationRow lat = rowNamed(groups[0], QString::fromUtf8("Широта"));
    QVERIFY2(std::abs(lat.peak - 1.11) < 0.01, qPrintable(QString::number(lat.peak)));
    QCOMPARE(lat.counts[2], 5);  // > 1 м — все пять точек

    const PdfReport::DeviationRow hdg = rowNamed(groups[1], QString::fromUtf8("Курс"));
    QVERIFY2(std::abs(hdg.peak - 1.0) < 1e-9, qPrintable(QString::number(hdg.peak)));

    // Без истины опора — эталон СНС строки результата (кадры коррекции).
    RunSeries ref_run = run;
    for (RunSeries::Row &r : ref_run.rows)
    {
        r.has_reference = true;
        r.reference[RunSeries::Alt] = 100.0 - 2.0;  // отклонение 2 м в каждой строке
    }
    const QVector<PdfReport::DeviationGroup> ref_only =
        PdfReport::deviations(ref_run, QVector<TruthSample>());
    QCOMPARE(int(ref_only.size()), 3);
    QCOMPARE(ref_only[0].samples, int(ref_run.rows.size()));
    const PdfReport::DeviationRow ref_alt = rowNamed(ref_only[0], QString::fromUtf8("Высота"));
    QVERIFY2(std::abs(ref_alt.peak - 2.0) < 1e-9, qPrintable(QString::number(ref_alt.peak)));
    QCOMPARE(ref_alt.counts[2], int(ref_run.rows.size()));  // > 1 м — все строки

    // Нет ни истины, ни эталона — таблица не строится.
    QVERIFY(PdfReport::deviations(run, QVector<TruthSample>()).isEmpty());
}

// Окно анализа: точки до конца выставки в таблицу отклонений не попадают —
// иначе предвыставочные всплески (курс/высота) портили бы метрики отчёта.
void TestPdfReport::deviationsRespectAlignWindow()
{
    RunSeries run;
    for (int i = 0; i < 10; i++)
    {
        RunSeries::Row r{};
        r.time = 0.1 * i;
        r.result[RunSeries::Alt] = 100.0;
        run.rows.push_back(r);
    }

    // Истина: до t = 0.5 с — большое отклонение, после — малое.
    QVector<TruthSample> truth;
    for (int i = 0; i < 10; i++)
    {
        TruthSample tr;
        tr.v[0] = 0.1 * i;
        tr.v[3] = 100.0 - (i < 5 ? 10.0 : 0.3);
        truth.push_back(tr);
    }

    // Без окна — 10 точек, максимум ~10 м.
    const QVector<PdfReport::DeviationGroup> all = PdfReport::deviations(run, truth);
    QCOMPARE(all[0].samples, 10);
    QCOMPARE(all.size(), 3);
    const auto find_alt = [](const PdfReport::DeviationGroup &g) {
        for (const PdfReport::DeviationRow &r : g.rows)
            if (r.name == QString::fromUtf8("Высота")) return r;
        return PdfReport::DeviationRow();
    };
    QVERIFY2(std::abs(find_alt(all[0]).peak - 10.0) < 1e-12, "peak_all");

    // С окном t >= 0.5 с — только 5 точек после выставки, максимум ~0.3 м.
    const QVector<PdfReport::DeviationGroup> windowed = PdfReport::deviations(run, truth, 0.5);
    QCOMPARE(windowed[0].samples, 5);
    QVERIFY2(std::abs(find_alt(windowed[0]).peak - 0.3) < 1e-12, "peak_windowed");
}

// График ошибок строится по тем же парам «строка — опора», что и таблица отклонений, и в тех же единицах: пик на графике равен peak строки таблицы, а знак показывает сторону ошибки (курс — по кратчайшей дуге).
void TestPdfReport::errorSeriesMatchesDeviations()
{
    RunSeries run;
    for (int i = 0; i < 10; i++)
    {
        RunSeries::Row r{};
        r.time = 0.1 * i;
        r.result[RunSeries::Lat] = 47.0;
        r.result[RunSeries::Alt] = 100.0;
        r.result[RunSeries::Heading] = 10.0;
        run.rows.push_back(r);
    }

    // Пять отсчётов истины (каждой второй строке): широта на 1e-5° меньше (ошибка на север +1.11 м), высота — на 0..8 м ниже, курс на 359° меньше (по кратчайшей дуге это −1°, а не −359°).
    QVector<TruthSample> truth;
    for (int k = 0; k < 5; k++)
    {
        TruthSample tr;
        tr.v[0] = run.rows[2 * k].time;
        tr.v[2] = 47.0 - 1e-5;
        tr.v[3] = 100.0 - 2.0 * k;
        tr.v[4] = 10.0 - 359.0;
        truth.push_back(tr);
    }

    const QVector<PdfReport::DeviationGroup> groups = PdfReport::deviations(run, truth);
    const auto rowNamed = [](const PdfReport::DeviationGroup &g, const QString &name) {
        for (const PdfReport::DeviationRow &r : g.rows)
            if (r.name == name) return r;
        return PdfReport::DeviationRow();
    };
    const PdfReport::DeviationRow alt_table = rowNamed(groups[0], QString::fromUtf8("Высота"));
    const PdfReport::DeviationRow lat_table = rowNamed(groups[0], QString::fromUtf8("Широта"));
    const PdfReport::DeviationRow hdg_table = rowNamed(groups[1], QString::fromUtf8("Курс"));

    // Ошибка высоты растёт линейно 0, 2, 4, 6, 8 м: оценка выше истины, знак положительный.
    const QVector<PlotSeries> alt = PdfReport::errorSeries(run, truth, RunSeries::Alt);
    QCOMPARE(int(alt.size()), 1);
    QCOMPARE(alt.first().name, QString::fromUtf8("Высота"));
    QCOMPARE(int(alt.first().points.size()), 5);
    for (int k = 0; k < 5; k++)
        QVERIFY2(std::abs(alt.first().points[k].y() - 2.0 * k) < 1e-9, "alt error");
    QVERIFY2(std::abs(std::abs(alt.first().points.last().y()) - alt_table.peak) < 1e-9,
             "пик графика ошибок равен peak таблицы");

    // Широта: оценка севернее истины — ошибка +1.11 м.
    const QVector<PlotSeries> lat = PdfReport::errorSeries(run, truth, RunSeries::Lat);
    QCOMPARE(int(lat.first().points.size()), 5);
    QVERIFY2(std::abs(lat.first().points.first().y() - 1.11) < 0.01, "lat error");
    QVERIFY2(std::abs(std::abs(lat.first().points.first().y()) - lat_table.peak) < 0.01,
             "|ошибку широты| = пике таблицы");

    // Курс: 359° по кругу дают −1°, а не −359°, и знак отрицательный.
    const QVector<PlotSeries> hdg = PdfReport::errorSeries(run, truth, RunSeries::Heading);
    QCOMPARE(int(hdg.first().points.size()), 5);
    QVERIFY2(std::abs(hdg.first().points.first().y() - (-1.0)) < 1e-9, "hdg signed");
    QVERIFY2(std::abs(std::abs(hdg.first().points.first().y()) - hdg_table.peak) < 1e-9,
             "|ошибку курса| = пике таблицы");

    // Единицы графика ошибок — единицы таблицы.
    QCOMPARE(PdfReport::errorUnit(RunSeries::Lat), QString::fromUtf8("м"));
    QCOMPARE(PdfReport::errorUnit(RunSeries::Alt), QString::fromUtf8("м"));
    QCOMPARE(PdfReport::errorUnit(RunSeries::Heading), QString::fromUtf8("°"));
    QCOMPARE(PdfReport::errorUnit(RunSeries::Vn), QString::fromUtf8("м/с"));
}

// Окно анализа: с align_time_s точки до выставки отбрасываются — как в таблице отклонений, чтобы предвыставочные всплески не искажали график.
void TestPdfReport::errorSeriesRespectsAlignWindow()
{
    RunSeries run;
    for (int i = 0; i < 10; i++)
    {
        RunSeries::Row r{};
        r.time = 0.1 * i;
        r.result[RunSeries::Alt] = 100.0;
        run.rows.push_back(r);
    }

    // Истина: до t = 0.5 с ошибка высоты 10 м, после — 0.3 м.
    QVector<TruthSample> truth;
    for (int i = 0; i < 10; i++)
    {
        TruthSample tr;
        tr.v[0] = 0.1 * i;
        tr.v[3] = 100.0 - (i < 5 ? 10.0 : 0.3);
        truth.push_back(tr);
    }

    // Без окна — 10 точек и пик 10 м.
    const QVector<PlotSeries> all = PdfReport::errorSeries(run, truth, RunSeries::Alt);
    QCOMPARE(int(all.size()), 1);
    QCOMPARE(int(all.first().points.size()), 10);
    QVERIFY2(std::abs(all.first().points.first().y() - 10.0) < 1e-12, "peak_all");

    // С окном t >= 0.5 с — только 5 точек после выставки, пик 0.3 м.
    const QVector<PlotSeries> win = PdfReport::errorSeries(run, truth, RunSeries::Alt, 0.5);
    QCOMPARE(int(win.first().points.size()), 5);
    QVERIFY2(std::abs(win.first().points.first().y() - 0.3) < 1e-12, "peak_windowed");
}

// Без истины опора — эталон СНС строки результата; строки без эталона в график не попадают.
void TestPdfReport::errorSeriesFallsBackToReference()
{
    RunSeries run;
    for (int i = 0; i < 6; i++)
    {
        RunSeries::Row r{};
        r.time = 0.1 * i;
        r.result[RunSeries::Alt] = 100.0;
        // У трёх строк из шести нет эталона — они пропускаются.
        r.has_reference = (i % 2 == 0);
        r.reference[RunSeries::Alt] = 100.0 - 2.0;
        run.rows.push_back(r);
    }

    const QVector<PlotSeries> err =
        PdfReport::errorSeries(run, QVector<TruthSample>(), RunSeries::Alt);
    QCOMPARE(int(err.size()), 1);
    QCOMPARE(int(err.first().points.size()), 3);
    for (const QPointF &p : err.first().points)
        QVERIFY2(std::abs(p.y() - 2.0) < 1e-9, "ошибка от эталона СНС");

    // Ни истины, ни эталона — графика ошибок нет.
    RunSeries bare;
    RunSeries::Row r{};
    r.time = 0.0;
    bare.rows.push_back(r);
    QVERIFY(PdfReport::errorSeries(bare, QVector<TruthSample>(), RunSeries::Alt).isEmpty());
}

void TestPdfReport::emptyRunIsRejected()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("empty.pdf");

    QString error;
    TimeSeriesPlot canvas;
    QVERIFY(!PdfReport::build(&canvas, RunSeries(), QVector<PdfReport::PlotBlock>(),
                              PdfReport::Options(), path, &error));
    QVERIFY(!error.isEmpty());
    QVERIFY(!QFileInfo::exists(path));
}

void TestPdfReport::buildsPdfFile()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("report.pdf");

    const RunSeries run = makeRun();

    // Холст: обычный график с чужим рядом — он не должен пострадать.
    TimeSeriesPlot canvas;
    QVector<double> t, v;
    for (int i = 0; i <= 50; i++)
    {
        t.push_back(0.2 * i);
        v.push_back(std::sin(0.2 * i));
    }
    canvas.setSeries({makeSeries(QStringLiteral("исходный"), t, v)});

    PdfReport::PlotBlock block;
    block.caption = QStringLiteral("Тестовый ряд");
    block.series = {makeSeries(QStringLiteral("ряд 1"), t, v)};
    block.sync_markers = {0.0, 5.0, 10.0};

    PdfReport::Options options;
    options.title = QStringLiteral("Тестовый отчёт");
    // Шапка: имя набора и пресет; параметры фильтра — отдельным разделом.
    options.dataset = QStringLiteral("static_300");
    options.preset = QStringLiteral("presets/tuned_static_300.ini");
    options.settings_ini = QStringLiteral("# комментарий\n[filter]\n"
                                          "sigma_g = 2.12428e-02\n"
                                          "sigma_a = 3e-03\n"
                                          "[run]\nalign_time_s = 120\n");

    QString error;
    QVERIFY2(PdfReport::build(&canvas, run, {block}, options, path, &error),
             qPrintable(error));
    QVERIFY(QFileInfo::exists(path));

    const QFileInfo info(path);
    QVERIFY2(info.size() > 1000, "PDF не должен быть пустым");

    // Сигнатура PDF и наличие страниц.
    QFile f(path);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString head = QString::fromUtf8(f.read(1024));
    QVERIFY2(head.startsWith(QStringLiteral("%PDF")), qPrintable(head.left(32)));

    // Ряды холста восстановлены: отчёт не оставил мусор в графике.
    QCOMPARE(canvas.seriesCount(), 1);
    QCOMPARE(canvas.reportSeries().size(), 1);
    QCOMPARE(canvas.reportSeries().first().name, QStringLiteral("исходный"));
}

void TestPdfReport::buildsPdfWithTruth()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("truth.pdf");

    const RunSeries run = makeRun();

    // Истина близка к результату, но не совпадает: раздел метрик должен
    // получить ненулевые, но малые ошибки.
    QVector<TruthSample> truth;
    for (int i = 0; i <= 100; i += 10)
    {
        TruthSample tr;
        tr.v[0] = run.rows[i].time;
        tr.v[1] = run.rows[i].result[RunSeries::Lon] + 1e-5;   // ~1 м по долготе
        tr.v[2] = run.rows[i].result[RunSeries::Lat];
        tr.v[4] = run.rows[i].result[RunSeries::Heading];
        tr.v[7] = run.rows[i].result[RunSeries::Vn] + 0.01;
        tr.v[9] = run.rows[i].result[RunSeries::Ve] + 0.02;
        truth.push_back(tr);
    }

    PdfReport::Options options;
    options.title = QStringLiteral("Отчёт с истиной");
    options.truth = truth;

    TimeSeriesPlot canvas;
    QString error;
    QVERIFY2(PdfReport::build(&canvas, run, QVector<PdfReport::PlotBlock>(), options,
                              path, &error),
             qPrintable(error));
    QVERIFY(QFileInfo::exists(path));
    const qint64 with_truth = QFileInfo(path).size();
    QVERIFY2(with_truth > 1000, "PDF не должен быть пустым");

    // Без истины раздел не печатается — отчёт должен быть заметно меньше.
    PdfReport::Options plain;
    plain.title = options.title;
    TimeSeriesPlot plain_canvas;
    const QString plain_path = dir.filePath("plain.pdf");
    QVERIFY2(PdfReport::build(&plain_canvas, run, QVector<PdfReport::PlotBlock>(), plain,
                              plain_path, &error),
             qPrintable(error));
    QVERIFY2(QFileInfo(plain_path).size() < with_truth,
             "раздел «Ошибка относительно истины» не добавился");
}

void TestPdfReport::blockAxisLabelsAreRestored()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    TimeSeriesPlot canvas;
    canvas.setAxisLabels(QStringLiteral("Время, с"), QStringLiteral("исходная ось"));
    QVector<double> t, v;
    for (int i = 0; i <= 20; i++)
    {
        t.push_back(i);
        v.push_back(i * 0.5);
    }
    canvas.setSeries({makeSeries(QStringLiteral("ряд"), t, v)});

    // У блока свои подписи осей (величина), они не должны «протечь» на холст.
    PdfReport::PlotBlock block;
    block.caption = QStringLiteral("Долгота");
    block.series = {makeSeries(QStringLiteral("Долгота"), t, v)};
    block.x_label = QStringLiteral("Время, с");
    block.y_label = QStringLiteral("°");

    QString error;
    QVERIFY2(PdfReport::build(&canvas, makeRun(), {block}, PdfReport::Options(),
                              dir.filePath("labels.pdf"), &error),
             qPrintable(error));

    QCOMPARE(canvas.xLabel(), QStringLiteral("Время, с"));
    QCOMPARE(canvas.yLabel(), QStringLiteral("исходная ось"));
}

void TestPdfReport::plotsDoNotChangeCanvasState()
{
    TimeSeriesPlot canvas;
    QVector<double> t, v;
    for (int i = 0; i <= 20; i++)
    {
        t.push_back(i);
        v.push_back(i * i);
    }
    canvas.setSeries({makeSeries(QStringLiteral("a"), t, v)});

    const double xmin_before = canvas.xMin();
    const double xmax_before = canvas.xMax();
    // Метки синхронизации принадлежат виджету, а не блоку отчёта:
    // после экспорта они должны остаться прежними.
    canvas.setSyncMarkers({2.0, 7.0});

    PdfReport::PlotBlock block;
    block.series = {makeSeries(QStringLiteral("b"), t, v),
                    makeSeries(QStringLiteral("c"), t, v)};
    block.sync_markers = {1.0, 4.0, 9.0, 16.0};

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    QString error;
    QVERIFY2(PdfReport::build(&canvas, makeRun(), {block}, PdfReport::Options(),
                              dir.filePath("state.pdf"), &error),
             qPrintable(error));

    QCOMPARE(canvas.reportSeries().size(), 1);
    QCOMPARE(canvas.xMin(), xmin_before);
    QCOMPARE(canvas.xMax(), xmax_before);
    QCOMPARE(canvas.syncMarkers(), QVector<double>({2.0, 7.0}));
}

// Смещения ИМУ (этап 6 плана): оценка фильтра, полоса ±2σ(P) и истина набора
// из params.json на одном графике.
void TestPdfReport::biasSeriesCarriesEstimateSigmaAndTruth()
{
    RunSeries run;
    run.has_bias = true;
    const int n = 5;
    for (int i = 0; i < n; i++)
    {
        RunSeries::Row r{};
        r.time = 1.0 * i;
        r.ba[0] = 1.0e-2;   // м/с²
        r.bg[2] = 1.0e-5;   // рад/с
        r.p[9] = 4.0e-6;    // σ(ba_X) = 2e-3 м/с²
        r.p[14] = 1.0e-10;  // σ(bg_Z) = 1e-5 рад/с
        run.rows.push_back(r);
    }

    nav::BiasTruth truth;
    truth.valid = true;
    truth.ba[0] = 5.0e-3;         // м/с²
    truth.bg[2] = 2.0e-5;         // рад/с
    truth.observable[3] = false;  // bg_X на стоянке ненаблюдаем

    // ba_X: оценка, +2σ, −2σ, истина — всё в мг.
    const double mg = 1.0 / 9.80665e-3;
    const QVector<PlotSeries> ba = PdfReport::biasSeries(run, truth, 0);
    QCOMPARE(ba.size(), 4);
    QCOMPARE(ba[0].points.size(), n);
    QVERIFY(std::abs(ba[0].points[0].y() - 1.0e-2 * mg) < 1e-9);
    QVERIFY(std::abs(ba[1].points[0].y() - (1.0e-2 + 2.0 * 2.0e-3) * mg) < 1e-9);
    QVERIFY(std::abs(ba[2].points[0].y() - (1.0e-2 - 2.0 * 2.0e-3) * mg) < 1e-9);
    QCOMPARE(ba[3].points.size(), 2);   // истина — горизонтальная линия
    QVERIFY(std::abs(ba[3].points[1].y() - 5.0e-3 * mg) < 1e-9);
    QVERIFY(ba[3].name.contains(QString::fromUtf8("истина")));

    // bg_Z: инженерные единицы °/ч.
    const double deg_hour = RAD_TO_DEG * 3600.0;
    const QVector<PlotSeries> bg = PdfReport::biasSeries(run, truth, 5);
    QCOMPARE(bg.size(), 4);
    QVERIFY(std::abs(bg[0].points[0].y() - 1.0e-5 * deg_hour) < 1e-9);

    // Ненаблюдаемая компонента подписана явно, линейка истины всё равно есть.
    const QVector<PlotSeries> bg_x = PdfReport::biasSeries(run, truth, 3);
    QCOMPARE(bg_x.size(), 4);
    QVERIFY(bg_x[3].name.contains(QString::fromUtf8("ненаблюдаема")));

    // Без params.json (истина не прочитана) остаются только оценка и полоса.
    QCOMPARE(PdfReport::biasSeries(run, nav::BiasTruth(), 0).size(), 3);

    // Подписи и единицы; индекс вне 0..5 — пустой ряд.
    QCOMPARE(PdfReport::biasLabel(0), QStringLiteral("ba_X"));
    QCOMPARE(PdfReport::biasLabel(5), QStringLiteral("bg_Z"));
    QCOMPARE(PdfReport::biasUnit(2), QString::fromUtf8("мг"));
    QCOMPARE(PdfReport::biasUnit(3), QString::fromUtf8("°/ч"));
    QVERIFY(PdfReport::biasSeries(run, truth, 6).isEmpty());
}

QTEST_MAIN(TestPdfReport)
#include "test_pdf_report.moc"