// test_pdf_report — Сводный PDF-отчёт по прогону (план 2.9).
//
// Проверяется, что отчёт действительно собирается в PDF-файл, содержит
// ожидаемые страницы/байты и что текстовая сводка считается по данным.
// Отчёт собирается напрямую из исходников Qt-слоя, как test_plots.

#include <QtTest>

#include <QTemporaryDir>

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
    void emptyRunIsRejected();
    void buildsPdfFile();
    void buildsPdfWithTruth();
    void blockAxisLabelsAreRestored();
    void plotsDoNotChangeCanvasState();
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

QTEST_MAIN(TestPdfReport)
#include "test_pdf_report.moc"