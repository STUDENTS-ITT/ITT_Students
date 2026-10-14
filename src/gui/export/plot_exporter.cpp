// plot_exporter.cpp — Экспорт графиков и данных.

#include "export/plot_exporter.h"

#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>

#include "models/nav_table_model.h"
#include "views/plots/plot_view.h"

namespace gui
{
namespace
{
QSize exportSize(PlotView *plot, int width, int height)
{
    if (width > 0 && height > 0) return QSize(width, height);
    const QSize current = plot->size();
    if (current.width() >= 200 && current.height() >= 150) return current;
    return QSize(1280, 800);
}

bool ensureParentDir(const QString &fileName)
{
    const QDir dir = QFileInfo(fileName).absoluteDir();
    if (dir.exists()) return true;
    return QDir().mkpath(dir.absolutePath());
}
} // namespace

bool PlotExporter::exportPlotImage(PlotView *plot, const QString &fileName,
                                   int width, int height, QString *error)
{
    if (plot == nullptr)
    {
        if (error) *error = QObject::tr("График не выбран");
        return false;
    }
    if (fileName.isEmpty())
    {
        if (error) *error = QObject::tr("Файл не указан");
        return false;
    }
    if (!ensureParentDir(fileName))
    {
        if (error) *error = QObject::tr("Не удалось создать каталог для %1").arg(fileName);
        return false;
    }

    if (!plot->saveImage(fileName, exportSize(plot, width, height)))
    {
        if (error) *error = QObject::tr("Не удалось сохранить %1").arg(fileName);
        return false;
    }
    return true;
}

bool PlotExporter::exportPlotPdf(PlotView *plot, const QString &fileName,
                                 int width, int height, QString *error)
{
    if (plot == nullptr)
    {
        if (error) *error = QObject::tr("График не выбран");
        return false;
    }
    if (fileName.isEmpty())
    {
        if (error) *error = QObject::tr("Файл не указан");
        return false;
    }
    if (!ensureParentDir(fileName))
    {
        if (error) *error = QObject::tr("Не удалось создать каталог для %1").arg(fileName);
        return false;
    }

    if (!plot->savePdf(fileName, exportSize(plot, width, height)))
    {
        if (error) *error = QObject::tr("Не удалось сохранить %1").arg(fileName);
        return false;
    }
    return true;
}

bool PlotExporter::exportPlotSvg(PlotView *plot, const QString &fileName,
                                 int width, int height, QString *error)
{
    if (plot == nullptr)
    {
        if (error) *error = QObject::tr("График не выбран");
        return false;
    }
    if (fileName.isEmpty())
    {
        if (error) *error = QObject::tr("Файл не указан");
        return false;
    }
    if (!ensureParentDir(fileName))
    {
        if (error) *error = QObject::tr("Не удалось создать каталог для %1").arg(fileName);
        return false;
    }

    if (!plot->saveSvg(fileName, exportSize(plot, width, height)))
    {
        if (error) *error = QObject::tr("Не удалось сохранить %1").arg(fileName);
        return false;
    }
    return true;
}

bool PlotExporter::exportTableCsv(NavTableModel *model, const QString &fileName,
                                  QString *error)
{
    if (model == nullptr)
    {
        if (error) *error = QObject::tr("Нет данных");
        return false;
    }
    if (model->rowCount() == 0)
    {
        if (error) *error = QObject::tr("Нет данных для экспорта");
        return false;
    }

    QSaveFile f(fileName);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error) *error = QObject::tr("Не удалось открыть %1 для записи").arg(fileName);
        return false;
    }

    f.write(model->toCsv().toUtf8());
    return f.commit();
}

bool PlotExporter::exportSeriesCsv(const RunSeries &series, const QString &fileName,
                                   QString *error)
{
    if (series.rows.empty())
    {
        if (error) *error = QObject::tr("Нет данных для экспорта");
        return false;
    }

    QSaveFile f(fileName);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error) *error = QObject::tr("Не удалось открыть %1 для записи").arg(fileName);
        return false;
    }

    QTextStream s(&f);
    s << "t_s,lon_deg,lat_deg,alt_m,heading_deg,pitch_deg,roll_deg,vn,vh,ve,"
      << "ref_lon_deg,ref_lat_deg,ref_alt_m,ref_heading_deg\n";

    for (const RunSeries::Row &r : series.rows)
    {
        s << r.time;
        for (int i = RunSeries::Lon; i <= RunSeries::Ve; i++)
        {
            s << ',' << r.result[i];
        }
        if (r.has_reference)
        {
            s << ',' << r.reference[RunSeries::Lon] << ',' << r.reference[RunSeries::Lat]
              << ',' << r.reference[RunSeries::Alt] << ',' << r.reference[RunSeries::Heading];
        }
        s << '\n';
    }

    s.flush();
    return f.commit();
}

namespace
{
// Недопустимые в имени файла Windows/UNIX символы и ведущие/троечные точки —
// заменяются подчёркиванием, чтобы имена вкладок нельзя было превратить в
// путь. Управляющие символы вырезаются.
QString sanitizeFileName(const QString &name)
{
    QString out;
    out.reserve(name.size());
    for (const QChar &c : name)
    {
        if (c.category() == QChar::Other_Control) continue;
        if (c == QLatin1Char('\\') || c == QLatin1Char('/') || c == QLatin1Char(':') ||
            c == QLatin1Char('*') || c == QLatin1Char('?') || c == QLatin1Char('"') ||
            c == QLatin1Char('<') || c == QLatin1Char('>') || c == QLatin1Char('|'))
        {
            out += QLatin1Char('_');
        }
        else
        {
            out += c;
        }
    }
    while (out.endsWith(QLatin1Char('.')) || out.endsWith(QLatin1Char(' '))) out.chop(1);
    if (out.isEmpty()) out = QLatin1String("plot");
    return out;
}

QString extensionForFormat(const QString &format)
{
    const QString f = format.toLower();
    if (f == QLatin1String("jpg") || f == QLatin1String("jpeg")) return QStringLiteral("jpg");
    if (f == QLatin1String("bmp")) return QStringLiteral("bmp");
    if (f == QLatin1String("svg")) return QStringLiteral("svg");
    if (f == QLatin1String("pdf")) return QStringLiteral("pdf");
    return QStringLiteral("png");
}
} // namespace

bool PlotExporter::exportAllPlots(const QList<PlotExportItem> &items,
                                  const QString &dir, const QString &format,
                                  int width, int height,
                                  const std::function<bool(int, int)> &progress,
                                  int *exported, QString *error)
{
    if (items.isEmpty())
    {
        if (error) *error = QObject::tr("Нет графиков для экспорта");
        return false;
    }
    if (dir.isEmpty())
    {
        if (error) *error = QObject::tr("Каталог не указан");
        return false;
    }

    const QDir out_dir(dir);
    if (!out_dir.exists() && !QDir().mkpath(out_dir.absolutePath()))
    {
        if (error) *error = QObject::tr("Не удалось создать каталог %1").arg(dir);
        return false;
    }

    const QString ext = extensionForFormat(format);
    int done = 0;
    for (const PlotExportItem &item : items)
    {
        // Прервать по желанию пользователя — не ошибка.
        if (progress && !progress(done, items.size())) break;
        if (item.plot == nullptr) continue;

        const QString fileName =
            out_dir.filePath(sanitizeFileName(item.name) + QLatin1Char('.') + ext);
        const bool ok =
            ext == QLatin1String("svg")   ? PlotExporter::exportPlotSvg(item.plot, fileName, width, height)
            : ext == QLatin1String("pdf") ? PlotExporter::exportPlotPdf(item.plot, fileName, width, height)
                                          : PlotExporter::exportPlotImage(item.plot, fileName, width, height);
        if (!ok)
        {
            if (error) *error = QObject::tr("Не удалось сохранить %1").arg(fileName);
            if (exported) *exported = done;
            return false;
        }
        ++done;
    }

    if (exported) *exported = done;
    return true;
}

} // namespace gui