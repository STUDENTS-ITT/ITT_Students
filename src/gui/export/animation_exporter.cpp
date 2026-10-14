// animation_exporter.cpp — Рендер кадров и их запись в GIF / PNG-файлы.

#include "export/animation_exporter.h"

#include "export/gif_writer.h"
#include "views/plots/plot_view.h"

#include <QDir>
#include <QPainter>

#include <algorithm>

namespace gui
{
namespace
{

// Задержка кадра в сотых долях секунды для заданного FPS.
int delayCsForFps(int fps)
{
    if (fps <= 0) return 4;
    return std::max(1, (100 + fps / 2) / fps);
}

} // namespace

QVector<QImage> AnimationExporter::renderFrames(
    PlotView *plot, int width, int height, int frames,
    const std::function<void(int, int)> &setup,
    const std::function<bool(int, int)> &progress, QString *error)
{
    QVector<QImage> out;
    if (plot == nullptr || width <= 0 || height <= 0 || frames <= 0)
    {
        if (error != nullptr) *error = QObject::tr("Некорректные параметры рендера анимации");
        return out;
    }

    out.reserve(frames);
    // Курсор не должен попадать в экспортируемые кадры.
    const bool prevCursor = plot->exportCursorEnabled();
    plot->setExportCursorEnabled(false);

    for (int i = 0; i < frames; i++)
    {
        if (progress != nullptr && !progress(i, frames)) break;
        if (setup != nullptr) setup(i, frames);

        QImage img(width, height, QImage::Format_RGB32);
        QPainter p(&img);
        plot->renderTo(p, QSize(width, height));
        p.end();
        out.push_back(std::move(img));
    }

    plot->setExportCursorEnabled(prevCursor);
    return out;
}

bool AnimationExporter::writeGif(const QVector<QImage> &frames, const QString &fileName,
                                 int fps, QString *error)
{
    GifWriter writer;
    const int delayCs = delayCsForFps(fps);
    for (const QImage &f : frames) writer.addFrame(f, delayCs);
    return writer.write(fileName, error);
}

bool AnimationExporter::writePngSequence(const QVector<QImage> &frames,
                                         const QString &dir, const QString &baseName,
                                         QString *error)
{
    if (frames.isEmpty())
    {
        if (error != nullptr) *error = QObject::tr("Нет кадров для сохранения");
        return false;
    }

    QDir d(dir);
    if (!d.exists() && !d.mkpath(QStringLiteral(".")))
    {
        if (error != nullptr) *error = QObject::tr("Не удалось создать каталог: %1").arg(dir);
        return false;
    }

    const QString base = baseName.isEmpty() ? QStringLiteral("frame") : baseName;
    const int digits = std::max(4, static_cast<int>(QString::number(frames.size()).size()));
    int number = 1;
    for (const QImage &f : frames)
    {
        const QString name = QStringLiteral("%1%2.png")
                                 .arg(base)
                                 .arg(number++, digits, 10, QLatin1Char('0'));
        if (!f.save(d.filePath(name), "PNG"))
        {
            if (error != nullptr)
                *error = QObject::tr("Не удалось сохранить кадр: %1").arg(d.filePath(name));
            return false;
        }
    }
    return true;
}

} // namespace gui