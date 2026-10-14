// animation_exporter.h — Рендер кадров анимации графика и их сохранение.
//
// Кадры рисуются вне кэша виджета напрямую в QImage (PlotView::renderTo),
// поэтому не портят состояние экрана и пригодны для тестирования в offscreen.

#pragma once

#include <QImage>
#include <QString>
#include <QVector>

#include <functional>

namespace gui
{

class PlotView;

class AnimationExporter
{
public:
    // Отрисовка frames кадров размера width×height. Перед каждым кадром
    // вызывается setup(frame, total) — в нём устанавливаются диапазон или
    // данные графика. Если progress(frame, total) вернула false, рендер
    // прерывается, и возвращается часть кадров.
    static QVector<QImage> renderFrames(
        PlotView *plot, int width, int height, int frames,
        const std::function<void(int frame, int total)> &setup = nullptr,
        const std::function<bool(int frame, int total)> &progress = nullptr,
        QString *error = nullptr);

    // Сохранить кадры как GIF-анимацию (собственный энкодер, не QImageWriter).
    static bool writeGif(const QVector<QImage> &frames, const QString &fileName,
                         int fps, QString *error = nullptr);

    // Сохранить кадры PNG-последовательностью frame0001.png, frame0002.png, …
    // в указанный каталог (создаётся при необходимости).
    static bool writePngSequence(const QVector<QImage> &frames,
                                 const QString &dir, const QString &baseName,
                                 QString *error = nullptr);
};

} // namespace gui