// gif_writer.h — Пишущий GIF89a-энкодер без внешних зависимостей.
//
// Qt умеет читать GIF, но не писать (QImageWriter формат не поддерживает),
// поэтому кадры анимации упаковываются здесь: палитра цвета строится
// методом медианного разреза, индексы сжимаются LZW по спецификации GIF89a.

#pragma once

#include <QImage>
#include <QSize>
#include <QString>
#include <QVector>

namespace gui
{

// Набор кадров одного размера. Кадры с прозрачностью кодируются через
// индекс прозрачного цвета, остальные — через палитру до 256 цветов.
class GifWriter
{
public:
    GifWriter() = default;

    // Добавить кадр; delay_cs — задержка перед следующим кадром в сотых
    // долях секунды (100 = 1 с), диапазон 1..65535.
    void addFrame(const QImage &frame, int delay_cs = 4);

    // Записать накопленные кадры в файл. Возвращает false и текст ошибки
    // в *error, если кадров нет или их размеры различаются.
    bool write(const QString &fileName, QString *error = nullptr) const;

    // Число добавленных кадров и размер первого из них.
    int frameCount() const { return frames_.size(); }
    QSize frameSize() const { return frames_.isEmpty() ? QSize() : frames_.front().image.size(); }

private:
    struct Frame
    {
        QImage image;
        int delay_cs = 4;
    };

    QVector<Frame> frames_;
};

} // namespace gui