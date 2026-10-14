// gif_writer.cpp — GIF89a-энкодер: палитра (медианный разрез) + LZW.
//
// Формат GIF89a: логический экран, NETSCAPE2.0-цикл, затем на каждый кадр
// расширение графического управления (GCE), дескриптор изображения с
// локальной палитрой и LZW-упакованные индексы. Все кадры одного размера.

#include "export/gif_writer.h"

#include <QColor>
#include <QFile>
#include <QHash>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace gui
{
namespace
{

struct ColorU8
{
    unsigned char r, g, b;
};

// Сколько бит нужно, чтобы закодировать индекс палитры (не меньше 2 —
// требование GIF для минимального размера кода).
int minLzwCodeSize(int paletteSize)
{
    int bits = 2;
    while ((1 << bits) < paletteSize) bits++;
    return std::max(bits, 2);
}

// Медианный разрез: сводит гистограмму цветов к палитре из maxColors.
void quantize(const QHash<quint32, int> &hist, int maxColors, QVector<ColorU8> *palette)
{
    struct Entry
    {
        ColorU8 c;
        int count = 0;
    };
    struct Box
    {
        std::vector<Entry> entries;
    };

    std::vector<Box> boxes(1);
    boxes.front().entries.reserve(hist.size());
    for (auto it = hist.cbegin(); it != hist.cend(); ++it)
    {
        const quint32 key = it.key();
        Entry e;
        e.c.r = static_cast<unsigned char>((key >> 16) & 0xFF);
        e.c.g = static_cast<unsigned char>((key >> 8) & 0xFF);
        e.c.b = static_cast<unsigned char>(key & 0xFF);
        e.count = it.value();
        boxes.front().entries.push_back(e);
    }
    if (boxes.front().entries.empty()) return;

    const auto axisRange = [](const Box &b, int axis) -> int {
        int lo = 255, hi = 0;
        for (const Entry &e : b.entries)
        {
            const int v = (axis == 0) ? e.c.r : (axis == 1) ? e.c.g : e.c.b;
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
        return hi - lo;
    };
    const auto volume = [&](const Box &b) -> long long {
        if (b.entries.size() < 2) return 0;
        return static_cast<long long>(b.entries.size()) *
               static_cast<long long>(axisRange(b, 0)) * axisRange(b, 1) * axisRange(b, 2);
    };

    while (static_cast<int>(boxes.size()) < maxColors)
    {
        // Самый «объёмный» бокс, который ещё можно разрезать.
        int bi = -1;
        long long best = 0;
        for (int i = 0; i < static_cast<int>(boxes.size()); i++)
        {
            if (boxes[i].entries.size() < 2) continue;
            const long long v = volume(boxes[i]);
            if (v > best)
            {
                best = v;
                bi = i;
            }
        }
        if (bi < 0) break;

        // Ось с наибольшим диапазоном.
        int axis = 0;
        const int r0 = axisRange(boxes[bi], 0);
        const int r1 = axisRange(boxes[bi], 1);
        const int r2 = axisRange(boxes[bi], 2);
        if (r1 >= r0 && r1 >= r2) axis = 1;
        else if (r2 >= r0 && r2 >= r1) axis = 2;

        // Разрез по счету: первая половина накопленной частоты.
        std::vector<Entry> &es = boxes[bi].entries;
        std::stable_sort(es.begin(), es.end(), [axis](const Entry &a, const Entry &b) {
            const int va = (axis == 0) ? a.c.r : (axis == 1) ? a.c.g : a.c.b;
            const int vb = (axis == 0) ? b.c.r : (axis == 1) ? b.c.g : b.c.b;
            return va < vb;
        });

        long long total = 0;
        for (const Entry &e : es) total += e.count;
        long long acc = 0;
        std::size_t split = 1;
        for (std::size_t i = 0; i < es.size(); i++)
        {
            acc += es[i].count;
            if (acc * 2 >= total)
            {
                split = i + 1;
                break;
            }
        }
        if (split <= 0 || split >= es.size()) break;

        Box second;
        second.entries.assign(es.begin() + static_cast<std::ptrdiff_t>(split), es.end());
        es.resize(split);
        boxes.push_back(std::move(second));
    }

    palette->clear();
    palette->reserve(static_cast<int>(boxes.size()));
    for (const Box &b : boxes)
    {
        long long r = 0, g = 0, bb = 0, cnt = 0;
        for (const Entry &e : b.entries)
        {
            r += static_cast<long long>(e.c.r) * e.count;
            g += static_cast<long long>(e.c.g) * e.count;
            bb += static_cast<long long>(e.c.b) * e.count;
            cnt += e.count;
        }
        if (cnt == 0) continue;
        palette->push_back(ColorU8{ static_cast<unsigned char>((r + cnt / 2) / cnt),
                                    static_cast<unsigned char>((g + cnt / 2) / cnt),
                                    static_cast<unsigned char>((bb + cnt / 2) / cnt) });
    }
}

// LZW-упаковка индексов. Таблица префикс/суффикс на 4096 входов — как в GIF.
QByteArray lzwEncode(const QByteArray &indices, int minCodeSize)
{
    QByteArray out;
    if (indices.isEmpty()) return out;

    const int clearCode = 1 << minCodeSize;
    const int endCode = clearCode + 1;
    int codeSize = minCodeSize + 1;
    int nextCode = endCode + 1;

    // Прямая таблица (префикс << 8 | суффикс) → код. Память ~2 МБ.
    std::vector<std::uint16_t> table(std::size_t(4096) * 256, 0xFFFF);
    const auto getCode = [&table](int prefix, int suffix) -> std::uint16_t {
        return table[static_cast<std::size_t>(prefix) * 256 + suffix];
    };
    const auto setCode = [&table](int prefix, int suffix, int code) {
        table[static_cast<std::size_t>(prefix) * 256 + suffix] = static_cast<std::uint16_t>(code);
    };

    // Битовый поток, LSB first.
    quint32 bitBuf = 0;
    int bitCnt = 0;
    const auto putCode = [&](int code) {
        bitBuf |= static_cast<quint32>(code) << bitCnt;
        bitCnt += codeSize;
        while (bitCnt >= 8)
        {
            out.append(static_cast<char>(bitBuf & 0xFF));
            bitBuf >>= 8;
            bitCnt -= 8;
        }
    };

    putCode(clearCode);

    int prefix = static_cast<unsigned char>(indices[0]);
    for (int i = 1; i < indices.size(); i++)
    {
        const int suffix = static_cast<unsigned char>(indices[i]);
        const int code = getCode(prefix, suffix);
        if (code != 0xFFFF)
        {
            prefix = code;
        }
        else
        {
            putCode(prefix);
            if (nextCode < 4096)
            {
                setCode(prefix, suffix, nextCode);
                nextCode++;
                // Декодер расширяет ширину на одну запись позже энкодера
                // (первая запись словаря строится им со второго считанного
                // кода после clear), поэтому момент роста сдвинут на +1.
                if (nextCode == (1 << codeSize) + 1 && codeSize < 12) codeSize++;
            }
            else
            {
                // Таблица полная — сброс по clear-коду. Записи старого словаря
                // обязательно удаляются, иначе getCode вернёт устаревший код,
                // который декодер построит лишь позже.
                putCode(clearCode);
                std::fill(table.begin(), table.end(), 0xFFFF);
                nextCode = endCode + 1;
                codeSize = minCodeSize + 1;
            }
            prefix = suffix;
        }
    }
    putCode(prefix);
    putCode(endCode);
    if (bitCnt > 0) out.append(static_cast<char>(bitBuf & 0xFF));
    return out;
}

// Кодирование одного кадра в палитру с индексами и LZW-поток.
struct EncodedFrame
{
    int delay_cs = 0;
    QVector<ColorU8> palette;
    QByteArray lzw;
    int transparent_index = -1;
};

EncodedFrame encodeFrame(const QImage &frame, int delayCs)
{
    EncodedFrame out;
    out.delay_cs = delayCs;

    const bool has_alpha = frame.hasAlphaChannel();
    QImage img = frame.convertToFormat(QImage::Format_RGB32);
    const int w = img.width();
    const int h = img.height();

    // Гистограмма непрозрачных цветов (прозрачные — зарезервированный индекс).
    QHash<quint32, int> hist;
    for (int y = 0; y < h; y++)
    {
        const QRgb *line = reinterpret_cast<const QRgb *>(img.constScanLine(y));
        for (int x = 0; x < w; x++)
        {
            if (has_alpha && qAlpha(line[x]) < 128) continue;
            const quint32 key = (static_cast<quint32>(qRed(line[x])) << 16) |
                                (static_cast<quint32>(qGreen(line[x])) << 8) |
                                static_cast<quint32>(qBlue(line[x]));
            hist[key] = hist.value(key) + 1;
        }
    }

    // Для прозрачности нужен лишний индекс, поэтому палитра — до 255 цветов.
    const int maxColors = has_alpha ? 255 : 256;
    if (hist.size() <= static_cast<quint32>(maxColors))
    {
        for (auto it = hist.cbegin(); it != hist.cend(); ++it)
        {
            const quint32 key = it.key();
            ColorU8 c;
            c.r = static_cast<unsigned char>((key >> 16) & 0xFF);
            c.g = static_cast<unsigned char>((key >> 8) & 0xFF);
            c.b = static_cast<unsigned char>(key & 0xFF);
            out.palette.push_back(c);
        }
    }
    else
    {
        quantize(hist, maxColors, &out.palette);
    }

    // Соответствие «цвет → индекс палитры» (для квантованной палитры —
    // поиск ближайшего).
    QHash<quint32, int> colorIndex;
    for (int i = 0; i < out.palette.size(); i++)
    {
        const quint32 key = (static_cast<quint32>(out.palette[i].r) << 16) |
                            (static_cast<quint32>(out.palette[i].g) << 8) |
                            static_cast<quint32>(out.palette[i].b);
        colorIndex[key] = i;
    }
    const auto toIndex = [&](quint32 key) -> int {
        const auto it = colorIndex.find(key);
        if (it != colorIndex.end()) return it.value();
        int best = 0;
        long long bestDist = -1;
        for (int i = 0; i < out.palette.size(); i++)
        {
            const int dr = static_cast<int>(out.palette[i].r) - static_cast<int>((key >> 16) & 0xFF);
            const int dg = static_cast<int>(out.palette[i].g) - static_cast<int>((key >> 8) & 0xFF);
            const int db = static_cast<int>(out.palette[i].b) - static_cast<int>(key & 0xFF);
            const long long d = static_cast<long long>(dr) * dr + dg * dg + db * db;
            if (bestDist < 0 || d < bestDist)
            {
                bestDist = d;
                best = i;
            }
        }
        colorIndex[key] = best;
        return best;
    };

    if (has_alpha) out.transparent_index = out.palette.size();

    QByteArray indices;
    indices.reserve(w * h);
    for (int y = 0; y < h; y++)
    {
        const QRgb *line = reinterpret_cast<const QRgb *>(img.constScanLine(y));
        for (int x = 0; x < w; x++)
        {
            if (has_alpha && qAlpha(line[x]) < 128)
            {
                indices.append(static_cast<char>(out.transparent_index));
            }
            else
            {
                const quint32 key = (static_cast<quint32>(qRed(line[x])) << 16) |
                                    (static_cast<quint32>(qGreen(line[x])) << 8) |
                                    static_cast<quint32>(qBlue(line[x]));
                indices.append(static_cast<char>(toIndex(key)));
            }
        }
    }

    out.lzw = lzwEncode(indices, minLzwCodeSize(out.palette.size()));
    return out;
}

void appendSubBlocks(QByteArray *dst, const QByteArray &data)
{
    const int total = static_cast<int>(data.size());
    int pos = 0;
    while (pos < total)
    {
        const int n = std::min(255, total - pos);
        dst->append(static_cast<char>(n));
        dst->append(data.mid(pos, n));
        pos += n;
    }
    dst->append('\0');
}

void put16(QByteArray *dst, int value)
{
    dst->append(static_cast<char>(value & 0xFF));
    dst->append(static_cast<char>((value >> 8) & 0xFF));
}

} // namespace

void GifWriter::addFrame(const QImage &frame, int delay_cs)
{
    Frame f;
    f.image = frame;
    f.delay_cs = qBound(1, delay_cs, 65535);
    frames_.push_back(f);
}

bool GifWriter::write(const QString &fileName, QString *error) const
{
    const auto fail = [error](const QString &message) {
        if (error != nullptr) *error = message;
        return false;
    };

    if (frames_.isEmpty()) return fail(QObject::tr("Нет кадров для анимации"));
    const QSize size = frames_.front().image.size();
    if (size.width() <= 0 || size.height() <= 0)
        return fail(QObject::tr("Некорректный размер кадра"));
    for (const Frame &f : frames_)
    {
        if (f.image.size() != size)
            return fail(QObject::tr("Кадры анимации имеют разный размер"));
    }

    QByteArray data;
    data.reserve(static_cast<int>(frames_.size()) * size.width() * size.height() / 4 + 1024);

    // Заголовок и логический экран.
    data.append("GIF89a", 6);
    put16(&data, size.width());
    put16(&data, size.height());
    data.append(char(0));  // без глобальной палитры
    data.append(char(0));  // фоновый цвет
    data.append(char(0));  // аспект

    // Бесконечный цикл (NETSCAPE2.0).
    data.append(char(0x21));
    data.append(char(0xFF));
    data.append(char(0x0B));
    data.append("NETSCAPE2.0", 11);
    data.append(char(0x03));
    data.append(char(0x01));
    put16(&data, 0);
    data.append(char(0x00));

    for (const Frame &f : frames_)
    {
        const EncodedFrame ef = encodeFrame(f.image, f.delay_cs);

        // GCE: задержка и возможный прозрачный индекс.
        data.append(char(0x21));
        data.append(char(0xF9));
        data.append(char(0x04));
        data.append(char(ef.transparent_index >= 0 ? 0x01 : 0x00));
        put16(&data, ef.delay_cs);
        data.append(char(ef.transparent_index >= 0 ? ef.transparent_index : 0));
        data.append(char(0x00));

        // Дескриптор изображения с локальной палитрой.
        data.append(char(0x2C));
        put16(&data, 0);
        put16(&data, 0);
        put16(&data, size.width());
        put16(&data, size.height());
        int tableBits = 0;
        while ((1 << tableBits) < ef.palette.size()) tableBits++;
        tableBits = std::max(tableBits, 1);
        const int tableSize = 1 << tableBits;
        data.append(char(0x80 | (7 << 4) | ((tableBits - 1) & 0x07)));
        for (const ColorU8 &c : ef.palette)
        {
            data.append(static_cast<char>(c.r));
            data.append(static_cast<char>(c.g));
            data.append(static_cast<char>(c.b));
        }
        for (int i = ef.palette.size(); i < tableSize; i++)
        {
            data.append(char(0));
            data.append(char(0));
            data.append(char(0));
        }

        data.append(char(minLzwCodeSize(tableSize)));
        appendSubBlocks(&data, ef.lzw);
    }

    data.append(char(0x3B));  // конец GIF-потока

    QFile f(fileName);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return fail(QObject::tr("Не удалось открыть файл: %1").arg(fileName));
    if (f.write(data) != data.size())
        return fail(QObject::tr("Не удалось записать файл: %1").arg(fileName));
    if (!f.flush())
        return fail(QObject::tr("Не удалось сохранить файл: %1").arg(fileName));
    return true;
}

} // namespace gui