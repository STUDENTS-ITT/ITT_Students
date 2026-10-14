// preset_meta.cpp — разбор секции [meta] пресета и сверка имени набора.
//
// Разбор намеренно простой: INI с одной интересующей секцией, ключи
// перечислены явно. Полноценный парсер ядра (utils::IniFile) здесь не нужен и
// потянул бы зависимость Qt-слоя от математики.

#include "settings/preset_meta.h"

#include <QDir>
#include <QFile>
#include <QObject>
#include <QStringList>

namespace gui
{

namespace
{
// Имя набора — последний компонент пути (как TuningReport::datasetFor и
// AppSettings::defaultReportsDir).
QString datasetName(const QString &dir)
{
    const QString t = dir.trimmed();
    if (t.isEmpty()) return QString();
    return QDir(t).dirName();
}
} // namespace

PresetMeta readPresetMeta(const QString &preset_path)
{
    PresetMeta meta;

    QFile f(preset_path);
    if (!f.open(QIODevice::ReadOnly)) return meta;

    const QStringList lines = QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'));

    bool in_meta = false;
    for (const QString &raw : lines)
    {
        const QString line = raw.trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#')) ||
            line.startsWith(QLatin1Char(';')))
        {
            continue;
        }

        if (line.startsWith(QLatin1Char('[')))
        {
            in_meta = (line.compare(QStringLiteral("[meta]"), Qt::CaseInsensitive) == 0);
            if (in_meta) meta.present = true;
            continue;
        }

        if (!in_meta) continue;

        const int eq = line.indexOf(QLatin1Char('='));
        if (eq <= 0) continue;

        const QString key = line.left(eq).trimmed().toLower();
        const QString val = line.mid(eq + 1).trimmed();
        if (key == QLatin1String("dataset"))
            meta.dataset = val;
        else if (key == QLatin1String("data_dir"))
            meta.data_dir = val;
        else if (key == QLatin1String("error_ref"))
            meta.error_ref = val;
    }

    return meta;
}

bool presetMatchesDataset(const PresetMeta &meta, const QString &data_dir,
                          QString *warning)
{
    if (warning != nullptr) warning->clear();

    const QString current = datasetName(data_dir);
    const QString tuned = meta.dataset.trimmed();

    // Сверять нечего: у старого пресета паспорта нет, либо каталог не задан.
    // Это не ошибка — не мешаем применению.
    if (!meta.present || tuned.isEmpty() || current.isEmpty()) return true;

    if (tuned == current) return true;

    if (warning != nullptr)
    {
        *warning = QObject::tr("Пресет подобран на наборе «%1», а выбран «%2».")
                       .arg(tuned, current);
    }
    return false;
}

} // namespace gui
