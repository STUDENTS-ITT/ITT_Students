// tuner_configs.cpp — перечень конфигов подбора в каталоге тюнера.

#include "settings/tuner_configs.h"

#include <QDir>

namespace gui
{

QStringList listTunerConfigs(const QString &tuner_dir)
{
    QStringList out;
    if (tuner_dir.isEmpty()) return out;

    const QDir d(tuner_dir);
    if (!d.exists()) return out;

    // Штатные конфиги наборов: tuner_config*.json в корне тюнера.
    const QStringList files =
        d.entryList({QStringLiteral("tuner_config*.json")}, QDir::Files, QDir::Name);
    for (const QString &name : files)
    {
        // smoke-конфиг служебный (быстрая проверка скрипта) — в списке не нужен.
        if (name.contains(QLatin1String("smoke"), Qt::CaseInsensitive)) continue;
        out << name;
    }

    // Конфиги экспериментов с истиной: truth_cfg*/cfg_*.json в подкаталогах.
    const QStringList subdirs =
        d.entryList({QStringLiteral("truth_cfg*")}, QDir::Dirs, QDir::Name);
    for (const QString &sub : subdirs)
    {
        const QDir sub_dir(d.filePath(sub));
        const QStringList cfgs =
            sub_dir.entryList({QStringLiteral("cfg_*.json")}, QDir::Files, QDir::Name);
        for (const QString &name : cfgs)
            out << (sub + QLatin1Char('/') + name);
    }

    return out;
}

} // namespace gui
