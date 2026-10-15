// tuning_report.h — Отчёт о подборе коэффициентов фильтра в PDF.
//
// Отчёт собирает само приложение из артефактов тюнера tools/kalman_tuner:
// конфига tuner_config*.json, best_params.txt и eval.json рабочего каталога.
// Сам подбор выполняет tune_kalman.py; здесь — только сборка PDF, поэтому
// математика в core/ не затрагивается.

#pragma once

#include <QString>

namespace gui
{

class TuningReport
{
public:
    // Собрать отчёт по конфигу подбора в файл pdf_path.
    // config_path — путь к tuner_config*.json; pdf_path — куда писать.
    // При ошибке возвращает false и заполняет error (если он передан).
    static bool build(const QString &config_path, const QString &pdf_path,
                      QString *error = nullptr);

    // Рабочий каталог подбора из конфига: относительный work_root разрешается
    // от каталога тюнера (ближайший предок конфига с tune_kalman.py) — так же,
    // как его трактует tune_kalman.py. Если каталог тюнера не найден, — от
    // каталога конфига. Пусто, если конфиг не читается.
    static QString workRootFor(const QString &config_path);

    // Имя набора (последний компонент data_dir из конфига) — для имени файла
    // отчёта и подпапки reports/<набор>/. Пусто, если конфиг не читается.
    static QString datasetFor(const QString &config_path);
};

} // namespace gui
