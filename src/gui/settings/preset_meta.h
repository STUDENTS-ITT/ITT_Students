// preset_meta.h — «паспорт набора» в пресете подобранных коэффициентов.
//
// Тюнер пишет в пресет секцию [meta] с идентификатором набора:
//   [meta]
//   dataset   = square_650
//   data_dir  = C:/.../dataset_shum/square_650
//   error_ref = truth.txt
// Ядро (loadFilterSettings) читает только ключи filter/tilt/run, поэтому
// секция [meta] на математику не влияет. GUI использует её, чтобы не
// применить пресет, подобранный на другом наборе, без предупреждения.
//
// Зависимостей от Qt-виджетов нет — только QtCore (QString/QDir).

#pragma once

#include <QString>

namespace gui
{

// Паспорт набора из секции [meta] пресета.
struct PresetMeta
{
    bool present = false;  // в файле есть секция [meta]
    QString dataset;       // имя набора (последний компонент data_dir)
    QString data_dir;      // каталог данных, по которому шёл подбор
    QString error_ref;     // truth.txt / reference.txt
};

// Читает [meta] из INI-пресета. present = false, если секции нет (старый
// пресет) или файл не открылся.
PresetMeta readPresetMeta(const QString &preset_path);

// Сверяет паспорт пресета с текущим каталогом данных. Возвращает true, если
// расхождения нет: паспорта нет, сверять нечего (каталог пуст) или имена
// совпадают. При расхождении возвращает false и (если warning != nullptr)
// заполняет текст предупреждения для интерфейса.
bool presetMatchesDataset(const PresetMeta &meta, const QString &data_dir,
                          QString *warning = nullptr);

} // namespace gui
