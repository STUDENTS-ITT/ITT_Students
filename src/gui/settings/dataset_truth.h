// dataset_truth.h — проверка файла наземной истины (truth.txt) набора
// данных. Используется вкладкой «Подбор», чтобы не считать ошибку по
// повреждённому файлу: тюнер умеет сравнивать с истиной (--ref-truth), но
// только для синтетических наборов, где truth.txt корректен.
//
// Зависимостей от Qt-виджетов нет — только QtCore (QString/QDir).

#pragma once

#include <QString>
#include <QVector>

namespace gui
{

// Состояние truth.txt в каталоге данных.
enum class TruthState
{
    Ok,       // файл есть и разбирается (заголовок + по 10 числовых полей)
    Missing,  // файла нет (набор без наземной истины)
    Corrupt   // файл есть, но формат повреждён
};

struct TruthCheck
{
    TruthState state = TruthState::Missing;
    int rows = 0;      // число корректных строк данных
    int bad_line = 0;  // 1-based номер первой проблемной строки (0 — нет)
    QString detail;    // краткое пояснение для интерфейса
};

// Одна строка траектории истины (формат reference.txt/result.txt):
// time, lon, lat, alt, heading, pitch, roll, vn, vh, ve.
struct TruthSample
{
    double v[10] = {0};
};

// Проверяет <data_dir>/truth.txt: наличие, строку заголовка, ровно 10
// числовых полей в каждой строке данных (формат reference.txt).
TruthCheck checkDatasetTruth(const QString &data_dir);

// Читает <data_dir>/truth.txt после проверки. rows — строки данных,
// check — результат проверки (может быть nullptr). true — файл корректен
// и строки прочитаны; при ошибке rows пуст.
bool loadDatasetTruth(const QString &data_dir, QVector<TruthSample> *rows,
                      TruthCheck *check = nullptr);

// Текст для интерфейса: «истина: truth.txt — ок (N строк)» и т. п.
QString truthStatusText(const TruthCheck &check);

} // namespace gui
