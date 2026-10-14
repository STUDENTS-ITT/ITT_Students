// dataset_truth.cpp — разбор и проверка <data_dir>/truth.txt.
//
// Формат совпадает с reference.txt: строка-заголовок (time lon lat alt
// heading pitch roll vn vh ve) и строки из 10 чисел. Повреждённый файл
// (как у набора square_noisefree) ловится по числу/типу полей.

#include "settings/dataset_truth.h"

#include <QDir>
#include <QFile>
#include <QObject>
#include <QRegularExpression>
#include <QStringList>

namespace gui
{

namespace
{
// Число колонок в truth.txt/reference.txt.
constexpr int kTruthColumns = 10;

// Общий разбор файла истины. rows заполняется только при collect == true.
TruthCheck parseTruth(const QString &data_dir, QVector<TruthSample> *rows, bool collect)
{
    TruthCheck res;

    const QString dir = data_dir.trimmed();
    if (dir.isEmpty())
    {
        res.state = TruthState::Missing;
        res.detail = QObject::tr("каталог данных не задан");
        return res;
    }

    const QString path = QDir(dir).filePath(QStringLiteral("truth.txt"));
    QFile f(path);
    if (!f.exists() || !f.open(QIODevice::ReadOnly))
    {
        res.state = TruthState::Missing;
        res.detail = QObject::tr("файл не найден");
        return res;
    }

    const QStringList lines = QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'));
    const QRegularExpression sep(QStringLiteral("\\s+"));

    int rows_count = 0;
    bool first_line = true;
    for (int i = 0; i < lines.size(); ++i)
    {
        const QString line = lines[i].trimmed();
        if (line.isEmpty()) continue;

        const QStringList tok = line.split(sep, Qt::SkipEmptyParts);

        // Первую непустую строку с «time» считаем заголовком и пропускаем.
        if (first_line)
        {
            first_line = false;
            if (!tok.isEmpty() && tok[0].compare(QStringLiteral("time"), Qt::CaseInsensitive) == 0)
                continue;
        }

        if (tok.size() != kTruthColumns)
        {
            res.state = TruthState::Corrupt;
            res.bad_line = i + 1;
            res.rows = rows_count;
            res.detail = QObject::tr("строка %1: %2 полей вместо %3")
                             .arg(i + 1)
                             .arg(tok.size())
                             .arg(kTruthColumns);
            return res;
        }

        TruthSample sample;
        for (int k = 0; k < kTruthColumns; ++k)
        {
            bool ok = false;
            sample.v[k] = tok[k].toDouble(&ok);
            if (!ok)
            {
                res.state = TruthState::Corrupt;
                res.bad_line = i + 1;
                res.rows = rows_count;
                res.detail = QObject::tr("строка %1: нечисловое поле «%2»").arg(i + 1).arg(tok[k]);
                return res;
            }
        }
        if (collect && rows != nullptr) rows->append(sample);
        rows_count++;
    }

    if (rows_count == 0)
    {
        res.state = TruthState::Corrupt;
        res.detail = QObject::tr("нет строк данных");
        return res;
    }

    res.state = TruthState::Ok;
    res.rows = rows_count;
    res.detail = QObject::tr("ок, %1 строк").arg(rows_count);
    return res;
}
} // namespace

TruthCheck checkDatasetTruth(const QString &data_dir)
{
    return parseTruth(data_dir, nullptr, false);
}

bool loadDatasetTruth(const QString &data_dir, QVector<TruthSample> *rows, TruthCheck *check)
{
    if (rows != nullptr) rows->clear();

    const TruthCheck res = parseTruth(data_dir, rows, true);
    if (check != nullptr) *check = res;

    if (res.state != TruthState::Ok)
    {
        if (rows != nullptr) rows->clear();
        return false;
    }
    return true;
}

QString truthStatusText(const TruthCheck &check)
{
    switch (check.state)
    {
    case TruthState::Ok:
        return QObject::tr("истина: truth.txt — ок (%1 строк)").arg(check.rows);
    case TruthState::Corrupt:
        return QObject::tr("истина: truth.txt повреждён (%1)").arg(check.detail);
    case TruthState::Missing:
    default:
        return QObject::tr("истина: truth.txt не найден");
    }
}

} // namespace gui
