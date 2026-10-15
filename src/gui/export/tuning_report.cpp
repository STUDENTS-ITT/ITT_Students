// tuning_report.cpp — Сборка PDF-отчёта о подборе коэффициентов фильтра.
//
// Источники (всё читает приложение):
//   * tuner_config*.json — конфигурация подбора (σ_ref/u_span, метрика,
//     горизонт, оптимизация, рабочий каталог и путь пресета);
//   * <work_root>/best_params.txt — итог подбора (J_train/J_valid, breakdown,
//     подобранные σ, плато, чувствительность);
//   * <work_root>/<этап>_*/eval.json — по одной оценке на пробу (для графика
//     сходимости J_train/J_valid по шагам).
//
// Формат PDF намеренно совпадает по оформлению с отчётом о прогоне
// (pdf_report.cpp): A4, поля 40 pt, 72 dpi.

#include "export/tuning_report.h"

#include <algorithm>
#include <cmath>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QFontMetricsF>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QMarginsF>
#include <QObject>
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QPdfWriter>
#include <QRegularExpression>

#include "views/plots/time_series_plot.h"

#include "settings/preset_meta.h"

namespace gui
{
namespace
{

constexpr int kPageWidthPt = 595;   // A4
constexpr int kPageHeightPt = 842;
constexpr int kMarginPt = 40;
constexpr double kContentWidth = kPageWidthPt - 2 * kMarginPt;   // 515

// Курсор по странице: следит за свободным местом и переводит на новую.
struct Cursor
{
    QPainter &p;
    QPdfWriter &w;
    Cursor(QPainter &painter, QPdfWriter &writer) : p(painter), w(writer) {}

    double y = kMarginPt;

    void ensure(double height)
    {
        if (y + height > kPageHeightPt - kMarginPt) newPage();
    }
    void advance(double dy) { y += dy; }
    void newPage()
    {
        w.newPage();
        y = kMarginPt;
    }
};

void drawHeading(QPainter &p, Cursor &c, const QString &text, double point_size,
                 double space_before)
{
    c.ensure(point_size * 3);
    c.advance(space_before);

    QFont f = p.font();
    f.setPointSizeF(point_size);
    f.setBold(true);
    p.setFont(f);
    const QFontMetricsF fm(f);

    c.ensure(fm.height() + 4);
    p.setPen(Qt::black);
    p.drawText(QRectF(kMarginPt, c.y, kContentWidth, fm.height()),
               Qt::AlignLeft | Qt::AlignVCenter, text);
    c.advance(fm.height() + 4);
    p.setFont(QFont());
}

// Строка «подпись слева — значение справа».
void drawRow(QPainter &p, Cursor &c, const QString &label, const QString &value,
             double label_width = 240.0)
{
    QFont f = p.font();
    f.setPointSizeF(9.0);
    p.setFont(f);
    const QFontMetricsF fm(f);

    c.ensure(fm.height() + 2);
    p.setPen(Qt::black);
    p.drawText(QRectF(kMarginPt, c.y, label_width, fm.height()),
               Qt::AlignLeft | Qt::AlignVCenter, label);
    p.drawText(QRectF(kMarginPt + label_width, c.y, kContentWidth - label_width, fm.height()),
               Qt::AlignRight | Qt::AlignVCenter, value);
    c.advance(fm.height() + 2);
    p.setFont(QFont());
}

// Таблица в несколько колонок: первая — по левому краю, остальные — по правому.
void drawColumns(QPainter &p, Cursor &c, const QStringList &headers,
                 const QVector<QStringList> &rows, const QVector<double> &widths)
{
    QFont base = p.font();
    base.setPointSizeF(9.0);
    p.setFont(base);
    const QFontMetricsF fm(base);
    const double h = fm.height() + 2;

    auto line = [&](const QStringList &cells, bool bold) {
        c.ensure(h);
        QFont f = base;
        f.setBold(bold);
        p.setFont(f);
        p.setPen(Qt::black);
        double x = kMarginPt;
        for (int i = 0; i < widths.size(); ++i)
        {
            const Qt::Alignment align = (i == 0) ? Qt::AlignLeft : Qt::AlignRight;
            p.drawText(QRectF(x, c.y, widths[i], h), align | Qt::AlignVCenter,
                       i < cells.size() ? cells[i] : QString());
            x += widths[i];
        }
        c.advance(h);
    };

    line(headers, true);
    for (const QStringList &r : rows) line(r, false);
    p.setFont(QFont());
    c.advance(4);
}

// --- источники -----------------------------------------------------------

struct ParamInfo
{
    QString name;
    double sigma_ref = 0.0;
    double u_span = 0.0;
};

struct SigmaRow
{
    QString name;
    double sigma = 0.0;   // σ* (СИ)
    double u = 0.0;       // u* = log10(σ*/σ_ref)
};

struct BpSection
{
    QString title;
    QStringList lines;
};

struct BestParams
{
    bool found = false;
    QStringList header;          // строки до первого «---»
    QVector<BpSection> sections;
};

BestParams parseBestParams(const QString &text)
{
    BestParams bp;
    bp.found = !text.trimmed().isEmpty();
    if (!bp.found) return bp;

    BpSection current;
    bool in_section = false;
    const QStringList lines = text.split(QLatin1Char('\n'));
    for (const QString &raw : lines)
    {
        if (raw.startsWith(QLatin1String("---")))
        {
            if (in_section) bp.sections.append(current);
            current = BpSection();
            QString t = raw;
            t.remove(QRegularExpression(QStringLiteral("^-+\\s*")));
            t.remove(QRegularExpression(QStringLiteral("\\s*-+$")));
            current.title = t.trimmed();
            in_section = true;
        }
        else if (in_section)
        {
            current.lines.append(raw);
        }
        else
        {
            bp.header.append(raw);
        }
    }
    if (in_section) bp.sections.append(current);
    return bp;
}

QVector<SigmaRow> parseSigmas(const BestParams &bp)
{
    QVector<SigmaRow> out;
    for (const BpSection &s : bp.sections)
    {
        if (!s.title.contains(QLatin1String("sigma"), Qt::CaseInsensitive)) continue;
        bool first = true;
        for (const QString &raw : s.lines)
        {
            const QString line = raw.trimmed();
            if (line.isEmpty()) continue;
            if (first)
            {
                first = false;   // строка заголовка «параметр ... sigma ... u»
                continue;
            }
            const QStringList tok =
                line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
            if (tok.size() < 3) continue;
            bool ok_sigma = false;
            bool ok_u = false;
            const double sigma = tok[1].toDouble(&ok_sigma);
            const double u = tok[2].toDouble(&ok_u);
            if (!ok_sigma || !ok_u) continue;
            out.append({tok[0], sigma, u});
        }
    }
    return out;
}

// Значение поля из шапки best_params.txt по началу строки («ключ : значение»).
QString headerField(const QStringList &header, const QString &key)
{
    for (const QString &raw : header)
    {
        const QString line = raw.trimmed();
        if (!line.startsWith(key)) continue;
        const int colon = line.indexOf(QLatin1Char(':'));
        if (colon > 0) return line.mid(colon + 1).trimmed();
    }
    return QString();
}

// Значение JSON как текст для строки таблицы.
QString jsonScalar(const QJsonValue &v)
{
    if (v.isDouble())
    {
        const double d = v.toDouble();
        if (std::abs(d) < 1e9 && std::abs(d - std::round(d)) < 1e-9)
            return QString::number(static_cast<long long>(std::llround(d)));
        return QString::number(d, 'g', 6);
    }
    if (v.isString()) return v.toString();
    if (v.isBool()) return v.toBool() ? QObject::tr("да") : QObject::tr("нет");
    if (v.isArray())
    {
        QStringList parts;
        const QJsonArray arr = v.toArray();
        for (const QJsonValue &e : arr) parts << jsonScalar(e);
        return parts.join(QStringLiteral(", "));
    }
    return QStringLiteral("—");
}

void drawJsonObject(QPainter &p, Cursor &c, const QJsonObject &o,
                    const QStringList &order)
{
    for (const QString &key : order)
    {
        if (!o.contains(key)) continue;
        drawRow(p, c, key, jsonScalar(o.value(key)));
    }
}

bool readConfig(const QString &path, QJsonObject *cfg, QString *error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly))
    {
        if (error) *error = QObject::tr("Не удалось открыть конфиг %1").arg(path);
        return false;
    }
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (error)
            *error = QObject::tr("Конфиг %1: %2").arg(path, pe.errorString());
        return false;
    }
    *cfg = doc.object();
    return true;
}

QString resolveWorkRoot(const QJsonObject &cfg, const QString &config_path)
{
    const QString wr = cfg.value(QStringLiteral("work_root")).toString(
        QStringLiteral("temp_tune_v3"));
    if (QFileInfo(wr).isAbsolute()) return QDir::cleanPath(wr);

    // Относительный work_root. Тюнер (tune_kalman.py, analyze_evals.py) резолвит
    // его от каталога самого тюнера, а не от каталога конфига, поэтому конфиги
    // в подкаталогах (truth_cfg, truth_cfg_13b, truth_cfg_13g) указывают на
    // каталог выше себя. Раньше отчёт по ним собирался пустым: каталог
    // truth_cfg_13g/temp_tune_13g_* не существует. Ищем каталог тюнера —
    // ближайший предок конфига с tune_kalman.py, поднимаясь вверх.
    QDir dir(QFileInfo(config_path).absoluteDir());
    for (int up = 0; up <= 4; ++up)
    {
        if (QFileInfo::exists(dir.filePath(QStringLiteral("tune_kalman.py"))))
            return QDir::cleanPath(dir.filePath(wr));
        if (!dir.cdUp()) break;
    }
    // Маркер не найден (конфиг вне дерева тюнера) — как раньше, каталог конфига.
    return QDir::cleanPath(QFileInfo(config_path).absoluteDir().filePath(wr));
}

struct EvalPoint
{
    QString name;      // каталог пробы (этап_номер)
    double j_train = 0.0;
    double j_valid = 0.0;
};

QVector<EvalPoint> readEvals(const QString &work_root)
{
    QVector<EvalPoint> out;
    const QDir dir(work_root);
    const QFileInfoList entries =
        dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QFileInfo &fi : entries)
    {
        const QString path = fi.absoluteFilePath() + QStringLiteral("/eval.json");
        if (!QFileInfo::exists(path)) continue;

        QFile f(path);
        if (!f.open(QIODevice::ReadOnly)) continue;
        const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        if (!doc.isObject()) continue;
        const QJsonObject o = doc.object();
        if (!o.contains(QStringLiteral("J_train"))) continue;

        EvalPoint pt;
        pt.name = fi.fileName();
        pt.j_train = o.value(QStringLiteral("J_train")).toDouble();
        pt.j_valid = o.value(QStringLiteral("J_valid")).toDouble();
        out.append(pt);
    }
    return out;
}

} // namespace

// ------------------------------------------------------------------------

QString TuningReport::workRootFor(const QString &config_path)
{
    QJsonObject cfg;
    if (!readConfig(config_path, &cfg, nullptr)) return QString();
    return resolveWorkRoot(cfg, config_path);
}

QString TuningReport::datasetFor(const QString &config_path)
{
    QJsonObject cfg;
    if (!readConfig(config_path, &cfg, nullptr)) return QString();
    const QString data_dir = cfg.value(QStringLiteral("data_dir")).toString();
    if (data_dir.isEmpty()) return QString();
    return QDir(data_dir).dirName();
}

bool TuningReport::build(const QString &config_path, const QString &pdf_path,
                         QString *error)
{
    QJsonObject cfg;
    if (!readConfig(config_path, &cfg, error)) return false;

    const QString data_dir = cfg.value(QStringLiteral("data_dir")).toString();
    const QString work_root = resolveWorkRoot(cfg, config_path);
    const QString preset = cfg.value(QStringLiteral("output")).toObject()
                               .value(QStringLiteral("preset")).toString();

    // --- Параметры σ из конфига ---
    QVector<ParamInfo> params;
    const QJsonArray params_arr = cfg.value(QStringLiteral("params")).toArray();
    for (const QJsonValue &v : params_arr)
    {
        const QJsonObject po = v.toObject();
        ParamInfo info;
        info.name = po.value(QStringLiteral("name")).toString();
        info.sigma_ref = po.value(QStringLiteral("sigma_ref")).toDouble();
        info.u_span = po.value(QStringLiteral("u_span")).toDouble();
        params.append(info);
    }

    // --- Артефакты подбора ---
    QString best_text;
    {
        QFile f(work_root + QStringLiteral("/best_params.txt"));
        if (f.open(QIODevice::ReadOnly)) best_text = QString::fromUtf8(f.readAll());
    }
    const BestParams best = parseBestParams(best_text);
    const QVector<SigmaRow> sigmas = parseSigmas(best);
    const QVector<EvalPoint> evals = readEvals(work_root);

    // Файл, по которому считалась ошибка (эталон СНС или наземная истина).
    // Строку пишет tune_kalman.py; в старых прогонах её может не быть.
    const QString error_ref = headerField(best.header, QStringLiteral("эталон ошибки"));

    // --- Запись PDF ---
    const QDir out_dir = QFileInfo(pdf_path).absoluteDir();
    if (!out_dir.exists() && !QDir().mkpath(out_dir.absolutePath()))
    {
        if (error)
            *error = QObject::tr("Не удалось создать каталог для %1").arg(pdf_path);
        return false;
    }

    QPdfWriter writer(pdf_path);
    // 72 dpi: координаты painter — пункты, A4 = 595×842; отступы рисуем сами,
    // поля самому QPdfWriter не отдаём (у setPageMargins единица по
    // умолчанию — миллиметры).
    writer.setResolution(72);
    writer.setPageSize(QPageSize(QPageSize::A4));
    writer.setPageMargins(QMarginsF(0, 0, 0, 0), QPageLayout::Point);
    writer.setTitle(QObject::tr("Отчёт о подборе коэффициентов фильтра"));

    QPainter p(&writer);
    if (!p.isActive())
    {
        if (error)
            *error = QObject::tr("Не удалось открыть %1 для записи").arg(pdf_path);
        return false;
    }
    p.setRenderHint(QPainter::Antialiasing, true);

    Cursor c(p, writer);

    // --- Титул и шапка ---
    drawHeading(p, c, QObject::tr("Отчёт о подборе коэффициентов фильтра"), 16.0, 0.0);
    drawRow(p, c, QObject::tr("Создан"),
            QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm")));
    drawRow(p, c, QObject::tr("Набор"),
            data_dir.isEmpty() ? QStringLiteral("—") : QDir(data_dir).dirName());
    drawRow(p, c, QObject::tr("Каталог данных"), data_dir.isEmpty() ? QStringLiteral("—") : data_dir);
    drawRow(p, c, QObject::tr("Конфиг"), config_path);
    drawRow(p, c, QObject::tr("Рабочий каталог"), work_root);
    drawRow(p, c, QObject::tr("Пресет"), preset.isEmpty() ? QStringLiteral("—") : preset);

    // Паспорт набора из [meta] пресета: должен совпадать с data_dir конфига
    // (сверка имён «набор ↔ пресет», см. settings/preset_meta.h).
    if (!preset.isEmpty())
    {
        const PresetMeta meta = readPresetMeta(preset);
        const QString tuned = meta.dataset.trimmed();
        const QString current = QDir(data_dir).dirName();
        QString passport;
        if (!meta.present || tuned.isEmpty())
            passport = QObject::tr("нет (пресет без секции [meta])");
        else if (!current.isEmpty() && tuned == current)
            passport = QObject::tr("совпадает");
        else
            passport = QObject::tr("пресет подобран на наборе «%1»").arg(tuned);
        drawRow(p, c, QObject::tr("Паспорт набора"), passport);
    }
    drawRow(p, c, QObject::tr("Ошибка считается по"),
            error_ref.isEmpty() ? QStringLiteral("—") : error_ref);
    drawRow(p, c, QObject::tr("Оценок (eval.json)"), QString::number(evals.size()));

    // --- Конфигурация подбора ---
    drawHeading(p, c, QObject::tr("Конфигурация подбора"), 13.0, 12.0);

    drawHeading(p, c, QObject::tr("Параметры σ"), 11.0, 4.0);
    {
        QVector<QStringList> rows;
        for (const ParamInfo &pi : params)
            rows.append({pi.name, QString::number(pi.sigma_ref, 'g', 6),
                         QString::number(pi.u_span, 'g', 4)});
        drawColumns(p, c, {QObject::tr("параметр"), QObject::tr("σ_ref"),
                           QObject::tr("u_span")},
                    rows, {150.0, 180.0, 185.0});
    }

    drawHeading(p, c, QObject::tr("Метрика"), 11.0, 4.0);
    drawJsonObject(p, c, cfg.value(QStringLiteral("metric")).toObject(),
                   {QStringLiteral("tol_pos"), QStringLiteral("tol_vel"),
                    QStringLiteral("tol_hdg"), QStringLiteral("tie_weight"),
                    QStringLiteral("ref_lambda"), QStringLiteral("tail_q"),
                    QStringLiteral("tail_weight")});

    drawHeading(p, c, QObject::tr("Горизонт"), 11.0, 4.0);
    drawJsonObject(p, c, cfg.value(QStringLiteral("horizon")).toObject(),
                   {QStringLiteral("skip_first_s"), QStringLiteral("train_frac")});

    drawHeading(p, c, QObject::tr("Оптимизация"), 11.0, 4.0);
    drawJsonObject(p, c, cfg.value(QStringLiteral("optim")).toObject(),
                   {QStringLiteral("n_global"), QStringLiteral("n_local"),
                    QStringLiteral("n_global2"), QStringLiteral("n_workers"),
                    QStringLiteral("seed"), QStringLiteral("early_stop_stale"),
                    QStringLiteral("valid_ovfit_ratio"),
                    QStringLiteral("polish_method"), QStringLiteral("b5_span"),
                    QStringLiteral("run_timeout_s")});

    // --- Результат подбора (из best_params.txt) ---
    drawHeading(p, c, QObject::tr("Результат подбора"), 13.0, 12.0);
    if (!best.found)
    {
        drawRow(p, c, QObject::tr("best_params.txt"),
                QObject::tr("не найден в %1").arg(work_root), 160.0);
    }
    else
    {
        for (const QString &raw : best.header)
        {
            const QString line = raw.trimmed();
            if (line.isEmpty() || line.startsWith(QLatin1String("==="))) continue;
            if (line.startsWith(QStringLiteral("эталон ошибки"))) continue;  // уже в шапке
            const int colon = line.indexOf(QLatin1Char(':'));
            if (colon > 0)
                drawRow(p, c, line.left(colon).trimmed(), line.mid(colon + 1).trimmed(), 210.0);
            else
                drawRow(p, c, line, QString(), 210.0);
        }
        if (sigmas.isEmpty())
            drawRow(p, c, QObject::tr("Подобранные σ"), QObject::tr("нет данных"), 210.0);
    }

    // --- Подобранные σ ---
    if (!sigmas.isEmpty())
    {
        drawHeading(p, c, QObject::tr("Подобранные σ"), 13.0, 12.0);
        QVector<QStringList> rows;
        for (const SigmaRow &s : sigmas)
        {
            double ref = 0.0;
            for (const ParamInfo &pi : params)
                if (pi.name == s.name) { ref = pi.sigma_ref; break; }
            const QString ratio = ref > 0.0 ? QString::number(s.sigma / ref, 'g', 4)
                                            : QStringLiteral("—");
            rows.append({s.name, QString::number(ref, 'g', 6),
                         QString::number(s.sigma, 'g', 6),
                         QString::number(s.u, 'f', 4), ratio});
        }
        drawColumns(p, c,
                    {QObject::tr("параметр"), QObject::tr("σ_ref"), QObject::tr("σ*"),
                     QObject::tr("u*"), QObject::tr("σ*/σ_ref")},
                    rows, {115.0, 100.0, 100.0, 90.0, 110.0});
    }

    // --- Breakdown, плато и чувствительность (остальные секции best_params) ---
    if (best.found)
    {
        for (const BpSection &s : best.sections)
        {
            if (s.title.contains(QLatin1String("sigma"), Qt::CaseInsensitive)) continue;
            drawHeading(p, c, s.title, 13.0, 12.0);
            for (const QString &raw : s.lines)
            {
                const QString line = raw.trimmed();
                if (line.isEmpty()) continue;
                const int colon = line.indexOf(QLatin1Char(':'));
                if (colon > 0)
                    drawRow(p, c, line.left(colon).trimmed(), line.mid(colon + 1).trimmed(),
                            240.0);
                else
                    drawRow(p, c, line, QString(), 240.0);
            }
        }
    }

    // --- Графики ---
    if (!evals.isEmpty() || !sigmas.isEmpty())
    {
        c.newPage();
        drawHeading(p, c, QObject::tr("Графики"), 13.0, 0.0);

        // Сходимость J по оценкам.
        if (!evals.isEmpty())
        {
            drawHeading(p, c, QObject::tr("Сходимость J по оценкам (eval.json)"), 11.0, 8.0);

            TimeSeriesPlot plot;
            plot.setTitle(QString());
            plot.setAxisLabels(QObject::tr("Оценка №"), QObject::tr("J"));
            plot.setExportCursorEnabled(false);

            QVector<double> xs;
            QVector<double> j_train;
            QVector<double> j_valid;
            for (int i = 0; i < evals.size(); ++i)
            {
                xs.append(i + 1);
                j_train.append(evals[i].j_train);
                j_valid.append(evals[i].j_valid);
            }
            plot.addSeries(QStringLiteral("J_train"), xs, j_train, QColor(0x00, 0x72, 0xbd));
            plot.addSeries(QStringLiteral("J_valid"), xs, j_valid, QColor(0xd9, 0x53, 0x19));

            const double height = 230.0;
            c.ensure(height + 20);
            const QRectF box(kMarginPt, c.y, kContentWidth, height);
            p.save();
            p.translate(box.topLeft());
            p.setClipRect(QRectF(0, 0, kContentWidth, height));
            plot.renderTo(p, QSize(static_cast<int>(kContentWidth), static_cast<int>(height)));
            p.restore();
            c.advance(height + 24);
        }

        // Отклонение подобранных σ от опорных: u* = log10(σ*/σ_ref).
        if (!sigmas.isEmpty())
        {
            drawHeading(p, c, QObject::tr("Отклонение σ от опорного (u* = log10(σ*/σ_ref))"),
                        11.0, 8.0);

            const double row_h = 17.0;
            const double label_w = 115.0;
            const double value_w = 70.0;
            const double track_w = kContentWidth - label_w - value_w;
            const double track_x = kMarginPt + label_w;
            const double center = track_x + track_w / 2.0;
            const double height = sigmas.size() * row_h + 8.0;
            c.ensure(height + 4);

            double umax = 0.5;
            for (const SigmaRow &s : sigmas) umax = std::max(umax, std::abs(s.u));

            QFont f = p.font();
            f.setPointSizeF(9.0);
            p.setFont(f);
            const QFontMetricsF fm(f);

            // Ось (нулевая линия) и рамка дорожки.
            p.setPen(QPen(QColor(150, 150, 150)));
            const double top = c.y;
            const double bottom = c.y + height;
            p.drawLine(QPointF(track_x, top), QPointF(track_x + track_w, top));
            p.drawLine(QPointF(track_x, bottom), QPointF(track_x + track_w, bottom));
            p.drawLine(QPointF(center, top), QPointF(center, bottom));

            for (int i = 0; i < sigmas.size(); ++i)
            {
                const SigmaRow &s = sigmas[i];
                const double cy = top + 4 + i * row_h + row_h / 2.0;

                p.setPen(Qt::black);
                p.drawText(QRectF(kMarginPt, cy - row_h / 2.0, label_w - 6.0, row_h),
                           Qt::AlignRight | Qt::AlignVCenter, s.name);

                const double len = (s.u / umax) * (track_w / 2.0);
                const QRectF bar(std::min(center, center + len), cy - 4.0,
                                 std::abs(len), 8.0);
                p.fillRect(bar, s.u >= 0.0 ? QColor(0x00, 0x72, 0xbd)
                                           : QColor(0xd9, 0x53, 0x19));

                p.drawText(QRectF(track_x + track_w, cy - row_h / 2.0, value_w, row_h),
                           Qt::AlignRight | Qt::AlignVCenter,
                           QString::number(s.u, 'f', 3));
            }

            c.advance(height + 6);
            p.setFont(QFont());
            p.setPen(QColor(90, 90, 90));
            QFont small = p.font();
            small.setPointSizeF(8.0);
            p.setFont(small);
            p.drawText(QRectF(track_x, c.y, track_w + value_w, 12.0), Qt::AlignRight,
                       QObject::tr("вправо — σ* больше опорного, влево — меньше"));
            p.setFont(QFont());
            c.advance(16.0);
        }
    }

    p.end();
    return true;
}

} // namespace gui
