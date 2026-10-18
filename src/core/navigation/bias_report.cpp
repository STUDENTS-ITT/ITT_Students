// bias_report.cpp — Реализация сводной таблицы по журналу смещений.
//
// params.json разбирается минимальным сканером: в ядре нет JSON-библиотеки,
// а файлы наборов генерируются нашим же dataset_generator в известном формате.
// Сканер ищет скаляр или массив из трёх чисел по ключу на любой глубине и
// отвергает NaN/Inf, чтобы повреждённый файл не дал ложных истин.

#include "bias_report.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "simulation.h"
#include "../utils/constants.h"

namespace nav
{

namespace
{

// Пропуск пробелов, переводов строк и табуляций.
void skipWs(const std::string &s, std::size_t &i)
{
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r'))
        i++;
}

// Разбор числа JSON. Возвращает false на не-числе и на NaN/Inf.
bool parseNumber(const std::string &s, std::size_t &i, double &out)
{
    skipWs(s, i);
    const char *begin = s.c_str() + i;
    char *end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end == begin) return false;
    i += static_cast<std::size_t>(end - begin);
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
}

// Пропуск строки JSON (включая кавычки и экранирование).
bool skipString(const std::string &s, std::size_t &i)
{
    if (i >= s.size() || s[i] != '"') return false;
    i++;
    while (i < s.size())
    {
        if (s[i] == '\\') { i += 2; continue; }
        if (s[i] == '"') { i++; return true; }
        i++;
    }
    return false;
}

// Пропуск значения JSON произвольной формы (объект/массив/строка/число/литерал).
// Нужен, чтобы обойти вложенные структуры в поиске ключа.
bool skipValue(const std::string &s, std::size_t &i)
{
    skipWs(s, i);
    if (i >= s.size()) return false;
    const char c = s[i];
    if (c == '{' || c == '[')
    {
        const char open = c;
        const char close = (c == '{') ? '}' : ']';
        int depth = 0;
        while (i < s.size())
        {
            if (s[i] == '"') { if (!skipString(s, i)) return false; continue; }
            if (s[i] == open) depth++;
            else if (s[i] == close)
            {
                depth--;
                if (depth == 0) { i++; return true; }
            }
            i++;
        }
        return false;
    }
    if (c == '"') return skipString(s, i);
    double dummy = 0.0;
    if (parseNumber(s, i, dummy)) return true;
    // true / false / null
    for (const char *lit : {"true", "false", "null"})
    {
        const std::size_t n = std::strlen(lit);
        if (s.compare(i, n, lit) == 0) { i += n; return true; }
    }
    return false;
}

// Поиск ключа на любой глубине. Если найден массив из трёх чисел — в out3.
// Если скаляр — в out1. true — ключ найден и разобран.
bool findScalarOrTriple(const std::string &s, const std::string &key,
                        double &out1, double out3[3])
{
    std::size_t i = 0;
    while (i < s.size())
    {
        if (s[i] == '"')
        {
            const std::size_t key_begin = i;
            if (!skipString(s, i)) return false;
            const std::string found = s.substr(key_begin + 1, i - key_begin - 2);
            skipWs(s, i);
            if (i < s.size() && s[i] == ':')
            {
                i++;
                skipWs(s, i);
                if (found == key)
                {
                    if (i < s.size() && s[i] == '[')
                    {
                        i++;
                        for (int k = 0; k < 3; k++)
                        {
                            if (k > 0)
                            {
                                skipWs(s, i);
                                if (i < s.size() && s[i] == ',') i++;
                            }
                            if (!parseNumber(s, i, out3[k])) return false;
                        }
                        return true;
                    }
                    return parseNumber(s, i, out1);
                }
                // Не наш ключ: значение пропускаем целиком. Вложенный объект
                // или массив не пропускаем, а заходим внутрь — нужный ключ
                // может лежать в секции (noise_si находится внутри корня).
                if (s[i] == '{' || s[i] == '[') { i++; continue; }
                if (!skipValue(s, i)) return false;
                continue;
            }
            continue;
        }
        i++;
    }
    return false;
}

} // namespace

bool loadBiasTruth(const std::string &params_json_path, BiasTruth &out,
                   std::string *error)
{
    out = BiasTruth{};

    std::ifstream in(params_json_path);
    if (!in.is_open())
    {
        if (error) *error = "cannot open " + params_json_path;
        return false;
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    const std::string s = buf.str();
    if (s.empty())
    {
        if (error) *error = params_json_path + ": empty file";
        return false;
    }

    double dummy = 0.0;
    double ba3[3] = {0, 0, 0};
    double bg3[3] = {0, 0, 0};

    // Имя набора — строка; ищем отдельно, т.к. сканер читает числа.
    {
        std::size_t i = s.find("\"name\"");
        if (i != std::string::npos)
        {
            i = s.find(':', i);
            if (i != std::string::npos)
            {
                i++;
                skipWs(s, i);
                if (i < s.size() && s[i] == '"')
                {
                    const std::size_t begin = i;
                    if (skipString(s, i))
                        out.name = s.substr(begin + 1, i - begin - 2);
                }
            }
        }
    }

    double align = 0.0;
    if (findScalarOrTriple(s, "align_time_s", align, ba3)) out.align_time_s = align;

    bool has_ba = findScalarOrTriple(s, "accel_bias_m_s2", dummy, ba3);
    bool has_bg = findScalarOrTriple(s, "gyro_bias_rad_s", dummy, bg3);
    if (!has_ba || !has_bg)
    {
        if (error)
            *error = params_json_path + ": noise_si.accel_bias_m_s2/gyro_bias_rad_s not found";
        return false;
    }
    for (int k = 0; k < 3; k++) { out.ba[k] = ba3[k]; out.bg[k] = bg3[k]; }

    // Наблюдаемость (план, раздел 2): на стоянке и на прямой без поворотов
    // горизонтальные ba/bg неотличимы от наклона и курса — считаем
    // наблюдаемой только вертикальную ось Y.
    bool any_motion = false;
    bool any_turn = false;
    std::size_t i = 0;
    while ((i = s.find("\"v_ms\"", i)) != std::string::npos)
    {
        std::size_t j = s.find(':', i);
        double v = 0.0;
        if (j != std::string::npos && parseNumber(s, ++j, v) && fabs(v) > 1e-9)
            any_motion = true;
        i += 6;
    }
    i = 0;
    while ((i = s.find("\"turn_deg_s\"", i)) != std::string::npos)
    {
        std::size_t j = s.find(':', i);
        double v = 0.0;
        if (j != std::string::npos && parseNumber(s, ++j, v) && fabs(v) > 1e-9)
            any_turn = true;
        i += 12;
    }

    out.valid = true;
    if (!any_motion)
    {
        for (int k = 0; k < 6; k++) out.observable[k] = (k % 3 == 1);
        out.observability_note =
            "стоянка: наблюдаемы только ba_Y (вертикаль) и bg_Y (по вращению Земли)";
    }
    else if (!any_turn)
    {
        for (int k = 0; k < 6; k++) out.observable[k] = (k % 3 == 1);
        out.observability_note =
            "прямая без поворотов: курс и горизонтальные смещения неотличимы";
    }
    else
    {
        out.observability_note = "манёвренный профиль: наблюдаемы все 6 компонент";
    }
    return true;
}

std::vector<BiasReportRow> buildBiasReport(const std::vector<BiasSample> &log,
                                           const BiasTruth &truth,
                                           const std::vector<double> &windows_s)
{
    std::vector<BiasReportRow> rows;
    if (log.empty()) return rows;

    std::vector<double> windows = windows_s;
    if (windows.empty())
        windows = {1.0, 10.0, 30.0, 60.0, 120.0, 300.0};

    // Окна считаются от конца выставки: в журнале первый кадр — это момент
    // после выставки, его время берём за ноль окон.
    const double t0 = log.front().time;
    const BiasSample &last = log.back();

    // ba_X, ba_Y, ba_Z, bg_X, bg_Y, bg_Z — оси тела: X вперёд (крен),
    // Y вверх (курс), Z вправо (тангаж).
    static const char *kNames[6] = {"ba_X", "ba_Y", "ba_Z", "bg_X", "bg_Y", "bg_Z"};

    for (double w : windows)
    {
        const double t_target = t0 + w;

        // Последний кадр с time <= t_target (или первый, если окно меньше
        // первого кадра — тогда берём первый).
        const BiasSample *pick = &log.front();
        for (const BiasSample &s : log)
        {
            if (s.time <= t_target) pick = &s;
            else break;
        }
        // Для последнего окна (конец прогона) берём финальный кадр.
        if (w >= windows.back()) pick = &last;

        for (int c = 0; c < 6; c++)
        {
            BiasReportRow r;
            r.component = kNames[c];
            r.window_s = w;
            r.observable = truth.valid ? truth.observable[c] : true;

            if (c < 3)
            {
                r.truth = truth.valid ? truth.ba[c] : 0.0;
                r.estimate = pick->ba[c];
                r.sigma = pick->sig_ba[c];
            }
            else
            {
                r.truth = truth.valid ? truth.bg[c - 3] : 0.0;
                r.estimate = pick->bg[c - 3];
                r.sigma = pick->sig_bg[c - 3];
            }

            r.norm_error = (r.sigma > 0.0) ? (r.estimate - r.truth) / r.sigma
                                           : std::nan("");
            rows.push_back(r);
        }
    }
    return rows;
}

std::string formatBiasReport(const std::vector<BiasReportRow> &rows,
                             const BiasTruth &truth)
{
    std::ostringstream out;
    out << std::scientific << std::setprecision(4);

    out << "=== Bias estimate report ===\n";
    if (truth.valid)
    {
        out << "dataset: " << (truth.name.empty() ? "(unnamed)" : truth.name) << "\n";
        out << "align_time_s: " << truth.align_time_s << "\n";
        out << "truth ba [m/s^2]: " << truth.ba[0] << " " << truth.ba[1] << " "
            << truth.ba[2] << "\n";
        out << "truth bg [rad/s]: " << truth.bg[0] << " " << truth.bg[1] << " "
            << truth.bg[2] << "\n";
    }
    else
    {
        out << "truth: unavailable (params.json not loaded) — "
               "norm_error columns are meaningless\n";
    }
    out << "observability: "
        << (truth.valid ? truth.observability_note : std::string("unknown"))
        << "\n\n";

    out << std::left << std::setw(8) << "comp"
        << std::right << std::setw(10) << "window_s"
        << std::setw(16) << "truth"
        << std::setw(16) << "estimate"
        << std::setw(16) << "sigma_P"
        << std::setw(12) << "norm_err"
        << std::setw(10) << "observable"
        << std::setw(12) << "verdict" << "\n";
    out << std::string(100, '-') << "\n";

    int converged = 0, diverged = 0, skipped = 0;
    for (const BiasReportRow &r : rows)
    {
        std::string verdict = "-";
        if (!r.observable)
        {
            verdict = "n/a";
            skipped++;
        }
        else if (std::isfinite(r.norm_error))
        {
            if (fabs(r.norm_error) <= 2.0) { verdict = "OK"; converged++; }
            else { verdict = "FAIL"; diverged++; }
        }
        else
        {
            verdict = "no sigma";
            skipped++;
        }

        out << std::left << std::setw(8) << r.component
            << std::right << std::setw(10) << r.window_s
            << std::setw(16) << r.truth
            << std::setw(16) << r.estimate
            << std::setw(16) << r.sigma;
        if (std::isfinite(r.norm_error)) out << std::setw(12) << r.norm_error;
        else out << std::setw(12) << "n/a";
        out << std::setw(10) << (r.observable ? "yes" : "no")
            << std::setw(12) << verdict << "\n";
    }

    out << std::string(100, '-') << "\n";
    out << "converged (|norm_err|<=2): " << converged
        << ", diverged: " << diverged
        << ", not observable / no sigma: " << skipped << "\n";
    out << "criterion: |estimate - truth| <= 2*sigma_P on observable components\n";

    // Итог в инженерных единицах: в СИ смещения порядка 1e-5..1e-2 и глазом
    // не читаются, а именно эта форма используется в params.json (mg, deg/h)
    // и в разделе 2 плана. Показываем финальное окно (последняя строка).
    if (!rows.empty())
    {
        const BiasReportRow &last_row = rows.back();
        const double t_end = last_row.window_s;
        out << "\nfinal state (window " << t_end << " s, engineering units)\n";
        out << std::left << std::setw(8) << "comp"
            << std::right << std::setw(16) << "truth"
            << std::setw(16) << "estimate"
            << std::setw(16) << "error"
            << std::setw(16) << "sigma_P"
            << std::setw(12) << "error/sigma"
            << std::setw(10) << "observable"
            << std::setw(12) << "verdict" << "\n";
        out << std::string(88, '-') << "\n";

        for (const BiasReportRow &r : rows)
        {
            if (r.window_s != t_end) continue;

            // ba — мг (1 мг = 9.80665e-3 м/с²), bg — °/ч. Отличаем по префиксу:
            // "ba_X" / "bg_X", ось стоит на 4-й позиции.
            const bool is_ba = (r.component.size() > 1 && r.component[0] == 'b' &&
                                r.component[1] == 'a');
            const double scale = is_ba ? (1.0 / 9.80665e-3) : (180.0 / PI * 3600.0);

            std::string verdict = "-";
            if (!r.observable) verdict = "n/a";
            else if (std::isfinite(r.norm_error)) verdict = (fabs(r.norm_error) <= 2.0) ? "OK" : "FAIL";
            else verdict = "no sigma";

            out << std::left << std::setw(8) << r.component
                << std::right << std::setprecision(4)
                << std::setw(16) << r.truth * scale
                << std::setw(16) << r.estimate * scale
                << std::setw(16) << (r.estimate - r.truth) * scale
                << std::setw(16) << r.sigma * scale;
            if (std::isfinite(r.norm_error)) out << std::setw(12) << r.norm_error;
            else out << std::setw(12) << "n/a";
            out << std::setw(10) << (r.observable ? "yes" : "no")
                << std::setw(12) << verdict << "\n";
        }
        out << "units: ba — mg (" << 1.0 / 9.80665e-3 << " per m/s^2), bg — deg/h\n";
    }

    return out.str();
}

bool runBiasReport(const RunOptions &options, RunSummary &summary,
                   std::string &report, std::string &error)
{
    // Журнал включаем принудительно: ради него отчёт и запускается.
    RunOptions opts = options;
    opts.write_bias_log = true;
    opts.write_files = true;

    if (!runSimulation(opts, summary, nullptr, nullptr, nullptr))
    {
        error = summary.error.empty() ? "simulation failed" : summary.error;
        return false;
    }

    // bias.txt рядом с result.txt — в output_dir прогона.
    const std::string bias_path =
        (std::filesystem::path(opts.output_dir) / "bias.txt").string();

    std::vector<BiasSample> log;
    std::string read_error;
    if (!readBiasLog(bias_path, log, &read_error))
    {
        error = read_error;
        return false;
    }

    // Истина — из params.json набора. Если его нет (набор сгенерирован без
    // него или каталог чужой), отчёт всё равно строим, но помечаем истины.
    BiasTruth truth;
    std::string truth_error;
    const std::string params_path =
        (std::filesystem::path(opts.data_dir) / "params.json").string();
    loadBiasTruth(params_path, truth, &truth_error);
    if (!truth.valid && !truth_error.empty())
        error.clear();  // не ошибка: отчёт без истины допустим

    report = formatBiasReport(buildBiasReport(log, truth), truth);
    report += "\nlog: " + bias_path + " (" + std::to_string(log.size()) + " rows)\n";
    if (!truth.valid)
        report += "truth: " + truth_error + "\n";
    return true;
}

} // namespace nav
