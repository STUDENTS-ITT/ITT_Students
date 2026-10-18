// bias_log.cpp — Реализация журнала оценок смещений (bias.txt).
//
// Формат: одна строка на кадр, 15 чисел через пробел, научная нотация с
// полной машинной точностью. Первая строка — заголовок с именами колонок
// (начинается с '#', при чтении пропускается).

#include "bias_log.h"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace nav
{

namespace
{

// Разбор одного числа. Возвращает false на NaN/Inf и на не-число, чтобы
// повреждённый файл не дал ложных «сходимостей».
bool parseDouble(const std::string &token, double &out)
{
    if (token.empty()) return false;
    char *end = nullptr;
    const double v = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') return false;
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
}

} // namespace

bool BiasLogWriter::open(const std::string &path)
{
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_.is_open()) return false;

    // Заголовок — имена колонок; парсер отчёта пропускает строки на '#'.
    out_ << "# time_s ba_x ba_y ba_z bg_x bg_y bg_z "
            "sig_ba_x sig_ba_y sig_ba_z sig_bg_x sig_bg_y sig_bg_z\n";
    // Полная машинная точность: σ(P_ba) порядка 1e-6 м/с², усечение до 4
    // значащих цифр обнуляет их (пробел, который фиксирует план, п. 6).
    out_ << std::scientific << std::setprecision(17);
    return out_.good();
}

void BiasLogWriter::write(const BiasSample &s)
{
    if (!out_.is_open()) return;

    out_ << s.time;
    for (int k = 0; k < 3; k++) out_ << ' ' << s.ba[k];
    for (int k = 0; k < 3; k++) out_ << ' ' << s.bg[k];
    for (int k = 0; k < 3; k++) out_ << ' ' << s.sig_ba[k];
    for (int k = 0; k < 3; k++) out_ << ' ' << s.sig_bg[k];
    out_ << '\n';
}

void BiasLogWriter::close()
{
    if (out_.is_open()) out_.close();
}

bool BiasLogWriter::isOpen() const
{
    return out_.is_open();
}

bool readBiasLog(const std::string &path, std::vector<BiasSample> &out,
                 std::string *error)
{
    out.clear();

    std::ifstream in(path);
    if (!in.is_open())
    {
        if (error) *error = "cannot open " + path;
        return false;
    }

    std::string line;
    std::size_t line_no = 0;
    while (std::getline(in, line))
    {
        line_no++;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::vector<double> values;
        std::string token;
        bool line_ok = true;
        while (ss >> token)
        {
            double v = 0.0;
            if (!parseDouble(token, v))
            {
                line_ok = false;
                break;
            }
            values.push_back(v);
        }
        if (!line_ok)
        {
            if (error)
            {
                std::ostringstream msg;
                msg << path << ": line " << line_no << ": malformed number";
                *error = msg.str();
            }
            return false;
        }
        // 13 колонок: время, ba×3, bg×3, σ_ba×3, σ_bg×3.
        // (Этап 1: трим×2 удалён из формата — см. bias_log.h.)
        constexpr std::size_t EXPECTED_COLUMNS = 13;
        if (values.size() != EXPECTED_COLUMNS)
        {
            if (error)
            {
                std::ostringstream msg;
                msg << path << ": line " << line_no << ": expected "
                    << EXPECTED_COLUMNS << " columns, got " << values.size();
                *error = msg.str();
            }
            return false;
        }

        BiasSample s;
        s.time = values[0];
        for (int k = 0; k < 3; k++)
        {
            s.ba[k] = values[1 + k];
            s.bg[k] = values[4 + k];
            s.sig_ba[k] = values[7 + k];
            s.sig_bg[k] = values[10 + k];
        }
        out.push_back(s);
    }

    if (out.empty())
    {
        if (error) *error = path + ": no data rows";
        return false;
    }
    return true;
}

} // namespace nav
