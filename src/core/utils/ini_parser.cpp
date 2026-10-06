// ini_parser.cpp — Реализация INI-ридера параметров.

#include "ini_parser.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace utils
{
namespace
{

// Обрезка пробелов и табуляций по краям строки.
std::string trim(const std::string &s)
{
    const auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
    std::size_t b = 0;
    std::size_t e = s.size();
    while (b < e && is_space(static_cast<unsigned char>(s[b]))) b++;
    while (e > b && is_space(static_cast<unsigned char>(s[e - 1]))) e--;
    return s.substr(b, e - b);
}

// Перевод ключа в нижний регистр (регистронезависимый поиск).
std::string lower(const std::string &s)
{
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

// Преобразование строки в double без исключений: при ошибке возвращает ok=false.
bool toDouble(const std::string &s, double &out)
{
    const std::string t = trim(s);
    if (t.empty()) return false;
    try
    {
        std::size_t used = 0;
        const double v = std::stod(t, &used);
        if (used != t.size()) return false;
        out = v;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace

std::string IniFile::normalize(const std::string &key)
{
    return lower(trim(key));
}

bool IniFile::load(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::ostringstream ss;
    ss << f.rdbuf();
    loadFromString(ss.str());
    return true;
}

void IniFile::loadFromString(const std::string &text)
{
    values_.clear();
    order_.clear();

    std::istringstream in(text);
    std::string line;
    std::string section;

    while (std::getline(in, line))
    {
        // Символ # в начале строки — комментарий (INI-совместимость).
        const std::string t = trim(line);
        if (t.empty()) continue;
        if (t[0] == ';' || t[0] == '#') continue;

        if (t.front() == '[')
        {
            const auto close = t.find(']');
            if (close != std::string::npos)
            {
                section = trim(t.substr(1, close - 1));
            }
            continue;
        }

        const auto eq = t.find('=');
        if (eq == std::string::npos) continue;

        const std::string key = trim(t.substr(0, eq));
        const std::string val = trim(t.substr(eq + 1));
        if (key.empty()) continue;

        const std::string full = section.empty() ? normalize(key)
                                                : normalize(section + "/" + key);
        if (values_.find(full) == values_.end())
        {
            order_.push_back(full);
        }
        values_[full] = val;
    }
}

bool IniFile::has(const std::string &key) const
{
    return values_.find(normalize(key)) != values_.end();
}

double IniFile::value(const std::string &key, double default_value) const
{
    const auto it = values_.find(normalize(key));
    if (it == values_.end()) return default_value;

    double v = 0.0;
    if (!toDouble(it->second, v)) return default_value;
    return v;
}

std::string IniFile::text(const std::string &key, const std::string &default_value) const
{
    const auto it = values_.find(normalize(key));
    return it == values_.end() ? default_value : it->second;
}

std::vector<std::string> IniFile::sections() const
{
    std::vector<std::string> out;
    for (const auto &k : order_)
    {
        const auto slash = k.find('/');
        if (slash == std::string::npos) continue;
        const std::string sec = k.substr(0, slash);
        if (std::find(out.begin(), out.end(), sec) == out.end())
        {
            out.push_back(sec);
        }
    }
    return out;
}

} // namespace utils