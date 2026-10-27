// Мелкие общие утилиты: строки, время, таблицы, журнал консоли.
#pragma once

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <iosfwd>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace vo {

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// Включает UTF-8 в консоли Windows (на других ОС - ничего не делает).
void init_console();

// printf-форматирование в std::string.
std::string fmt(const char* format, ...)
#if defined(__MINGW32__) && !defined(__clang__)
    __attribute__((format(gnu_printf, 1, 2)))
#elif defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

std::string lower(std::string s);
std::string upper(std::string s);
std::string trim(const std::string& s);
std::vector<std::string> split(const std::string& s, char delim);
bool starts_with(const std::string& s, const std::string& prefix);
bool ends_with_ci(const std::string& s, const std::string& suffix);

// Заменяет все символы, кроме [A-Za-z0-9_-], на '_' (порт safe_file_name).
std::string safe_file_name(const std::string& text);

// Среднее по конечным значениям; NaN, если таких нет.
double safe_mean(const std::vector<double>& values);
double median(std::vector<double> values);  // NaN для пустого вектора

// Python round(): к ближайшему чётному при .5.
inline double round_half_even(double x) { return std::nearbyint(x); }

inline double now_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

// Создаёт каталог со всеми родителями.
void make_dirs(const std::string& path);
std::string parent_path(const std::string& path);

// Формат числа как в Python-версии: f'{v:.15g}', NaN для нечисел.
std::string fmt_g15(double v);

// Простейшая CSV-таблица со строковыми ячейками.
struct CsvTable {
    std::vector<std::string> columns;
    std::vector<std::vector<std::string>> rows;
    void write(const std::string& path) const;   // UTF-8, '\n'
    void print(std::ostream& os) const;          // выровненный вывод
};

// Дублирует std::cout в файл (аналог diary/ConsoleLog из Python).
// Пока путь неизвестен, вывод копится в памяти; set_path() сбрасывает его в
// файл. Вложенные журналы допустимы: внутренний пишет во внешний.
class ConsoleLog {
public:
    explicit ConsoleLog(const std::string& path = "");
    ~ConsoleLog();
    ConsoleLog(const ConsoleLog&) = delete;
    ConsoleLog& operator=(const ConsoleLog&) = delete;

    void set_path(const std::string& path);

    // Строка хода работы: только в терминал, обновляется на месте. Возвращает
    // false, если вывод перенаправлен (тогда вызывающий печатает сводку сам).
    bool progress(const std::string& text);
    void end_progress();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vo
