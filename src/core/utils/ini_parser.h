// ini_parser.h — Минимальный INI-ридер для параметров фильтра (core, без Qt).
//
// Поддерживаемый формат (совместим с QSettings INI):
//   ; комментарий
//   # комментарий
//   [section]
//   key = value
//
// Ключи хранятся с полным именем "section/key", поэтому обращение к параметру
// не зависит от того, в какой секции он объявлен. Регистр ключей не важен.

#pragma once

#include <map>
#include <string>
#include <vector>

namespace utils
{

// Хранилище «ключ → значение». Ключ в нижнем регистре, с префиксом секции.
class IniFile
{
public:
    // Разбор INI из файла. Возвращает false, если файл не открылся.
    bool load(const std::string &path);

    // Разбор INI из строки (для тестов и встроенных значений).
    void loadFromString(const std::string &text);

    // Значение по ключу; default, если ключ отсутствует.
    double value(const std::string &key, double default_value) const;

    // Значение по ключу; пустая строка, если ключ отсутствует.
    std::string text(const std::string &key, const std::string &default_value = "") const;

    bool has(const std::string &key) const;

    // Все ключи в порядке появления (для редактора параметров в GUI).
    const std::vector<std::string> &keys() const { return order_; }

    // Секция → список ключей (для группировки в GUI).
    std::vector<std::string> sections() const;

private:
    static std::string normalize(const std::string &key);

    std::map<std::string, std::string> values_;
    std::vector<std::string> order_;
};

// Экранирование/подстановка не требуются: значения хранятся как есть,
// лишние пробелы по краям обрезаются.

} // namespace utils