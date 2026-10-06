// filter_settings.cpp — Загрузка/выгрузка настраиваемых параметров фильтра.

#include "filter_settings.h"

#include <cmath>
#include <fstream>
#include <ostream>
#include <sstream>
#include <vector>

namespace nav
{
namespace
{

// Вспомогательная структура «ключ INI → поле структуры».
struct FieldBinding
{
    const char *key;
    double *field;
};

// Привязка полей KalmanConfig к ключам INI.
std::vector<FieldBinding> filterBindings(ins::KalmanConfig &c)
{
    return {
        {"filter/sigma_g", &c.sig_g},
        {"filter/sigma_a", &c.sig_a},
        {"filter/sigma_bg", &c.sig_bg},
        {"filter/sigma_ba", &c.sig_ba},
        {"filter/sigma_pos", &c.sig_pos},
        {"filter/sigma_h", &c.sig_h},
        {"filter/sigma_v", &c.sig_v},
        {"filter/sigma_hdg", &c.sig_hdg},
        {"filter/outage_start_s", &c.outage_start_s},
        {"filter/outage_end_s", &c.outage_end_s},
    };
}

// Привязка полей TiltManeuverPolicy к ключам INI.
std::vector<FieldBinding> tiltBindings(TiltManeuverPolicy &p)
{
    return {
        {"tilt/gyro_off_rad_s", &p.gyro_off_rad_s},
        {"tilt/settle_s", &p.settle_s},
        {"tilt/brake_body_accel", &p.brake_body_accel},
        {"tilt/tau_fast_s", &p.tau_fast_s},
        {"tilt/tau_slow_s", &p.tau_slow_s},
        {"tilt/fast_err_rad", &p.fast_err_rad},
        {"tilt/level_acc_rad", &p.level_acc_rad},
        {"tilt/bias_gain", &p.bias_gain},
        {"tilt/bias_max_rad_s", &p.bias_max_rad_s},
        {"tilt/divergence_deadband_rad", &p.divergence_deadband_rad},
    };
}

std::string fmt(double v)
{
    std::ostringstream ss;
    ss.precision(17);
    ss << v;
    return ss.str();
}

// Имя ключа без префикса группы: в INI ключ пишется внутри своей секции,
// а парсер собирает обратно «секция/ключ».
std::string keyInGroup(const char *key)
{
    const std::string k(key);
    const std::size_t slash = k.find('/');
    return slash == std::string::npos ? k : k.substr(slash + 1);
}

} // namespace

TiltManeuverPolicy &tiltPolicy()
{
    static TiltManeuverPolicy policy;  // значения по умолчанию
    return policy;
}

void applySettings(const FilterSettings &s)
{
    ins::kalman_cfg = s.filter;
    tiltPolicy() = s.tilt;
}

bool loadFilterSettings(const std::string &path, FilterSettings &s)
{
    utils::IniFile ini;
    if (!ini.load(path)) return false;

    for (const auto &b : filterBindings(s.filter))
    {
        if (ini.has(b.key)) *b.field = ini.value(b.key, *b.field);
    }
    for (const auto &b : tiltBindings(s.tilt))
    {
        if (ini.has(b.key)) *b.field = ini.value(b.key, *b.field);
    }
    if (ini.has("run/align_time_override_s"))
    {
        s.align_time_override_s = ini.value("run/align_time_override_s", s.align_time_override_s);
    }
    if (ini.has("run/align_fallback_s"))
    {
        s.align_fallback_s = ini.value("run/align_fallback_s", s.align_fallback_s);
    }
    return true;
}

bool settingsFileHasSections(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open()) return true;

    std::string line;
    while (std::getline(f, line))
    {
        const std::size_t t = line.find_first_not_of(" \t");
        if (t == std::string::npos) continue;
        if (line[t] == ';' || line[t] == '#') continue;
        return line[t] == '[';  // первая содержательная строка — заголовок секции
    }
    return true;  // пустой файл — считаем секционным
}

// Старый плоский params.ini: ключи без секций пишутся прямо в глобальную
// конфигурацию фильтра. Применяется поверх значений по умолчанию, поэтому
// вызывается после applySettings().
void applyLegacyParamsFile(const std::string &path, std::ostream *log)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        if (log) *log << "params.ini: cannot open " << path << std::endl;
        return;
    }

    struct Entry
    {
        const char *name;
        double *field;
    };
    const Entry entries[] = {
        {"sigma_g", &ins::kalman_cfg.sig_g},
        {"sigma_a", &ins::kalman_cfg.sig_a},
        {"sigma_bg", &ins::kalman_cfg.sig_bg},
        {"sigma_ba", &ins::kalman_cfg.sig_ba},
        {"sigma_pos", &ins::kalman_cfg.sig_pos},
        {"sigma_v", &ins::kalman_cfg.sig_v},
        {"sigma_ang", &ins::kalman_cfg.sig_hdg},
        {"outage_start_s", &ins::kalman_cfg.outage_start_s},
        {"outage_end_s", &ins::kalman_cfg.outage_end_s},
    };

    std::string line;
    while (std::getline(f, line))
    {
        const std::size_t t = line.find_first_not_of(" \t");
        if (t == std::string::npos) continue;
        if (line[t] == ';' || line[t] == '#') continue;

        std::string key = line.substr(0, line.find('='));
        std::string val = line.substr(line.find('=') + 1);
        while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
        while (!val.empty() && (val.front() == ' ' || val.front() == '\t')) val.erase(val.begin());
        while (!val.empty() && (val.back() == ' ' || val.back() == '\t' || val.back() == '\r')) val.pop_back();
        if (key.empty() || val.empty()) continue;

        for (const auto &e : entries)
        {
            if (key != e.name) continue;
            try
            {
                *e.field = std::stod(val);
            }
            catch (const std::exception &)
            {
                // Мусор в значении игнорируем: остаётся значение по умолчанию.
                if (log) *log << "params.ini: bad value for " << key << ": " << val << std::endl;
            }
            break;
        }
    }

    if (log)
    {
        *log << "params.ini: sigma_g=" << ins::kalman_cfg.sig_g
             << " sigma_a=" << ins::kalman_cfg.sig_a
             << " sigma_bg=" << ins::kalman_cfg.sig_bg
             << " sigma_ba=" << ins::kalman_cfg.sig_ba
             << " sigma_pos=" << ins::kalman_cfg.sig_pos
             << " sigma_v=" << ins::kalman_cfg.sig_v
             << " sigma_ang=" << ins::kalman_cfg.sig_hdg << std::endl;
    }
}

std::string dumpFilterSettings(const FilterSettings &s)
{
    FilterSettings def;

    std::ostringstream out;
    out << "# Настройки фильтра ESKF. Создаётся и редактируется в imitator_gui.\n"
        << "# Отсутствующие ключи берутся из значений по умолчанию.\n\n";

    const auto dump_group = [&out](const char *group,
                                   std::vector<FieldBinding> values,
                                   const std::vector<FieldBinding> &defaults) {
        out << "[" << group << "]\n";
        for (std::size_t i = 0; i < values.size(); i++)
        {
            const std::string name = keyInGroup(values[i].key);
            const double dv = defaults[i].field != nullptr ? *defaults[i].field : 0.0;
            out << "; " << name << " = " << fmt(dv) << "\n";
            out << name << " = " << fmt(*values[i].field) << "\n";
        }
        out << "\n";
    };

    FilterSettings tmp = s;
    dump_group("filter", filterBindings(tmp.filter), filterBindings(def.filter));
    dump_group("tilt", tiltBindings(tmp.tilt), tiltBindings(def.tilt));

    out << "[run]\n";
    out << "align_time_override_s = " << fmt(s.align_time_override_s) << "\n";
    out << "align_fallback_s = " << fmt(s.align_fallback_s) << "\n";
    return out.str();
}

std::vector<SettingDescriptor> filterSettingDescriptors()
{
    const FilterSettings d;

    std::vector<SettingDescriptor> out;
    const auto add = [&out](const char *key, const char *label, const char *units,
                            double lo, double hi) {
        SettingDescriptor sd;
        sd.key = key;
        sd.label = label;
        sd.units = units;
        sd.min_value = lo;
        sd.max_value = hi;
        sd.has_range = true;
        out.push_back(sd);
    };

    add("filter/sigma_g", "Шум гироскопа", "рад/√с", 0.0, 1.0);
    add("filter/sigma_a", "Шум акселерометра", "м/с²/√с", 0.0, 100.0);
    add("filter/sigma_bg", "Дрейф смещения гироскопа", "рад/с/√с", 0.0, 1e-2);
    add("filter/sigma_ba", "Дрейф смещения акселерометра", "м/с²/√с", 0.0, 1e-2);
    add("filter/sigma_pos", "Шум позиции СНС", "рад", 0.0, 1e-4);
    add("filter/sigma_h", "Шум высоты СНС", "м", 0.0, 100.0);
    add("filter/sigma_v", "Шум скорости СНС", "м/с", 0.0, 100.0);
    add("filter/sigma_hdg", "Шум курса СНС", "рад", 0.0, 1.0);
    add("filter/outage_start_s", "Начало окна без СНС", "с", 0.0, 1e5);
    add("filter/outage_end_s", "Конец окна без СНС", "с", 0.0, 1e5);

    add("tilt/gyro_off_rad_s", "Порог гироскопа для tilt-режима", "рад/с", 0.0, 1.0);
    add("tilt/settle_s", "Время успокоения после манёвра", "с", 0.0, 10.0);
    add("tilt/brake_body_accel", "Порог торможения (tanгаж/крен)", "м/с²", 0.0, 50.0);
    add("tilt/tau_fast_s", "Постоянная времени быстрого сведения", "с", 0.0, 10.0);
    add("tilt/tau_slow_s", "Постоянная времени медленного сведения", "с", 0.0, 100.0);
    add("tilt/fast_err_rad", "Порог быстрого сведения", "рад", 0.0, 1.0);
    add("tilt/level_acc_rad", "Порог «аксель смотрит в горизонт»", "рад", 0.0, 1.0);
    add("tilt/bias_gain", "Усиление интегральной оценки дрейфа", "1/с²", 0.0, 10.0);
    add("tilt/bias_max_rad_s", "Ограничение оценки дрейфа", "рад/с", 0.0, 1.0);
    add("tilt/divergence_deadband_rad", "Мёртвая зона расхождения", "рад", 0.0, 1.0);

    add("run/align_time_override_s", "Выставка (0 — из StartupNav.ini)", "с", 0.0, 1e5);
    add("run/align_fallback_s", "Выставка без StartupNav.ini", "с", 0.0, 1e5);

    (void)d;
    return out;
}

} // namespace nav