// filter_settings.h — Настраиваемые параметры фильтра, вынесенные из заголовков.
//
// Значения по умолчанию совпадают с ранее захардкоженными константами
// (ins::KalmanConfig и nav::TiltManeuverPolicy), поэтому запуск без
// settings.ini даёт побитово тот же результат, что и до выноса параметров.
//
// Загрузка: settings.ini (формат QSettings-совместимого INI):
//   [filter]
//   sigma_g = 2.12428e-02
//   [tilt]
//   gyro_off_rad_s = 0.08
//   [run]
//   align_time_s = 120

#pragma once

#include <ostream>
#include <string>

#include "../ins/ins_filter.h"
#include "../navigation/maneuver_tilt.h"
#include "../utils/ini_parser.h"

namespace nav
{

// Полный набор настраиваемых параметров прогона.
struct FilterSettings
{
    // Шумы и окна фильтра Калмана (копия ins::KalmanConfig).
    ins::KalmanConfig filter;

    // Пороги сведения тангажа/крена после манёвра.
    TiltManeuverPolicy tilt;

    // Параметры прогона, не влияющие на математику такта.
    double align_time_override_s = 0.0;  // 0 — брать время выставки из StartupNav.ini
    double align_fallback_s = 120.0;     // выставка, если StartupNav.ini отсутствует
};

// Применить настройки к глобальной конфигурации фильтра.
// После вызова ins::kalman_cfg содержит значения из settings.
void applySettings(const FilterSettings &s);

// Загрузить настройки из INI-файла. Отсутствующие ключи сохраняют значения
// по умолчанию. Возвращает false, если файл не открылся.
bool loadFilterSettings(const std::string &path, FilterSettings &s);

// Формат файла настроек: true — секции ([filter], [tilt], [run]),
// false — старый плоский params.ini без секций. Нечитаемый файл считается
// секционным (парсер сам сообщит об ошибке).
bool settingsFileHasSections(const std::string &path);

// Применить старый плоский params.ini (ключи без секций) к глобальной
// конфигурации фильтра. Вызывается после applySettings(), иначе значения
// по умолчанию затрут плоские ключи.
void applyLegacyParamsFile(const std::string &path, std::ostream *log = nullptr);

// Сформировать INI-текст с текущими значениями (для записи из GUI).
std::string dumpFilterSettings(const FilterSettings &s);

// Описание параметра для редактора в GUI.
struct SettingDescriptor
{
    std::string key;     // ключ в INI
    std::string label;   // подпись в GUI
    std::string units;   // единицы измерения
    double min_value = 0.0;
    double max_value = 0.0;
    bool has_range = false;
};

// Полный список параметров: ключ, подпись, единицы, диапазон.
std::vector<SettingDescriptor> filterSettingDescriptors();

} // namespace nav