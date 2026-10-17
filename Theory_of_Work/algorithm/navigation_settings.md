# `src/core/navigation/filter_settings.*` — настраиваемые параметры

Вынос параметров фильтра и tilt-политики из заголовков в `settings.ini`.
Значения по умолчанию совпадают с прежними захардкоженными константами, поэтому
запуск без файла настроек даёт побитово тот же результат.

Формат файла — совместимый с `QSettings` INI:

```ini
[filter]
sigma_g = 2.12428e-02
[tilt]
gyro_off_rad_s = 0.005
[run]
align_time_s = 120
```

Пространство имён `nav`.

---

## `navigation/filter_settings.h`

### `struct FilterSettings`

Полный набор настраиваемых параметров прогона:
- `ins::KalmanConfig filter` — шумы и окна фильтра Калмана;
- `TiltManeuverPolicy tilt` — пороги сведения тангажа/крена;
- `double align_time_override_s = 0.0` — 0 — брать время выставки из
  `StartupNav.ini`;
- `double align_fallback_s = 120.0` — выставка, если `StartupNav.ini` нет.

### Функции

- `void applySettings(const FilterSettings &s)` — применяет настройки к
  глобальным `ins::kalman_cfg` и `tiltPolicy()`.
- `bool loadFilterSettings(const std::string &path, FilterSettings &s)` —
  загрузка из INI-файла (отсутствующие ключи сохраняют значения по умолчанию;
  `false`, если файл не открылся).
- `bool parseFilterSettings(const std::string &iniText, FilterSettings &s)` —
  разбор из строки (снимок параметров истории прогонов).
- `bool settingsFileHasSections(const std::string &path)` — формат файла: `true`
  — с секциями, `false` — старый плоский `params.ini`. Нечитаемый файл считается
  секционным.
- `void applyLegacyParamsFile(const std::string &path, std::ostream *log = nullptr)` —
  применяет старый плоский `params.ini` (ключи без секций) к глобальной
  конфигурации; вызывается **после** `applySettings()`, иначе значения по
  умолчанию затрут плоские ключи.
- `std::string dumpFilterSettings(const FilterSettings &s)` — формирует INI-текст
  с текущими значениями (для записи из GUI); рядом с каждым ключом пишется
  комментарий со значением по умолчанию.

### `struct SettingDescriptor`

Описание параметра для редактора в GUI:
- `std::string key` — ключ в INI;
- `std::string label` — подпись;
- `std::string units` — единицы;
- `double min_value`, `max_value`; `bool has_range`.

### Функция

- `std::vector<SettingDescriptor> filterSettingDescriptors()` — полный список
  параметров с подписями, единицами и диапазонами (используется GUI).

---

## `navigation/filter_settings.cpp`

### Вспомогательное (анонимное пространство имён)

- `struct FieldBinding { const char *key; double *field; }` — связь «ключ INI →
  поле структуры».
- `std::vector<FieldBinding> filterBindings(ins::KalmanConfig &c)` — привязки
  фильтра: `filter/sigma_g`, `sigma_a`, `sigma_bg`, `sigma_ba`, `sigma_pos`,
  `sigma_h`, `sigma_v`, `sigma_hdg`, `outage_start_s`, `outage_end_s`,
  `gnss_guard_on`, `gnss_vel_plaus`, `gnss_acc_bound`,
  `gnss_silence_timeout_s`, `gnss_guard_log`.
- `std::vector<FieldBinding> tiltBindings(TiltManeuverPolicy &p)` — привязки
  tilt: все поля `TiltManeuverPolicy` под ключами `tilt/...`.
- `bool applyIni(const utils::IniFile &ini, FilterSettings &s)` — применяет
  присутствующие ключи (то, чего нет в файле, не трогает). Используется и
  файлом, и строковым снимком. Также читает `run/align_time_override_s` и
  `run/align_fallback_s`.
- `std::string fmt(double v)` — формат `double` с `precision(17)`.
- `std::string keyInGroup(const char *key)` — имя ключа без префикса группы
  (для записи внутри секции).

### Функции с внешней связностью

- `TiltManeuverPolicy &tiltPolicy()` — глобальная политика (объявлена в
  `maneuver_tilt.h`; здесь определение со `static` экземпляром).
- `void applySettings(const FilterSettings &s)` — `ins::kalman_cfg = s.filter`;
  `tiltPolicy() = s.tilt`.
- `bool loadFilterSettings(const std::string &path, FilterSettings &s)` — через
  `utils::IniFile::load` + `applyIni`.
- `bool parseFilterSettings(const std::string &iniText, FilterSettings &s)` —
  через `utils::IniFile::loadFromString` + `applyIni`.
- `bool settingsFileHasSections(const std::string &path)` — первая содержательная
  строка (не пустая, не `;`/`#`): если начинается с `[` — секции есть.
- `void applyLegacyParamsFile(const std::string &path, std::ostream *log)` —
  читает старый формат вручную: таблица `Entry{name, field}` из 15 ключей
  (в том числе `sigma_ang` → `sig_hdg`), парсит `key = value` через
  `std::stod`; мусор в значении игнорируется; в `log` пишет итоговый набор σ.
- `std::string dumpFilterSettings(const FilterSettings &s)` — пишет заголовок,
  затем секции `[filter]` и `[tilt]` (по каждому ключу — комментарий со
  значением по умолчанию и строка `key = value`) и секцию `[run]`.
- `std::vector<SettingDescriptor> filterSettingDescriptors()` — таблица
  дескрипторов для GUI: все `filter/*`, `tilt/*`, `run/*` с подписями и
  диапазонами.

### Совместимость

Старый `params.ini` (плоские ключи) поддерживается: `runSimulation` определяет
формат через `settingsFileHasSections`, загружает секционный файл обычным
образом, а при legacy-формате дополнительно вызывает `applyLegacyParamsFile`.
