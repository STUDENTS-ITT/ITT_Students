# `src/core/utils/` — базовые типы, константы, пути, INI

Служебный слой: псевдонимы типов, физические константы, поиск каталогов и
минимальный INI-ридер. Qt здесь нет.

---

## `utils/types.h`

Псевдонимы базовых типов линейной алгебры.

- `using Matrix = std::vector<double>;` — матрица, хранится **построчно**
  (row-major) в одномерном векторе.
- `using Vector = std::vector<double>;` — вектор (тот же тип).

Число столбцов матрицы всегда передаётся отдельным аргументом функций
(`math_lib/matrix_ops.h`), потому что из `size()` оно не выводится.

---

## `utils/constants.h`

Физические и модельные константы, используемые всем ядром. Исторически
константы переехали сюда из `math_lib/constants.hpp` (остался прокси-заголовок,
см. `math_lib.md`).

### Константы

| Имя | Значение | Смысл |
|---|---|---|
| `M_PI` | `3.14159265358979323846` | если не определён (страховка для `cmath`) |
| `PI` | `3.14159265358979323846` | число π |
| `U_EARTH` | `7.292115e-5` | угловая скорость вращения Земли, рад/с |
| `R_EARTH` | `6371e3` | средний радиус Земли, м |
| `G_EQ` | `9.780327` | ускорение свободного падения на экваторе, м/с² |
| `GRAVITY_CONSTANT` | `0.0053024` | коэффициент формулы Клеро |
| `NAV_R` | `R_EARTH` | алиас для обратной совместимости (`aligner.hpp`) |
| `IMU_RATE_HZ` | `200.0` | частота ИМУ, Гц |
| `IMU_PERIOD` | `1.0 / IMU_RATE_HZ` | шаг ИМУ, с (0.005) |
| `SNS_RATE_HZ` | `10.0` | частота обновления СНС, Гц |
| `DEG_TO_RAD` | `PI / 180.0` | градусы → радианы |
| `RAD_TO_DEG` | `180.0 / PI` | радианы → градусы |

Все — `constexpr double`, кроме `M_PI` (макрос).

### Функции

- `inline double normalGravity(double lat, double h)` — нормальная сила тяжести
  по формуле Клеро с поправкой на высоту:
  `g0 = G_EQ·(1 + GRAVITY_CONSTANT·sin²lat)`, затем масштаб
  `(R_EARTH/(R_EARTH+h))²`. Аргументы: `lat` — широта в **радианах**,
  `h` — высота, м. Используется в счислении и генераторе данных.

---

## `utils/paths.h`

Поиск каталогов независимо от того, откуда запущена программа. Всё в
пространстве имён `utils`.

### Функции

- `inline std::filesystem::path exeDir(const char *argv0)` — каталог исполняемого
  файла. Возвращает `{}` при пустом `argv0` или ошибке.
- `inline std::filesystem::path dataDir(const char *argv0, const std::string &probe)` —
  каталог данных. Логика: если `probe` (обычно `imu.dat`) существует в текущем
  рабочем каталоге — возвращается `{}` (значит, работать из cwd); иначе, если
  `probe` есть рядом с exe — возвращается `exeDir`; иначе `{}`.
  В текущем коде ядра вызовов `dataDir` нет (точка входа использует каталог по
  умолчанию — см. `console.md`); функция оставлена как публичный помощник.
- `inline std::filesystem::path toolsDir(const char *argv0)` — каталог `tools/`
  для `result.txt`/`reference.txt`. Перебирает по порядку:
  `cwd/tools`, `exeDir/../tools`, `exeDir/tools`; первый существующий
  возвращает (`weakly_canonical`). Если ни одного нет — создаёт `cwd/tools` и
  возвращает его. Используется консольным `main`.

---

## `utils/ini_parser.h` + `utils/ini_parser.cpp`

Минимальный INI-ридер параметров фильтра (в ядре, без Qt). Формат совместим с
`QSettings` INI:

```
; комментарий
# комментарий
[section]
key = value
```

Ключи хранятся с полным именем `section/key` в нижнем регистре, поэтому
обращение к параметру не зависит от фактического места объявления и регистра.

### Класс `utils::IniFile`

Хранилище «ключ → значение».

Поля (приватные):
- `std::map<std::string, std::string> values_` — значения по нормализованному
  ключу.
- `std::vector<std::string> order_` — ключи в порядке появления (для GUI).

Методы:
- `bool load(const std::string &path)` — разбор из файла. `false`, если файл не
  открылся.
- `void loadFromString(const std::string &text)` — разбор из строки (тесты,
  снимки параметров). Очищает прежнее содержимое.
- `double value(const std::string &key, double default_value) const` — значение
  как `double`; при отсутствии ключа или нечисловом значении — `default_value`.
- `std::string text(const std::string &key, const std::string &default_value = "") const` —
  значение как строка.
- `bool has(const std::string &key) const` — есть ли ключ.
- `const std::vector<std::string> &keys() const` — все ключи в порядке появления.
- `std::vector<std::string> sections() const` — список секций (в порядке
  появления, без повторов).
- `static std::string normalize(const std::string &key)` (приватный) — `trim` +
  нижний регистр.

### Внутренние помощники (анонимное пространство имён в `.cpp`)

- `std::string trim(const std::string &s)` — обрезка пробелов/табов по краям.
- `std::string lower(const std::string &s)` — нижний регистр.
- `bool toDouble(const std::string &s, double &out)` — разбор `double` без
  исключений; `false`, если строка не является полностью числом.

### Логика разбора (`loadFromString`)

- Пустые строки пропускаются; строки, начинающиеся с `;` или `#`, — комментарии.
- `[name]` задаёт текущую секцию.
- `key = value` собирается в `full = "section/key"` (или просто `key` вне
  секции), нормализуется, кладётся в `values_`; при первом появлении ключ
  добавляется в `order_`.

Секции, встречающиеся в проекте: `[filter]`, `[tilt]`, `[run]`
(см. `navigation_settings.md`).
