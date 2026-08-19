# Issue: Объединение конвейеров БИНС: выставка, счисление, фильтр Калмана, вывод

## Обзор

Полная переработка навигационного конвейера БИНС: объединение кода выставки (Vasya) и счисления с фильтром Калмана (Katya), исправление ошибок, реорганизация ввода/вывода и аудит единиц измерения.

## История коммитов

```
1391e4a Исправлены единицы: angle.dat→SnsSample (DEG_TO_RAD), Aligner.cpp (убрана лишняя конверсия PI/180)
53a66b8 Разделение вывода: result.txt + reference.txt + errors.txt, get_angle_start принимает параметры
c74c572 Удалён bins_alignment.h, StartupNav.ini: 4 строки (lon, lat, alt, time), fallback на gps.dat
```

---

## Детальное описание изменений

### 1. Удаление `bins_alignment.h` и переход на параметрическую выставку

**Удалено:** `src/navigation/bins_alignment.h` (187 строк) — дублировал логику выставки, не использовался.

**Изменения в `get_angle_start()`:**
- Функция больше не читает `StartupNav.ini` самостоятельно
- Принимает параметры: `lat_deg` (град), `h` (м), `iter` (сек)
- Координаты и время выставки передаются из `main.cpp`

**Изменения в `main.cpp`:**
- Добавлена функция `readStartupNav()` — чтение конфигурации
- `StartupNav.ini`: 4 строки (lon, lat, alt, time)
- Запасной вариант: если файл отсутствует — координаты из первого отсчёта `gps.dat`, время = 120 с

**Изменения в `aligner.h`:**
- Новая перегрузка `initialAlignment(lat_rad, lon_rad, alt, yaw, pitch, roll)`
- Координаты из конфига, скорости нулевые, углы из выставки

### 2. Разделение выходных файлов

**Было:** один файл `kalman15_line2.txt` (16 столбцов)

**Стало:** три файла:

#### `result.txt` — результат БИНС (10 столбцов)

Записывается при коррекции по СНС (раз в 200 отсчётов ИМУ = 1 Гц). Содержит скорректированное состояние БИНС с вычитанием оценки ошибок фильтра Калмана.

| Столбец | Единицы | Описание |
|---------|---------|----------|
| `time` | с | Время отсчёта (с начала счисления) |
| `lon` | град | Долгота (−180..+180) |
| `lat` | град | Широта (−90..+90) |
| `alt` | м | Высота над уровнем моря |
| `heading` | град | Курс (0..360 или −180..+180) |
| `pitch` | град | Тангаж (−90..+90) |
| `roll` | град | Крен (−180..+180) |
| `vn` | м/с | Скорость на север |
| `vh` | м/с | Вертикальная скорость (вверх положительна) |
| `ve` | м/с | Скорость на восток |

**Формула:** `result = NavState − x[Kalman]`, где `x` — вектор ошибок фильтра.

#### `reference.txt` — эталон СНС (10 столбцов)

Записывается при коррекции по СНС (раз в 200 отсчётов ИМУ = 1 Гц). Содержит данные из `gps.dat` + `angle.dat` (внешний эталон для фильтра).

| Столбец | Единицы | Описание |
|---------|---------|----------|
| `time` | с | Время отсчёта |
| `lon` | град | Долгота из `gps.dat` |
| `lat` | град | Широта из `gps.dat` |
| `alt` | м | Высота из `gps.dat` |
| `heading` | град | Курс из `angle.dat` |
| `pitch` | град | Тангаж из `angle.dat` |
| `roll` | град | Крен из `angle.dat` |
| `vn` | м/с | Скорость на север из `gps.dat` |
| `vh` | м/с | Вертикальная скорость из `gps.dat` |
| `ve` | м/с | Скорость на восток из `gps.dat` |

#### `errors.txt` — ошибки фильтра Калмана (16 столбцов)

Записывается при коррекции по СНС (раз в 200 отсчётов ИМУ = 1 Гц). Содержит текущую оценку вектора ошибок `x` **до** обнуления (т.е. до применения коррекции к состоянию).

| Столбец | Индекс | Единицы | Описание |
|---------|--------|---------|----------|
| `time` | — | с | Время отсчёта |
| `x0` | 0 | рад | Ошибка широты (δφ) |
| `x1` | 1 | рад | Ошибка долготы (δλ) |
| `x2` | 2 | м | Ошибка высоты (δh) |
| `x3` | 3 | м/с | Ошибка скорости на север (δVn) |
| `x4` | 4 | м/с | Ошибка вертикальной скорости (δVh) |
| `x5` | 5 | м/с | Ошибка скорости на восток (δVe) |
| `x6` | 6 | рад | Ошибка курса (δψ) |
| `x7` | 7 | рад | Ошибка тангажа (δθ) |
| `x8` | 8 | рад | Ошибка крена (δφ) |
| `x9` | 9 | м/с² | Смещение акселерометра по X (δba_x) |
| `x10` | 10 | м/с² | Смещение акселерометра по Y (δba_y) |
| `x11` | 11 | м/с² | Смещение акселерометра по Z (δba_z) |
| `x12` | 12 | рад/с | Смещение гироскопа по X (δbg_x) |
| `x13` | 13 | рад/с | Смещение гироскопа по Y (δbg_y) |
| `x14` | 14 | рад/с | Смещение гироскопа по Z (δbg_z) |

**Структуры данных:**
- `NavResult` — строка результата
- `NavReference` — строка эталона
- `NavLogger` — управление тремя выходными файлами

**Удалено:**
- `writeAlignment()` — запись файла выставки
- Столбцы `hdg_true` и `lat_bins` из вывода

**Изменения в `trajectory.h`:**
- `makeResult()` — формирование строки результата (БИНС − ошибки)
- `makeReference()` — формирование строки эталона (СНС)
- Обе функции вызываются в `step()` при коррекции по СНС

### 3. Исправление ошибок единиц измерения

#### 3a. `angle.dat` → `SnsSample`

**Файл:** `src/data_io/data_reader.cpp`

**Проблема:** `angle.dat` содержит углы в **градусах**, но `SnsReader::next()` записывал их без конвертации. `SnsSample` ожидает радианы.

**Исправление:**

```cpp
// Было:
out.heading = std::stod(ang[ANG_COL_YAW]);
out.roll = std::stod(ang[ANG_COL_ROLL]);
out.pitch = std::stod(ang[ANG_COL_PITCH]);

// Стало:
out.heading = std::stod(ang[ANG_COL_YAW]) * DEG_TO_RAD;
out.roll = std::stod(ang[ANG_COL_ROLL]) * DEG_TO_RAD;
out.pitch = std::stod(ang[ANG_COL_PITCH]) * DEG_TO_RAD;
```

**Влияние:** Инновация Калмана `z = bins - sns` вычитала радианы из градусов — фильтр корректировал ошибочно.

#### 3b. `Aligner.cpp` — единицы гироскопа

**Файл:** `src/navigation/Aligner.cpp`

**Проблема:** `imu.dat` содержит угловые скорости в **рад/с** (подтверждено README и `imu_processor.h`). `get_angle_start()` обрабатывал данные как град/с и умножал на `PI / 180.0` в 9 местах.

**Исправление:** Убраны все 9 умножений `(PI / 180.0)`:
- `Wx/Wy/Wz_arr.push_back()` — массивы фильтрации
- `Wx/Wy/Wz_sm +=` — суммы среднего
- `Wx/Wy/Wz_blc[]` — буферы блочной фильтрации

**Влияние:** Латентный баг — в текущем коде atan2 компенсировал масштаб, но при использовании W в sin/cos/интегрировании ошибка ~57.3x была бы катастрофической.

---

## Полный аудит цепочек единиц (все корректны)

| Цепочка | Статус |
|---------|--------|
| StartupNav.ini (град) → readStartupNav → main → get_angle_start(lat_deg) → initialAlignment(rad) → NavState | OK |
| imu.dat (м/с²) → `ins::accel()` → bodyToNav → интегрирование скоростей | OK |
| imu.dat (рад/с) → `ins::gyro()` → eulerRates → integrate → NavState.att | OK |
| angle.dat (град) → SnsReader (*DEG_TO_RAD) → SnsSample (рад) → makeReference (*RAD_TO_DEG) → reference.txt (град) | OK |
| gps.dat (град) → SnsReader (*DEG_TO_RAD) → SnsSample (рад) → ins::correct → Kalman | OK |
| NavState (рад) → makeResult (*RAD_TO_DEG) → result.txt (град) | OK |
| bins (рад) − measurement (рад) = инновация (рад) | OK |
| Q-матрица: sigma_g^2*dt (рад²), sigma_a^2*dt (м/с²)² | OK |
| R-матрица: sig_pos = 5/R_EARTH (рад), sig_ang = 1°*DEG_TO_RAD (рад) | OK |
| bodyToNavMatrix(hdg, pitch, roll) — рад | OK |
| normalize_angle — [-pi, pi] | OK |
| integrate() — трапеция рад/с → рад | OK |
| lat clamp ±pi/2, защита от полюса cos(lat) | OK |

---

## Единицы измерения (справочник)

| Расположение | Единицы |
|-------------|---------|
| `imu.dat` wx/wy/wz | рад/с |
| `imu.dat` ax/ay/az | м/с² |
| `gps.dat` lat/lon | градусы |
| `gps.dat` vn/vh/ve | м/с |
| `angle.dat` roll/pitch/yaw | градусы |
| `StartupNav.ini` lon/lat | градусы |
| `SnsSample` lat/lon/heading/roll/pitch | **радианы** |
| `NavState` att/lat/lon | рад |
| `NavState` V | м/с |
| `NavState` ba | м/с² |
| `NavState` bg | рад/с |
| `Kalman x[0..2]` | рад (ошибки координат) |
| `Kalman x[3..5]` | м/с (ошибки скорости) |
| `Kalman x[6..8]` | рад (ошибки ориентации) |
| `Kalman x[9..11]` | м/с² (дрейф акселя) |
| `Kalman x[12..14]` | рад/с (дрейф гиро) |
| `result.txt` / `reference.txt` | градусы для углов |
| `errors.txt` | как в векторе x |

---

## Затронутые файлы

| Файл | Изменения |
|------|-----------|
| `src/navigation/bins_alignment.h` | **Удалён** |
| `src/main.cpp` | readStartupNav(), запасной вариант, get_angle_start(params), initialAlignment(rad) |
| `src/navigation/Aligner.cpp` | get_angle_start() принимает параметры, удалены 9 конверсий PI/180 |
| `src/navigation/aligner.hpp` | Объявление get_angle_start() обновлено |
| `src/navigation/aligner.h` | Новая перегрузка initialAlignment(lat/lon/alt/yaw/pitch/roll) |
| `src/data_io/data_reader.cpp` | +DEG_TO_RAD для angle.dat |
| `src/data_io/data_writer.h` | NavResult, NavReference, NavLogger |
| `src/data_io/data_writer.cpp` | writeResult(), writeReference(), writeErrors() |
| `src/navigation/trajectory.h` | makeResult(), makeReference(), step() |
| `tools/plot_trajectory.py` | Чтение двух файлов (result + reference) |
| `README.md` | Обновлены форматы входных/выходных данных |

---

## Как проверить

1. Запустить программу с тестовыми данными в `data/raw/`
2. Убедиться что `result.txt`, `reference.txt`, `errors.txt` созданы
3. Координаты в `result.txt` и `reference.txt` в пределах маршрута
4. Углы в пределах ±180°
5. Ошибки фильтра `|x[i]|` малы относительно состояния
6. `Aligner.dat` создаётся (промежуточные данные выставки)
