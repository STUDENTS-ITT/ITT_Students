# Полный отчёт по программе БИНС с фильтром Калмана

Подробное описание всех модулей, структур данных, алгоритмов, формул и логики работы программы бортовой инерциальной навигационной системы.

---

## Содержание

1. [Общее описание системы](#1-общее-описание-системы)
2. [Структура файлов](#2-структура-файлов)
3. [Входные и выходные данные](#3-входные-и-выходные-данные)
4. [Точка входа main.cpp](#4-точка-входа-maincpp)
5. [Константы и физические модели](#5-константы-и-физические-модели)
6. [Структуры данных](#6-структуры-данных)
7. [Модуль ввода данных (data_reader)](#7-модуль-ввода-данных-data_reader)
8. [Модуль вывода данных (data_writer)](#8-модуль-вывода-данных-data_writer)
9. [Обработка ИМУ (imu_processor)](#9-обработка-иму-imu_processor)
10. [Обработка СНС (gps_processor)](#10-обработка-снс-gps_processor)
11. [Математическая библиотека](#11-математическая-библиотека)
12. [Вычисление ориентации (attitude_calc)](#12-вычисление-ориентации-attitude_calc)
13. [Счисление позиции (position_calc)](#13-счисление-позиции-position_calc)
14. [Автономная выставка (aligner)](#14-автономная-выставка-aligner)
15. [Фильтр Калмана (ins_filter)](#15-фильтр-калмана-ins_filter)
16. [Основной цикл счисления (trajectory)](#16-основной-цикл-счисления-trajectory)
17. [Полный алгоритм работы](#17-полный-алгоритм-работы)
18. [Матрицы фильтра Калмана](#18-матрицы-фильтра-калмана)
19. [Настройка параметров](#19-настройка-параметров)

---

## 1. Общее описание системы

### 1.1. Назначение

Программа реализует бортовую инерциальную навигационную систему (БИНС) с фильтром Калмана. Система счисляет траекторию движения по данным инерциального измерительного блока (ИМУ) и корректирует результаты по данным глобальной навигационной спутниковой системы (GNSS/СНС).

### 1.2. Принцип работы

```
ИМУ (200 Гц) → БИНС (интегрирование) → приближённое решение
                        ↓
              Фильтр Калмана (коррекция)
                        ↑
GPS (10 Гц) + angle.dat → СНС (эталон)
```

1. БИНС интегрирует показания ИМУ (200 Гц) → координаты, скорости, углы
2. Фильтр Калмана 15-го порядка оценивает **ошибки** этих величин
3. По расхождению с данными СНС (~10 Гц) ошибки корректируются
4. Результат записывается в файл (200 Гц)

### 1.3. Системы координат

| СК | Описание | Оси |
|----|----------|-----|
| **СК тела** (body) | Связана с корпусом БИНС | X=вперёд, Y=вверх, Z=вправо |
| **Навигационная СК** (nav) | Связана с Землёй | N=север, U=вверх, E=восток |

### 1.4. Единицы измерения

| Величина | В файлах | Внутри программы |
|----------|----------|------------------|
| Углы координат (lat, lon) | градусы | радианы |
| Высота | метры | метры |
| Скорости | м/с | м/с |
| Углы ориентации (heading, pitch, roll) | градусы | радианы |
| Гироскоп | рад/с | рад/с |
| Акселерометр | м/с² | м/с² |

---

## 2. Структура файлов

```
src/
├── main.cpp                          — точка входа
├── data_io/
│   ├── data_reader.h                 — класс чтения imu.dat, gps.dat, angle.dat
│   ├── data_reader.cpp               — реализация чтения файлов
│   ├── data_writer.h                 — класс записи result.txt, reference.txt, errors.txt
│   ├── data_writer.cpp               — реализация записи файлов
│   └── io_error.h                    — сообщения об ошибках ввода/вывода
├── ins/
│   ├── imu_processor.h               — разбор строк imu.dat
│   ├── ins_filter.h                  — фильтр Калмана 15-го порядка
│   └── attitude_calc.h               — расчёт углов ориентации
├── math_lib/
│   ├── matrix_ops.h                  — операции с матрицами
│   ├── transformations.h             — переходы между СК
│   ├── interpolation.h               — метод трапеций
│   └── constants.hpp                 — константы (выставка)
├── navigation/
│   ├── trajectory.h                  — основной цикл счисления (NavState, step)
│   ├── position_calc.h               — интегрирование координат и скоростей
│   ├── gps_processor.h               — структура SnsSample
│   ├── aligner.h                     — начальное состояние (initialAlignment)
│   └── aligner.hpp                   — автономная выставка (Median + EMA)
└── utils/
    ├── constants.h                   — физические константы
    ├── types.h                       — типы Vector, Matrix
    └── paths.h                       — пути к файлам
```

---

## 3. Входные и выходные данные

### 3.1. Входные файлы

**imu.dat** (обязательный, 200 Гц):

| Столбец | Индекс | Единицы | Описание |
|---------|--------|---------|----------|
| time_s | 0 | с | Время |
| timestamp_ns | 1 | нс | Временная метка |
| wx | 2 | рад/с | Угловая скорость гироскопа X |
| wy | 3 | рад/с | Угловая скорость гироскопа Y |
| wz | 4 | рад/с | Угловая скорость гироскопа Z |
| ax | 5 | м/с² | Ускорение акселерометра X |
| ay | 6 | м/с² | Ускорение акселерометра Y |
| az | 7 | м/с² | Ускорение акселерометра Z |

**gps.dat** (обязательный, ~10 Гц):

| Столбец | Индекс | Единицы | Описание |
|---------|--------|---------|----------|
| time_s | 0 | с | Время |
| timestamp_ns | 1 | нс | Временная метка |
| latitude | 2 | **градусы** | Широта |
| longitude | 3 | **градусы** | Долгота |
| altitude | 4 | м | Высота |
| vx | 5 | м/с | Скорость на север |
| vy | 6 | м/с | Вертикальная скорость |
| vz | 7 | м/с | Скорость на восток |

**angle.dat** (опциональный, ~10 Гц):

| Столбец | Индекс | Единицы | Описание |
|---------|--------|---------|----------|
| time_s | 0 | с | Время |
| timestamp_ns | 1 | нс | Временная метка |
| roll | 2 | **градусы** | Крен |
| pitch | 3 | **градусы** | Тангаж |
| yaw | 4 | **градусы** | Курс |

**StartupNav.ini** (опциональный):

| Строка | Единицы | Описание |
|--------|---------|----------|
| 1 | градусы | Долгота |
| 2 | градусы | Широта |
| 3 | метры | Высота |
| 4 | секунды | Время выставки |

### 3.2. Выходные файлы

**result.txt** (200 Гц, каждый такт ИМУ):

| Столбец | Единицы | Описание |
|---------|---------|----------|
| time | с | Время |
| lon | градусы | Долгота (БИНС − ошибка Калмана) |
| lat | градусы | Широта (БИНС − ошибка Калмана) |
| alt | м | Высота (БИНС − ошибка Калмана) |
| heading | градусы | Курс (БИНС − ошибка Калмана) |
| pitch | градусы | Тангаж (БИНС − ошибка Калмана) |
| roll | градусы | Крен (БИНС − ошибка Калмана) |
| vn | м/с | Скорость N (БИНС − ошибка Калмана) |
| vh | м/с | Скорость H (БИНС − ошибка Калмана) |
| ve | м/с | Скорость E (БИНС − ошибка Калмана) |

**reference.txt** (~10 Гц, только при обновлении GPS):

| Столбец | Единицы | Описание |
|---------|---------|----------|
| time | с | Время |
| lon | градусы | Долгота СНС |
| lat | градусы | Широта СНС |
| alt | м | Высота СНС |
| heading | градусы | Курс СНС |
| pitch | градусы | Тангаж СНС |
| roll | градусы | Крен СНС |
| vn | м/с | Скорость N СНС |
| vh | м/с | Скорость H СНС |
| ve | м/с | Скорость E СНС |

**errors.txt** (~10 Гц, только при обновлении GPS):

| Столбец | Описание |
|---------|----------|
| time | Время |
| x0..x14 | 15 компонент вектора ошибок фильтра Калмана |

---

## 4. Точка входа main.cpp

### 4.1. Конфигурация

```cpp
const std::string data_dir = "../data/raw";
const std::string imu_file = data_dir + "/imu.dat";
const std::string gps_file = data_dir + "/gps.dat";
const std::string angle_file = data_dir + "/angle.dat";
const std::string startup_file = data_dir + "/StartupNav.ini";
const std::string result_file = "result.txt";
const std::string reference_file = "reference.txt";
const std::string err_file = "errors.txt";
```

### 4.2. Алгоритм main()

```
Этап 1: Чтение конфигурации (StartupNav.ini)
    │
    ├── Файл существует → lon_deg, lat_deg, alt, align_time
    │
    └── Файл не существует → fallback:
         ├── Чтение первого отсчёта gps.dat
         ├── lon = gps[lon] * RAD_TO_DEG  (обратное преобразование)
         ├── lat = gps[lat] * RAD_TO_DEG
         └── align_time = 120.0 секунд

Этап 2: Автономная выставка (Median + EMA фильтры)
    │
    ├── get_angle_start(&Yaw, &Pitch, &Roll, imu_file, lat, alt, time)
    │   (читает imu.dat, вычисляет начальные углы)
    └── Возвращает: Yaw_0, Pitch_0, Roll_0 (радианы)

Этап 3: Формирование начального состояния
    │
    ├── initialAlignment(lat_rad, lon_rad, alt, Yaw_0, Pitch_0, Roll_0)
    │   ├── lat = lat_rad
    │   ├── lon = lon_rad
    │   ├── alt = alt
    │   ├── V = {0, 0, 0}
    │   ├── att = {heading, pitch, roll}
    │   └── P = initialCovariance()  (15×15 диагональная)
    └── NavState state

Этап 4: Открытие потоков данных
    │
    ├── ImuReader::open(imu_file)
    ├── SnsReader::open(gps_file, angle_file)
    └── NavLogger::open(result, reference, errors)

Этап 5: Основной цикл счисления
    │
    while (imu.next(row)):
        │
        ├── Синхронизация gps.dat:
        │   while (last_ref.time <= imu_time):
        │       sns.next(last_ref)
        │
        ├── Флаг коррекции: do_correction = (last_ref.time != prev_gps_time)
        │
        └── step(row, ref, state, log, do_correction)
            (интегрирование + коррекция + запись)

Этап 6: Завершение
    │
    ├── imu.close()
    ├── sns.close()
    └── log.close()
```

---

## 5. Константы и физические модели

### 5.1. Константы Земли (`constants.h`)

| Константа | Значение | Описание |
|-----------|----------|----------|
| PI | 3.141592653589793 | Число π |
| U_EARTH | 7.292115e-5 рад/с | Угловая скорость вращения Земли |
| R_EARTH | 6371000 м | Средний радиус Земли |
| G_EQ | 9.780327 м/с² | Ускорение свободного падения на экваторе |
| GRAVITY_CONSTANT | 0.0053024 | Коэффициент формулы Клеро |
| IMU_RATE_HZ | 200.0 Гц | Частота ИМУ |
| IMU_PERIOD | 0.005 с | Период дискретизации ИМУ |
| DEG_TO_RAD | π/180 | Перевод градусов в радианы |
| RAD_TO_DEG | 180/π | Перевод радиан в градусы |

### 5.2. Нормальная сила тяжести (формула Клеро)

```
g(φ, h) = g₀ · (1 + K · sin²(φ)) · (R / (R + h))²

Где:
  g₀ = G_EQ = 9.780327 м/с²
  K = GRAVITY_CONSTANT = 0.0053024
  R = R_EARTH = 6371000 м
  φ = широта (рад)
  h = высота (м)
```

Реализация: `normalGravity(lat, alt)` в `constants.h`.

---

## 6. Структуры данных

### 6.1. Базовые типы (`types.h`)

```cpp
using Vector = std::vector<double>;     // одномерный массив
using Matrix = std::vector<double>;     // двумерный массив (построчно)
```

Матрица хранится в одномерном векторе. Доступ: `A[i * cols + j]`.

### 6.2. NavState — полное состояние навигации (`trajectory.h`)

```cpp
struct NavState
{
    // Координаты и ориентация (интегрированные по ИМУ).
    double lat = 0;              // широта, рад
    double lon = 0;              // долгота, рад
    double alt = 0;              // высота, м
    ins::Attitude att;           // {heading, pitch, roll}, рад
    Vector V = Vector(3, 0.0);  // [Vn, Vh, Ve], м/с

    // Производные на предыдущем такте (для метода трапеций).
    Vector V_dot_prev = Vector(3, 0.0);
    double lat_dot_prev = 0;
    double lon_dot_prev = 0;
    double alt_dot_prev = 0;
    ins::AttitudeRates rates_prev;

    double time_prev = 0;

    // Смещения датчиков (корректируются фильтром Калмана).
    Vector ba = Vector(3, 0.0);  // смещение акселерометра [X,Y,Z]
    Vector bg = Vector(3, 0.0);  // смещение гироскопа [X,Y,Z]

    // Фильтр Калмана.
    Vector x = Vector(15, 0.0);          // вектор ошибок (15×1)
    Matrix P = Matrix(225, 0.0);         // ковариационная матрица (15×15)
};
```

### 6.3. Attitude — углы ориентации (`attitude_calc.h`)

```cpp
struct Attitude
{
    double heading = 0;  // курс ψ, рад
    double pitch = 0;    // тангаж θ, рад
    double roll = 0;     // крен φ, рад
};
```

### 6.4. AttitudeRates — производные углов ориентации (`attitude_calc.h`)

```cpp
struct AttitudeRates
{
    double heading_dot = 0;  // ψ̇, рад/с
    double pitch_dot = 0;    // θ̇, рад/с
    double roll_dot = 0;     // φ̇, рад/с
};
```

### 6.5. Position — результат интегрирования координат (`position_calc.h`)

```cpp
struct Position
{
    double lat = 0;      // широта, рад
    double lon = 0;      // долгота, рад
    double alt = 0;      // высота, м
    double lat_dot = 0;  // φ̇, рад/с
    double lon_dot = 0;  // λ̇, рад/с
};
```

### 6.6. SnsSample — один отсчёт эталона СНС (`gps_processor.h`)

```cpp
struct SnsSample
{
    double time = 0;      // время, с
    double lat = 0;       // широта, рад
    double lon = 0;       // долгота, рад
    double alt = 0;       // высота, м
    double vn = 0;        // скорость на север, м/с
    double vh = 0;        // вертикальная скорость, м/с
    double ve = 0;        // скорость на восток, м/с
    double heading = 0;   // курс, рад
    double roll = 0;      // крен, рад
    double pitch = 0;     // тангаж, рад
};
```

### 6.7. NavResult / NavReference — строки для записи (`data_writer.h`)

```cpp
struct NavResult
{
    double time = 0;
    double lon = 0;       // град
    double lat = 0;       // град
    double alt = 0;       // м
    double heading = 0;   // град
    double pitch = 0;     // град
    double roll = 0;      // град
    double vn = 0;        // м/с
    double vh = 0;        // м/с
    double ve = 0;        // м/с
};
```

---

## 7. Модуль ввода данных (data_reader)

### 7.1. ImuReader — чтение imu.dat

```cpp
class ImuReader
{
    bool open(const std::string &path);   // открытие + пропуск заголовка
    bool next(std::vector<double> &row);  // чтение следующей строки
    void close();
};
```

**Формат строки:** `time_s  timestamp_ns  wx  wy  wz  ax  ay  az`

### 7.2. SnsReader — чтение gps.dat + angle.dat

```cpp
class SnsReader
{
    bool open(const std::string &gps_path, const std::string &angle_path = "");
    bool next(nav::SnsSample &out);  // чтение следующего отсчёта
    void close();
};
```

**Логика чтения:**

```
Если angle.dat задан и открыт:
    │
    └── Совместное чтение gps.dat и angle.dat (построчно):
        ├── gps → time, lat(DEG→RAD), lon(DEG→RAD), alt, vn, vh, ve
        └── angle → roll(DEG→RAD), pitch(DEG→RAD), yaw(DEG→RAD)

Иначе:
    │
    └── Чтение только gps.dat:
        ├── gps → time, lat(DEG→RAD), lon(DEG→RAD), alt, vn, vh, ve
        └── heading=0, roll=0, pitch=0
```

---

## 8. Модуль вывода данных (data_writer)

### 8.1. NavLogger

```cpp
class NavLogger
{
    bool open(const std::string &result_path,
              const std::string &reference_path,
              const std::string &error_path);
    void writeHeader();                          // заголовки столбцов
    void writeResult(const NavResult &r);        // 200 Гц
    void writeReference(const NavReference &r);  // ~10 Гц
    void writeErrors(double time, const Vector &x);  // ~10 Гц
    void close();
};
```

**Формат записи:**
- `result.txt`: 10 столбцов, разделитель — табуляция
- `reference.txt`: 10 столбцов, разделитель — табуляция
- `errors.txt`: 16 столбцов (time + x0..x14), разделитель — табуляция

---

## 9. Обработка ИМУ (imu_processor)

### 9.1. Доступ к данным

```cpp
constexpr int IMU_COL_TIME = 0;
constexpr int IMU_COL_GYRO = 2;
constexpr int IMU_COL_ACCEL = 5;
```

### 9.2. Извлечение компонентов

```cpp
// Ускорение с вычитанием смещения: f_body = f_raw - ba
inline Vector accel(const std::vector<double> &row, const Vector &ba)
{
    return {row[5] - ba[0], row[6] - ba[1], row[7] - ba[2]};
}

// Угловая скорость с вычитанием смещения: ω_body = ω_raw - bg
inline Vector gyro(const std::vector<double> &row, const Vector &bg)
{
    return {row[2] - bg[0], row[3] - bg[1], row[4] - bg[2]};
}
```

---

## 10. Обработка СНС (gps_processor)

### 10.1. SnsSample — упаковка для фильтра

```cpp
Vector measurement() const
{
    return {lat, lon, alt, vn, vh, ve, heading, pitch, roll};
}
```

---

## 11. Математическая библиотека

### 11.1. Операции с матрицами (`matrix_ops.h`)

| Функция | Формула | Описание |
|---------|---------|----------|
| `at(A, i, j, cols)` | A[i·cols + j] | Доступ к элементу |
| `multiply_m(A, v, cols)` | r[i] = Σⱼ A[i,j]·v[j] | Матрица × вектор |
| `multiply_matrix(A, B, ca, cb)` | C[i,j] = Σₖ A[i,k]·B[k,j] | Матрица × матрица |
| `transpose_m(A, cols)` | A^T[j,i] = A[i,j] | Транспонирование |
| `matrix_sum(A, B, cols)` | C[i,j] = A[i,j] + B[i,j] | Сложение |
| `matrix_diff(A, B, cols)` | C[i,j] = A[i,j] - B[i,j] | Вычитание |
| `return_matrix(A, cols)` | A⁻¹ | Обратная матрица (Гаусс-Жордан) |
| `E_matrix(n)` | I (n×n) | Единичная матрица |
| `H_matrix(m, n)` | H (m×n) | Матрица наблюдения |
| `vector_product(a, b)` | a × b | Векторное произведение |
| `vector_diff(a, b)` | a - b | Разность векторов |
| `vector_sum(a, b)` | a + b | Сумма векторов |

### 11.2. Обратная матрица (метод Гаусса-Жордана)

```
Вход: A (n×n)
Выход: A⁻¹ (n×n)

1. Формирование расширенной матрицы [A | I]
2. Для каждой строки i:
   a. Выбор ведущего элемента (max |M[k,i]|)
   b. Обмен строк
   c. Нормализация строки: M[i,:] /= pivot
   d. Исключение столбца: M[k,:] -= factor · M[i,:] для k ≠ i
3. Извлечение правой половины → A⁻¹
```

**Особенность:** если pivot < 1e-15 (сингулярная матрица), возвращается единичная матрица.

### 11.3. Переходы между СК (`transformations.h`)

**Нормализация угла:**
```cpp
inline double normalize_angle(double a)
{
    while (a > PI)  a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    return a;
}
```

**Матрица перехода СК тела → навигационная СК (Эйлер Z-Y-X):**

```
C = bodyToNavMatrix(ψ, θ, φ)

C = {{cos(θ)cos(ψ),  -cos(φ)cos(ψ)sin(θ)+sin(φ)sin(ψ),  sin(φ)cos(ψ)sin(θ)+cos(φ)sin(ψ)},
     {sin(θ),         cos(φ)cos(θ),                          -sin(φ)cos(θ)},
     {-cos(θ)sin(ψ),  cos(φ)sin(ψ)sin(θ)+sin(φ)cos(ψ),  -sin(φ)sin(ψ)sin(θ)+cos(φ)cos(ψ)}}
```

**Преобразование векторов:**
```cpp
v_nav = C · v_body          // bodyToNav(C, v_body)
v_body = C^T · v_nav        // navToBody(C, v_nav)
```

### 11.4. Метод трапеций (`interpolation.h`)

**Одномерная интеграция:**
```
V(n) = V(n-1) + (V̇(n) + V̇(n-1))/2 · Δt
```

**Векторная интеграция:** поэлементно для каждого компонента.

---

## 12. Вычисление ориентации (attitude_calc)

### 12.1. Относительная угловая скорость

У гироскопа измеряется **абсолютная** угловая скорость (относительно инерциальной СК). Для счисления в навигационной СК нужно вычесть вращение Земли:

```
ω_отн = ω_гиро - (C_б^н)^T · ω_Земля
```

Реализация: `relativeRate(w_abs, w_nav, C)`

### 12.2. Угловая скорость навигационной СК

```
ω_N = U_EARTH · cos(φ) + V_E / R_h
ω_H = U_EARTH · sin(φ) + V_E · tan(φ) / R_h
ω_E = -V_N / R_h

Где:
  R_h = R_EARTH + alt
  φ = широта
  V_N, V_E = скорости
```

Реализация: `navAngularRate(lat, V, Rh)`

### 12.3. Скорости Эйлера

Преобразование угловой скорости тела в производные углов Эйлера:

```
ψ̇ = (ω_y · cos(φ) - ω_z · sin(φ)) / cos(θ)
θ̇ = ω_y · sin(φ) + ω_z · cos(φ)
φ̇ = ω_x - tan(θ) · (ω_y · cos(φ) - ω_z · sin(φ))
```

Реализация: `eulerRates(w_rel, pitch, roll)`

**Сингулярность:** при θ = ±90° → `cos(θ)` заменяется на `fmax(fabs(cos(θ)), 1e-10)`.

### 12.4. Интегрирование углов

```
heading(n) = normalize_angle(heading(n-1) + (ψ̇(n) + ψ̇(n-1))/2 · Δt)
pitch(n)   = normalize_angle(pitch(n-1)   + (θ̇(n) + θ̇(n-1))/2 · Δt)
roll(n)    = normalize_angle(roll(n-1)    + (φ̇(n) + φ̇(n-1))/2 · Δt)
```

Реализация: `integrate(att, rates, rates_prev, dt)`

---

## 13. Счисление позиции (position_calc)

### 13.1. Вредные ускорения

На приборе.gravity не действует. Нужно учесть:

```
a_вред = a_Кориолис + a_центробежное + g
```

**Кориолисово ускорение:**
```
a_K = 2 · ω_Земля × V
```

**Центробежное ускорение:**
```
a_ц = ω̂ × V
```

Где `ω̂` — угловая скорость вращения СК:
```
ω̂_N = λ̇ · cos(φ)
ω̂_H = λ̇ · sin(φ)
ω̂_E = -φ̇
```

**Гравитация:** `g = {0, normalGravity(φ, h), 0}` (только вертикальная компонента).

Реализация: `harmfulAccel(lat, alt, V, lat_dot, lon_dot)`

### 13.2. Производная скорости

```
V̇ = C_б^н · (f_body - ba) - a_вред
```

Реализация: `velocityDot(n_nav, a_harm)`

### 13.3. Интегрирование скоростей (метод трапеций)

```
V(n) = V(n-1) + (V̇(n) + V̇(n-1))/2 · Δt
```

Реализация: `integrateVelocity(V_dot, V_prev, V_dot_prev, dt)`

### 13.4. Интегрирование координат

**Производная широты:**
```
φ̇ = V_N / R_h
```

**Производная долготы:**
```
λ̇ = V_E / (R_h · cos(φ))
```

**Интегрирование:**
```
φ(n) = φ(n-1) + (φ̇(n) + φ̇(n-1))/2 · Δt
λ(n) = λ(n-1) + (λ̇(n) + λ̇(n-1))/2 · Δt
h(n) = h(n-1) + (V_H(n) + V_H(n-1))/2 · Δt
```

**Защита от сингулярности:**
- `cos(φ)` → `fmax(fabs(cos(φ)), 1e-10)`
- `lat` ограничена `[-π/2, π/2]`

Реализация: `integratePosition(lat, lon, alt, V, lat_dot_prev, lon_dot_prev, alt_dot_prev, dt)`

---

## 14. Автономная выставка (aligner)

### 14.1. Функция выставки

```cpp
void get_angle_start(double *Yaw, double *Pitch, double *Roll,
                     const char *IMU_path, double lat_deg, double h, double iter);
```

**Вход:** путь к imu.dat, широта (град), высота (м), время выставки (сек).
**Выход:** начальные углы (радианы).

### 14.2. Алгоритм выставки

1. Чтение imu.dat
2. Разделение на блоки по 128 отсчётов
3. Для каждого блока:
   a. Медианный фильтр (окно 5) — удаление выбросов
   b. EMA-фильтр (коэффициент α) — сглаживание
4. Накопление данных за `align_time` секунд
5. Вычисление средних компонент:
   - Ускорения → тангаж, крен
   - Угловые скорости → курс (через компоненты вращения Земли)

### 14.3. Фильтры

**Медианный фильтр (окно 5):**
```
Для каждого элемента i:
  Берём 5 соседних: [i-2, i-1, i, i+1, i+2]
  Сортируем
  output[i] = медиана (третий элемент)
```

**EMA-фильтр:**
```
output[0] = input[0]  (для первого элемента)
output[i] = α · input[i] + (1 - α) · output[i-1]

Где α = dt / (RC + dt), RC = 1 / (2π · f_cutoff)
```

---

## 15. Фильтр Калмана (ins_filter)

### 15.1. Размерность

```cpp
constexpr int KF_STATE = 15;  // размерность вектора состояний
constexpr int KF_MEAS = 9;    // размерность вектора измерений
```

### 15.2. Матрица шума процесса Q (15×15, диагональная)

```
Q = diag(0, 0, 0,                    // координаты — без шума
         σ_a²·T, σ_a²·T, σ_a²·T,   // скорости
         σ_g²·T, σ_g²·T, σ_g²·T,   // углы
         σ_ba²·T, σ_ba²·T, σ_ba²·T, // смещения акселя
         σ_bg²·T, σ_bg²·T, σ_bg²·T) // смещения гиро
```

**Параметры шума:**
| Параметр | Значение | Единицы |
|----------|----------|---------|
| σ_g | 3.394e-4 | рад/√с |
| σ_a | 3.05e-3 | м/с²/√с |
| σ_bg | 1.16e-5 | рад/с/√с |
| σ_ba | 2e-5 | м/с²/√с |

Реализация: `Qj_matrix(T)`

### 15.3. Матрица шума измерений R (9×9, диагональная)

```
R = diag(σ_pos², σ_pos², σ_h², σ_v², σ_v², σ_v², σ_ang², σ_ang², σ_ang²)
```

**Параметры точности:**
| Параметр | Значение | Формула |
|----------|----------|---------|
| σ_pos | 7.85e-7 рад | 5м / R_EARTH |
| σ_h | 5.0 м | — |
| σ_v | 0.1 м/с | — |
| σ_ang | 0.0175 рад | 1° × π/180 |

Реализация: `Rj_matrix()`

### 15.4. Матрица перехода F (15×15)

**Общая формула:** `F = I + F_c · T`

**Структура F:**

```
     δφ     δλ     δh     δVn    δVh    δVe    δψ     δθ     δφ     δba_x  δba_y  δba_z  δbg_x  δbg_y  δbg_z
δφ  [ 1      0      0      T/R    0      0      0      0      0      0      0      0      0      0      0    ]
δλ  [ 0      1      0      0      0      T/(R·c) 0      0      0      0      0      0      0      0      0    ]
δh  [ 0      0      1      0      T      0      0      0      0      0      0      0      0      0      0    ]
δVn [ 0      0      0      1      0      0      T·(n_psi×f)[0]  T·(n_theta×f)[0]  T·(n_gamma×f)[0]  T·C[0,0]  T·C[0,1]  T·C[0,2]  0      0      0    ]
δVh [ 0      0      T·2g/R 0      1      0      T·(n_psi×f)[1]  T·(n_theta×f)[1]  T·(n_gamma×f)[1]  T·C[1,0]  T·C[1,1]  T·C[1,2]  0      0      0    ]
δVe [ 0      0      0      0      0      1      T·(n_psi×f)[2]  T·(n_theta×f)[2]  T·(n_gamma×f)[2]  T·C[2,0]  T·C[2,1]  T·C[2,2]  0      0      0    ]
δψ  [ 0      0      0      0      0      0      1      0      0      0      0      0      T·e_rate[0,0]  T·e_rate[0,1]  T·e_rate[0,2]]
δθ  [ 0      0      0      0      0      0      0      1      0      0      0      0      T·e_rate[1,0]  T·e_rate[1,1]  T·e_rate[1,2]]
δφ  [ 0      0      0      0      0      0      0      0      1      0      0      0      T·e_rate[2,0]  T·e_rate[2,1]  T·e_rate[2,2]]
δba [ 0      0      0      0      0      0      0      0      0      1      0      0      0      0      0    ]
δbg [ 0      0      0      0      0      0      0      0      0      0      0      0      1      0      0    ]
```

Где `c = cos(lat)`, `f = f_nav`.

**Блоки F:**

| Блок | Строки | Столбцы | Формула | Физический смысл |
|------|--------|---------|---------|-------------------|
| Координаты ← Скорости | 0-2 | 3-5 | T/R, T/(R·cosφ), T | δφ̇=δVn/R, δλ̇=δVe/(R·cosφ), δḣ=δVh |
| Скорости ← Координаты | 4 | 2 | T·2g/R | Гравитационный градиент |
| Скорости ← Ориентация | 3-5 | 6-8 | T·(n×f) | Ошибка углов → ошибка скорости |
| Скорости ← Смещения акселя | 3-5 | 9-11 | T·C | Смещение акселя → ошибка скорости |
| Ориентация ← Смещения гиро | 6-8 | 12-14 | T·e_rate | Смещение гиро → ошибка углов |

**Связывающая матрица Эйлера:**
```
e_rate = {{0,       cos(φ)/cos(θ),  -sin(φ)/cos(θ)},
          {0,       sin(φ),          cos(φ)},
          {1,       -tan(θ)cos(φ),   tan(θ)sin(φ)}}
```

Реализация: `Fj_matrix(T, lat, alt, C, f_nav, att)`

### 15.5. Этап предсказания (predict)

На каждом такте ИМУ (200 Гц):

```
F = Fj_matrix(T, lat, alt, C, f_nav, att)
F^T = transpose_m(F, 15)

x = F · x
P = F · P · F^T + Q
```

Реализация: `predict(T, lat, alt, C, f_nav, att, x, P)`

### 15.6. Этап коррекции (correct)

При обновлении GPS (~10 Гц):

```
1. Инновация: z = bins - sns
   z[6] = normalize_angle(z[6])   (курс)
   z[7] = normalize_angle(z[7])   (тангаж)
   z[8] = normalize_angle(z[8])   (крен)

2. Матрица наблюдения H (9×15):
   H = [I₉ | 0₉×6]

3. Ковариация инновации: S = H · P · H^T + R

4. Коэффициент усиления: K = P · H^T · S⁻¹

5. Инновация: innov = z - H · x
   innov[6..8] = normalize_angle(innov[6..8])

6. Коррекция состояния: x = x + K · innov

7. Коррекция ковариации: P = (I - K · H) · P
```

Реализация: `correct(bins, sns, x, P)`

### 15.7. Применение коррекции к состоянию

```
lat     -= x[0]    lon     -= x[1]    alt     -= x[2]
V[0]    -= x[3]    V[1]    -= x[4]    V[2]    -= x[5]
att.heading -= x[6]  att.pitch -= x[7]  att.roll  -= x[8]
ba[0..2] += x[9..11]
bg[0..2] += x[12..14]
x = 0                  (обнуление)
```

---

## 16. Основной цикл счисления (trajectory)

### 16.1. Функция step() — один такт счисления

```cpp
void step(const std::vector<double> &row,      // строка imu.dat
          const SnsSample &ref,                  // эталон СНС
          NavState &st,                          // состояние навигации
          data_io::NavLogger &log,               // логгер
          bool do_correction)                    // флаг коррекции
```

### 16.2. Алгоритм step()

```
1. Чтение времени и вычисление dt
   ├── time_s = row[0]
   └── dt = time_s - st.time_prev (если dt ≤ 0 → dt = 0.005)

2. Матрица направляющих косинусов
   └── C = bodyToNavMatrix(ψ, θ, φ)

3. Ускорение в навигационной СК
   └── n_nav = C · (f_raw - ba)

4. Предсказание фильтра Калмана
   └── predict(dt, lat, alt, C, n_nav, att, x, P)

5. Вредные ускорения
   └── a_harm = harmfulAccel(lat, alt, V, lat_dot_prev, lon_dot_prev)

6. Производная скорости
   └── V_dot = n_nav - a_harm

7. Интегрирование скоростей (метод трапеций)
   └── V = V_prev + (V_dot + V_dot_prev)/2 · dt

8. Интегрирование координат
   ├── φ̇ = V_N / R_h
   ├── λ̇ = V_E / (R_h · cos(φ))
   └── pos = integratePosition(lat, lon, alt, V, lat_dot_prev, lon_dot_prev, alt_dot_prev, dt)

9. Интегрирование углов
   ├── ω_nav = navAngularRate(lat, V, R_h)
   ├── ω_отн = ω_гиро - C^T · ω_nav
   ├── rates = eulerRates(ω_отн, pitch, roll)
   └── att = integrate(att, rates, rates_prev, dt)

10. Обновление состояния
    ├── V = V, lat = pos.lat, lon = pos.lon, alt = pos.alt, att = att
    └── Сохранение производных для следующего такта

11. Если do_correction:
    ├── Формирование инновации:
    │   ├── bins = {lat, lon, alt, Vn, Vh, Ve, ψ, θ, φ}
    │   └── sns = {lat_gps, lon_gps, alt_gps, Vn_gps, Vh_gps, Ve_gps, ψ_gps, θ, φ}
    │       (тангаж и крен = из БИНС, инновация = 0)
    ├── correct(bins, sns, x, P)
    ├── writeErrors(time_s, x)
    ├── Применение ошибок к состоянию:
    │   ├── lat -= x[0], lon -= x[1], alt -= x[2]
    │   ├── V[0] -= x[3], V[1] -= x[4], V[2] -= x[5]
    │   ├── att.heading -= x[6], att.pitch -= x[7], att.roll -= x[8]
    │   ├── ba += x[9..11], bg += x[12..14]
    │   └── x = 0
    └── writeReference(makeReference(time_s, ref))

12. Запись результата (200 Гц)
    └── writeResult(makeResult(time_s, st))
```

### 16.3. Формирование результата

```cpp
NavResult makeResult(double time, const NavState &st)
{
    NavResult r;
    r.time = time;
    r.lon = (st.lon - st.x[1]) * RAD_TO_DEG;    // вычитание ошибки
    r.lat = (st.lat - st.x[0]) * RAD_TO_DEG;
    r.alt = st.alt - st.x[2];
    r.heading = normalize_angle(st.att.heading - st.x[6]) * RAD_TO_DEG;
    r.pitch = normalize_angle(st.att.pitch - st.x[7]) * RAD_TO_DEG;
    r.roll = normalize_angle(st.att.roll - st.x[8]) * RAD_TO_DEG;
    r.vn = st.V[0] - st.x[3];
    r.vh = st.V[1] - st.x[4];
    r.ve = st.V[2] - st.x[5];
    return r;
}
```

---

## 17. Полный алгоритм работы

### 17.1. Блок-схема main()

```
                    ┌─────────────────────┐
                    │      START          │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  Чтение StartupNav  │
                    │  .ini               │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  Автономная         │
                    │  выставка           │
                    │  (Median+EMA)       │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  initialAlignment   │
                    │  (NavState)         │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  Открытие файлов    │
                    │  imu, gps, angle    │
                    └──────────┬──────────┘
                               │
              ┌────────────────▼────────────────┐
              │  while (imu.next(row))           │◄────────┐
              │                                  │         │
              │  ┌───────────────────────────┐   │         │
              │  │ Синхронизация gps.dat     │   │         │
              │  │ (временное сравнение)     │   │         │
              │  └─────────────┬─────────────┘   │         │
              │                │                  │         │
              │  ┌─────────────▼─────────────┐   │         │
              │  │ do_correction =            │   │         │
              │  │ (last_ref.time != prev)    │   │         │
              │  └─────────────┬─────────────┘   │         │
              │                │                  │         │
              │  ┌─────────────▼─────────────┐   │         │
              │  │ step(row, ref, state,      │   │         │
              │  │       log, do_correction)  │   │         │
              │  └─────────────┬─────────────┘   │         │
              │                │                  │         │
              │  ┌─────────────▼─────────────┐   │         │
              │  │   Интегрирование БИНС     │   │         │
              │  │   1. C = bodyToNavMatrix   │   │         │
              │  │   2. n_nav = C·(f-ba)      │   │         │
              │  │   3. predict(KF)           │   │         │
              │  │   4. a_harm                │   │         │
              │  │   5. V_dot = n_nav - a_harm│   │         │
              │  │   6. V = ∫V_dot dt         │   │         │
              │  │   7. (lat,lon,alt) = ∫V dt │   │         │
              │  │   8. att = ∫ω dt           │   │         │
              │  └─────────────┬─────────────┘   │         │
              │                │                  │         │
              │  ┌─────────────▼─────────────┐   │         │
              │  │   Если do_correction:      │   │         │
              │  │   9. z = bins - sns        │   │         │
              │  │  10. K = P·H^T·S⁻¹       │   │         │
              │  │  11. x += K·(z-H·x)       │   │         │
              │  │  12. P = (I-K·H)·P        │   │         │
              │  │  13. Применение ошибок    │   │         │
              │  │  14. x = 0                │   │         │
              │  └─────────────┬─────────────┘   │         │
              │                │                  │         │
              │  ┌─────────────▼─────────────┐   │         │
              │  │  writeResult (200 Гц)     │   │         │
              │  │  writeReference (~10 Гц)  │   │         │
              │  │  writeErrors (~10 Гц)     │   │         │
              │  └─────────────┬─────────────┘   │         │
              │                │                  │         │
              └────────────────┼──────────────────┘         │
                               │                            │
                               └────────────────────────────┘
```

### 17.2. Детальный алгоритм на каждом такте ИМУ

```
Шаг 1: Чтение imu.dat → row
        row = [time_s, timestamp_ns, wx, wy, wz, ax, ay, az]

Шаг 2: Синхронизация gps.dat
        while (last_ref.time <= imu_time):
            sns.next(last_ref)
        prev_gps_time = last_ref.time
        do_correction = (last_ref.time != prev_gps_time)

Шаг 3: Вычисление dt
        dt = time_s - st.time_prev
        if dt ≤ 0: dt = 0.005

Шаг 4: Матрица направляющих косинусов
        C = bodyToNavMatrix(st.att.heading, st.att.pitch, st.att.roll)

Шаг 5: Ускорение в навигационной СК
        f_body = accel(row, st.ba)     // f_raw - ba
        n_nav = bodyToNav(C, f_body)    // C · f_body

Шаг 6: Предсказание фильтра Калмана
        F = Fj_matrix(dt, st.lat, st.alt, C, n_nav, st.att)
        x = F · x
        P = F · P · F^T + Q

Шаг 7: Вредные ускорения
        ω_nav = navAngularRate(st.lat, st.V, R_EARTH + st.alt)
        a_K = 2·ω_Земля × V
        a_ц = ω̂ × V
        g = {0, normalGravity(st.lat, st.alt), 0}
        a_harm = a_K + a_ц + g

Шаг 8: Производная скорости
        V_dot = n_nav - a_harm

Шаг 9: Интегрирование скоростей (трапеция)
        V[i] = st.V[i] + (V_dot[i] + st.V_dot_prev[i])/2 · dt

Шаг 10: Интегрирование координат (трапеция)
        φ̇ = V_N / (R + alt)
        λ̇ = V_E / ((R + alt) · cos(φ))
        lat = lat_prev + (φ̇ + φ̇_prev)/2 · dt
        lon = lon_prev + (λ̇ + λ̇_prev)/2 · dt
        alt = alt_prev + (V_H + V_H_prev)/2 · dt

Шаг 11: Интегрирование углов (трапеция)
        ω_body = gyro(row, st.bg)       // ω_raw - bg
        ω_отн = ω_body - C^T · ω_nav
        rates = eulerRates(ω_отн, pitch, roll)
        att = integrate(att, rates, rates_prev, dt)

Шаг 12: Обновление состояния
        st.V = V, st.lat = lat, st.lon = lon, st.alt = alt
        st.att = att
        st.V_dot_prev = V_dot, st.lat_dot_prev = φ̇, ...
        st.time_prev = time_s

Шаг 13: Если do_correction:
        bins = {lat, lon, alt, Vn, Vh, Ve, ψ, θ, φ}
        sns = {lat_gps, lon_gps, alt_gps, Vn_gps, Vh_gps, Ve_gps, ψ_gps, θ, φ}
        z = bins - sns
        K = P·H^T·(H·P·H^T + R)^{-1}
        x = x + K·(z - H·x)
        P = (I - K·H)·P
        Применение ошибок: lat -= x[0], V -= x[3..5], ...
        x = 0

Шаг 14: Запись result.txt (всегда)
        Запись reference.txt (только при коррекции)
        Запись errors.txt (только при коррекции)
```

---

## 18. Матрицы фильтра Калмана

### 18.1. Вектор состояний x (15×1)

Это **вектор ошибок** (не само состояние):

| Индекс | Обозначение | Физический смысл | Единицы |
|--------|-------------|-------------------|---------|
| x[0] | δφ | Ошибка широты | рад |
| x[1] | δλ | Ошибка долготы | рад |
| x[2] | δh | Ошибка высоты | м |
| x[3] | δVn | Ошибка скорости на север | м/с |
| x[4] | δVh | Ошибка вертикальной скорости | м/с |
| x[5] | δVe | Ошибка скорости на восток | м/с |
| x[6] | δψ | Ошибка курса | рад |
| x[7] | δθ | Ошибка тангажа | рад |
| x[8] | δφ | Ошибка крена | рад |
| x[9..11] | δba | Смещение акселерометра (3 оси) | м/с² |
| x[12..14] | δbg | Смещение гироскопа (3 оси) | рад/с |

### 18.2. Начальная ковариация P₀ (15×15, диагональная)

| Элемент | Значение | Неопределённость |
|---------|----------|------------------|
| P[0,0] | 6.14e-9 | ~50 м |
| P[1,1] | 6.14e-9 | ~50 м |
| P[2,2] | 100 | ~10 м |
| P[3,3] | 6.25 | ~2.5 м/с |
| P[4,4] | 6.25 | ~2.5 м/с |
| P[5,5] | 6.25 | ~2.5 м/с |
| P[6,6] | 6.85e-2 | ~15° |
| P[7,7] | 3.05e-6 | ~0.1° |
| P[8,8] | 3.05e-6 | ~0.1° |
| P[9..11] | 9e-6 | ~0.003 м/с² |
| P[12..14] | 1e-8 | ~0.006°/с |

### 18.3. Матрица шума процесса Q (15×15)

```
Q[0..2, 0..2] = 0                            (координаты)
Q[3..5, 3..5] = σ_a² · T = 4.65e-8          (скорости)
Q[6..8, 6..8] = σ_g² · T = 5.75e-10         (углы)
Q[9..11, 9..11] = σ_ba² · T = 2e-12         (смещения акселя)
Q[12..14, 12..14] = σ_bg² · T = 6.73e-13    (смещения гиро)
```

### 18.4. Матрица наблюдения H (9×15)

```
H = [ I₉  |  0₉×6 ]
```

### 18.5. Матрица шума измерений R (9×9)

```
R = diag(6.14e-9, 6.14e-9, 25, 0.01, 0.01, 0.01, 3.05e-4, 3.05e-4, 3.05e-4)
```

### 18.6. Уравнения фильтра

**Предсказание (200 Гц):**
```
x = F · x
P = F · P · F^T + Q
```

**Коррекция (~10 Гц):**
```
z = БИНС − СНС
S = H · P · H^T + R
K = P · H^T · S⁻¹
x = x + K · (z − H · x)
P = (I − K · H) · P
```

**Применение ошибок:**
```
lat  -= x[0]    Vn   -= x[3]    ψ    -= x[6]    ba   += x[9..11]
lon  -= x[1]    Vh   -= x[4]    θ    -= x[7]    bg   += x[12..14]
alt  -= x[2]    Ve   -= x[5]    φ    -= x[8]    x = 0
```

---

## 19. Настройка параметров

### 19.1. Влиание параметров

| Параметр | Увеличить | Уменьшить |
|----------|-----------|-----------|
| σ_g (Q гиро) | Курс корректируется сильнее, но шумнее | Курс стабильнее, но слабее |
| σ_a (Q акселя) | Скорость корректируется сильнее | Скорость стабильнее |
| σ_bg (Q dr gyro) | bg оценивается быстрее | bg оценивается медленнее |
| σ_ba (Q dr accel) | ba оценивается быстрее | ba оценивается медленнее |
| σ_pos (R коорд) | Координаты «плывут» по БИНС | Координаты «прилипают» к GPS |
| σ_v (R скорость) | Скорость «плывёт» по БИНС | Скорость «прилипает» к GPS |
| σ_ang (R углы) | Уги «плывут» по БИНС | Уги «прилипают» к angle.dat |

### 19.2. Типичные ошибки

- Слишком маленький Q → фильтр «не помнит» ошибки, коррекция слабая
- Слишком большой Q → коррекция сильная, но шумная
- Слишком маленький R → результат «дёргается» за шумом
- Слишком большой R → измерения игнорируются

---

## Приложение: Связь файлов

| Модуль | Файл | Функции |
|--------|------|---------|
| Точка входа | src/main.cpp | readStartupNav, main |
| Ввод данных | src/data_io/data_reader.h | ImuReader, SnsReader |
| Вывод данных | src/data_io/data_writer.h | NavLogger, NavResult, NavReference |
| ИМУ | src/ins/imu_processor.h | accel, gyro, sampleTime |
| СНС | src/navigation/gps_processor.h | SnsSample |
| Матрицы | src/math_lib/matrix_ops.h | multiply_m, multiply_matrix, return_matrix, ... |
| СК | src/math_lib/transformations.h | bodyToNavMatrix, bodyToNav, navToBody, normalize_angle |
| Интегрирование | src/math_lib/interpolation.h | v_integral (метод трапеций) |
| Ориентация | src/ins/attitude_calc.h | eulerRates, integrate, relativeRate |
| Позиция | src/navigation/position_calc.h | integratePosition, integrateVelocity, harmfulAccel |
| Выставка | src/navigation/aligner.hpp | get_angle_start, FastMedian, EMA_Filter |
| Начальное состояние | src/navigation/aligner.h | initialAlignment, initialCovariance |
| Фильтр Калмана | src/ins/ins_filter.h | predict, correct, Qj_matrix, Rj_matrix, Fj_matrix, H_matrix |
| Основной цикл | src/navigation/trajectory.h | step, NavState, makeResult, makeReference |
| Константы | src/utils/constants.h | PI, U_EARTH, R_EARTH, DEG_TO_RAD, normalGravity |
| Типы | src/utils/types.h | Vector, Matrix |
