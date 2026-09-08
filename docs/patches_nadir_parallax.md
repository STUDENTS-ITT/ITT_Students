# Правка ЕКФ: растущая погрешность видеонавигации (вариант A)

Готовые к копированию изменения в `E:\24.08.26\ITT_Students`. Три файла, около 30 строк.

## Зачем

В текущем виде погрешность измерения СНС зашита константой:

```cpp
// src/ins/ins_filter.h
const double sig_pos = 5.0 / R_EARTH;   // позиция в радианах
```

Для настоящего GPS это правильно: его ошибка случайна и не растёт. Видеоодометрия — это счисление, её ошибка накапливается монотонно. Если оставить константу, фильтр будет одинаково доверять отметке на первой секунде и на трёхсотой, когда её ошибка выросла до сотен метров, и **потянет решение за дрейфом**. Заодно поедут оценки смещений акселерометра и гироскопа: фильтр попытается объяснить растущее расхождение смещениями датчиков.

Правка делает `sig_pos` и `sig_hdg` функциями времени, прошедшего с момента привязки.

> Сначала сохраните копии изменяемых файлов.

---

## Шаг 1. `src/ins/ins_filter.h`

### 1.1. Матрица R принимает погрешности параметром

Было:

```cpp
inline Matrix Rj_matrix(double sig_hdg, double sig_pitch, double sig_roll)
{
    const double sig_pos = 5.0 / R_EARTH;   // позиция в радианах
    const double sig_h = 5.0;               // высота, м
    const double sig_v = 0.1;               // скорость, м/с
```

Стало:

```cpp
// sig_pos_m — погрешность горизонтальных координат, м;
// sig_v_ms  — погрешность скорости, м/с.
// Значения по умолчанию соответствуют штатному приёмнику СНС, поэтому
// вызовы без новых аргументов ведут себя как раньше.
inline Matrix Rj_matrix(double sig_hdg, double sig_pitch, double sig_roll,
                        double sig_pos_m = 5.0, double sig_v_ms = 0.1)
{
    const double sig_pos = sig_pos_m / R_EARTH;   // позиция в радианах
    const double sig_h = sig_pos_m;               // высота, м
    const double sig_v = sig_v_ms;                // скорость, м/с
```

### 1.2. Коррекция пробрасывает параметры дальше

Было:

```cpp
inline void correct(const Vector &bins, const Vector &sns, Vector &x, Matrix &P,
                    double sig_hdg, double sig_pitch, double sig_roll)
{
```

Стало:

```cpp
inline void correct(const Vector &bins, const Vector &sns, Vector &x, Matrix &P,
                    double sig_hdg, double sig_pitch, double sig_roll,
                    double sig_pos_m = 5.0, double sig_v_ms = 0.1)
{
```

И в теле той же функции, было:

```cpp
        Rj_matrix(sig_hdg, sig_pitch, sig_roll), KF_MEAS);
```

Стало:

```cpp
        Rj_matrix(sig_hdg, sig_pitch, sig_roll, sig_pos_m, sig_v_ms), KF_MEAS);
```

---

## Шаг 2. `src/navigation/trajectory.h`

### 2.1. Запомнить момент начала работы видеонавигации

В структуру `NavState`, рядом с `tilt_maneuver_time`, добавьте:

```cpp
    // Время первой коррекции по видеонавигации. Погрешность видеоодометрии
    // отсчитывается от этого момента.
    double vo_start_time = -1.0;
```

### 2.2. Модель роста погрешности

Перед функцией `step` добавьте:

```cpp
// Погрешность видеонавигации в зависимости от времени автономной работы.
//
// Видеоодометрия — это счисление: её ошибка накапливается, в отличие от
// ошибки СНС. Коэффициенты берутся из измеренного на EuRoC относительного
// дрейфа (RPE). Например, при дрейфе 0.5 % пути и скорости 10 м/с
// погрешность растёт примерно на 0.05 м/с.
struct VoNoisePolicy
{
    double sigma_pos0 = 3.0;        // начальная погрешность координат, м
    double sigma_pos_rate = 0.05;   // рост погрешности координат, м/с
    double sigma_pos_max = 500.0;   // потолок, чтобы не переполнить фильтр

    double sigma_hdg0 = 1.0;        // начальная погрешность курса, град
    double sigma_hdg_rate = 0.05;   // рост погрешности курса, град/с
    double sigma_hdg_max = 30.0;    // потолок, град
};

inline double voSigmaPos(double dt, const VoNoisePolicy &pol)
{
    return fmin(pol.sigma_pos0 + pol.sigma_pos_rate * dt, pol.sigma_pos_max);
}

inline double voSigmaHdg(double dt, const VoNoisePolicy &pol)
{
    return fmin(pol.sigma_hdg0 + pol.sigma_hdg_rate * dt, pol.sigma_hdg_max)
           * DEG_TO_RAD;
}
```

### 2.3. Использовать растущую погрешность в коррекции

В функции `step`, блок `if (do_correction)`. Было:

```cpp
        const double SIG_HDG = 1.0 * DEG_TO_RAD;
        const double SIG_TILT_IGNORE = 1e6 * DEG_TO_RAD;

        ins::correct(bins, sns, st.x, st.P, SIG_HDG, SIG_TILT_IGNORE, SIG_TILT_IGNORE);
```

Стало:

```cpp
        const double SIG_TILT_IGNORE = 1e6 * DEG_TO_RAD;
        constexpr VoNoisePolicy VO_POL;

        if (st.vo_start_time < 0.0)
            st.vo_start_time = time_s;
        const double dt_vo = time_s - st.vo_start_time;

        const double sig_hdg = voSigmaHdg(dt_vo, VO_POL);
        const double sig_pos_m = voSigmaPos(dt_vo, VO_POL);

        ins::correct(bins, sns, st.x, st.P, sig_hdg, SIG_TILT_IGNORE, SIG_TILT_IGNORE,
                     sig_pos_m);
```

---

## Шаг 3. Сборка и проверка

```bash
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cd build && ./imitator
```

Контрольная проверка перед прогоном на видеоданных: подставьте **штатные** `gps.dat` и `angle.dat` и убедитесь, что `result.txt` изменился незначительно. Значения по умолчанию (3 м начальной погрешности против прежних 5 м) дадут небольшое расхождение, но характер траектории сохранится. Если результат изменился радикально — где-то ошибка в правке.

---

## Как подобрать коэффициенты

Не берите их с потолка, они должны следовать из измерений:

1. Прогоните `evo_rpe` на EuRoC, получите относительный дрейф в процентах пройденного пути (раздел 9 отчёта).
2. Умножьте на характерную путевую скорость вашего полёта. Дрейф 0.5 % при 10 м/с даёт `sigma_pos_rate ≈ 0.05` м/с.
3. `sigma_pos0` берите равной ATE на первых 10–20 секундах.
4. Для курса точно так же: измерьте дрейф курса в градусах в минуту, поделите на 60.

В отчёте укажите, откуда взялось каждое число.

---

## Что дальше (вариант B)

Растущая сигма — это компромисс. Она не даёт фильтру уехать вслед за видеонавигацией, но и пользы приносит всё меньше: через несколько минут `sigma_pos` упирается в потолок, и коррекция фактически выключается.

Методически правильный путь — подавать в фильтр **приращения положения** между эпохами, а не абсолютные координаты. Приращения видеоодометрия знает хорошо, и их погрешность **не растёт** со временем, потому что каждое приращение независимо. Тогда фильтр корректирует дрейф скорости БИНС постоянно и одинаково эффективно на любой минуте полёта.

Что для этого нужно:

- добавить матрицу наблюдений на 3 измерения, отображающую состояния `x[3..5]` (ошибки скорости), по образцу уже имеющейся `H_tilt_matrix`;
- измерением сделать `V_БИНС − (P_VO(t) − P_VO(t−Δt)) / Δt`, приведённое в оси NUE;
- матрицу R задать постоянной: `sigma_v ≈ sigma_приращения / Δt`.

Объём — примерно как у `correctTilt`, который у вас уже написан и может служить образцом.
