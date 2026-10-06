# Отчёт: изменения фильтра Калмана

---

## Аннотация

Фильтр Калмана обновлён до актуальной версии. Изменены пять исходных файлов;
математическое ядро (`Fj_matrix`,
`predict`, габариты `Q`/`R`, измерение `z = БИНС − СНС`, `H = [I₉ | 0]`,
эйлерова параметризация) не изменилось. Изменения касаются:

- выноса шумов в настраиваемую конфигурацию `KalmanConfig`;
- Joseph-формы обновления ковариации (П3);
- рабочего reset ковариации `apply_reset_covariance` (П1);
- развязки tilt-контура (П2);
- переноса `predict()` после интегрирования номинала (П5);
- калибровки P₀ по σ (aligner.h);
- входных параметров `argv[1]`/`argv[2]` и окна outage (main.cpp);
- чтения `angle.dat` как радианов (data_reader.cpp).

---

## 1. Сводка изменений

| Файл | Изменение | Строк |
|---|---|---|
| `src/ins/ins_filter.h` | `KalmanConfig`, Joseph (П3), развязка tilt (П2), `apply_reset_covariance` (П1) | +104 |
| `src/main.cpp` | `applyParamsFile`, `argv[1]/argv[2]`, окно outage | +78 |
| `src/navigation/aligner.h` | P₀ из `kalman_cfg` | +35/−19 |
| `src/navigation/trajectory.h` | `predict` после номинала (П5), вызовы reset | +29/−22 |
| `src/data_io/data_reader.cpp` | `angle.dat` — радианы как есть | +5/−5 |

---

## 2. `ins/ins_filter.h`

### 2.1 Новое: `KalmanConfig` и `kalman_cfg`

Было — сигмы захардкожены литералами внутри `Qj_matrix`/`Rj_matrix`:
```cpp
const double sig_g = 3.394e-4;      const double sig_pos = 5.0 / R_EARTH;
const double sig_a = 3.05e-3;       const double sig_h   = 5.0;
const double sig_bg = 1.16e-5;      const double sig_v   = 0.1;
const double sig_ba = 2e-5;
```

Стало — структура конфигурации плюс глобальный экземпляр (`inline`, C++17:
одна инстанция на программу):
```cpp
struct KalmanConfig
{
    double sig_g   = 2.12428e-02;   // шум гироскопа, рад/√с (прогон №6)
    double sig_a   = 1.5276e+00;    // шум акселерометра, м/с²/√с (прогон №6)
    double sig_bg  = 1.05526e-05;   // дрейф смещения гиро, рад/с/√с (прогон №6)
    double sig_ba  = 1.10162e-05;   // дрейф смещения акселя, м/с²/√с (прогон №6)
    double sig_pos = 2.64401e-08;   // СНС позиция, рад (прогон №6)
    double sig_h   = 5.0;           // СНС высота, м
    double sig_v   = 4.81061e-02;   // СНС скорость, м/с (прогон №6)
    double sig_hdg = 1.23106e-03;   // СНС курс, рад (прогон №6)
    double outage_start_s = 0.0;    // начало окна без коррекций СНС, с
    double outage_end_s   = 0.0;    // конец окна (0 — выкл)
};
inline KalmanConfig kalman_cfg;
```
`Qj_matrix`/`Rj_matrix` читают `kalman_cfg.sig_*`.

Значения σ получены подбором на реальном полёте (прогон №6, 2026-09-27,
рекомендованный): J_train 0.1768 / J_valid 0.1835, P95 pos 0.255/0.259 м.
Прогон №7 не используется — артефакт «копии СНС» (`sigma_pos` вплотную к нижней
границе диапазона). Обратите внимание: значения σ отличаются от прежних дефолтов
на 1.5–3 порядка; они подобраны на конкретном полёте и не переносимы на другие
данные без повторного тюнинга.

### 2.2 П3 — Joseph-форма в `correct()`

Было:
```cpp
P = multiply_matrix(E_KH, P, KF_STATE, KF_STATE);   // P = (I − K·H)·P
```
Стало:
```cpp
const Matrix E_KH_T = transpose_m(E_KH, KF_STATE);
const Matrix Rj = Rj_matrix(sig_hdg, sig_pitch, sig_roll);
const Matrix KR = multiply_matrix(Kj, Rj, KF_MEAS, KF_MEAS);
P = matrix_sum(
    multiply_matrix(multiply_matrix(E_KH, P, KF_STATE, KF_STATE), E_KH_T,
                    KF_STATE, KF_STATE),
    multiply_matrix(KR, transpose_m(Kj, KF_MEAS), KF_MEAS, KF_STATE),
    KF_STATE);
```
`P = (I−K·H)·P·(I−K·H)ᵀ + K·R·Kᵀ` — симметрия и положительная
полуопределённость гарантированы при любом K. Побочно: `Rj` теперь вычисляется
один раз и переиспользуется в `S` и в `K·R·Kᵀ`.

### 2.3 П2 — развязка tilt-контура в `correctTilt()`

Добавлено обнуление строк K для x[0..6] (`K` сделана не-`const`):
```cpp
Matrix K = multiply_matrix(P_HT, return_matrix(S, KF_TILT_MEAS), KF_TILT_MEAS, KF_TILT_MEAS);
for (int r = 0; r < 7; r++)
    for (int c = 0; c < KF_TILT_MEAS; c++)
        at(K, r, c, KF_TILT_MEAS) = 0.0;
```
Акселерометрическая коррекция (200 Гц) больше не «протягивается» в ошибки
координат/скорости/курса, которые не скорректированы СНС.

### 2.4 П3 в `correctTilt()`

Аналогичная замена на Joseph-форму с `R_tilt_matrix` и `KF_TILT_MEAS = 2`.

### 2.5 П1 — `apply_reset_covariance` вместо no-op `resetCovariance`

Прежде (промежуточная версия) была заглушка:
```cpp
inline void resetCovariance(Matrix &P) { (void)P; }   // G = I
```
Теперь рабочая функция:
```cpp
inline void apply_reset_covariance(Matrix &P)
{
    const int n = KF_STATE;
    for (int i = 0; i < n; i++)                       // 1) симметризация
        for (int j = i + 1; j < n; j++)
        {
            const double s = 0.5 * (at(P, i, j, n) + at(P, j, i, n));
            at(P, i, j, n) = s;
            at(P, j, i, n) = s;
        }
    for (int i = 0; i < n; i++)                       // 2) страховка диагонали
        if (at(P, i, i, n) < 0.0)
            at(P, i, i, n) = 1e-15;
}
```
При аддитивной эйлеровой параметризации G = I, поэтому P не пересчитывается
по существу; практический смысл — численное согласование: симметрия и
неотрицательность диагонали удерживаются на длинных прогонах, что защищает от
раздувания K и режима «копии СНС».

---

## 3. `navigation/trajectory.h` — П5

Было (`predict` до интегрирования, по состоянию прошлого такта):
```cpp
const Matrix C = bodyToNavMatrix(st.att.heading, st.att.pitch, st.att.roll);
const Vector n_nav = bodyToNav(C, ins::accel(row, st.ba));
ins::predict(dt, st.lat, st.alt, C, n_nav, st.att, st.x, st.P);   // ← до номинала
const Vector V = integrateVelocity(...);
const Position pos = integratePosition(...);
const ins::Attitude att = ins::integrate(...);
```

Стало (`predict` после номинала, включая trim углов, по свежему состоянию):
```cpp
trimManeuverTilt(st.att, st.tilt_trim, f_body, gyro_mag, V_dot_body, dt, TILT_POL);

const Matrix C_cur = bodyToNavMatrix(st.att.heading, st.att.pitch, st.att.roll);
const Vector n_nav_cur = bodyToNav(C_cur, f_body);
ins::predict(dt, st.lat, st.alt, C_cur, n_nav_cur, st.att, st.x, st.P);
```
Устранён систематический сдвиг на `dt` (5 мс) между предсказанием и коррекцией:
инновация СНС считается против только что проинтегрированного номинала.
Побочно `f_body` теперь вычисляется один раз и переиспользуется.

Также вызовы `apply_reset_covariance(st.P)` — после обнуления `x` в
`applyTiltKalmanCorrections` и `applyKalmanCorrections`.

---

## 4. `navigation/aligner.h` — калибровка P₀

Было — `initialCovariance()` без аргументов, P₀ из жёстко заданных дисперсий
(позиция 50 м, высота 10 м, скорость 2.5 м/с, курс 15°, ba 9e-6, bg 1e-8).

Стало — `initialCovariance(lat_rad)`, три компоненты берутся из `kalman_cfg`:
- позиция (широта и долгота): `2·σ_pos`;
- высота: `2·σ_h`;
- курс: `σ_bg / (|cos(lat)|·ω_E)` — неснимаемое смещение гиро искажает
  проекцию скорости вращения Земли, т.е. курс определяется точностью курсовой
  выставки.

Остальные 7 диагоналей P₀ остались константами (2.5 м/с ×3, 1° ×2, 3e-3 ×3,
1e-4 ×3) — тюнером не подбираются, физически описывают стартовую
неопределённость выставки. Оба вызова обновлены: `st.P = initialCovariance(st.lat)`.

---

## 5. `main.cpp` — интерфейс запуска

- `applyParamsFile(path)`: таблица ключ→поле, читает `sigma_g`, `sigma_a`,
  `sigma_bg`, `sigma_ba`, `sigma_pos`, `sigma_v`, `sigma_ang` (→ `sig_hdg`),
  `outage_start_s`, `outage_end_s`; формат `ключ = значение`, пробелы
  обрезаются, неизвестные ключи и строки без `=` пропускаются; печатает
  применённые значения.
- `argv[1]` — каталог данных (по умолчанию `../data/raw`), `argv[2]` —
  `params.ini`. Раньше каталог был жёстко `../data/raw`.
- Окно outage: `in_outage` по `outage_start_s`..`outage_end_s`;
  `do_correction = !in_outage && (hold_ref.time != prev_gps_time)` — имитация
  пропадания СНС (проверка свободного счисления).
- Порядок вызовов важен: `applyParamsFile` выполняется до `initialAlignment`,
  чтобы P₀ и Q/R считались с новыми σ.

---

## 6. `data_io/data_reader.cpp` — единицы angle.dat

Было: `ang_roll_ = std::stod(...) * DEG_TO_RAD` (трактовка как градусов).
Стало: `ang_roll_ = std::stod(...)` — радианы как есть, в соответствии с
заголовком `roll_pitch_yaw_rad` файла `angle.dat`.

Расхождение устранено в пользу варианта «радианы как есть»: внутренняя СК
проекта — радианы, а заголовок файла явно указывает `_rad`.

---

## 7. Проверка

Сборка: `cmake --build build --target imitator` — успешно (MinGW, C++17).

Прогон на датасете `data/raw` (после приведения данных к согласованному виду —
см. п. 8):
- выставка: Yaw −0.33°, Pitch 1.55°, Roll −0.42°;
- начало координат: `lon 114.04269051, lat 22.41611152, alt 99.73` — совпадает
  с эталоном СНС;
- конец (t = 365.68 с): `lon 114.04268963, lat 22.41611459`, скорости ≈ 0,
  углы в допустимом диапазоне; 73 104 строки `result.txt` (200 Гц), траектория
  не расходится.

---

## 8. Риски и замечания

1. **σ привязаны к конкретному полёту.** Значения прогона №6 (особенно
   `sigma_a = 1.53` на верхней границе диапазона и `sigma_pos = 2.64e-8`) — не
   универсальные константы. На другом полёте требуется повторный тюнинг.
2. **`main.cpp` жёстко использует `../data/raw`** как каталог данных по
   умолчанию и пишет результаты в `utils::toolsDir(argv[0])`; при запуске вне
   `build/` каталог `tools` может отсутствовать.

---

*Привязка к коду: `correct()`, `correctTilt()`, `apply_reset_covariance()` —
`src/ins/ins_filter.h`; `step()` и apply*Corrections — `src/navigation/trajectory.h`;
`initialCovariance()` — `src/navigation/aligner.h`; `applyParamsFile` — `src/main.cpp`.*
