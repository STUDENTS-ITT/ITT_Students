# `src/core/ins/` — ориентация, ИМУ, фильтр Калмана

Бесплатформенная часть: обработка показаний ИМУ, кинематика углов ориентации и
расширенный фильтр Калмана 15-го порядка (ESKF) с аддитивной моделью ошибок.

---

## `ins/attitude_calc.h`

Расчёт углов ориентации по угловым скоростям гироскопа. Пространство имён `ins`.

### Структуры

- `struct Attitude` — углы ориентации, радианы:
  - `double heading = 0` — курс (ψ);
  - `double pitch = 0` — тангаж (θ);
  - `double roll = 0` — крен (φ).
- `struct AttitudeRates` — производные углов, рад/с:
  - `double heading_dot = 0`;
  - `double pitch_dot = 0`;
  - `double roll_dot = 0`.

### Функции

- `inline Vector relativeRate(const Vector &w_abs, const Vector &w_nav, const Matrix &C)` —
  относительная угловая скорость тела: `ω_отн = ω_гиро − Cᵀ·ω_Земля`. Вычитает
  вращение Земли (переведённое в СК тела) из показаний гироскопа.
  Внутри: `vector_diff(w_abs, navToBody(C, w_nav))`.
- `inline AttitudeRates eulerRates(const Vector &w_rel, double pitch, double roll)` —
  преобразование относительной угловой скорости тела в скорости Эйлера. Это
  обратная связывающая матрица Эйлера (есть сингулярность при `θ = ±90°`,
  `cos_pitch` ограничен снизу `1e-10`):
  - `heading_dot = −(wy·cosφ − wz·sinφ) / cosθ` — знак минус, т.к. курс растёт
    по часовой стрелке;
  - `pitch_dot = wy·sinφ + wz·cosφ`;
  - `roll_dot = wx − tanθ·(wy·cosφ − wz·sinφ)`.
- `inline Attitude integrate(const Attitude &prev, const AttitudeRates &now, const AttitudeRates &prev_rates, double dt)` —
  интегрирование углов методом трапеций. Считает в `long double`, затем
  `normalize_angle`. `prev_rates` — производные на предыдущем такте, `now` — на
  текущем.
- `inline double headingFromEarthRate(double wx, double wz)` —
  `ψ = atan2(−wz, wx)` — вспомогательная оценка курса по компонентам вращения
  Земли.

Внутренний помощник (анонимное пространство имён):
- `inline long double v_integral_ld(...)` — трапеция в `long double` (локальная
  копия для точности интегрирования углов).

---

## `ins/imu_processor.h`

Разбор строк `imu.dat` и извлечение компонентов ИМУ. Формат (tab-separated):

```
time_s  timestamp_ns  wx  wy  wz  ax  ay  az
```

### Константы

- `IMU_COL_TIME = 0` — столбец времени;
- `IMU_COL_GYRO = 2` — первый столбец угловых скоростей (`wx wy wz`);
- `IMU_COL_ACCEL = 5` — первый столбец ускорений (`ax ay az`);
- `IMU_MIN_COLS = 8` — минимальное число столбцов валидной строки.

### Функции

- `inline bool isValidRow(const std::vector<double> &row)` — `row.size() >= 8`.
- `inline double sampleTime(const std::vector<double> &row)` — `row[0]`.
- `inline Vector accel(const std::vector<double> &row, const Vector &ba)` —
  ускорение с вычитанием смещения `ba`: `{ax−ba[0], ay−ba[1], az−ba[2]}`.
- `inline Vector gyro(const std::vector<double> &row, const Vector &bg)` —
  угловая скорость с вычитанием смещения `bg`.

---

## `ins/ins_filter.h`

Фильтр Калмана 15-го порядка (ESKF, аддитивные ошибки). Пространство имён `ins`.
Это ключевой файл оценивания.

### Размерности

- `constexpr int KF_STATE = 15;` — размер вектора состояния.
- `constexpr int KF_MEAS = 9;` — размер вектора измерений.
- `constexpr int KF_ACCEL_MEAS = 2;` — размер измерений аксель-контура
  (горизонт по осям X и Z тела; позже в файле).

### Вектор состояния `x` (15)

| Индекс | Компонента | Смысл |
|---|---|---|
| `x[0]` | `δφ` | ошибка широты |
| `x[1]` | `δλ` | ошибка долготы |
| `x[2]` | `δh` | ошибка высоты |
| `x[3]` | `δVn` | ошибка скорости на север |
| `x[4]` | `δVh` | ошибка вертикальной скорости |
| `x[5]` | `δVe` | ошибка скорости на восток |
| `x[6]` | `δψ` | ошибка курса |
| `x[7]` | `δθ` | ошибка тангажа |
| `x[8]` | `δφ` | ошибка крена |
| `x[9..11]` | `δba` | смещения акселерометра |
| `x[12..14]` | `δbg` | смещения гироскопа |

### Вектор измерений `z` (9)

| Индекс | Компонента |
|---|---|
| `z[0..2]` | разность координат (БИНС − СНС): широта, долгота, высота |
| `z[3..5]` | разность скоростей: Vn, Vh, Ve |
| `z[6..8]` | разность углов: курс, тангаж, крен |

### `struct KalmanConfig`

Настраиваемые шумы фильтра. Поля и значения по умолчанию:

| Поле | Значение | Смысл |
|---|---|---|
| `sig_g` | `2.12428e-02` | шум гироскопа, рад/√с |
| `sig_a` | `1.5276e+00` | шум акселерометра, м/с²/√с |
| `sig_bg` | `1.05526e-05` | дрейф смещения гироскопа, рад/с/√с |
| `sig_ba` | `1.96133e-04` | дрейф смещения акселерометра, м/с²/√с (паспорт, Этап 3) |
| `sig_pos` | `2.64401e-08` | шум позиции СНС, рад |
| `sig_h` | `5.0` | шум высоты СНС, м |
| `sig_v` | `4.81061e-02` | шум скорости СНС, м/с |
| `sig_hdg` | `1.23106e-03` | шум курса СНС, рад |
| `outage_start_s` | `0.0` | начало окна без коррекций СНС, с |
| `outage_end_s` | `0.0` | конец окна без коррекций СНС, с (0 — выкл) |
| `gnss_guard_on` | `1.0` | защита СНС вкл/выкл |
| `gnss_vel_plaus` | `60.0` | предел скорости для правдоподобия, м/с |
| `gnss_acc_bound` | `50.0` | допуск кинематической проверки, м |
| `gnss_silence_timeout_s` | `0.5` | порог молчания СНС, с |
| `gnss_guard_log` | `1.0` | писать лог решений защиты |

Все поля `double` — таблица привязок INI принимает только `double`.

- `inline KalmanConfig kalman_cfg;` — **глобальная** текущая конфигурация шумов.
  Переопределяется из `settings.ini` через `nav::applySettings`. Явные
  перегрузки функций принимают `cfg` аргументом, чтобы не зависеть от глобальной
  переменной (это же используется для экспорта/документации).

### Шум процесса — `Qj_matrix`

- `inline Matrix Qj_matrix(const KalmanConfig &cfg, double T)` — диагональная
  матрица `Q` (15×15):
  - координаты (`0..2`) — 0;
  - скорости (`3..5`) — `q_v = sig_a²·T`;
  - углы (`6..8`) — `q_att = sig_g²·T`;
  - смещения акселя (`9..11`) — `q_ba = sig_ba²·T`;
  - смещения гиро (`12..14`) — `q_bg = sig_bg²·T`.
- `inline Matrix Qj_matrix(double T)` — то же по глобальному `kalman_cfg`.

### Шум измерений — `Rj_matrix`

- `inline Matrix Rj_matrix(const KalmanConfig &cfg, double sig_hdg, double sig_pitch, double sig_roll)` —
  диагональная `R` (9×9):
  - `z[0],z[1]` — `sig_pos²`; `z[2]` — `sig_h²`;
  - `z[3..5]` — `sig_v²`;
  - `z[6]` — `sig_hdg²`; `z[7]` — `sig_pitch²`; `z[8]` — `sig_roll²`.
- `inline Matrix Rj_matrix(double sig_hdg, double sig_pitch, double sig_roll)` —
  по глобальному `kalman_cfg`.

### Матрица перехода — `Fj_matrix`

- `inline Matrix Fj_matrix(double T, double lat, double alt, const Matrix &C, const Vector &f_nav, const Attitude &att)` —
  матрица `F` (15×15), дискретизация за шаг `T`. Единичная диагональ, затем
  блоки:
  - `δφ̇ = δVn/R`: `F[0][3] = T/R_EARTH`;
  - `δλ̇ = δVe/(R·cosφ)`: `F[1][5] = T/(R·cosφ)`;
  - `δḣ = δVh`: `F[2][4] = T`;
  - гравитационный градиент: `F[4][2] = T·2g/R_EARTH`;
  - кориолисовы связи скоростей (`2Ω×δV`):
    `F[3][5] += T·(−2ω·sinφ)`, `F[4][5] += T·(2ω·cosφ)`,
    `F[5][3] += T·(2ω·sinφ)`;
  - ошибки скорости ← ошибки ориентации: через `f_nav × n̂` для трёх осей
    (`n_psi = {0,−1,0}`, `n_theta = {−sinψ, 0, cosψ}`, `n_gamma` — первый
    столбец `C`): заполняются `F[3+r][6..8]`;
  - ошибки скорости ← смещения акселя: `F[3+r][9+j] = T·C[r][j]`;
  - ошибки ориентации ← смещения гиро: `F[6+r][12+c] = T·e_rate[r][c]`, где
    `e_rate` — связывающая матрица Эйлера (те же коэффициенты, что в
    `eulerRates`).

Здесь `g = normalGravity(lat, alt)`, `ω = U_EARTH`.

### Предсказание — `predict`

- `inline void predict(double T, double lat, double alt, const Matrix &C, const Vector &f_nav, const Attitude &att, Vector &x, Matrix &P)` —
  этап прогноза: `x = F·x`, `P = F·P·Fᵀ + Q`.

### Диагностика — `struct CorrectionStats`

- `double nis = 0` — `innovᵀ·S⁻¹·innov` (Normalized Innovation Squared);
- `double nees = 0` — `innovᵀ·P_meas⁻¹·innov` (Normalized Estimation Error Squared);
- `int dof = KF_MEAS` — число степеней свободы;
- `bool valid = false` — заполнено ли (кадр с обратимой `S`).

Обе величины хи-квадратные: у согласованного фильтра среднее равно `dof`, а
отношение `NIS/NEES` близко к 1.

### Коррекция — `correct`

- `inline void correct(const Vector &bins, const Vector &sns, Vector &x, Matrix &P, double sig_hdg, double sig_pitch, double sig_roll, CorrectionStats *stats = nullptr)` —
  этап коррекции по СНС:
  1. инновация `z = bins − sns` (углы `z[6..8]` приводятся `normalize_angle`);
  2. `H = H_matrix(9, 15)`;
  3. `S = H·P·Hᵀ + R`;
  4. `K = P·Hᵀ·S⁻¹`;
  5. `x += K·(z − H·x)` (инновация углов снова нормализуется);
  6. `P` обновляется в **устойчивой форме Джозефа**:
     `P = (I−K·H)·P·(I−K·H)ᵀ + K·R·Kᵀ` (симметрична и сохраняет
     положительную полуопределённость).
  - `stats != nullptr` — заполняет NIS/NEES (считает инверсии `S` и верхнего
    блока `P` 9×9). В консоли передаётся `nullptr`, поэтому вычислительный путь
    и выходные файлы не меняются.
  - Параметры `sig_pitch`/`sig_roll` в кадре СНС обычно равны
    `1e6·DEG_TO_RAD` (углы тангажа/крена на этом кадре не измеряются — они
    обновляются аксель-контуром по горизонту, см. ниже).

### Патч результатов

Важно для понимания `makeResult`/`applyKalmanCorrections` (`navigation.md`):
оценки `x` — это **ошибки**, они **вычитаются** из номинала (БИНС), а смещения
`x[9..14]` **прибавляются** к `ba`/`bg`.

### Аксель-контур (горизонт, Этап 2 R2)

- `inline Matrix R_accel_matrix(double sig_a_sample)` — `R` 2×2,
  `R = sig_a_sample²·I`; вызывающему передаётся `sig_a·√dt`.
- `inline Matrix H_accel_matrix(const Matrix &C, const Vector &f_nav, const Attitude &att)` —
  `H` 2×15 по модели `h(x̂) = Ĉ_б^нᵀ·a_вред + b̂a` (строки — связанные оси X и
  Z, `KF_ACCEL_ROW = {0, 2}`):
  - `∂h/∂x[6..8] = −Ĉ_б^нᵀ·(n_i × f̂_н)` (`n_ψ = (0,−1,0)`,
    `n_θ = (−sinψ, 0, cosψ)`, `n_γ` — первый столбец `C`);
  - `∂h/∂x[9..11] = −I` (знак минус: `h` содержит `+b̂a`, а инъекция смещения —
    сложение).
- `inline void correctAccel(const Matrix &C, const Vector &f_nav, const Attitude &att, const Vector &z_body, double sig_a_sample, Vector &x, Matrix &P)` —
  коррекция по горизонту (вызывается на тактах ИМУ, где открыт гейт
  `shouldApplyAccelHorizon`, 200 Гц):
  - измерение берётся по осям `KF_ACCEL_ROW` из трёхмерной инновации
    `z_body` (ось Y — модуль удельной силы — исключена: её единственный сигнал
    `b̂a_Y` полностью забирает неучтённое `V̇_Y`);
  - `K` считается как обычно, но строки `K` для `x[0..6]` **обнуляются**
    (развязка от координат/скорости/курса: акселерометрическая коррекция должна
    трогать только ошибки углов и смещений датчиков `x[7..14]`);
  - `x += K·innov`; `P` — в форме Джозефа.

### Согласование ковариации — `apply_reset_covariance`

- `inline void apply_reset_covariance(Matrix &P)` — вызывается после ESKF-reset
  (инъекция ошибок в номинал и обнуление `δx`). Канонически `P ← G·P·Gᵀ`, где
  `G` — якобиан reset; для аддитивных групп `G = I`. Практический вклад функции —
  числовое согласование:
  1. симметризация `P ← (P + Pᵀ)/2` (после `predict`/коррекций симметрия
     нарушается на ~10⁻¹⁵);
  2. страховка диагонали: отрицательные дисперсии заменяются на `1e-15`
     (хвостовые вычитания в `(I−K·H)·P` при малых σ).
