# `src/core/navigation/` — счисление, такт, прогон

Основной вычислительный модуль. Здесь собраны: один отсчёт СНС, уравнения
счисления, полное состояние, такт `step()` и прогон целиком `runSimulation()`.
Остальные файлы модуля — в отдельных документах:
[`navigation_alignment.md`](navigation_alignment.md),
[`navigation_tilt.md`](navigation_tilt.md),
[`navigation_gnss_guard.md`](navigation_gnss_guard.md),
[`navigation_settings.md`](navigation_settings.md).

---

## `navigation/gps_processor.h`

Один отсчёт эталона СНС и вспомогательные перечисления защиты.

### Перечисления

- `enum class SnsStatus` — статус отсчёта (свойство **кадра**, а не пары
  «кадр ↔ состояние»):
  - `VALID = 0` — самодостаточные проверки пройдены;
  - `INVALID = 1` — данные есть, но кадр непригоден;
  - `STALE = 2` — корректных отсчётов нет дольше таймаута.
- `enum class SnsReject` — причина отклонения кадра (уровень 1):
  - `NONE`, `NONFINITE` (нечисловое значение), `RANGE` (|lat| > 90° или
    |lon| > 180°), `ZERO_FIX` (нулевые координаты — «нет решения»), `SPEED`
    (недостижимая скорость), `ALTITUDE` (выход из полосы высоты),
    `KINEMATICS` (расхождение с последним принятым фиксом), `TIME`
    (немонотонное или слишком большое `dt`).

### Функции

- `inline const char *snsStatusName(SnsStatus s)` — строка статуса для лога
  (`VALID`/`INVALID`/`STALE`).
- `inline const char *snsRejectName(SnsReject r)` — строка причины для лога
  (`-`, `nonfinite`, `range`, `zero_fix`, `speed`, `altitude`, `kinematics`,
  `time`).

### `struct SnsSample`

Поля:
- `double time` — время, с;
- `double lat`, `lon` — координаты, **рад**; `double alt` — высота, м;
- `double vn`, `vh`, `ve` — скорости (Север, Вверх, Восток), м/с;
- `double heading`, `roll`, `pitch` — углы, **рад**.

Методы:
- `Vector measurement() const` — упаковка в вектор измерений фильтра в порядке
  `z[0..8]`: `{lat, lon, alt, vn, vh, ve, heading, pitch, roll}`.
- `double maxSpeed() const` — максимум модулей `vn`, `vh`, `ve` (порог движения).

---

## `navigation/position_calc.h`

Счисление координат и скоростей в навигационной СК. Пространство имён `nav`.

### `struct Position`

- `double lat = 0` — широта, рад;
- `double lon = 0` — долгота, рад;
- `double alt = 0` — высота, м;
- `double lat_dot = 0` — производная широты, рад/с;
- `double lon_dot = 0` — производная долготы, рад/с.

### Функции

- `inline Vector navAngularRate(double lat, const Vector &V, double Rh)` —
  угловая скорость навигационной СК:
  - `ω_N = U_EARTH·cos(lat) + V_E / Rh`;
  - `ω_H = U_EARTH·sin(lat) + V_E·tan(lat) / Rh`;
  - `ω_E = −V_N / Rh`.
  `Rh = R_EARTH + alt` передаётся аргументом.
- `inline Vector harmfulAccel(double lat, double alt, const Vector &V, double lat_dot, double lon_dot)` —
  вредные ускорения (то, что надо вычесть из `C·f_б`):
  - кориолисово `2·ω_Земля × V`, где `ω_Земля = {U_EARTH·cos(lat), U_EARTH·sin(lat), 0}`,
    а множитель 2 добавлен: `U2 = {2·U_EARTH·cos(lat), 2·U_EARTH·sin(lat), 0}`;
  - центробежное `ω̂ × V`, где `ω̂ = {lon_dot·cos(lat), lon_dot·sin(lat), −lat_dot}`
    (угловая скорость вращения самой навигационной СК);
  - гравитация `{0, normalGravity(lat, alt), 0}`.
- `inline Vector velocityDot(const Vector &n_nav, const Vector &a_harm)` —
  `V̇ = C_б^н·f_б − a_вред` (то есть `n_nav − a_harm`).
- `inline Vector integrateVelocity(const Vector &V_dot, const Vector &V_prev, const Vector &V_dot_prev, double dt)` —
  трапеция `v_integral`.
- `inline Position integratePosition(double lat, double lon, double alt, const Vector &V, double lat_dot_prev, double lon_dot_prev, double alt_dot_prev, double dt)` —
  интегрирование координат:
  - `lat_dot = V[0] / Rh`; `lat` — с ограничением в `[−π/2, π/2]`, трапеция;
  - `lon_dot = V[2] / (Rh·cos(lat))` (`cos` защищён `fmax(|·|, 1e-10)`);
    `lon` — трапеция и `normalize_angle`;
  - `alt = v_integral(V[1], alt, alt_dot_prev, dt)`.

---

## `navigation/trajectory.h`

Сердце счисления: полное состояние `NavState`, формирование строк вывода и один
такт `step()`.

### `struct NavState`

Полное состояние навигационной системы. Поля:
- `double lat`, `lon` — координаты, рад; `double alt` — высота, м;
- `ins::Attitude att` — углы ориентации;
- `Vector V = Vector(3, 0)` — скорости `{Vn, Vh, Ve}`;
- `Vector V_dot_prev = Vector(3, 0)` — производная скорости на прошлом такте;
- `double lat_dot_prev`, `lon_dot_prev`, `alt_dot_prev` — производные координат
  на прошлом такте (для трапеций);
- `ins::AttitudeRates rates_prev` — производные углов на прошлом такте;
- `double time_prev` — время прошлого такта;
- `Vector ba = Vector(3, 0)` — смещение акселерометра (корректируется фильтром);
- `Vector bg = Vector(3, 0)` — смещение гироскопа;
- `SnsGuard sns_guard` — состояние защиты СНС (см. `navigation_gnss_guard.md`);
- `Vector x = Vector(KF_STATE, 0)` — 15-компонентный вектор ошибок фильтра;
- `Matrix P = Matrix(KF_STATE*KF_STATE, 0)` — ковариация фильтра;
- `Vector bg_static = {0,0,0}` — статическое смещение гироскопа (для истинного
  курса);
- `double tilt_maneuver_time = −1e9` — время последнего такта, когда горизонт
  акселя считался недостоверным (`|ω| > gyro_off_rad_s` или торможение);
  от него отсчитывается пауза `settle_s` перед доверием акселю.
  (Поле `TiltTrimState tilt_trim` удалено на Этапе 1 — скрытый оцениватель
  дрейфа тангажа/крена вне фильтра, см. `docs/bias_estimation_stages.md`, §4.)

### Формирование вывода

- `inline data_io::NavResult makeResult(double time, const NavState &st)` —
  строка результата БИНС: из номинала **вычитаются** ошибки фильтра,
  углы — `normalize_angle`, координаты/углы — в градусы:
  - `lon = (lon − x[1])·RAD_TO_DEG`, `lat = (lat − x[0])·RAD_TO_DEG`,
    `alt = alt − x[2]`;
  - `heading/pitch/roll = normalize_angle(att − x[6..8])·RAD_TO_DEG`;
  - `vn/vh/ve = V − x[3..5]`.
- `inline data_io::NavReference makeReference(double time, const SnsSample &ref)` —
  строка эталона: координаты/углы в градусы, скорости как есть.

### Оценка горизонта по акселерометру (Этап 2, R2)

Отдельного `evalAccelAttitude` больше нет: аксель входит в фильтр штатным
измерением (`ins::correctAccel`, `src/core/ins/ins_filter.h`), а не как
готовые углы. Модель:

```text
h(x̂) = Ĉ_б^нᵀ·a_вред(φ̂, ĥ, V̂) + b̂a      (V̇ ≈ 0),
y     = f_б^изм,   z = h − y,
H: столбцы x[6..8] (углы) и x[9..11] (смещения акселя),   R = (σ_a·√dt)²·I.
```

Измерение — **горизонт**, то есть только связанные оси X и Z; ось Y
(модуль удельной силы) исключена, потому что её единственный сигнал —
`b̂a_Y`, и неучтённое `V̇_Y` уходит туда целиком (вывод и замер — §5
`docs/bias_estimation_stages.md`). `V̇` из БИНС в модель не подставляется:
там он сам построен по `f_б` и обращает `z` в тождественный нуль.

Годность измерения определяется политикой `TiltManeuverPolicy`
(`src/core/navigation/maneuver_tilt.h`) в `shouldApplyAccelHorizon`:

- `gyro_mag ≤ gyro_off_rad_s` — нет вращения;
- не торможение (`!isBrakingManeuver(V_dot_body)`);
- с последнего манёвра прошло не меньше `settle_s`;
- `‖z‖ ≤ accel_gate_ms2` — грубая защита от расхождения.

Ключи политики живут в `presets/*.ini` (`filter/*` и `tilt/*`:
`gyro_off_rad_s`, `settle_s`, `brake_body_accel`, `level_acc_rad`,
`divergence_deadband_rad`, `accel_gate_ms2`), читаются `filter_settings.cpp`.
### Телеметрия для GUI

- `struct StepTelemetry`:
  - `double time`; `int step_index`;
  - `bool corrected` — была ли коррекция по СНС;
  - `double P_diag[KF_STATE]` — диагонали ковариации после `predict`;
  - `Vector innovation` — инновация `z` (9) при коррекции;
  - `ins::CorrectionStats stats` — NIS/NEES;
  - `Vector ba`, `bg`.

Заполняется только при ненулевом указателе, поэтому консольный прогон
(`telemetry == nullptr`) не тратит время и результат не меняется.

### Применение коррекций к номиналу

- `inline void applyTiltKalmanCorrections(NavState &st)` — после `correctAccel`:
  вычитает только ошибки тангажа/крена `x[7..8]`, прибавляет `ba`/`bg`,
  обнуляет `x[7..14]`, вызывает `ins::apply_reset_covariance(P)`.
- `inline void applyKalmanCorrections(NavState &st)` — после полной коррекции по
  СНС: вычитает все ошибки `x[0..8]` (с `normalize_angle` для углов), прибавляет
  `ba`/`bg`, обнуляет весь `x`, вызывает `ins::apply_reset_covariance(P)`.

### `step()` — один такт

`inline void step(const std::vector<double> &row, const SnsSample &ref, NavState &st, data_io::NavSink &sink, bool do_correction, bool has_heading_ref, StepTelemetry *telemetry = nullptr)`

Порядок (полностью соответствует коду):

1. `dt = row[0] − st.time_prev`; если `dt ≤ 0` — `IMU_PERIOD`.
2. `C = bodyToNavMatrix(att)`.
3. `f_body = ins::accel(row, st.ba)`; `n_nav = C·f_body`.
4. `a_harm = harmfulAccel(...)`; `V_dot = n_nav − a_harm`;
   `V = integrateVelocity(...)`.
5. `pos = integratePosition(...)`.
6. `V_dot_body = navToBody(C, V_dot)`; `gyro_raw = ins::gyro(row, st.bg)`;
   `gyro_mag`.
7. Если `gyro_mag > TILT_POL.gyro_off_rad_s` или `isBrakingManeuver(...)` —
   `st.tilt_maneuver_time = time_s`.
8. `w_nav = navAngularRate(...)`; `w_rel = ins::relativeRate(...)`;
   `rates = ins::eulerRates(...)`; `limitTiltDivergence(...)`;
   `att = ins::integrate(st.att, rates, st.rates_prev, dt)`.
   *(Этап 1: вычет трима `applyTiltRateBias(rates, tilt_trim)` убран.)*
9. Обновление `st.V/lat/lon/alt/att`; сохранение производных
   (`V_dot_prev`, `lat_dot_prev`, `lon_dot_prev`, `alt_dot_prev = V[1]`,
   `rates_prev`, `time_prev`).
10. `C_cur = bodyToNavMatrix(новые att)`; `n_nav_cur = C_cur·f_body`;
    `ins::predict(dt, lat, alt, C_cur, n_nav_cur, att, x, P)`.
11. Заполнение `telemetry` (время, `P_diag`, `ba`, `bg`).
12. `f_meas = f_body + st.ba`; `z_accel = (Ĉ^T·a_вред + st.ba) − f_meas`.
13. Если `shouldApplyAccelHorizon(...)` (тихий ДУС, не торможение,
    `settle_s` истекло, `‖z‖ ≤ accel_gate_ms2`) — `ins::correctAccel(...)`
    (оси X и Z тела) + `applyTiltKalmanCorrections(st)`, иначе
    `clearTiltKalmanErrors(st.x)`.
14. `sns_ok = st.sns_guard.update(ref, time_s, kalman_cfg)`;
    `manual = sns_guard.is_new_frame && !do_correction`;
    `apply_correction = do_correction && sns_ok`.
15. При `apply_correction`:
    - `bins = {lat, lon, alt, Vn, Vh, Ve, heading, pitch, roll}`;
    - `sns = {ref.lat, ref.lon, ref.alt, ref.vn, ref.vh, ref.ve,`
      `has_heading_ref ? ref.heading : att.heading, att.pitch, att.roll}`
      (без `angle.dat` инновация курса нулевая — фильтр не тянет курс к нулю);
    - `ins::correct(bins, sns, x, P, sig_hdg, 1e6·DEG_TO_RAD, 1e6·DEG_TO_RAD,`
      `telemetry ? &stats : nullptr)`;
    - заполнение телеметрии (инновация, статистика);
    - `sink.writeErrors(time_s, st.x)`; `applyKalmanCorrections(st)`;
      `sink.writeReference(makeReference(time_s, ref))`.
16. Если `kalman_cfg.gnss_guard_log > 0.5` — сформировать `SnsGuardRecord` и
    `sink.writeGuard(...)`.
17. `sink.writeResult(makeResult(time_s, st))` на каждом такте.

Замечание: угол курса корректируется только при наличии эталона курса
(`has_heading_ref` = найден `angle.dat`/`angle.data`).

---

## `navigation/simulation.h`

Точка входа прогона, разделяемая консолью и GUI.

### `struct RunOptions`

- `std::string data_dir` — каталог данных;
- `std::string output_dir` — каталог для `result.txt`/`reference.txt`/`errors.txt`;
- `std::string settings_file` — `settings.ini` (пусто — значения по умолчанию);
- `bool write_files = true` — писать ли файлы;
- `bool capture_telemetry = false` — считать ли `P_diag` и инновацию;
- `data_io::IDataSource *source = nullptr` — источник (nullptr — файловый по
  `data_dir`; заданный позволяет считать без файлов);
- `RunPacer *pacer = nullptr` — пауза и темп;
- `double speed_factor = 0.0` — темп: > 0 — во столько раз медленнее реального
  времени, 0 — максимально быстро.

### `struct RunSummary`

Итоги: `ok`, `error`, `has_angle`, `start_lon_deg`, `start_lat_deg`, `start_alt`,
`align_time_s`, `yaw0_deg`, `pitch0_deg`, `roll0_deg`, `ba[3]`, `steps`
(число тактов), `corrections` (число коррекций), `time_first`, `time_last`,
`wall_seconds`.

### Интерфейсы обратной связи

- `class SimulationObserver` — подписка GUI. Виртуальные методы (по умолчанию
  пустые): `onAlignment(summary)`, `onResult(result)` (200 Гц),
  `onCorrection(reference, x)` (1 Гц), `onState(state, telemetry)` (только при
  `capture_telemetry`), `onProgress(t, done, total)`. Все вызываются из рабочего
  потока.
- `class CancellationToken` — `virtual bool cancelled() const { return false; }`.
- `class RunPacer` — `virtual void beforeStep(double sim_time_s, double speed_factor)`;
  реализация (в GUI) может задержать вызов (пауза/медленное воспроизведение).

### Функции

- `bool runSimulation(const RunOptions &options, RunSummary &summary, SimulationObserver *observer = nullptr, CancellationToken *cancel = nullptr, std::ostream *log = nullptr)` —
  полный прогон (см. ниже). `false` при ошибке, `summary.error` — описание;
  отмена тоже даёт `false` и `error = "cancelled"`.
- `bool readStartupNav(const std::string &path, double &lon_deg, double &lat_deg, double &alt, double &time_s)` —
  чтение `StartupNav.ini` (4 строки: долгота, широта, высота, время выставки).
- `std::size_t countImuRows(const std::string &imu_path)` — число строк ИМУ без
  заголовка (бинарный режим, чтобы `0x1A` не обрезал подсчёт).

---

## `navigation/simulation.cpp`

Реализация прогона. Порядок операций намеренно повторяет прежний `main.cpp`,
поэтому консольный прогон даёт побитово те же файлы.

### Вспомогательное (анонимное пространство имён)

- `class ForwardingSink : public data_io::NavSink` — приёмник, транслирующий
  результат в файловый `NavSink*` и/или `SimulationObserver*`. Хранит
  последние `last_result_`, `last_errors_` (чтобы `onCorrection` шёл после
  `writeErrors`).

### Функции

- `readStartupNav` — читает 4 строки через `sscanf("%lf")`.
- `countImuRows` — считает непустые строки, вычитает заголовок.
- `runSimulation` — основной алгоритм:
  1. `summary = RunSummary{}`.
  2. Источник: `FileDataSource owned_source(data_dir)` либо `options.source`;
     `source_is_file` — задан ли внешний источник.
  3. Чтение `settings.ini` (если задан): `loadFilterSettings`; определяется
     legacy-формат (`settingsFileHasSections`) для старого плоского `params.ini`;
     `applySettings(settings)`; при необходимости `applyLegacyParamsFile`.
  4. `has_angle = src->hasAngle()`.
  5. `src->open()`; чтение `StartupNav` (`src->startupNav(...)`); если нет —
     берётся первый отсчёт СНС (`src->nextSns`).
  6. `align_time` переопределяется `settings.align_time_override_s`, если > 0.
  7. Проверка отмены.
  8. Перемотка `src->open()` и выставка: `AlignImuProvider align_provider`
     оборачивает `src->nextImu`; вызывается `alignFromImuProvider(...)`
     (`write_dump = source_is_file`). Результат — `Yaw_0/Pitch_0/Roll_0` и
     `ba_x/y/z`.
  9. `state = initialAlignment(start_lat·DEG_TO_RAD, start_lon·DEG_TO_RAD, start_alt, Yaw_0, Pitch_0, Roll_0, ba0)`.
  10. Заполнение `summary`; `observer->onAlignment(summary)`.
  11. Третий `src->open()` для основного цикла. Открытие файловых выходов
      (`NavLogger::open`) при `write_files`; иначе `file_sink = nullptr`.
  12. `ForwardingSink sink(file_sink, observer)`; `sink.writeHeader()`.
  13. Цикл `while (src->nextImu(row))`:
      - пропуск невалидных строк (`ins::isValidRow`);
      - `pacer->beforeStep` и проверка отмены;
      - подтягивание отсчётов СНС: `hold_ref` — последний кадр с
        `time ≤ imu_time` (удержание между кадрами);
      - `in_outage` — окно `outage_start_s..outage_end_s` (проверка свободного
        счисления); `do_correction = !in_outage && (hold_ref.time != prev_gps_time)`;
      - `step(row, ref, state, sink, do_correction, has_angle, tp)`;
      - учёт `summary.steps`/`corrections`/`time_first`/`time_last`;
      - `observer->onState` при телеметрии;
      - `observer->onProgress` не чаще 4 Гц;
      - проверка отмены.
  14. `src->close(); sink.close()`; финальный `onProgress`; `summary.wall_seconds`;
      `summary.ok = true`.

### Логика `hold_ref` (удержание кадра СНС)

Внутренний цикл подтягивает из источника отсчёты, пока их время не превысит
время ИМУ; между кадрами СНС используется последний подходящий (`hold_ref`).
Коррекция выполняется только когда время удерживаемого кадра изменилось
(`hold_ref.time != prev_gps_time`), то есть ровно один раз на кадр СНС
(константа `SNS_RATE_HZ` в `utils/constants.h` равна 10 Гц; в тестовых наборах
СНС идёт с частотой, заданной `DatasetConfig::gps_hz`).
