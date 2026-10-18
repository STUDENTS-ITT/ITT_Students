# `src/core/navigation/maneuver_tilt.h` — горизонт по акселерометру

Держит тангаж и крен «у горизонта» в тех случаях, когда СНС не даёт эталона
углов (нет `angle.dat`, или кадр СНС отклонён). Модуль больше **не** сводит
углы сам и **не** оценивает дрейф: он только (1) ограничивает уход углов от
показаний акселя и (2) разрешает штатный аксель-апдейт фильтра. Углы и
смещения обновляет один общий контур — `ins::correctAccel`
(`src/core/ins/ins_filter.h`), см. `theory/estimation.md` и
`docs/bias_estimation_stages.md`, §5.

Логика:

1. пока гироскоп активен или идёт торможение/разгон — углы ведём
   интегрированием гироскопа;
2. в покое разрешается аксель-апдейт фильтра (горизонт как измерение
   `Ĉ_б^нᵀ·a_вред + b̂a`), который уточняет и углы, и смещения акселя;
3. скорость, уводящую угол **от** показаний акселя, гасит
   `limitTiltDivergence` — единственный оставшийся ограничитель
   интегрирования.

Пространство имён `nav`.

---

## `struct TiltManeuverPolicy`

Политика. Поля и значения по умолчанию:

| Поле | Значение | Смысл |
|---|---|---|
| `gyro_off_rad_s` | `0.005` | порог «тихого» гироскопа: ниже него акселю можно верить как горизонту |
| `settle_s` | `0.35` | пауза после манёвра перед доверием акселю, с |
| `brake_body_accel` | `0.7` | порог \|V̇_body x/y\| (торможение/разгон), м/с² |
| `level_acc_rad` | `1.5°` | порог «аксель смотрит в горизонт» (для `limitTiltDivergence`) |
| `divergence_deadband_rad` | `0.25°` | мёртвая зона расхождения с акселем |
| `accel_gate_ms2` | `3.0` | порог модуля инновации аксель-апдейта, м/с² |

`accel_gate_ms2` добавлен на Этапе 2 (R2). Модель апдейта принимает `V̇ ≈ 0`,
поэтому ошибка допущения равна неучтённому `V̇` и входит в инновацию целиком.
3 м/с² соответствуют ошибке наклона около 17°, то есть порог — грубая
страховка от выбросов, а не отбор манёвров (отбор делают признаки манёвра).

## Глобальная политика

- `TiltManeuverPolicy &tiltPolicy()` — глобальный экземпляр политики.
  Объявлена в этом заголовке, определена в `filter_settings.cpp` (внутри —
  `static TiltManeuverPolicy`), чтобы `trajectory.h` не зависел от
  `filter_settings.h`. Переопределяется из `settings.ini` через
  `nav::applySettings`.

---

## Функции

### Условия манёвра/покоя

- `inline bool isBrakingManeuver(const Vector &V_dot_body, const TiltManeuverPolicy &pol = {})` —
  `|V̇_body[0]| > brake_body_accel || |V̇_body[1]| > brake_body_accel`
  (продольная и вертикальная компоненты ускорения тела).
- `inline bool isLevelTiltSettled(double gyro_mag, const Vector &V_dot_body, const TiltManeuverPolicy &pol = {})` —
  `gyro_mag ≤ gyro_off_rad_s && !isBrakingManeuver(...)`.

### Углы по сырому акселю

- `inline double rawAccelPitch(const Vector &f_body)` — `atan2(f_body[0], f_body[1])`.
- `inline double rawAccelRoll(const Vector &f_body)` — `−atan2(f_body[2], f_body[1])`.

Без компенсации `V̇_body`, чтобы ошибочная `C(θ)` не «подтверждала» саму себя.

### Ограничение расхождения с акселем

- `inline void limitAxisDivergence(double &rate, double att, double raw, const TiltManeuverPolicy &pol)` —
  если расхождение `raw − att` больше мёртвой зоны и скорость уводит угол **от**
  акселя (`delta·rate < 0`), скорость обнуляется.
- `inline bool limitTiltDivergence(ins::AttitudeRates &rates, const ins::Attitude &att, const Vector &f_body, double gyro_mag, const Vector &V_dot_body, const TiltManeuverPolicy &pol = {})` —
  применяет `limitAxisDivergence` к тангажу/крену, только когда аппарат
  «успокоился» и аксель смотрит в горизонт (`|raw| < level_acc_rad`).

### Разрешение аксель-апдейта (Этап 2, R2)

- `inline bool shouldApplyAccelHorizon(double time_s, double maneuver_time_s, double gyro_mag, double innov_mag, const Vector &V_dot_body, const TiltManeuverPolicy &pol = {})` —
  условия входа в коррекцию фильтра по акселерометру:
  - `gyro_mag ≤ gyro_off_rad_s` (нет вращения);
  - не торможение (`!isBrakingManeuver`);
  - прошло не меньше `settle_s` с последнего манёвра;
  - `innov_mag ≤ accel_gate_ms2`.
- `inline void clearTiltKalmanErrors(Vector &x)` — обнуляет `x[7]`, `x[8]`
  (тангаж/крен), когда коррекция по акселю не применялась.

### Применение к фильтру

- `ins::correctAccel(C, f_nav, att, z_body, sig_a_sample, x, P)`
  (`src/core/ins/ins_filter.h`) — сам апдейт: `H` 2×15 (столбцы `x[6..8]` и
  `x[9..11]`) по осям X и Z тела, `R = (σ_a·√dt)²·I`; строки `K` для
  координат/скорости/курса обнулены.
- `nav::applyTiltKalmanCorrections(st)` (`trajectory.h`) — переносит ошибки
  `x[7..8]` в номинал и обнуляет `x[7..14]`.

---

## Удалено

Этап 1 (R1) удалил `TiltTrimState`, `applyTiltRateBias`, `trimTiltAxis`,
`trimManeuverTilt` и обслуживавшие их поля политики (`tau_fast_s`,
`tau_slow_s`, `fast_err_rad`, `bias_gain`, `bias_max_rad_s`). Это был скрытый
оцениватель дрейфа тангажа/крена вне фильтра.

Этап 2 (R2) удалил `evalAccelAttitude`, `AccelAttitude`,
`shouldApplyAccelTilt` и `applyAccelTiltCorrection`: вместо готовых углов
аксель входит в фильтр штатным измерением с горизонтом по двум осям.
Разбор и замеры — `docs/bias_estimation_stages.md`, §4–5.
