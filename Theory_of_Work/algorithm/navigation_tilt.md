# `src/core/navigation/maneuver_tilt.h` — коррекция тангажа/крена после манёвра

Держит тангаж и крен «у горизонта» в тех случаях, когда СНС не даёт эталона
углов (нет `angle.dat`, или кадр СНС отклонён). Логика нескольких уровней:

1. пока гироскоп активен или идёт торможение/разгон — углы ведём
   интегрированием гироскопа;
2. в покое тангаж/крен сводятся к показаниям акселерометра (апериодически,
   быстро при большой ошибке и медленно у нуля);
3. накопленный дрейф оценивается интегратором и вычитается из скоростей Эйлера;
4. на каждом такте ИМУ добавляется отдельная ковариационная коррекция фильтра
   только по тангажу/крену (`ins::correctTilt`).

Пространство имён `nav`.

---

## `struct TiltManeuverPolicy`

Политика сведения. Поля и значения по умолчанию:

| Поле | Значение | Смысл |
|---|---|---|
| `gyro_off_rad_s` | `0.005` | порог «тихого» гироскопа: ниже него акселю можно верить как горизонту |
| `settle_s` | `0.35` | пауза после манёвра перед доверием акселю, с |
| `brake_body_accel` | `0.7` | порог \|V̇_body x/y\| (торможение/разгон), м/с² |
| `tau_fast_s` | `0.3` | постоянная быстрого сведения при большой ошибке, с |
| `tau_slow_s` | `4.0` | постоянная медленного сведения вблизи нуля, с |
| `fast_err_rad` | `1.0°` | порог переключения быстрого/медленного сведения |
| `level_acc_rad` | `1.5°` | порог «аксель смотрит в горизонт» |
| `bias_gain` | `0.2` | усиление интегральной оценки дрейфа, 1/с² |
| `bias_max_rad_s` | `0.3°` | ограничение оценки дрейфа, рад/с |
| `divergence_deadband_rad` | `0.25°` | мёртвая зона расхождения с акселем |

## `struct TiltTrimState`

- `double bias_pitch = 0` — оценка дрейфа тангажа (интегральный член);
- `double bias_roll = 0` — оценка дрейфа крена.

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
  (горизонтальные компоненты ускорения тела).
- `inline bool isLevelTiltSettled(double gyro_mag, const Vector &V_dot_body, const TiltManeuverPolicy &pol = {})` —
  `gyro_mag ≤ gyro_off_rad_s && !isBrakingManeuver(...)`.

### Углы по сырому акселю

- `inline double rawAccelPitch(const Vector &f_body)` — `atan2(f_body[0], f_body[1])`.
- `inline double rawAccelRoll(const Vector &f_body)` — `−atan2(f_body[2], f_body[1])`.

Без компенсации `V̇_body`, чтобы ошибочная `C(θ)` не «подтверждала» саму себя.

### Вычитание дрейфа и ограничение расхождения

- `inline void applyTiltRateBias(ins::AttitudeRates &rates, const TiltTrimState &trim)` —
  `rates.pitch_dot −= trim.bias_pitch`, `rates.roll_dot −= trim.bias_roll`.
- `inline void limitAxisDivergence(double &rate, double att, double raw, const TiltManeuverPolicy &pol)` —
  если расхождение `raw − att` больше мёртвой зоны и скорость уводит угол **от**
  акселя (`delta·rate < 0`), скорость обнуляется.
- `inline bool limitTiltDivergence(ins::AttitudeRates &rates, const ins::Attitude &att, const Vector &f_body, double gyro_mag, const Vector &V_dot_body, const TiltManeuverPolicy &pol = {})` —
  применяет `limitAxisDivergence` к тангажу/крену, только когда аппарат
  «успокоился» и аксель смотрит в горизонт (`|raw| < level_acc_rad`).

### Сведение к горизонту (trim)

- `inline void trimTiltAxis(double &att, double &bias, double raw, double dt, double trust, const TiltManeuverPolicy &pol)` —
  апериодическое сведение одной оси:
  - `tau = (|delta| > fast_err_rad) ? tau_fast_s : tau_slow_s`;
  - `att = normalize_angle(att + min(1, trust·dt/tau)·delta)`;
  - `bias` ограничивается `±bias_max_rad_s` и обновляется по
    `bias − trust·bias_gain·delta·dt`.
- `inline bool trimManeuverTilt(ins::Attitude &att, TiltTrimState &trim, const Vector &f_body, double gyro_mag, const Vector &V_dot_body, double dt, const TiltManeuverPolicy &pol = {})` —
  сводит тангаж и крен к сырому акселю, когда аппарат «успокоился» и аксель
  смотрит в горизонт. `trust = 1 − gyro_mag/gyro_off_rad_s` (чем тише гироскоп,
  тем достовернее «аксель = горизонт»).

### Разрешение коррекции по акселю

- `inline bool shouldApplyAccelTilt(double time_s, double maneuver_time_s, double gyro_mag, bool acc_ok, double acc_pitch, double acc_roll, const Vector &V_dot_body, const ins::Attitude &att, const ins::AttitudeRates &rates, const TiltManeuverPolicy &pol = {})` —
  условия входа в коррекцию фильтра по акселерометру:
  - `acc_ok` (аксель годен);
  - `gyro_mag ≤ gyro_off_rad_s` и не торможение;
  - прошло не меньше `settle_s` с последнего манёвра
    (`time_s − maneuver_time_s ≥ settle_s`);
  - не глубокий «нос вниз» и не динамический крен
    (`pitch_dynamic`/`roll_dynamic` — защита от того, чтобы аксель ≈ 0° не
    тянул INS вверх при торможении);
  - угловые скорости `|rates.pitch_dot|`, `|rates.roll_dot|` не больше `2°`.

### Применение к фильтру

- `inline void applyAccelTiltCorrection(const ins::Attitude &att, double pitch_meas, double roll_meas, double sig_pitch, double sig_roll, Vector &x, Matrix &P)` —
  вызывает `ins::correctTilt` по текущему номиналу и измерению акселя.
- `inline void clearTiltKalmanErrors(Vector &x)` — обнуляет `x[7]`, `x[8]`
  (тангаж/крен), когда коррекция по акселю не применялась.
