# `src/core/navigation/gnss_guard.h` — защита комплексирования от отказов СНС

Реализованы уровни 1–2 по `docs/gnss_guard_design.md`:

1. **Самодостаточная валидация отсчёта** (диапазоны, нулевой фикс, скорость,
   высота, кинематика, монотонность времени) → статус `VALID`/`INVALID`.
2. **Детектор молчания СНС** по времени ИМУ → статус `STALE`.

Уровень 0 (устойчивый разбор строк) выполнен в `data_io/data_reader.cpp`.
Уровни 3–6 (гейт по инновации, детектор скачка, блокировка с backoff, мягкое
пересопряжение) не реализованы.

Пространство имён `nav`.

---

## Пороговые константы

- `constexpr double SNS_ALT_BAND_M = 5000.0` — полоса высоты вокруг первого
  правдоподобного фикса, м.
- `constexpr double SNS_ZERO_FIX_EPS_RAD = 1e-7` — порог «нулевого фикса»
  (|lat|,|lon| меньше — значит решения нет), рад.
- `constexpr double SNS_DT_MAX_S = 3600.0` — предельный интервал между кадрами,
  с.

Это величины-предохранители, а не «настройки подбора», поэтому не вынесены в
`KalmanConfig`.

## Функция расстояния

- `inline double snsHorizontalDistance(double lat1, double lon1, double lat2, double lon2)` —
  горизонтальное расстояние между двумя точками по локальной плоскости:
  `dlon` умножается на `cos(0.5·(lat1+lat2))`, результат — `R_EARTH·√(dlat² + dlon²)`.

---

## `struct SnsGuard`

Состояние защиты, живёт в `NavState` и переживает такты. Опора кинематической
проверки — **последний принятый** фикс, а не последний прочитанный (иначе мусор
принимается за опору).

Поля:
- `SnsSample last_accepted{}`; `bool has_accepted = false` — последний VALID
  кадр;
- `double alt0 = 0`; `bool has_alt0 = false` — центр полосы высоты;
- `double last_seen_time`, `first_seen_time`; `bool has_seen = false` — для
  детектора молчания, когда ни один кадр ещё не принят;
- `SnsStatus status = SnsStatus::STALE` — решение по текущему кадру;
- `SnsReject reject = SnsReject::NONE` — причина отклонения;
- `bool is_new_frame = false` — пришёл ли новый кадр на этом такте;
- `double last_dt`, `last_dpos`, `last_dvel` — метрики последнего нового кадра
  (для лога);
- `std::size_t accepted_frames`, `rejected_frames` — счётчики наблюдаемости.

### Метод `update`

`bool update(const SnsSample &s, double imu_time, const ins::KalmanConfig &cfg)`

- `is_new_frame = !has_seen || s.time != last_seen_time`; при новом кадре
  обновляются `first_seen_time`/`last_seen_time` и вызывается `validate(s, cfg)`.
- Повторные вызовы с тем же отсчётом решение не пересчитывают (кадр оценивается
  один раз).
- Детектор молчания (уровень 2): если кадр не новый и с момента последнего
  принятого (или первого увиденного) прошло больше
  `cfg.gnss_silence_timeout_s` по времени ИМУ — статус `STALE`. Статус только
  что пришедшего кадра не перекрывается.
- **Возвращает** `true`, если коррекцию по этому отсчёту применять разрешено:
  `cfg.gnss_guard_on ≤ 0.5 || status == VALID` (с выключенной защитой разрешение
  прежнее, но наблюдение продолжается — для сравнения прогонов).

### Приватные методы

- `void rejectFrame(SnsReject why)` — `status = INVALID`, `reject = why`,
  `rejected_frames++`.
- `void validate(const SnsSample &s, const ins::KalmanConfig &cfg)` — уровень 1
  (порядок проверок):
  1. конечность всех девяти компонент измерения и `s.time` → `NONFINITE`;
  2. `|lat| ≤ 90°`, `|lon| ≤ 180°` → `RANGE`;
  3. не нулевой фикс → `ZERO_FIX`;
  4. `s.maxSpeed() ≤ cfg.gnss_vel_plaus` → `SPEED`;
  5. высота в полосе вокруг `alt0` → `ALTITUDE`;
  6. `dt = s.time − last_accepted.time > 0` и `≤ SNS_DT_MAX_S` → `TIME`
     (метрики `last_dt` пишутся и при нарушении);
  7. `last_dpos ≤ cfg.gnss_vel_plaus·dt + cfg.gnss_acc_bound` → `KINEMATICS`
     (закрывает заморозку, телепорт и правдоподобный, но неверный фикс);
     затем считается `last_dvel`.
  - При успехе: при первом кадре задаётся `alt0`, обновляется `last_accepted`,
    `has_accepted = true`, `status = VALID`, `accepted_frames++`.

### Известное ограничение

Повтор времени (`dt = 0`) не наблюдается: строка с прежним временем не считается
новым кадром в `update()` и в лог как отказ не попадает (разбор — в
`docs/gnss_guard_design.md`, раздел 14.2).

Связь с тактом: `nav::step` вызывает `sns_guard.update(...)` на каждом такте,
а разрешение на коррекцию получает как `apply_correction = do_correction && sns_ok`
(см. [`navigation.md`](navigation.md)).
