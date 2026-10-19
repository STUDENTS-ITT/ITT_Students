# `src/core/navigation/` — начальная выставка

Формирование начального состояния навигации: ковариация `P₀`, начальные углы
ориентации и смещение акселерометра. Складывается из современного заголовка
`aligner.h` (интерфейс фильтра) и исторической пары `aligner.hpp` / `Aligner.cpp`
(собственно алгоритм выставки по потоку ИМУ).

---

## `navigation/aligner.h`

Связка «фильтр ↔ выставка». Пространство имён `nav`.

### Функции

- `inline Matrix initialCovariance(const ins::KalmanConfig &cfg, double lat_rad, double sig_heading = -1.0)` —
  начальная ковариационная матрица `P₀` (15×15, диагональная). `P₀` не
  захардкожен, а калибруется по σ из `cfg`:
  - позиция — `2·sig_pos` (рад), высота — `2·sig_h`;
  - скорость — `2.5` м/с (каждая компонента);
  - курс — `sig_bg / (cos(lat)·U_EARTH)` — неснимаемое смещение гиро искажает
    проекцию вращения Земли, поэтому курс определяется точностью курсовой
    выставки, а не свободным масштабом; при заданном `sig_heading ≥ 0` курс
    берётся равным этому σ (Этап 4: курс из эталона СНС, а не из
    `gyro-compassing`);
  - тангаж/крен — `1.0°`;
  - смещение акселя — `3e-3`; смещение гиро — `1e-4`.
  Возвращает диагональную матрицу с квадратами этих σ.
- `inline Matrix initialCovariance(double lat_rad)` — то же по глобальному
  `ins::kalman_cfg`.
- `inline ins::Attitude referenceAttitude(const SnsSample &first)` — начальная
  ориентация прямо из первого отсчёта СНС (углы копируются как есть).
- `inline NavState initialAlignment(const SnsSample &first, const ins::Attitude &att, double sig_heading = -1.0)` —
  состояние: координаты, скорости из первого отсчёта СНС, заданные углы,
  `P = initialCovariance(st.lat, sig_heading)`.
- `inline NavState initialAlignment(const SnsSample &first, double yaw, double pitch, double roll)` —
  перегрузка с явными углами.
- `inline NavState initialAlignment(double lat_rad, double lon_rad, double alt, double yaw, double pitch, double roll, const Vector &ba_init = {0,0,0}, double sig_heading = -1.0)` —
  состояние по координатам и углам (нулевая скорость, заданное `ba`), используется
  в `runSimulation` после автономной выставки; `sig_heading` задаёт σ курса в `P₀`.

---

## `navigation/aligner.hpp`

Алгоритм автономной выставки. Исторически написан с табуляцией и расширенными
doc-комментариями. Включает `math_lib/constants.hpp` (то есть физические
константы через прокси).

### Функции

- `inline double calculate_g(double lat_rad, double h)` — ускорение свободного
  падения по формуле Клеро с поправкой на высоту. Дублирует
  `utils::normalGravity` (историческая копия из исходного кода).
- `inline void FastMedian(const std::vector<double> &input, std::vector<double> &output, int n)` —
  медианный фильтр с окном 5 для подавления выбросов. Крайние два элемента
  копируются без фильтрации; в центре — медиана по 5 точкам (пузырьковая
  сортировка).
- `inline double calculate_alpha(double cutoff_freq, double sample_rate)` —
  коэффициент сглаживания EMA: `dt = 1/sample_rate`,
  `RC = 1/(2π·cutoff_freq)`, `alpha = dt/(RC + dt)`.
- `inline void EMA_Filter(const std::vector<double> &input, std::vector<double> &output, double alpha, int n, double *last_val, int is_first_block)` —
  экспоненциальное сглаживание с продолжением между блоками: первый блок
  инициализируется `input[0]`, последующие стартуют от `*last_val`; в конце
  `*last_val = output[n-1]`.
- `inline void update_recursive_mean(double &current_mean, double new_val, long n)` —
  рекурсивное среднее: при `n ≤ 1` присвоение, иначе
  `mean += (new_val − mean)/n`.
- `void get_angle_start(double *Yaw, double *Pitch, double *Roll, double *ba_x, double *ba_y, double *ba_z, const char *IMU_path, double lat_deg, double h, double iter)` —
  выставка по файлу ИМУ (обёртка над `alignFromImuProvider`).
- `using AlignImuProvider = std::function<bool(double &Time, double &Wx, double &Wy, double &Wz, double &Ax, double &Ay, double &Az)>` —
  поставщик строк ИМУ (заполняет поля, `false` — конец потока). Развязывает
  выставку с источником (файл/память).
- `void alignFromImuProvider(const AlignImuProvider &next, double *Yaw, double *Pitch, double *Roll, double *ba_x, double *ba_y, double *ba_z, double lat_deg, double h, double iter, bool write_dump = true, double *omega_h_horiz = nullptr)` —
  ядро выставки. `omega_h_horiz` (можно `nullptr`) — измеренный средний модуль
  горизонтальной составляющей гироскопа `mean(√(Wn_sm² + We_sm²))` за окно
  выставки; `-1`, если хвост окна обработан не был (`final_angles_count == 0`).
  Используется для проверки годности курсовой выставки (Этап 4, R4).

### Алгоритм `alignFromImuProvider`

1. `g = calculate_g(lat_deg·DEG_TO_RAD, h)`.
2. Отладочный дамп `../data/processed/Aligner.dat` (при `write_dump`); если путь
   недоступен — файл не пишется, выставка продолжается.
3. Данные читаются в векторы `time_arr`, `Ax_arr…Wz_arr`; параллельно
   накапливаются суммы (`Ax_sm…Wz_sm`, `total_samples`).
4. Блочная фильтрация блоками `BUF_rot = 3000`: `FastMedian` → `EMA_Filter`
   (`SAMPLE_RATE = 200 Гц`, `CUTOFF_FREQ = 1 Гц`, `alpha` из
   `calculate_alpha`). EMA хранит состояние между блоками
   (`last_Ax…last_Wz`).
5. Для каждой точки считаются:
   - средние по всей истории (`Ax_sm/total_samples` и т. д.);
   - сглаженные углы: `pitch_smooth = asin(Ax_mean/g)`, `roll_smooth = atan2(−Az_mean, Ay_mean)`;
   - проекции вращения Земли и `yaw_smooth = atan2(−We_sm, Wn_sm)`, где
     `Wn_sm = Wx_mean·cos p − Wy_mean·sin p·cos r + Wz_mean·sin p·sin r`,
     `We_sm = Wy_mean·sin r + Wz_mean·cos r`;
   - сырые углы аналогично по нефильтрованным данным (для дампа).
6. Средние углы накапливаются рекурсивно
   (`update_recursive_mean`) → `final_yaw_mean`, `final_pitch_mean`,
   `final_roll_mean`. Там же (в том же стоке данных) накапливается
   `final_wh_mean` — средний модуль горизонтальной составляющей гироскопа
   `√(Wn_sm² + We_sm²)`; наружу он отдаётся через `omega_h_horiz` и служит
   проверке годности курсовой выставки (Этап 4, см. последний раздел).
7. Смещение акселерометра: `ba = f_mean − Cᵀ·{0, g, 0}` при покое, где
   `C = bodyToNavMatrix(final_yaw, final_pitch, final_roll)`.
8. Запись результатов в `*Yaw`, `*Pitch`, `*Roll`, `*ba_x/y/z` (углы — радианы).

Ключевая идея: курс определяется по проекции вращения Земли
(`yaw = atan2(−We, Wn)`) после фильтрации показаний ИМУ, а тангаж/крен — по
акселерометру.

### `get_angle_start`

Открывает `IMU_path` в **бинарном** режиме (чтобы байт `0x1A` в повреждённых
наборах не обрезал чтение), пропускает заголовок, формирует `AlignImuProvider`
поверх `parseAlignImuRow` и вызывает `alignFromImuProvider(..., write_dump=true)`.

---

## `navigation/Aligner.cpp`

Реализация выставки.

### Константа

- `constexpr int BUF_rot = 3000` — размер буфера блочной фильтрации.

### Вспомогательная функция (анонимное пространство имён)

- `static bool parseAlignImuRow(const std::string &line, double &Time, double &Wx, double &Wy, double &Wz, double &Ax, double &Ay, double &Az)` —
  разбор строки ИМУ для выставки. Два формата:
  - новый (`dataset_1786981733`): первый токен содержит `:`
    (`ЧЧ:ММ:СС.дробь wx wy wz ax ay az`), разбор через `data_io::splitLine` и
    `data_io::parseTimestamp`;
  - старый: `time_s timestamp_ns wx wy wz ax ay az`, разбор через `sscanf` с
    пропуском второго столбца (`%*s`).
  Возвращает `false`, если строку нужно пропустить.

### Функции

- `void alignFromImuProvider(...)` — полная реализация алгоритма из
  `aligner.hpp` (см. выше).
- `void get_angle_start(...)` — обёртка над файлом.

Обе функции находятся в пространстве имён `nav` (не анонимном), поскольку
объявлены в `aligner.hpp`.

---

## Начальный курс и его достоверность (Этап 4)

Логика выбора начального курса живёт в `simulation.cpp` (перед
`initialAlignment`), потому что ей нужны и ИМУ-выставка, и поток СНС.

### R4 — проверка `gyro-compassing`

Курсовая выставка опирается на проекцию вращения Земли
(`|Ω_h| = |cos lat|·U_EARTH ≈ 10.1 °/ч` на широте наборов). Смещение гироскопа
искажает её неучтённо, поэтому по окну выставки считается

```text
ratio = ||ω_h| − Ω_h| / Ω_h,   ω_h — средний модуль ω_h из alignFromImuProvider
```

и сравнивается с порогом `GYRO_COMPASS_LIMIT = 0.3`. По обратному неравенству
треугольника `||ω_h| − Ω_h| ≤ |bg_гор|`, значит отношение — заниженная оценка
`|bg_гор|/Ω_h` (перпендикулярная Земле составляющая смещения в модуль не
входит). Превышение порога — достаточное основание признать курс по вращению
Земли недостоверным. Недоступное отношение (`ω_h = -1`, хвост окна не
обработан) достоверностью тоже не считается.

### R5 — курс из эталона СНС

Если `angle.dat` найден, `Yaw_0` заменяется на курс первого отсчёта СНС
(эталон), а `P₀` курса берётся равным
`max(cfg.sig_hdg, σ̂_эталона)²`, где `σ̂` — СКО отклонений
`normalize_angle(yaw_i − yaw_0)` по отсчётам СНС в окне `[0, align_time]`
(на окне выставки покой, поэтому отклонения — шум эталона; нормировка угла
нужна там, где курс проходит 180°). Тангаж, крен и `ba0` остаются от
акселерометрической выставки — `ba0` от курса не зависит.

### Без эталона

Курс остаётся от `gyro-compassing`, но:

- предупреждение пишется в лог и в `RunSummary` (`heading_reliable = false`);
- `P₀` курса — формула выставки `σ_bg/Ω_h` (честно большой);
- строка курса в измерении обезвешивается (`trajectory.h`), иначе фиктивный
  эталон сжал бы `P[6][6]` и заморозил курс.

Диагностика в логе: `heading source: …`, `sigma(heading0) = … rad`,
`gyro-compassing check: ||w_h|-Omega_h|/Omega_h = …`, `note: …`/`WARNING`.
Поля `RunSummary`: `heading_from_sns`, `heading_reliable`,
`gyro_compass_ratio`, `sig_heading0`.
