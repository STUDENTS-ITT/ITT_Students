# `src/core/data_io/` — чтение, запись, синтез данных

Ввод/вывод ядра: устойчивое чтение `imu.dat`/`gps.dat`/`angle.dat`, абстракция
источника, запись результатов и генератор тестовых наборов. Qt здесь нет.

Общий формат входа (старый, `tab`-separated):

```
imu.dat:   time_s  timestamp_ns  wx wy wz ax ay az
gps.dat:   time_s  timestamp_ns  latitude longitude altitude vx vy vz
angle.dat: time_s  timestamp_ns  roll pitch yaw
```

Новый формат `dataset_1786981733` (`*.data`) — время в первом столбце
`ЧЧ:ММ:СС.дробь` без `timestamp_ns`. Формат определяется по первому столбцу
строки: наличие `:` — новый формат.

---

## `data_io/io_error.h`

- `inline void reportOpenError(const std::string &path)` — вывод в `stderr`
  строки `error open: <path>` и текущего рабочего каталога (для отладки
  относительных путей).

---

## `data_io/data_reader.h` + `data_reader.cpp`

Последовательное чтение `imu.dat` и `gps.dat`+`angle.dat`.

### Свободные функции

- `std::vector<double> &parseLine(const std::string &line, std::vector<double> &items)` —
  разбор строки на числа (`std::stringstream >> item` + `std::stod`).
- `std::vector<std::string> splitLine(const std::string &line)` — разбор на строки.
- `bool parseTimestamp(const std::string &token, double &seconds)` — разбор
  `ЧЧ:ММ:СС.дробь` (через `sscanf("%d:%d:%lf")`) или обычного числа
  (`std::stod`).

### Внутренние помощники (`data_reader.cpp`, анонимное пространство имён)

Индексы столбцов старого формата:
- `GPS_COL_TIME = 0`, `GPS_COL_LAT = 2`, `GPS_COL_LON = 3`, `GPS_COL_ALT = 4`,
  `GPS_COL_VN = 5`, `GPS_COL_VH = 6`, `GPS_COL_VE = 7`, `GPS_MIN_COLS = 8`;
- `ANG_COL_TIME = 0`, `ANG_COL_ROLL = 2`, `ANG_COL_PITCH = 3`, `ANG_COL_YAW = 4`,
  `ANG_MIN_COLS = 5`.

Новый формат:
- `IMU_NEW_COLS = 7`;
- `GPS_NEW_COLS = 7`, `GPS_NEW_COL_LAT = 1 … GPS_NEW_COL_VE = 6`;
- `ANG_NEW_COLS = 14`, `ANG_NEW_COL_ROLL = 11`, `ANG_NEW_COL_PITCH = 12`,
  `ANG_NEW_COL_YAW = 13`.

Функции:
- `bool parseAngleLine(const std::string &line, double &t, double &roll, double &pitch, double &yaw)` —
  разбор углов; оба формата трактуются как **радианы**.
- `bool parseGpsLine(const std::string &line, nav::SnsSample &out)` —
  разбор СНС; координаты обоих форматов переводятся **градусы → радианы**.

### `class ImuReader`

Поля: `std::ifstream file_`, `std::string line_`.
- `bool open(const std::string &path)` — открывает в **бинарном** режиме (чтобы
  `0x1A` не обрезал чтение), сбрасывает состояние (перемотка), пропускает
  заголовок.
- `bool next(std::vector<double> &row)` — следующая строка. Пустая строка →
  пустой ряд и `true`. Новый формат приводится к старому виду:
  `{time_s, 0.0, wx, wy, wz, ax, ay, az}`. Битые строки пропускаются.
- `void close()`.

### `class SnsReader`

Поля: `gps_`, `angle_`, буферы строк, `has_angle_`, окно углов (`ang_t_prev_`,
`ang_t_curr_`, `have_ang_prev_`, `have_ang_curr_`, `ang_roll/pitch/yaw_prev/curr_`).
- `bool open(const std::string &gps_path, const std::string &angle_path = "")` —
  открывает `gps.dat` (обязательный) и `angle.dat` (опциональный); при наличии
  углов сразу читает первый отсчёт в окно `curr_`.
- `bool next(nav::SnsSample &out)` — следующий отсчёт СНС. Если углов нет —
  `heading/roll/pitch = 0`. Иначе подтягивает `angle` вперёд, пока его время
  меньше времени `gps`, и берёт **ближайший по времени** отсчёт среди `prev` и
  `curr` (стыковка по времени, а не по номеру строки).
- `void close()`.
- `bool readNextAngle()` (приватный) — сдвигает окно углов.

---

## `data_io/data_source.h` + `data_source.cpp`

Абстракция источника входных данных (развязывает ядро от файлов; тот же
интерфейс в перспективе использует поток с последовательного порта).

### `class IDataSource` (интерфейс)

- `virtual bool open() = 0` — открыть/перемотать на начало;
- `virtual bool hasAngle() const = 0`;
- `virtual std::size_t imuRowCount() const = 0`;
- `virtual bool nextImu(std::vector<double> &row) = 0` — строка ИМУ в
  каноническом виде `time_s, timestamp_ns, wx, wy, wz, ax, ay, az`;
- `virtual bool nextSns(nav::SnsSample &out) = 0`;
- `virtual bool startupNav(double &lon_deg, double &lat_deg, double &alt, double &time_s) = 0`;
- `virtual void close() = 0`;
- `virtual std::string error() const { return {}; }` — описание ошибки `open()`.

### `class FileDataSource : public IDataSource`

Поля: `data_dir_`, `imu_file_`, `gps_file_`, `angle_file_`, `startup_file_`,
`angle_exists_`, `error_`, `ImuReader imu_`, `SnsReader sns_`.
- Конструктор выбирает файлы через `pickFile` (`imu.dat`/`imu.data`,
  `gps.dat`/`gps.data`, `angle.dat`/`angle.data`), `startup_file_ =
  data_dir + "/StartupNav.ini"`.
- `open()` открывает `imu_` и `sns_`; `imuRowCount()` — `nav::countImuRows`;
  `startupNav(...)` — `nav::readStartupNav`.

### `class MockDataSource : public IDataSource`

Источник в памяти (тесты, отладка): публичные поля `imu_rows`,
`sns_samples`, `has_angle`, `has_startup`, `startup_*`; индексы `imu_index_`,
`sns_index_`.

### `pickFile` (`data_source.cpp`, анонимное пространство имён)

`std::string pickFile(const std::string &dir, const std::string &preferred, const std::string &fallback)` —
возвращает `dir/preferred`, если он есть, иначе `dir/fallback`; если нет ни
одного — `dir/preferred` (для сообщения об ошибке).

---

## `data_io/data_writer.h` + `data_writer.cpp`

Запись результатов и логов.

### `struct NavResult`

Строка результата БИНС (10 колонок): `time`, `lon`(град), `lat`(град),
`alt`(м), `heading`, `pitch`, `roll`(град), `vn`, `vh`, `ve`(м/с).

### `struct NavReference`

Строка эталона СНС (10 колонок, те же поля).

### `struct SnsGuardRecord`

Строка лога защиты СНС: `time`, `status` (`SnsStatus`), `reject` (`SnsReject`),
`new_frame`, `applied`, `manual`, `dt`, `dpos`, `dvel`, `accepted`, `rejected`.
Поля `gate/nis/sig_pos_pred/thresh/infl` из раздела 8 дизайна не пишутся
(уровни 3–5 не реализованы); `gate` в выводе всегда `ENABLED`.

### `class NavSink` (интерфейс приёмника)

- `virtual void writeHeader() {}`;
- `virtual void writeResult(const NavResult &r) = 0`;
- `virtual void writeReference(const NavReference &r) = 0`;
- `virtual void writeErrors(double time, const Vector &x) = 0`;
- `virtual void writeGuard(const SnsGuardRecord &r) {}` — по умолчанию ничего;
- `virtual void close() {}`.

`nav::step` принимает `NavSink&`, поэтому один расчёт пишет и в файлы, и в
память (GUI), и в оба сразу.

### `class NavLogger : public NavSink`

Запись в `result.txt`, `reference.txt`, `errors.txt` и лениво `gnss_guard.txt`.
- `bool open(result_path, reference_path, error_path)` — открывает три файла;
  путь к `gnss_guard.txt` выводится из пути `result.txt` (рядом с ним).
- `writeHeader()` — заголовки: результат/эталон — 10 колонок (`time lon lat alt
  heading pitch roll vn vh ve`), ошибки — `time x0 … x14`.
- `writeResult`/`writeReference` — `fixed`, `setprecision(8)`, ширина колонки
  `COL = 15`.
- `writeErrors` — `setprecision(4)`, время и весь вектор `x`.
- `writeGuard` — лениво открывает `gnss_guard.txt` (`openGuardFile`), ведёт
  сводку (отрезки STALE, причины отклонений по новым кадрам, счётчики);
  `setprecision(6)`.
- `close()` — дописывает итоговую сводку в `gnss_guard.txt` и закрывает файлы.
- Поля приватные: три `ofstream`, `guard_path_`, `guard_file_`, `guard_tried_`,
  счётчики `guard_records_/accepted_/rejected_/applied_/manual_frames_`,
  `guard_rejected_by_[8]`, `guard_stale_seconds_`, `guard_stale_last_`,
  `guard_first_time_`, `guard_last_time_`.

### `class TeeSink : public NavSink`

Дублирует поток в два приёмника `a_` и `b_` (файлы + память GUI).

---

## `data_io/dataset_generator.h` + `dataset_generator.cpp`

Синтез физически согласованных тестовых наборов: по аналитической траектории
формируются показания ИМУ, измерения СНС (`gps.dat`, `angle.dat`) и истина
(`truth.txt` в формате `reference.txt`). Модель построена **теми же формулами**,
что и счисление ядра, поэтому «чистые» показания уходят от истины лишь на ошибку
дискретизации интеграторов:

- `f_body = Cᵀ·(V̇ + a_harm)`, где `a_harm = harmfulAccel(...)`,
  `C = bodyToNavMatrix(heading, pitch, roll)`;
- `ω_gyro = Cᵀ·ω_н + ω_отн`, где `ω_н = navAngularRate(...)`, а `ω_отн` —
  угловые скорости тела из скоростей Эйлера (инверсия `eulerRates`).

Смещения/шумы добавляются **в файлы** как измерения; истина пишется без них.
Пространство имён `dataset`.

### Структуры

- `struct ProfileSegment` — сегмент профиля: `dur_s`, `v_ms` (горизонтальная
  скорость), `v_up_ms` (вертикальная), `turn_deg_s` (скорость разворота).
- `struct NoiseConfig` — шумы и ошибки:
  - гироскоп: `gyro_sigma = 0.0016` рад/с, `gyro_bias_deg_h = {20, −15, 10}`,
    `gyro_bias_rw_deg_h_sqrt_s = 0`;
  - аксель: `accel_sigma = 0.043` м/с², `accel_bias_mg = {1.5, −2, 1}`,
    `accel_bias_rw_mg_sqrt_s = 0`;
  - СНС: `gps_pos_h_sigma = 1.5` м, `gps_pos_v_sigma = 2.5` м,
    `gps_vel_sigma = 0.05` м/с, `gps_pos_offset = {0,0,0}`,
    `gps_pos_corr_sigma = 0.8` м, `gps_pos_corr_tau_s = 90` с;
  - углы: `angle_sigma_deg = {0.05, 0.05, 0.2}` (roll, pitch, yaw);
  - метки времени: `imu_jitter_s = 0.0015`, `imu_dropout_p = 0.003`,
    `sns_jitter_s = 0.006`.
- `struct DatasetConfig` — конфигурация:
  - `name`, `out_dir`;
  - `imu_hz = 200`, `gps_hz = 10`, `angle_hz = 10`;
  - начальные условия: `lat_deg = 47.641468`, `lon_deg = −122.1401651`,
    `alt_m = 116.975`, `heading_deg = 5`, `pitch_deg = −0.33`, `roll_deg = 0`,
    `align_time_s = 120`;
  - сглаживание команд: `tau_speed_s = 4`, `tau_climb_s = 4`, `tau_turn_s = 2`,
    `tau_roll_s = 1.5`;
  - `std::vector<ProfileSegment> profile`;
  - `NoiseConfig noise`; `uint64_t seed = 42`.
- `struct DatasetReport` — итоги: `imu_rows`, `gps_rows`, `angle_rows`,
  `truth_rows`, `duration_s`, `final_alt_m`, `traveled_m`.

### Функция

- `bool generateDataset(const DatasetConfig &cfg, DatasetReport &report, std::string *error)` —
  создаёт `out_dir`, синтезирует траекторию, пишет `imu.dat`, `gps.dat`,
  `angle.dat` (при `gps_hz > 0`), `truth.txt` и `StartupNav.ini`. `false` при
  ошибке (описание в `*error`).

### Внутренние функции (`dataset_generator.cpp`, анонимное пространство имён)

- `Vector eulerToBodyRates(double heading_dot, double pitch_dot, double roll_dot, double pitch, double roll)` —
  согласованная с `eulerRates` трансформация скоростей Эйлера в угловые скорости
  тела:
  - `wx = roll_dot − heading_dot·sin(pitch)`;
  - `wy = −heading_dot·cos(pitch)·cos(roll) + pitch_dot·sin(roll)`;
  - `wz = heading_dot·cos(pitch)·sin(roll) + pitch_dot·cos(roll)`.
- `struct TrajPoint` — точка чистой траектории: `t`, `lat`, `lon`, `alt`, `vn`,
  `vh`, `ve`, `heading`, `pitch`, `roll`, `fx/fy/fz` (удельная сила тела),
  `wx/wy/wz` (угловая скорость тела). Модуль также пишет тип `TrajPoint` в
  заголовочной области.
- `struct Rng` — ГПСЧ (`std::mt19937_64` + `std::normal_distribution`):
  `nextNormal()`, `nextUniform(a, b)`.
- `inline double approach(double current, double target, double tau, double dt)` —
  экспоненциальное приближение к цели (`1 − exp(−dt/tau)`).
- `void vector_sum_values(const Vector &a, const Vector &b, double &x, double &y, double &z)` —
  сумма векторов в скаляры.
- `TrajPoint interpolate(const TrajPoint &a, const TrajPoint &b, double t)` —
  линейная интерполяция между точками.
- `bool synthesize(const DatasetConfig &cfg, std::vector<TrajPoint> &out, DatasetReport *report, std::string *error)` —
  строит чистую траекторию и чистые показания ИМУ на сетке `imu_hz`.
- `void writeImu(const std::vector<TrajPoint> &traj, const DatasetConfig &cfg, Rng &rng, std::ostream &f, std::size_t &rows)` —
  пишет `imu.dat` (шум/смещения добавляются как измерения).
- `bool writeSns(const std::vector<TrajPoint> &traj, const DatasetConfig &cfg, Rng &rng, std::ostream &gps, std::ostream &angle, std::ostream &truth, std::size_t &gps_rows, std::size_t &angle_rows, std::size_t &truth_rows, std::string *error)` —
  пишет `gps.dat`, `angle.dat`, `truth.txt`.
- `bool generateDataset(...)` — как выше.

### Формат `StartupNav.ini`

4 строки: `lon_deg`, `lat_deg`, `alt_m`, `align_time_s` (с комментариями после
табуляции).

---

## `data_io/file_parser.py`

Пустой (0 байт) файл-наследство от Python-версии. В сборке C++ не участвует.
Такой же пустой файл есть в `ins/__init__.py` (см. `ins.md`).
