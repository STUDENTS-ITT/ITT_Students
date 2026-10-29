# NoQT-версия и ветка `add-kalman-report`: разбор отличий

Подробный отчёт о различиях кода и функционала между консольной
NoQT-версией (`C:\ITT_Students_NoQT`) и локальной веткой
`add-kalman-report` (последний коммит `4f3d99a`, 27.09.2026).

Источник истины — **код**, а не README/комментарии. Часть комментариев
устарела (см. `docs/coordinate_frames.md`, раздел 10), поэтому выводы ниже
сопровождаются ссылками `файл:строка`.

Дата разбора: 2026-10-23.

---

## 0. Краткое резюме

Обе версии — это один и тот же проект `imitator`: 15-состоянийный
ESKF поверх БИНС с коррекцией по СНС. Ядро фильтра (уравнения, матрицы
`F`/`H`, форма Джозефа, сброс ковариации) в обеих версиях одно и то же.

Различия — в **обвязке вокруг фильтра**, и они принципиальные:

| Тема | NoQT | `add-kalman-report` |
|---|---|---|
| Выставка | окно усреднения от начала файла до `T_align` (по умолчанию 1 с) | Median + EMA, по умолчанию 120 с |
| Начальные ошибки `x0` | «последний отсчёт − среднее» (ненулевые) | нули |
| Начальная ковариация `P0` | из разброса окна + паспортные полы, с корреляциями наклон↔`ba` | захардкоженная диагональ |
| Канал наклона | аксель-вектор (X, Z) + оценка `ba` в фильтре | прямые измерения pitch/roll + скрытый трим-контур |
| Защита СНС | есть (`gnss_guard.h`, уровни 1–2) | нет |
| Диагностика | NIS/NEES, `consistency.txt`, `bias-log/report`, `init.txt` | нет |
| Форматы входа | `*.dat` и `*.data`, плюс бинарный режим | только `*.dat` |
| Настройки шумов | `sig_ba` ×17.8, `sig_pos` ×8.9 относительно ветки | базовые значения |
| `R` по долготе | штраф `1/cos φ` | без штрафа |
| Объём `src/` (строк) | ~5.5 тыс. | ~3.3 тыс. |

Коротко: ветка — это «наивная» версия с ручной выставкой и трим-контуром;
NoQT — экспериментальная линия, где выставка переведена на окно
усреднения, добавлена защита СНС, диагностика согласованности и
расширена поддержка форматов. Обратной совместимости по результату между
ними нет и не должно быть: `docs/noqt_deviations.md` явно фиксирует
сознательный отказ от побитового паритета.

---

## 1. Как сравнивалось

- Ветка взята **локально** (`git show-ref`), без сетевой загрузки;
  `add-kalman-report` = `origin/add-kalman-report` = `4f3d99a`.
- Merge-base с текущей линией (`Anton`) отсутствует: это отдельная
  история (Anton +62 коммита, branch +39 коммитов).
- Ветка выгружена в отдельный worktree:
  `C:\Users\anton\AppData\Local\Temp\opencode\wt-add-kalman-report`.
- Пофайловые unified diff сохранены в
  `C:\Users\anton\AppData\Local\Temp\opencode\diffs\*.diff`.
- Дополнительно сравнивались счётчики строк и содержимое `docs/`,
  `tools/`, `data/`.

Команды воспроизведения:

```powershell
git -C C:\ITT_Students_Qt_Version worktree add <путь> add-kalman-report
git diff --no-index -- src\ins\ins_filter.h <путь>\src\ins\ins_filter.h
```

или пакетно:

```powershell
# список общих файлов src/ и счётчики строк
python <скрипт сравнения деревьев>
```

---

## 2. Структура репозиториев

```
src/
  main.cpp                    ← вся обвязка прогона
  ins/                        ← imu_processor, attitude_calc, ins_filter
  navigation/                 ← Aligner, aligner, trajectory, position_calc,
                                gps_processor, maneuver_tilt + расширения NoQT
  math_lib/                   ← matrix_ops, transformations, interpolation, statistics
  data_io/                    ← data_reader, data_writer, io_error
  utils/                      ← constants, types
```

Отличия по составу:

| Файл / каталог | NoQT | ветка |
|---|:---:|:---:|
| `src/navigation/averaging.{h,cpp}` | **есть** | нет |
| `src/navigation/bias_log.{h,cpp}` | **есть** | нет |
| `src/navigation/bias_report.{h,cpp}` | **есть** | нет |
| `src/navigation/gnss_guard.h` | **есть** | нет |
| `data/raw/{imu,gps,angle}.dat` | **есть** (реальные данные) | нет (только `StartupNav.ini`) |
| `data/raw/StartupNav.ini` | есть | есть |
| `tools/plot_errors.py`, `tools/compare_truth.py` | **есть** | нет |
| `tools/{result,reference,errors}.txt`, `result_comparison.png` | есть | есть |
| `docs/` (21 файл) | полный набор | 3 файла: `kalman_filter_report.md`, `Protocol_data.md`, `AirSim-1.pdf` |

Важная деталь: `docs/kalman_filter_report.md` **идентичен** в обеих
версиях (байт-в-байт), а `Protocol_data.md` отличается. То есть отчёт по
фильтру в NoQT не «новый», он унаследован из ветки.

Qt-версия (`C:\ITT_Students_Qt_Version\src\core`) по содержимому совпадает
с NoQT `src/` за вычетом `main.cpp`, `averaging.*` и правок путей
включения (`"../x/y.h"` → `"x/y.h"`), а также добавленных позже
`filter_settings`, `simulation`, `data_source`, `dataset_generator`,
`ini_parser`, `CMakeLists.txt`.

---

## 3. Функциональное сравнение

| Функция | NoQT | `add-kalman-report` |
|---|---|---|
| Выставка по файлу | `AveragingWindow` (`navigation/averaging.*`) | `get_angle_start` (`Aligner.cpp`) |
| Метод выставки | средние/максимумы по окну; наклон и курс из средних | Median + EMA по буферу `BUF_rot = 3000` |
| Длительность выставки | `T_align` из `StartupNav.ini`, fallback 1 с (`main.cpp:66`) | по умолчанию 120 с |
| Начальное состояние | координаты/скорости — из окна, углы и `ba`/`bg` — из средних | углы — из выставки, координаты — из `StartupNav.ini`, скорости нулевые |
| Начальные ошибки `x0` | «последний отсчёт − среднее» | 0 |
| `P0` | `initialCovarianceFromAveraging`, `initialErrorsFromAveraging`, корреляции | `initialCovariance(lat)` — диагональ, константы |
| Канал наклона | `shouldApplyAccelHorizon` + `correctAccel` (оси X, Z) | `shouldApplyAccelTilt` + `applyAccelTiltCorrection` + `correctTilt` (pitch/roll) |
| Трим-контур (скрытая оценка `bg`) | **удалён** | `TiltTrimState`, `trimManeuverTilt`, `applyTiltRateBias` |
| Защита СНС | `gnss_guard.h`: конечность, скачки, молчание, устаревание, NIS-гейт | нет (только ручное окно «пропуск СНС») |
| Диагностика согласованности | NIS/NEES, `consistency.txt`, `cholesky_solve` | нет |
| Журнал смещений | `--bias-log` → `bias.txt`, `--bias-report` → сводка | нет |
| Лог выставки | `init.txt` (`writeInitLog`) | нет |
| Форматы входа | `*.dat` и `*.data` (dataset_1786981733) | только `*.dat` |
| Бинарный режим чтения | есть (с флагом) | нет |
| Провайдер ИМУ для выставки | `AlignImuProvider` (`std::function`) | нет (только файл) |
| Выбор файла | `pickFile(dir, "imu.dat", "imu.data")` | фиксированные имена |
| Флаги CLI | `--bias-log`, `--bias-report`, `--consistency` | только каталог и `params.ini` |
| Выход `result/reference/errors` | 10/10/15 колонок (формат сохранён) | 10/10/15 колонок |
| Доп. выходные файлы | `gnss_guard.txt`, `consistency.txt`, `init.txt`, `bias.txt` | нет |

---

## 4. Пофайловые отличия

Счётчики строк (NoQT / ветка) и объём изменений:

| Файл | NoQT | ветка | Δ (+/−) |
|---|---:|---:|---:|
| `main.cpp` | 774 | 272 | +575 / −73 |
| `ins/ins_filter.h` | 513 | 349 | +234 / −70 |
| `navigation/aligner.h` | 334 | 192 | +288 / −146 |
| `navigation/Aligner.cpp` | 414 | 307 | +372 / −265 |
| `navigation/maneuver_tilt.h` | 153 | 210 | — |
| `navigation/aligner.hpp` | 184 | 159 | — |
| `navigation/trajectory.h` | 341 | 317 | — |
| `navigation/gps_processor.h` | 103 | 47 | — |
| `data_io/data_writer.cpp` | 296 | 136 | +161 / −1 |
| `data_io/data_writer.h` | 214 | 65 | +157 / −8 |
| `data_io/data_reader.cpp` | 447 | 236 | — |
| `data_io/data_reader.h` | 87 | 78 | +19 / −10 |
| `math_lib/matrix_ops.h` | 349 | 269 | +81 / −1 |
| `ins/attitude_calc.h` | 101 | 101 | +5 / −5 (пути включения) |
| `math_lib/transformations.h` | 77 | 77 | +2 / −2 (пути включения) |

Ниже — что именно изменено.

### 4.1. `ins/ins_filter.h` (ядро фильтра)

Общая часть (одинакова): вектор состояния 15, `Fj_matrix`, форма
Джозефа, `apply_reset_covariance`, модель измерений СНС.

Отличия:

1. **Шумы модели.**
   - NoQT: `sig_ba = 1.96133e-04`, `sig_pos = 2.35442e-07`
     (`ins_filter.h:42, 48`).
   - Ветка: `sig_ba = 1.10162e-05`, `sig_pos = 2.64401e-08`
     (`ins_filter.h:38-39`).
   - Отношение: `sig_ba` ×17.8, `sig_pos` ×8.9. Смысл: NoQT быстрее
     учит смещения акселерометра и меньше доверяет координатам СНС.
2. **`R` по долготе.** NoQT делит `sig_pos` на `cos φ` (с ограничением
   снизу 0.05), чтобы на высоких широтах долгота не «весила» столько же,
   сколько широта (`ins_filter.h:117-138`). В ветке `R` диагональная с
   одинаковым `sig_pos` на все три координаты (`ins_filter.h:90`).
   На широте 47.6° вес долготы отличается в `1/cos²φ ≈ 2.2` раза.
3. **Аксель-апдейт.** NoQT: `KF_ACCEL_ROW = {0, 2}` — наблюдаются только
   X и Z тела; `H_accel_matrix` строится через `−Cᵀ·(n_i × f_н)`
   (`ins_filter.h:357-424`). Ветка: `KF_TILT_MEAS = 2`, `R_tilt_matrix`
   и `correctTilt(pitch, roll)`, где измеряются **сами углы**, а строки
   `H` — единичные (`ins_filter.h:245-...`). Это разные физические
   модели: NoQT отделяет `ba` от наклона, ветка — нет.
4. **Диагностика.** NoQT добавляет `CorrectionStats` (NIS/NEES),
   `cholesky_solve` и счётчики коррекций (`ins_filter.h:242-337`).
   В ветке этого нет.
5. **Настройки защиты СНС** (порог NIS, скачки, таймауты) добавлены в
   `KalmanConfig` только в NoQT (`ins_filter.h:55-61`).

### 4.2. `navigation/maneuver_tilt.h`

- Ветка: полноценный **трим-контур** — `TiltTrimState`,
  `applyTiltRateBias`, `trimManeuverTilt`, `shouldApplyAccelTilt`,
  `applyAccelTiltCorrection`. Контур подмешивает оценку `bg` вне фильтра
  и правит углы напрямую.
- NoQT: трим-контур **удалён**. Остались `isBrakingManeuver`,
  `isLevelTiltSettled`, `rawAccelPitch`, `rawAccelRoll`,
  `shouldApplyAccelHorizon`. Порог `gyro_off_rad_s` снижен с 0.08 до
  0.005 рад/с, добавлен `divergence_deadband_rad` (0.25°) и
  `accel_gate_ms2` (3 м/с²).
- Причина удаления зафиксирована в `docs/noqt_deviations.md`: скрытая
  оценка `bg` конфликтовала с оценкой `bg` в фильтре (двойной учёт) и
  давала «тихую» подмену состояния.

### 4.3. `navigation/trajectory.h`

- NoQT: тип приёмника — `NavSink&` (интерфейс), а не `NavLogger&`.
  Появились `writeGuard`, `writeConsistency`, `wantsConsistency`.
- NoQT: перед коррекцией по СНС работает `SnsGuard` (`sns_guard` в
  `NavState`); отклонённый кадр не применяется, но пишется в лог.
- Оба: ручное окно «отказа СНС» (`outage_start_s` / `outage_end_s` в
  `KalmanConfig`). В ветке решение принимается в `main.cpp:247-254`
  (`in_outage`), в NoQT — там же, но дополнительно через `SnsGuard`.
- NoQT: канал наклона — `shouldApplyAccelHorizon` + `correctAccel` с
  измерением в осях тела; ветка — `shouldApplyAccelTilt` +
  `applyAccelTiltCorrection` + `correctTilt` + `applyTiltRateBias` +
  `trimManeuverTilt`.
- Оба: инъекция ошибок и сброс `P` после коррекции; знаки одинаковые
  (`x = номинал − истина`).

### 4.4. `navigation/aligner.h`

- Ветка: `initialCovariance(double lat)` — диагональ, константы
  `σ_ba = 3e-3 м/с²`, `σ_bg = 1e-4 рад/с`; `initialAlignment`
  перегружен для координат из СНС/конфига и углов из выставки.
- NoQT: `initialCovariance(cfg, lat, sig_heading)` — паспортные полы
  `σ_ba = 5e-2 м/с²`, `σ_bg = 5e-4 рад/с`, плюс перекрёстные члены
  наклон↔`ba` (где угол связан с `ba_X`/`ba_Z`); `initialCovariance`
  с перегрузкой без `sig_heading`, `initialCovarianceFromAveraging` /
  `initialErrorsFromAveraging` / `initialAlignment(..., sig_heading)`,
  `ba_init`.
- Практический эффект: `P0` NoQT существенно «шире» по смещениям (в
  ~17 раз по `ba`, в ~5 раз по `bg`), поэтому фильтр активнее
  переоценивает датчики и меньше «доверяет» модели.

### 4.5. `navigation/Aligner.cpp` / `aligner.hpp`

- Ветка: монолитная `get_angle_start` — читает файл, копит буфер
  `BUF_rot = 3000`, Median + EMA, пишет `Aligner.dat`.
- NoQT: ядро вынесено в `alignFromImuProvider(const AlignImuProvider&)`,
  где провайдер — `std::function<bool(...)>`; файловая `get_angle_start`
  стала тонкой обёрткой. Добавлены:
  - разбор нового формата (`ЧЧ:ММ:СС.дробь`);
  - диагностика `omega_h_horiz`;
  - дамп `Aligner.dat` — best-effort (при недоступности пути выставка
    продолжается, `Aligner.cpp:85-101`);
  - поддержка бинарного чтения.
- **Важно:** в NoQT `get_angle_start`/`alignFromImuProvider` в `main.cpp`
  **не вызываются** — выставка идёт через `AveragingWindow`. Код
  оставлен для совместимости и для Qt-версии/тестов. В ветке
  `get_angle_start` — единственный путь выставки.

### 4.6. `data_io/data_reader.*`

- Ветка: только старый формат `time_s | timestamp_ns | ...`, фиксированные
  индексы столбцов, углы `angle.dat` читаются «как есть» (радианы).
- NoQT: два формата (`*.dat` и `*.data`), автоопределение по `:` в первом
  столбце, `parseTimestamp`, `parseAngleLine`, `parseGpsLine`, устойчивый
  разбор (битые строки пропускаются), бинарный режим, `SnsReader`
  хранит предыдущий кадр `angle.dat` для интерполяции.
- Единицы: **в обеих версиях** `gps.dat` — градусы → радианы, а
  `angle.dat` — радианы (без `DEG_TO_RAD`). Разница только в
  комментариях.

### 4.7. `data_io/data_writer.*`

- Ветка: один класс `NavLogger`, три файла (`result`, `reference`,
  `errors`).
- NoQT: интерфейс `NavSink` + `NavLogger` + `TeeSink` (дублирование
  вывода), ленивое создание `gnss_guard.txt` и `consistency.txt`,
  структуры `SnsGuardRecord` и `ConsistencyRecord`, флаг
  `wantsConsistency()`.
- **Формат** `result.txt` / `reference.txt` / `errors.txt` не изменён:
  10 / 10 / 15 колонок, те же единицы и порядок. Это принципиально:
  Qt-версия и внешние скрипты (`plot_trajectory.py`, `plot_errors.py`)
  парсят именно его.

### 4.8. `main.cpp`

- Ветка (272 строки):
  1. аргументы: каталог данных, `params.ini`;
  2. `StartupNav.ini` — координаты и `T_align` (120 с);
  3. `get_angle_start` (Median + EMA);
  4. `initialAlignment`, нулевые `x0`;
  5. цикл: интегрирование, аксель-трим, коррекция по СНС на новом кадре.
- NoQT (774 строки):
  1. аргументы + флаги `--bias-log/--bias-report/--consistency`;
  2. `pickFile` для `imu/gps/angle` (`*.dat` или `*.data`);
  3. `StartupNav.ini`, fallback `T_align = 1 с`;
  4. `AveragingWindow` (окно = весь отрезок файла до `T_align`);
  5. «лишний» отсчёт ИМУ и кадр СНС возвращаются в основной цикл
     (потоки не перематываются, `main.cpp:394-399`);
  6. `x0`/`P0` из окна, `writeInitLog` → `init.txt`;
  7. цикл: `SnsGuard`, аксель-горизонт, `correctAccel`, NIS/NEES;
  8. `--bias-report` прогоняет набор и печатает сводку.

### 4.9. `math_lib/matrix_ops.h`

- NoQT добавляет `cholesky_solve` (≈81 строка) — нужен для NIS/NEES.
- Остальные функции идентичны; конвенция хранения (row-major, `cols`
  явно) не менялась.

### 4.10. `navigation/gps_processor.h`

- Ветка: `SnsSample` (time, lat, lon, alt, vn, vh, ve, heading, pitch,
  roll).
- NoQT: добавлены `SnsStatus` (`VALID/INVALID/STALE/...`) и `SnsReject`
  (причины отказа) — их использует `gnss_guard.h`.

---

## 5. Что осталось общим

Чтобы не создавать ложного впечатления «двух разных фильтров»:

- Вектор состояния 15, порядок и соглашение `x = номинал − истина` —
  одинаковые.
- `Fj_matrix`, линеаризация ошибок ориентации (`n_ψ`, `n_θ`, `n_γ`) —
  одинаковые.
- Форма Джозефа и сброс ковариации — одинаковые (коммит ветки про
  «reset P0, predict после номинала» вошёл в обе линии).
- `position_calc.h`, `attitude_calc.h`, `transformations.h`,
  `imu_processor.h`, `interpolation.h`, `statistics.h`, `constants.h` —
  идентичны по смыслу (различия только в путях включения).
- Формат выходных файлов — идентичен.
- `docs/kalman_filter_report.md` — идентичен байт-в-байт.

---

## 6. Влияние на результат

1. **Выставка.** Ветка: 120 с Median+EMA. NoQT: окно до `T_align`
   (обычно 1 с). На реальных наборах NoQT-выставка проходит гейт покоя
   на 11 наборах (`rest_ok = 1`), но начальный курс/наклон и `ba`
   получаются другими — сравнение «строка в строку» с веткой
   бессмысленно.
2. **`x0` и `P0`.** В ветке `x0 = 0`, `P0` — узкая диагональ. В NoQT
   `x0` ненулевой (последний отсчёт минус среднее), `P0` шире и с
   корреляциями. Это меняет переходный процесс в первые секунды и
   скорость обучения `ba`/`bg`.
3. **Канал наклона.** Ветка измеряет pitch/roll напрямую (через аксель и
   трим-контур). NoQT измеряет аксель-вектор по X/Z и оценивает `ba`
   фильтром. Плюс NoQT: нет двойного учёта `bg`; минус: `ba_Y` не
   наблюдается, а при больших углах линеаризация приближённая.
4. **Шумы.** `sig_ba` ×17.8 и `sig_pos` ×8.9 делают NoQT-фильтр более
   «подвижным» по смещениям и менее доверчивым к координатам СНС.
   Это осознанная настройка (см. `docs/bias_estimation_stages.md`), а не
   случайность.
5. **`R` по долготе.** Штраф `1/cos φ` усиливает вес долготы на широте
   47.6° в ~2.2 раза. На средних широтах это заметно меняет
   соотношение коррекций широта/долгота.
6. **Защита СНС.** NoQT может отклонить кадр (скачок, молчание,
   устаревание, NIS-гейт), ветка применяет любой новый кадр. На «чистых»
   данных результат близок; на выбросах — расходится принципиально.
7. **Диагностика.** NIS/NEES и `bias-log/report` в NoQT позволяют
   оценить согласованность фильтра; в ветке таких измерений нет.

---

## 7. Регрессии и риски NoQT относительно ветки

Это места, где NoQT **потерял** функциональность ветки (часть — осознанно):

| Что потеряно | Оценка |
|---|---|
| Трим-контур `TiltTrimState` / `trimManeuverTilt` / `applyTiltRateBias` | осознанно: конфликт с оценкой `bg` в фильтре |
| Прямая коррекция pitch/roll (`correctTilt`, `H_tilt`, `R_tilt`) | заменено аксель-апдейтом; для `ba_Y` и больших углов — слабее |
| Выставка 120 с по Median+EMA | заменено окном 1 с; на шумных/подвижных данных может быть менее устойчиво |
| Обязательный дамп `Aligner.dat` | стал best-effort (тихая деградация диагностики) |
| `get_angle_start` в основном потоке | не вызывается вовсе; мёртвый код нужно либо использовать, либо удалить |

Риски (не регрессии, а особенности):

- `ba_Y` наблюдается только выставкой и СНС.
- При больших тангаже/крене `H_accel_matrix` приближённая.
- `gnss_guard.h:140-141` путает порядок `roll`/`pitch` в массиве
  (сейчас безвредно, но опасно при развитии).
- `angle.dat` трактуется как радианы — на «градусных» наборах результат
  будет неверным.
- `main.cpp` NoQT в 2.8 раза больше ветки: сложнее сопровождать.

---

## 8. Чего нет в ветке относительно NoQT

Полезные вещи, которые существуют только в NoQT и могут быть перенесены
(или уже перенесены в Qt-версию):

1. Окно усреднения (`averaging.*`) и `initialCovarianceFromAveraging`.
2. `gnss_guard.h` — защита СНС и лог `gnss_guard.txt`.
3. `NIS/NEES` + `cholesky_solve` + `consistency.txt`.
4. `bias_log` / `bias_report` (`--bias-log`, `--bias-report`).
5. `NavSink` / `TeeSink` — вывод в несколько приёмников.
6. Поддержка `*.data` (dataset_1786981733) и бинарного чтения.
7. `pickFile`, `writeInitLog` (`init.txt`), `AlignImuProvider`.
8. Устойчивый разбор входных строк (битые строки пропускаются).
9. `api`-дружелюбные флаги CLI.

---

## 9. Рекомендации

1. **Не пытаться получить побитовый паритет** NoQT ↔ ветка: это разные
   методики выставки и канала наклона. Паритет фиксируется только для
   Qt-версии относительно NoQT (`docs/noqt_deviations.md`).
2. В Qt-версии сохранять формат `result/reference/errors` — он общий для
   всех трёх линий.
3. Если нужен режим «как в ветке» — это отдельный флаг (Median+EMA,
   120 с, `x0 = 0`, `correctTilt`), а не изменение поведения по
   умолчанию.
4. Перед переносом трим-контура в Qt-версию ещё раз проверить конфликт
   с оценкой `bg` (есть готовое объяснение в `docs/noqt_deviations.md`).
5. Починить косметику, которая может стать ошибкой:
   - порядок `roll`/`pitch` в `gnss_guard.h:140-141`;
   - устаревший комментарий `transformations.h:43-44`;
   - заголовок `gps.dat` (`vx vy vz` → фактические N/U/E);
   - либо использовать `get_angle_start`, либо удалить.

---

## 10. Приложение: как воспроизвести

Полный diff по файлу:

```powershell
$wt = "C:\Users\anton\AppData\Local\Temp\opencode\wt-add-kalman-report"
git diff --no-index -- C:\ITT_Students_NoQT\src\ins\ins_filter.h $wt\src\ins\ins_filter.h
```

Список сохранённых диффов:

```
diffs\ins__ins_filter.h.diff
diffs\navigation__Aligner.cpp.diff
diffs\navigation__aligner.h.diff
diffs\navigation__trajectory.h.diff
diffs\navigation__maneuver_tilt.h.diff
diffs\navigation__gps_processor.h.diff
diffs\data_io__data_reader.cpp.diff
diffs\data_io__data_reader.h.diff
diffs\data_io__data_writer.cpp.diff
diffs\data_io__data_writer.h.diff
diffs\math_lib__matrix_ops.h.diff
diffs\main.cpp.diff
...
```

Связанные документы:

- `docs/coordinate_frames.md` — разбор СК/осей (общий для NoQT и core).
- `C:\ITT_Students_NoQT\docs\noqt_deviations.md` — сознательные отклонения
  NoQT от Qt-версии.
- `C:\ITT_Students_NoQT\docs\fast_alignment_plan.md` — схема выставки 1 с.
- `C:\ITT_Students_NoQT\docs\bias_estimation_stages.md` — этапы оценки
  смещений и почему `ba_Y` не наблюдается.
