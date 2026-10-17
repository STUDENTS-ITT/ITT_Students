# Руководство по алгоритму

Справочник по коду ядра проекта (`esfcore`) — для тех, кому нужно разобраться,
как именно считает комплекс БИНС/СНС. Описаны все файлы ядра и консольного
запуска, их классы, структуры, функции и ключевые переменные.

Правило чтения: документ идёт **за кодом**. Для каждого файла указан его путь,
назначение, затем — сущности (структуры/классы с полями, функции с сигнатурами,
константы). Если формулировка расходится с кодом — верен код.

## Границы руководства

Описывается вычислительное ядро и консоль:

| Каталог | Сборка | Что это |
|---|---|---|
| `src/core/` | статическая библиотека `esfcore` | Qt-независимая математика (STL, C++17) |
| `src/console/` | цель `imitator` | консольная точка входа |

GUI (`src/gui/`) в этом руководстве не разбирается: он не считает, а вызывает
то же ядро (`nav::runSimulation`). Интерфейс ядра для GUI — `SimulationObserver`,
`RunPacer`, `CancellationToken`, `IDataSource` (см. `navigation.md`).

## Карта документов

| Документ | Файлы |
|---|---|
| [`utils.md`](utils.md) | `utils/types.h`, `constants.h`, `paths.h`, `ini_parser.h`, `ini_parser.cpp` |
| [`math_lib.md`](math_lib.md) | `math_lib/constants.hpp`, `interpolation.h`, `matrix_ops.h`, `statistics.h`, `transformations.h` |
| [`ins.md`](ins.md) | `ins/attitude_calc.h`, `imu_processor.h`, `ins_filter.h` |
| [`navigation.md`](navigation.md) | `navigation/gps_processor.h`, `position_calc.h`, `trajectory.h`, `simulation.h`, `simulation.cpp` |
| [`navigation_alignment.md`](navigation_alignment.md) | `navigation/aligner.h`, `aligner.hpp`, `Aligner.cpp` |
| [`navigation_tilt.md`](navigation_tilt.md) | `navigation/maneuver_tilt.h` |
| [`navigation_gnss_guard.md`](navigation_gnss_guard.md) | `navigation/gnss_guard.h` |
| [`navigation_settings.md`](navigation_settings.md) | `navigation/filter_settings.h`, `filter_settings.cpp` |
| [`data_io.md`](data_io.md) | `data_io/data_reader.h/.cpp`, `data_source.h/.cpp`, `data_writer.h/.cpp`, `dataset_generator.h/.cpp`, `io_error.h` |
| [`console.md`](console.md) | `console/main.cpp`, `console/CMakeLists.txt` |

Сборка `esfcore` и `imitator` — в `console.md` (там же полный список
компилируемых единиц).

> В `src/core/data_io/file_parser.py` и `src/core/ins/__init__.py` — пустые
> (0 байт) Python-заглушки, оставшиеся от прежней структуры. В сборке не
> участвуют, к алгоритму отношения не имеют.

## Поток данных

```
 data_dir/                     data_io/                         nav/
 imu.dat|imu.data  ──► ImuReader ─┐
 gps.dat|gps.data  ──► SnsReader ─┼──► IDataSource ──► runSimulation
 angle.dat|.data   ──►           ─┘        (FileDataSource)
 StartupNav.ini    ──► readStartupNav
                                   │
                                   ├─ выставка: alignFromImuProvider ─► Yaw,Pitch,Roll,ba
                                   │
                                   └─ цикл по строкам ИМУ ─► nav::step()
                                        ├─ интегрирование ИМУ (позиция, скорость, углы)
                                        ├─ коррекция tilt по акселю (200 Гц)
                                        ├─ защита СНС (SnsGuard)
                                        ├─ коррекция Калмана по СНС (1 Гц)
                                        └─ запись: sink.writeResult / writeReference /
                                                   writeErrors / writeGuard
                                   │
 output_dir/  ◄── NavLogger ───────┘
 result.txt, reference.txt, errors.txt, gnss_guard.txt
```

`NavSink` — приёмник результатов (файлы `NavLogger`, память GUI, либо `TeeSink`
в оба сразу). `SimulationObserver` — подписка для GUI; в консоли `nullptr`.

## Один такт счисления (`nav::step`)

Порядок операций важен для понимания кода (полное описание — в
[`navigation.md`](navigation.md)):

1. `dt = time − time_prev` (если `dt ≤ 0` — подставить `IMU_PERIOD`).
2. `C = bodyToNavMatrix(heading, pitch, roll)` — матрица СК тела → навигационная.
3. Ускорение тела `f_body = accel(row, ba)` (минус смещение акселя),
   в навигационной СК `n_nav = C · f_body`.
4. `a_harm = harmfulAccel(...)` — Кориолис + центробежное + гравитация;
   `V̇ = n_nav − a_harm`; `V = integrateVelocity(...)` (метод трапеций).
5. `pos = integratePosition(...)` — широта/долгота/высота.
6. `gyro_raw = gyro(row, bg)`, `V̇_body = Cᵀ·V̇`, `gyro_mag`.
7. Отметка времени последнего манёвра, если `gyro_mag > порог` или торможение.
8. `evalAccelAttitude(...)` — тангаж/крен по акселю и их доверие (σ).
9. `w_nav = navAngularRate(...)`, `w_rel = relativeRate(gyro_raw, w_nav, C)`,
   `rates = eulerRates(w_rel, pitch, roll)`, `integrate(...)` → новые углы.
10. Обновление состояния; сохранение производных для трапеций.
11. `trimManeuverTilt(...)` — сведение тангажа/крена к горизонту после манёвра.
12. `ins::predict(...)` — предсказание фильтра по свежему номиналу.
13. Если разрешено `shouldApplyAccelTilt(...)` — `correctTilt(...)` и
    `applyTiltKalmanCorrections(st)`, иначе `clearTiltKalmanErrors(x)`.
14. `SnsGuard::update(ref, time, cfg)` — валидация кадра и детектор молчания.
15. При разрешённой коррекции (`do_correction && sns_ok`) — `ins::correct(...)`,
    запись ошибок и эталона, `applyKalmanCorrections(st)`.
16. Запись лога защиты (`writeGuard`) и результата (`writeResult`).

## Конвенции (важно при чтении кода)

- **Углы.** Внутри — радианы; в файлы и на вывод — градусы
  (`RAD_TO_DEG`). Приведение в `[-π, π]` — `normalize_angle`.
- **Матрицы.** Хранятся в `std::vector<double>` построчно (row-major). Число
  столбцов передаётся явно: `multiply_matrix(A, B, cols_A, cols_B)`,
  `transpose_m(A, cols)`, `multiply_m(A, v, cols_A)`, `at(A, i, j, cols)`.
- **Системы координат.** Навигационная СК — (Север, Вверх, Восток); скорости
  `V = {Vn, Vh, Ve}`. СК тела — X вперёд, Y вверх, Z вправо. Курс отсчитывается
  от Севера по часовой стрелке (к Востоку).
- **Состояние фильтра.** 15 компонент: координаты (3), скорости (3), углы (3),
  смещения акселя (3), смещения гиро (3). Порядок — в [`ins.md`](ins.md).
- **Типы.** `Vector` и `Matrix` — оба `std::vector<double>`
  (`utils/types.h`).

## Что где считать «сердцем»

- Уравнения движения и счисление — `navigation/position_calc.h`.
- Один такт — `nav::step` в `navigation/trajectory.h`.
- Фильтр Калмана — `ins/ins_filter.h`.
- Проведение прогона целиком — `nav::runSimulation` в `navigation/simulation.cpp`.
- Выставка — `navigation/aligner.hpp` + `navigation/Aligner.cpp`.
- Генератор тестовых данных — `data_io/dataset_generator.cpp`.

## Смежное

- **Теория** (вывод формул, реализованных в коде) — [`../theory/README.md`](../theory/README.md).
- Руководства пользователя — [`../user/`](../user/).
- Карта репозитория — `../../PROJECT_MAP.md`; архитектура и отладка —
  `../../DEVELOPER_GUIDE.md`; правила работы с ядром — `../../AGENTS.md`.
