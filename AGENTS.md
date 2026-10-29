# AGENTS.md — контекст для ИИ по проекту ITT_Students_NoQT

## Цель проекта

Консольная версия комплекса БИНС/СНС с фильтром Калмана 15-го порядка
(`imitator`), собранная **без Qt** из чистого C++17 (STL). Это копия текущего
ядра Qt-версии (`C:\ITT_Students_Qt_Version`), но с исходной (плоской)
структурой `src/` и штатным запуском, как в папке-предшественнике
`C:\ITT_Students`.

## Структура

```
src/
├── main.cpp                 # Точка входа и движок прогона целиком (аргументы,
│                            #   выбор файлов, выставка, основной цикл, запись)
├── data_io/                 # data_reader, data_writer
├── ins/                     # attitude_calc, imu_processor, ins_filter
├── math_lib/                # matrix_ops, statistics, transformations,
│                            #   interpolation, constants
├── navigation/              # Aligner.cpp, aligner, averaging, trajectory,
│                            #   gps_processor, position_calc, maneuver_tilt,
│                            #   gnss_guard, bias_log, bias_report
└── utils/                   # constants, types, paths
```

Заголовки подключаются от корня `src` (например, `"navigation/trajectory.h"`).
Относительных `../` в include нет — раскладка плоская, как в оригинале.

Отдельного модуля прогона нет: `simulation.*`, `data_source.*`,
`filter_settings.*` и `ini_parser.*` удалены — движок живёт в `main.cpp`,
как в исходной версии `C:\ITT_Students`.

## Сборка

```bash
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build --target imitator
```

Единственная цель — `imitator`. Ни Qt, ни тестов, ни GUI в сборке нет.

То же через пресеты (ими пользуется VS Code, расширение CMake Tools):

```bash
cmake --preset mingw && cmake --build --preset mingw   # build/imitator.exe
cmake --preset ninja && cmake --build --preset ninja   # build/ninja/imitator.exe
```

В VS Code открывать каталог `C:\ITT_Students_NoQT`: `CMakePresets.json` +
`.vscode/` (settings/tasks/launch) задают генератор, компилятор MinGW-w64 и
задачи сборки — выбор kit не нужен.

## Ключевые правила работы с кодом

- **Никакого Qt.** Ядро — Qt-независимое: ни одного `#include <Q...>`.
- **Сохранять алгоритмы ESKF** (15 состояний). Матрица хранится построчно
  (row-major), число столбцов передаётся явно: `multiply_matrix(A,B,cols_A,cols_B)`,
  `transpose_m(A,cols)`, `multiply_m(A,v,cols_A)`, `at(A,i,j,cols)`.
- **Углы внутри — радианы, на выход — градусы** (`RAD_TO_DEG`).
- **Формат `errors.txt`** (`time` + `x0..x14`) не менять.
- Комментарии — на русском, без лишних восклицательных знаков.
- Минимальные правки: не переписывать фильтр без необходимости.
- После правок собирать `cmake --build build --target imitator`.

## Аналитика (в коде есть, в штатном запуске — нет)

Нововведённые диагностические возможности сохранены, но **по умолчанию
выключены** и включаются только явными флагами:

- `--bias-log` — писать `tools/bias.txt` (журнал оценок смещений ДУС/акселя);
- `--consistency` — писать `tools/consistency.txt` (согласованность фильтра, NIS);
- `--bias-report` — прогон со сводной таблицей по смещениям.

Штатный запуск `imitator <каталог_данных> [params.ini]` их не включает.

## Настройки

Тюнера и пресетов нет: `settings.ini` не читается, INI-ридер в проекте
отсутствует. Пороги сведения наклона заданы умолчаниями `TiltManeuverPolicy`,
шумы фильтра — умолчаниями `ins::kalman_cfg`.

Необязательный второй позиционный аргумент — плоский `params.ini` в формате
исходной версии (`sigma_g`, `sigma_a`, `sigma_bg`, `sigma_ba`, `sigma_pos`,
`sigma_v`, `sigma_ang`, `outage_start_s`, `outage_end_s`). Его разбирает сам
`main.cpp` (`applyParamsFile`).

## Связь с Qt-версией

Ядро (`src/`) совпадает с `C:\ITT_Students_Qt_Version\src\core\` (с точностью до
перехода от `../include` к плоским include). Выход консольного `imitator`
бит-в-бит совпадает с `imitator` Qt-версии на одном и том же ядре (проверено
сравнением md5 `result.txt`/`reference.txt`/`errors.txt`/`gnss_guard.txt`/
`bias.txt`/`consistency.txt`).

Телеметрии GUI в консольной версии нет (`StepTelemetry` удалён): NIS остаётся
доступен через `sink.wantsConsistency()`.

## Формат данных

- **imu.dat** (200 Гц): `time_s timestamp_ns wx wy wz ax ay az` — рад/с и м/с²
  в СК тела (X — вперёд, Y — вверх, Z — вправо).
- **gps.dat** (запись 200 Гц, обновление ~10 Гц): `time_s timestamp_ns lat lon alt
  vn vh ve` — градусы, метры, м/с.
- **angle.dat** (200 Гц): `time_s timestamp_ns roll pitch yaw` — радианы.
- **StartupNav.ini**: 4 строки — долгота(град), широта(град), высота(м), время
  выставки(с); без файла — координаты из первого отсчёта gps.dat, t=1 с.
  Окно выставки — от начала файла до `T_align` (файлы читаются с начала,
  перемотки нет; сейчас 1 с): средние и максимумы отклонения окна → уровень,
  курс, `ba`/`bg`, номинал, `x0`/`P0`;
  первая коррекция СНС — по первому кадру за окном
  (`docs/fast_alignment_plan.md`, `tools/init.txt`).

`gps.dat` и `angle.dat` читаются синхронно построчно (`SnsReader`).
