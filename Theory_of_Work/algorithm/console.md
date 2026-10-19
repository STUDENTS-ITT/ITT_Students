# `src/console/` — консольный запуск (`imitator`)

Тонкая оболочка вокруг ядра: разбор аргументов, поиск каталогов, вызов
`nav::runSimulation`. Вся математика — в `esfcore`, поэтому файлы результатов
консоли и Qt-приложения совпадают.

---

## `console/main.cpp`

Единственная функция:

- `int main(int argc, char **argv)`:
  1. разбор аргументов: флаги `--bias-log`, `--bias-report`, `--consistency`
     распознаются в любой позиции, остальное — позиционные аргументы
     (сохраняется старый вызов `imitator <каталог> <settings.ini>`);
  2. `data_dir = positional[0]` (по умолчанию `../data/raw`) — каталог данных;
  3. `settings_file = positional[1]` (по умолчанию пусто) — `settings.ini`
     (пусто — значения по умолчанию). Совместимость со старым плоским
     `params.ini` обеспечивает сам runner;
  4. `tools_dir = utils::toolsDir(argv[0])` — каталог для результатов;
  5. заполняет `nav::RunOptions`: `data_dir`, `output_dir = tools_dir`,
     `settings_file`, `write_files = true`, `write_bias_log`, `write_consistency`,
     `capture_telemetry = false`;
  6. при `--bias-report` вызывает `nav::runBiasReport(...)`, печатает таблицу в
     `stdout` (плюс пишет `bias.txt` и обычные три файла);
  7. иначе `nav::runSimulation(options, summary, nullptr, nullptr, &std::cout)` —
     вывод лога в `stdout`, без наблюдателя и отмены;
  8. при ошибке печатает `imitator: run failed` в `stderr` и возвращает 1;
     иначе печатает `summary.wall_seconds` и возвращает 0.

Использование:

```
imitator [--bias-log] [--bias-report] [--consistency] [каталог_данных] [settings.ini]
```

Каталог данных по умолчанию — `../data/raw`. Результаты (`result.txt`,
`reference.txt`, `errors.txt`, `gnss_guard.txt`) пишутся в `tools/` рядом с
исполняемым файлом (см. `utils::toolsDir` в [`utils.md`](utils.md)).

Дополнительные файлы:

- `--bias-report` → `bias.txt` (журнал смещений ИМУ) и таблица в `stdout`;
- `--consistency` → `consistency.txt` (NIS на кадрах коррекции по СНС).
  Флаг выключен по умолчанию: без него ядро не считает NIS и не тратит время
  (важно для тюнера, который делает тысячи прогонов).

---

## Сборка (`CMake`)

Ядро и консоль собираются двумя целями CMake (см. `src/core/CMakeLists.txt` и
`src/console/CMakeLists.txt`, а также корневой `CMakeLists.txt`).

### `esfcore` — статическая библиотека (Qt-независимое ядро)

Компилируются только `.cpp` (заголовки подключены по месту):

- `data_io/data_reader.cpp`
- `data_io/data_source.cpp`
- `data_io/data_writer.cpp`
- `data_io/dataset_generator.cpp`
- `navigation/Aligner.cpp`
- `navigation/filter_settings.cpp`
- `navigation/simulation.cpp`
- `utils/ini_parser.cpp`

Свойства:
- `target_include_directories(esfcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)` —
  корень `src/`, поэтому включают как `"core/..."`;
- `target_compile_features(esfcore PUBLIC cxx_std_17)`;
- `POSITION_INDEPENDENT_CODE ON`;
- MSVC: `/utf-8`.

Правило проекта: в этом каталоге не должно быть ни одного `#include <Q...>`;
проверяется ctest-тестом `core_has_no_qt`
(`tools/check_no_qt_in_core.cmake`).

### `imitator` — консольная цель

- `add_executable(imitator main.cpp)`;
- `target_link_libraries(imitator PRIVATE esfcore)`;
- `target_include_directories(imitator PRIVATE ${CMAKE_SOURCE_DIR}/src)`;
- C++17; MSVC: `/utf-8`.

GUI-цель `imitator_gui` линкуется с тем же `esfcore`; её код в этом руководстве
не разбирается.

Пример сборки:

```powershell
cmake --build build --target imitator
# или
cmake --build build --target imitator_gui
```
