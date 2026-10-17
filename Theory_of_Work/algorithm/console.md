# `src/console/` — консольный запуск (`imitator`)

Тонкая оболочка вокруг ядра: разбор аргументов, поиск каталогов, вызов
`nav::runSimulation`. Вся математика — в `esfcore`, поэтому файлы результатов
консоли и Qt-приложения совпадают.

---

## `console/main.cpp`

Единственная функция:

- `int main(int argc, char **argv)`:
  1. `data_dir = (argc > 1) ? argv[1] : "../data/raw"` — каталог данных;
  2. `settings_file = (argc > 2) ? argv[2] : ""` — `settings.ini`
     (пусто — значения по умолчанию). Совместимость со старым плоским
     `params.ini` обеспечивает сам runner;
  3. `tools_dir = utils::toolsDir(argv[0])` — каталог для результатов;
  4. заполняет `nav::RunOptions`: `data_dir`, `output_dir = tools_dir`,
     `settings_file`, `write_files = true`, `capture_telemetry = false`;
  5. `nav::runSimulation(options, summary, nullptr, nullptr, &std::cout)` —
     вывод лога в `stdout`, без наблюдателя и отмены;
  6. при ошибке печатает `imitator: run failed` в `stderr` и возвращает 1;
     иначе печатает `summary.wall_seconds` и возвращает 0.

Использование:

```
imitator [каталог_данных] [settings.ini]
```

Каталог данных по умолчанию — `../data/raw`. Результаты (`result.txt`,
`reference.txt`, `errors.txt`, `gnss_guard.txt`) пишутся в `tools/` рядом с
исполняемым файлом (см. `utils::toolsDir` в [`utils.md`](utils.md)).

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
