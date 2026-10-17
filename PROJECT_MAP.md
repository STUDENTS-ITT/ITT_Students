# PROJECT_MAP — что где лежит

Карта репозитория `ITT_Students_Qt_Version`. Отвечает на вопрос «что в каком каталоге
и почему», отделяя исходники (в git) от данных, сборок и сгенерированных артефактов
(вне git). Краткий состав и команды — в `README.md`; архитектура — в `DEVELOPER_GUIDE.md`;
правила работы — в `AGENTS.md`. Правила игнорирования — в `.gitignore`.

Проект: Qt-GUI-обвязка над Qt-независимым ядром ESKF/БИНС-СНС. Ядро (`esfcore`) общее
для консоли `imitator` и GUI `imitator_gui`.

---

## 1. Код и сборка (в git)

| Путь | Назначение |
|---|---|
| `src/core/` | Qt-независимое ядро (C++17, STL), статическая библиотека `esfcore`. Внутри: `data_io/`, `ins/`, `math_lib/`, `navigation/`, `utils/` |
| `src/console/` | `main.cpp` консольной цели `imitator` |
| `src/gui/` | Qt 6 GUI-приложение `imitator_gui` (`core_bridge/`, `export/`, `models/`, `settings/`, `views/`, `workers/`); графика на `QPainter` |
| `tests/` | Тесты ctest (QtTest), включая `test_parity` и `core_has_no_qt` |
| `CMakeLists.txt` | Корневая сборка: цели `esfcore`, `imitator`, `imitator_gui`, тесты |
| `ci.ps1` | Локальный прогон: configure + build + ctest |

Правило: в `src/core/` не должно быть ни одного `#include <Q...>` (проверяет тест
`core_has_no_qt`, скрипт `tools/check_no_qt_in_core.cmake`).

## 2. Документация (в git)

| Путь | Назначение |
|---|---|
| `README.md` | Краткий состав, сборка, запуск |
| `Theory_of_Work/` | Среда документации: `user/` (руководства пользователя), `algorithm/` (руководство по алгоритму), `theory/` (теоретическая справка) |
| `DEVELOPER_GUIDE.md` | Архитектура, ключевые классы, сборка, отладка |
| `AGENTS.md` | Контекст для ИИ (ядро ESKF, конвенции) |
| `PROJECT_MAP.md` | Этот файл |
| `docs/` | Проектная документация: `DEVELOPMENT_PLAN.md`, `PROJECT_REPORT.md`, дизайн-документы (`gnss_guard_design.md`, `navigation_enhancements.md`, …) |
| `FUTURE_PROPOSALS.md`, `BackToTheFuture.md`, `eskf_brief_report.md` | Планы, сводки, история решений |

## 3. Инструменты и пресеты (в git)

| Путь | Назначение |
|---|---|
| `tools/kalman_tuner/` | Подбор коэффициентов КФ: `tune_kalman.py`, `tuner_config*.json`, запись прогонов `full_runs/`, конфиги истины `truth_cfg*` |
| `tools/plot_trajectory.py`, `plot_errors.py`, `compare_truth.py` | Постобработка и сравнение результатов |
| `tools/check_no_qt_in_core.cmake` | Проверка отсутствия Qt в `src/core/` |
| `presets/tuned_*.ini` | Пресеты коэффициентов (в т.ч. `[meta] data_dir`) |

## 4. Данные и артефакты вне git

Эти каталоги/файлы намеренно не коммитятся (крупные, бинарные или генерируемые).
Часть из них нужна тестам и пресетам — пути `data_dir` заданы абсолютно.

| Путь | Что это | Воспроизводимость |
|---|---|---|
| `data/raw/` | Входные данные: `imu.dat` (200 Гц), `gps.dat` (5 Гц), `angle.dat` (100 Гц), `StartupNav.ini` | внешний набор; нужен `test_parity` (тест не регистрируется без `data/raw/imu.dat`) |
| `data/processed/Aligner.dat` | Дамп выставки (на расчёт не влияет) | создаётся прогоном |
| `run_hk/data/raw/` | Копия `data/raw` (идентична) + `StartupNav.ini` | дефолт `presets/tuned.ini` и `tools/kalman_tuner/tuner_config.json` |
| `ITT_Students-Katya/` | Корпус наборов: `dataset_shum/*` (9 наборов + `truth.txt`), `imitator_sq/test/line_dataset`, `step_session_1791149911/step_session` | данные; ссылки из `presets/tuned_*.ini`, `tools/kalman_tuner/tuner_config*.json`, `tests/CMakeLists.txt` |
| `reports/` | Сгенерированные PDF-отчёты по наборам (в git только `reports/README.md`) | собирает `imitator_gui` |
| `out_first_run/` | Пример вывода из `FIRST_RUN.md` | создаётся при первом запуске |
| `dist/` | Portable-сборка GUI (exe + Qt-DLL + плагины + доки) | `windeployqt` из `build/` |
| `imitator_gui_portable.zip` | Упакованный `dist/` | выведен из git (`git rm --cached`, 2026-10-17); публикуется в GitHub Releases (`v1.0-qt-gui`) |
| `build/`, `build_debug/` | Каталоги сборки (Release / Debug) | CMake |
| `.venv/` | Python-окружение для `tools/*.py` (numpy, pandas, matplotlib, scipy, PIL) | `python -m venv` + пакеты |
| `Report_*.docx`, `*.zip` | Черновики отчётов и архивы | генерируемые |

`tools/kalman_tuner/temp_tune*/` — временные каталоги подбора (создаются и удаляются
тюнером), намеренно игнорируются и не хранятся.

## 5. Первичный материал (в git, вне сборки)

Справочные материалы, не входящие в сборку и не используемые тестами. Хранятся как
первоисточники исходной (до-Qt) линии проекта.

| Путь | Что это |
|---|---|
| `SINS/` | Исходники БИНС на углах Эйлера–Крылова (предок `src/core/`), `docs/` |
| `Smoothing/` | Исходники сглаживания временных рядов (Python/C++), графики |
| `Documentation/` | Описание методов БИНС и сглаживания (PDF/tex/md); 9 файлов дублируют `SINS/docs/` |

## 6. Сборка, запуск, тесты

```bash
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH="C:/QT6.10/6.12.0/mingw_64"
cmake --build build
./build/imitator.exe        # консоль (данные по умолчанию ../data/raw/)
./build/imitator_gui.exe    # GUI
ctest --test-dir build --output-on-failure
pwsh -File ci.ps1           # configure + build + ctest
```

## 7. Куда смотреть дальше

- краткий состав и команды — `README.md`;
- архитектура и отладка — `DEVELOPER_GUIDE.md`;
- конвенции ядра ESKF — `AGENTS.md`;
- форматы входных/выходных файлов — `Theory_of_Work/user/DATA_FORMATS.md`;
- руководства пользователя — `Theory_of_Work/user/`;
- план развития — `docs/DEVELOPMENT_PLAN.md`, `FUTURE_PROPOSALS.md`.
