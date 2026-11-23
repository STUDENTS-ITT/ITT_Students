### Статус выполнения

- **Итерация A (2026-11-19, `src/`):** создан `src/common/` (`types.h`, `constants.h`), `normalGravity` выделён в `src/math_lib/gravity.h`, `src/utils/` приведён к правилу «только вспомогательные функции» (оставлен `paths.h`), proxy `src/math_lib/constants.hpp` удалён, legacy-навигация перенесена в `src/navigation/legacy/` и исключена из сборки, обновлены include-пути. Пересборка: `[100%] Built target imitator` (`build_align`).
- **Итерация B (2026-11-19, реструктуризация по плану):**
  - **Код/сборка:** `src/main.cpp` → `src/app/imitator/main.cpp`, `CMakeLists.txt` обновлён, сборка подтверждена.
  - **Документация:** разнесена по `docs/{protocols,guides,reports,plans,reference}` (включая корневые `Example.txt`, `sensor_frames.md`, руководство, PDF/figures из `report/`).
  - **Инструменты:** `tools/` → `tools/{analysis,pipelines,scenarios,reporting}`; обновлены пути/корни в ключевых скриптах (`run_all_HKairport03.py`, `plot_fusion_results.py`, `run_fusion_pipeline.py`, генераторы отчётов) и в инструментах эксперимента.
  - **Конфиги/данные:** добавлен `configs/calibration/camera.json`, `configs/datasets/HKairport03/StartupNav.ini`; данные нормализованы по набору `data/raw/HKairport03/` и `data/processed/HKairport03/` (`StartupNav.ini` сохранён в наборе для совместимости с текущим `imitator`).
  - **Результаты/тесты/эксперименты:** `results_fusion` → `results/fusion`, `results_HKairport03` → `results/HKairport03`, `test/test.md` → `tests/fixtures/README.md`, `add-kalman-report` → `experiments/add-kalman-report`.
  - **Корень:** обновлены ключевые пути в `README.md` и `AGENTS.md`.
- **Итерация C / Фаза 4 (2026-11-19, VO-домен):**
  - Принята стратегия размещения VO как доменного кода в `src/video/` (решение 3).
  - `vo_final2.py` → `src/video/vo_final2.py`, `vo_cpp/` → `src/video/vo/vo_cpp/`.
  - Добавлен `src/video/README.md` (границы модуля и интеграция).
  - Обновлены ссылки/пути в `tools/scenarios/run_all_HKairport03.py`, `tools/pipelines/run_fusion_pipeline.py`, `README.md`, `AGENTS.md`.
- **Отложено / требует решений и валидации:** эталонная регрессия выходов на контрольных данных (Фаза 0), полная проверка всех Python/VO/отчётных сценариев и сборки `vo_cpp` в новой раскладке, актуализация исторических описательных документов (напр. `docs/guides/file_functional_guide.md`), удаление `src/navigation/legacy/`, унификация хранения конфигов (`configs/` vs дубли в `data/raw/<dataset>/`). Рабочая копия не является git-репозиторием.

---

## 1. Цели

1. **Семантическая однозначность:** каждый файл и каталог имеет чёткую роль; размещение определяется назначением, а не историей появления.
2. **Разделение сущностей:** исходный код ≠ конфигурация ≠ данные ≠ результаты ≠ документация ≠ инструменты ≠ тесты ≠ эксперименты/архив.
3. **Чистая архитектура ядра C++:** модули разнесены по доменам (общие определения, I/O, INS/фильтр, математика, навигация, утилиты).
4. **Воспроизводимость:** входы, конфиги, сценарии запуска, выходы и отчёты имеют фиксированные и предсказуемые места.
5. **Управляемая эволюция:** legacy, эксперименты и тяжёлые артефакты изолированы от канонического кода.

## 2. Базовые принципы и правила

- **`src/common/`** — только межмодульные определения (типы, канонические константы-данные), без алгоритмов и без «хелперов».
- **`src/utils/`** — только вспомогательные функции общего назначения (пути/ФС, строки, простые утилиты). Запрещены: типы, доменные/проектные константы-данные, структуры состояния, алгоритмы предметной области.
- **`src/math_lib/`** — математика, преобразования, статистика и вычислительные модели (например, `gravity.h` / `normalGravity`).
- **`src/navigation/`** — навигационная логика, выставка, СНС, траектория, защита GNSS. Устаревшие реализации изолируются в `src/navigation/legacy/` до подтверждённого удаления.
- **`src/ins/`** — IMU-обработка, ориентация, модель/фильтр (ESKF).
- **`src/data_io/`** — чтение/запись/парсинг данных и ошибки I/O.
- **`src/video/`** — код предметной области видео/VO (при выделении); сценарные оркестраторы и вспомогательные скрипты запуска — в `tools/`.
- **Единый источник истины для констант:** без дублирования и прокси-цепочек; зависимости ацикличные.
- **Конфиги отдельно от кода и данных:** профили запуска, калибровки, параметры — в `configs/` (с явной привязкой к наборам/сценариям).
- **Данные отдельно от результатов:** `data/raw|processed` — входы/подготовка; `results/` — воспроизводимые выходы прогонов.
- **Документация по типу:** протоколы, руководства, отчёты, планы, справочные материалы — в соответствующих подпапках `docs/`.
- **Артефакты сборки и кэши не являются исходниками:** локальные `build/`, промежуточные выходы, временные файлы — вне канонической структуры исходников (игнорируются VCS).

## 3. Целевая структура проекта (эталон)

```text
ITT_Students-VO-FastFreak-homography/
├─ src/                                  # Канонический исходный код
│  ├─ app/                               # Точки входа и композиция приложений
│  │  └─ imitator/
│  │     └─ main.cpp                    # CLI/запуск БИНС+ESKF (бывш. src/main.cpp)
│  │
│  ├─ common/                            # Межмодульные определения (не helpers)
│  │  ├─ types.h                         # Matrix, Vector и др. общие типы/алиасы
│  │  └─ constants.h                     # Канонические физические/дискретизационные константы
│  │
│  ├─ ins/                               # Инерциальная навигация и фильтр
│  │  ├─ __init__.py                     # (если требуется совместимость с Python-обвязкой)
│  │  ├─ attitude_calc.h
│  │  ├─ imu_processor.h
│  │  └─ ins_filter.h                    # ESKF, Q/R/F/H, predict/correct, конфиг
│  │
│  ├─ navigation/                        # Навигация, выставка, СНС, траектория
│  │  ├─ aligner.h                       # Формирование x0/P0, начальная выставка (канон)
│  │  ├─ averaging.h
│  │  ├─ averaging.cpp                   # Окно усреднения, статистики, ba/bg, rest gate
│  │  ├─ gnss_guard.h                    # Валидация/защита GNSS (VALID/INVALID/STALE)
│  │  ├─ gps_processor.h                 # Типы/измерения СНС
│  │  ├─ position_calc.h                 # Кинематика/интегрирование координат и скоростей
│  │  ├─ trajectory.h                    # NavState, цикл step, интеграция с фильтром
│  │  └─ legacy/                         # Устаревшие реализации (до удаления)
│  │     ├─ Aligner.cpp
│  │     ├─ aligner.hpp
│  │     └─ bins_alignment.h
│  │
│  ├─ data_io/                           # Ввод/вывод и форматы
│  │  ├─ data_reader.h
│  │  ├─ data_reader.cpp
│  │  ├─ data_writer.h
│  │  ├─ data_writer.cpp
│  │  ├─ io_error.h
│  │  └─ file_parser.py                  # (если используется; иначе в tools/)
│  │
│  ├─ math_lib/                          # Математика и вычислительные модели
│  │  ├─ matrix_ops.h
│  │  ├─ transformations.h
│  │  ├─ interpolation.h
│  │  ├─ statistics.h
│  │  └─ gravity.h                       # normalGravity и связанные модели
│  │
│  ├─ video/                             # [опц., итеративно] Код VO/видео-домена
│  │  ├─ vo/                             # Нативная/обёрточная реализация VO
│  │  │  └─ vo_cpp/                      # Переносимый подпроект vo_cpp (apps/include/src/tools)
│  │  ├─ vo_final2.py                    # Python VO-пайплайн (если относится к domain-коду)
│  │  └─ README.md                       # Границы модуля и контракты интеграции
│  │
│  └─ utils/                             # Только вспомогательные функции
│     ├─ paths.h                         # Разрешение путей/каталогов данных
│     └─ (другие helpers общего назначения при необходимости)
│
├─ configs/                              # Конфигурации, профили, калибровки
│  ├─ datasets/
│  │  └─ HKairport03/
│  │     ├─ StartupNav.ini               # Параметры выставки/инициализации набора
│  │     └─ params.ini                   # Параметры фильтра/прогона (если применимо)
│  ├─ calibration/
│  │  └─ camera.json
│  └─ scenarios/                         # [опц.] профильные конфиги сценариев
│
├─ data/                                 # Входные и подготовленные данные
│  ├─ raw/
│  │  └─ HKairport03/
│  │     ├─ bag_info.json
│  │     ├─ raw.md
│  │     ├─ camera/
│  │     │  ├─ timestamps.csv
│  │     │  └─ timestamps.csv.md
│  │     ├─ gps/
│  │     ├─ imu/
│  │     ├─ other/
│  │     ├─ time_sync/
│  │     └─ reference/
│  │        ├─ altitude_per_frame.csv
│  │        └─ gps_reference.csv
│  └─ processed/
│     └─ HKairport03/
│        └─ data.md
│
├─ results/                              # Воспроизводимые выходы прогонов
│  ├─ fusion/
│  │  ├─ errors.txt
│  │  ├─ reference.txt
│  │  ├─ metrics_summary.json
│  │  ├─ timing.json
│  │  └─ plots/
│  │     ├─ fusion_comparison.png
│  │     └─ trajectory_comparison.png
│  └─ HKairport03/
│     └─ vo/
│        └─ homography/FAST-FREAK/       # (сохранение смысловой иерархии VO-результатов)
│
├─ tools/                                # Скрипты и утилиты вне канонического src
│  ├─ scenarios/
│  │  └─ run_all_HKairport03.py          # Оркестрация полного сценария HKairport03
│  ├─ analysis/
│  │  ├─ plot_trajectory.py
│  │  └─ plot_fusion_results.py
│  ├─ pipelines/
│  │  └─ run_fusion_pipeline.py
│  └─ reporting/
│     ├─ make_report.py
│     ├─ make_fusion_report.py
│     └─ make_tracking_takeoff_video.py
│
├─ tests/                                # Тесты и тестовые материалы
│  ├─ unit/
│  ├─ regression/
│  └─ fixtures/
│     └─ README.md
│
├─ docs/                                 # Документация проекта
│  ├─ protocols/                         # Форматы, СК, интерфейсы данных
│  │  ├─ Protocol_data.md
│  │  └─ sensor_frames.md
│  ├─ guides/                            # Руководства пользователя/разработчика
│  │  └─ Руководство_по_vo_final.docx
│  ├─ reports/                           # Отчёты, аналитика, фигуры
│  │  ├─ gnss_guard_report.md
│  │  ├─ alignment_theory_Alignment2019.md
│  │  ├─ VO_HKairport03.pdf
│  │  ├─ VO_INS_Fusion_HKairport03.pdf
│  │  └─ figures/
│  ├─ plans/                             # Планы, roadmap, архитектурные решения
│  │  ├─ restructuring_plan.md           # Данный план
│  │  ├─ alignment_x0_p0_transfer_plan.md
│  │  └─ gnss_guard_development_plan.md
│  └─ reference/                         # Справочные/эталонные материалы
│     ├─ Example.txt
│     └─ AirSim-1.pdf                    # (по роли можно оставить в reports/reference)
│
├─ experiments/                          # Изолированные эксперименты/альтернативные реализации
│  └─ add-kalman-report/                 # Полный подпроект-эксперимент (как есть внутри)
│
├─ archive/                              # [опц.] Устаревшие материалы, снятые с поддержки
├─ third_party/                          # [опц.] Внешние зависимости/вендоринг (если появятся)
├─ build/                                # Локальная сборка (в .gitignore)
├─ CMakeLists.txt
├─ README.md
├─ AGENTS.md
├─ .gitignore
└─ requirements.txt
```

> Примечание: `src/app/imitator/main.cpp` — целевое размещение точки входа; допустима совместимость через тонкий `src/main.cpp`-обёртку на переходный период.

## 4. Семантика ключевых каталогов

| Каталог | Назначение | Что должно находиться | Что не должно находиться |
|---|---|---|---|
| `src/common` | Общие определения | Типы, канонические константы-данные | Функции-хелперы, алгоритмы, I/O, доменная логика |
| `src/utils` | Вспомогательные функции | `paths`, общие утилиты без предметной логики | Типы, константы-данные, навигация/фильтр/модели |
| `src/ins` | Инерциальный контур | IMU, ориентация, ESKF, матрицы/шаги фильтра | Чтение файлов, UI, отчёты, оркестрация сценариев |
| `src/navigation` | Навигационный контур | Выставка, траектория, позиционирование, GNSS guard | Конфиги/данные, генерация отчётов, VO-пайплайн |
| `src/data_io` | Границы ввода/вывода | Парсинг, чтение, запись, ошибки I/O | Алгоритмы оценивания/навигации |
| `src/math_lib` | Математическая база | Лин. алгебра, преобразования, статистика, модели | Доменные состояния, файловый I/O |
| `configs` | Конфигурация | INI/JSON профилей, калибровки | Сырые измерения, выходы прогонов |
| `data` | Данные | `raw/` и `processed/` по наборам | Код, отчёты, итоговые сводки прогонов |
| `results` | Выходы | Логи/CSV/метрики/графики прогонов | Исходники, сырые датасеты, руководства |
| `tools` | Инструментарий | Скрипты запуска, анализа, конвертации, отчётности | Каноническое ядро алгоритмов (кроме явных обёрток) |
| `docs` | Документация | Протоколы, гайды, отчёты, планы, справка | Исполняемые артефакты прогонов, сырые данные |
| `tests` | Проверки | Unit/regression/fixtures | Большие «сырые» наборы, прод-выходы |
| `experiments` | Исследования | Изолированные альтернативы и подпроекты | Канонический production-код ядра |

## 5. Текущее состояние и соответствие целевой структуре

### 5.1. Уже приведено к целевой семантике (в `src/`, итерация A)
- `src/common/types.h`, `src/common/constants.h` — созданы и используются.
- `src/math_lib/gravity.h` — выделен (`normalGravity`).
- `src/utils/paths.h` — единственный helper в `src/utils/`.
- `src/navigation/legacy/{Aligner.cpp,aligner.hpp,bins_alignment.h}` — legacy изолирован.
- Прокси `src/math_lib/constants.hpp` — удалён.
- `CMakeLists.txt` и include-пути обновлены, `imitator` собирается (`build_align`, 2026-11-19).

### 5.2. Частично выполнено / требует доработки (после итераций B–C)
- **Точка входа:** `src/main.cpp` → `src/app/imitator/main.cpp` выполнено; shim `src/main.cpp` не добавлялся (открытое решение 1).
- **Конфиги:** введён `configs/` (`configs/calibration/camera.json`, `configs/datasets/HKairport03/StartupNav.ini`); для совместимости `StartupNav.ini` также сохранён в `data/raw/HKairport03/` (требуется решение по единому источнику, см. разд. 8).
- **Данные HKairport03:** корневые датасетные каталоги/файлы перенесены в `data/raw/HKairport03/...` (+ `reference/`), `data/processed/data.md` → `data/processed/HKairport03/data.md`.
- **Результаты:** `results_fusion/` → `results/fusion/`, `results_HKairport03/` → `results/HKairport03/`.
- **Инструменты:** разнесены в `tools/{analysis,pipelines,scenarios,reporting}/`; ключевые скрипты и генераторы отчётов обновлены, но требуется полная runtime-проверка всех сценариев.
- **Документация:** базовая таксономия `docs/{protocols,guides,reports,plans,reference}` применена; исторические/описательные документы (напр. `docs/guides/file_functional_guide.md`) могут содержать устаревшие пути и требуют актуализации.
- **VO-домен:** выполнено размещение в `src/video/` — `src/video/vo_final2.py`, `src/video/vo/vo_cpp/`, `src/video/README.md`; основные проектные ссылки обновлены, требуется валидация запуска/сборки VO в новой раскладке.
- **Эксперимент:** `add-kalman-report/` → `experiments/add-kalman-report/` выполнено; проектные ссылки обновлены точечно.
- **Тестовые материалы:** `test/test.md` → `tests/fixtures/README.md` выполнено.
- **Артефакты сборки:** `build_align/` остаётся локальным артефактом.

## 6. Отображение «текущее → целевое» (сводно)

| Текущее | Целевое | Примечание |
|---|---|---|
| `src/main.cpp` | `src/app/imitator/main.cpp` | допустим временный shim |
| `src/common/{types.h,constants.h}` | `src/common/{types.h,constants.h}` | соответствует (готово) |
| `src/math_lib/gravity.h` | `src/math_lib/gravity.h` | соответствует (готово) |
| `src/utils/paths.h` | `src/utils/paths.h` | соответствует (готово) |
| `src/navigation/legacy/*` | `src/navigation/legacy/*` | соответствует (готово), удалить после верификации |
| `src/{ins,data_io,math_lib,navigation}/*` (актуальные) | те же доменные каталоги | сохранить роли, уточнить границы при необходимости |
| `data/raw/StartupNav.ini` | `configs/datasets/HKairport03/StartupNav.ini` | конфиг, привязанный к набору |
| `camera/*`, `gps/*`, `imu/*`, `other/*`, `time_sync/*` | `data/raw/HKairport03/{camera,gps,imu,other,time_sync}/` | сырые потоки набора |
| `altitude_per_frame.csv`, `gps_reference.csv` | `data/raw/HKairport03/reference/` | входные референсы |
| `bag_info.json` | `data/raw/HKairport03/bag_info.json` | метаданные набора |
| `data/processed/data.md` | `data/processed/HKairport03/data.md` | нормализация по набору |
| `calibration/camera.json` | `configs/calibration/camera.json` | калибровка, не raw-измерение |
| `results_fusion/*` | `results/fusion/*` (+ `plots/`) | группировка артефактов |
| `results_HKairport03/*` | `results/HKairport03/*` | сохранить смысловую иерархию |
| `run_all_HKairport03.py` | `tools/scenarios/run_all_HKairport03.py` | оркестрация |
| `tools/plot_*.py`, `tools/run_fusion_pipeline.py` | `tools/analysis/*`, `tools/pipelines/*` | разделение по роли |
| `report/make_*.py` | `tools/reporting/*` | генераторы — инструменты |
| `report/*.pdf`, `report/figures/*`, `report/*.txt` | `docs/reports/*` или `results/*` | по типу артефакта |
| `docs/AirSim-1.pdf` | `docs/reference/AirSim-1.pdf` (или `reports/`) | по роли |
| `docs/Protocol_data.md` | `docs/protocols/Protocol_data.md` | протокол |
| `docs/alignment_theory_Alignment2019.md` | `docs/reports/alignment_theory_Alignment2019.md` | аналитический материал |
| `docs/alignment_x0_p0_transfer_plan.md` | `docs/plans/alignment_x0_p0_transfer_plan.md` | план |
| `docs/gnss_guard_development_plan.md` | `docs/plans/gnss_guard_development_plan.md` | план |
| `docs/gnss_guard_report.md` | `docs/reports/gnss_guard_report.md` | отчёт |
| `docs/restructuring_plan.md` | `docs/plans/restructuring_plan.md` | данный документ |
| `Example.txt` | `docs/reference/Example.txt` | справочный эталон |
| `sensor_frames.md` | `docs/protocols/sensor_frames.md` | спецификация СК |
| `Руководство_по_vo_final.docx` | `docs/guides/Руководство_по_vo_final.docx` | руководство |
| `vo_final2.py` | `src/video/vo_final2.py` (или `tools/video/`) | по стратегии VO |
| `vo_cpp/**` | `src/video/vo/vo_cpp/**` (или `tools/video/vo_cpp/**`) | по стратегии VO |
| `add-kalman-report/**` | `experiments/add-kalman-report/**` | изолированный эксперимент |
| `test/test.md` | `tests/fixtures/README.md` | тестовые материалы |
| `build_align/**` | `build/` (локально, игнор) | не мигрировать содержимое |

## 7. Фазы миграции (рекомендуемый порядок)

### Фаза 0 — Подготовка
- Зафиксировать рабочий эталон (сборка, контрольный прогон, ключевые выходы/метрики).
- Выполнять миграцию атомарными шагами; при возможности — в git-ветке/копии.
- Согласовать открытые решения (раздел 8).

### Фаза 1 — Завершение нормализации канонического C++-кода
- Перенести `src/main.cpp` в `src/app/imitator/main.cpp` (обновить `CMakeLists.txt`; опционально оставить совместимую обёртку).
- Подтвердить отсутствие не-helper сущностей в `src/utils/`.
- Принять решение об удалении `src/navigation/legacy/` после регрессии.

### Фаза 2 — Конфиги, данные, результаты
- Ввести `configs/` и перенести профили/калибровки.
- Сгруппировать `data/raw|processed` по наборам (`HKairport03` и др.).
- Нормализовать `results/` (разделить логи/метрики/графики по сценариям).

### Фаза 3 — Инструменты, тесты, документация
- Разнести `tools/` по ролям (`scenarios`, `analysis`, `pipelines`, `reporting`).
- Привести `docs/` к таксономии `protocols/guides/reports/plans/reference`.
- Нормализовать `tests/` (`unit`, `regression`, `fixtures`).

### Фаза 4 — VO-домен и эксперименты
- Принять и реализовать стратегию размещения `vo_cpp/` и `vo_final2.py`.
- Перенести `add-kalman-report/` в `experiments/` (или по решению).
- Обновить скрипты, пути, документацию интеграции INS↔VO.

### Фаза 5 — Финализация
- Обновить `README.md`, `AGENTS.md`, `.gitignore`, `CMakeLists.txt`, пути в скриптах/конфигах.
- Выполнить регрессию (сборка, контрольные сценарии, сравнение выходов/метрик).
- Удалить подтверждённо неиспользуемый legacy и временные совместимости.

## 8. Открытые решения (требуют явного выбора)

1. **Размещение точки входа:** `src/app/imitator/main.cpp` с обязательным shim `src/main.cpp` или только новый путь.
2. **Конфиги наборов:** `configs/datasets/<dataset>/` vs хранение рядом с `data/raw/<dataset>/`.
3. ~~**Стратегия VO:** `src/video/` как доменный код (с `vo/vo_cpp`) vs вынос реализации в `tools/video/` при тонких контрактах в `src/`.~~ **Решено (2026-11-19):** принят вариант `src/video/` (`src/video/vo_final2.py`, `src/video/vo/vo_cpp/`, `src/video/README.md`).
4. **Оркестрация сценариев:** хранить только в `tools/scenarios/` (рекомендуется) или дублировать entrypoints в корне.
5. **Отчётные артефакты:** финальные PDF/figures в `docs/reports/` (документация) vs в `results/.../reports/` (артефакты прогонов).
6. **`add-kalman-report/`:** `experiments/` в репозитории, `archive/` или вне дерева/отдельный репозиторий.
7. **Удаление legacy:** момент удаления `src/navigation/legacy/*` после регрессии и подтверждения неиспользуемости.
8. **Политика сборочных/крупных артефактов:** `build*/`, `results/` (полностью/частично), бинарные отчёты и архивы — в git или только локально/во внешнем хранилище.
9. **Поддержка Python-частей в `src/`:** оставить `src/ins/__init__.py`, `src/data_io/file_parser.py` в каноническом коде или перенести в `tools/`/пакетную структуру по фактическому использованию.

## 9. Критерии готовности целевой структуры

- [ ] Все элементы проекта размещены согласно разделам 3–6 (или явно зафиксированы исключения).
- [ ] В `src/utils/` отсутствуют типы, константы-данные и доменные алгоритмы.
- [ ] Нет дублирующих источников констант/прокси; include-граф ацикличен.
- [ ] `CMakeLists.txt`, скрипты и конфиги используют актуальные пути.
- [ ] `imitator` собирается из новой структуры; контрольные прогоны воспроизводимы (в согласованных допусках).
- [ ] Данные, результаты, конфиги, инструменты, тесты и документация физически и семантически разделены.
- [ ] Legacy и эксперименты изолированы и не смешиваются с каноническим `src/`.

---

**Примечание по текущему состоянию:** семантика `src/` (common/utils/math_lib/navigation legacy, канонические константы/типы/gravity) и большая часть целевой физической структуры (docs/tools/configs/data/results/tests/experiments, перенос `main.cpp`, нормализация ключевых путей в скриптах) приведены в соответствие плану (итерации A–B). Остаются открытые решения и валидация: стратегия `vo_cpp`/`vo_final2.py`, полная регрессия сценариев/выходов, актуализация исторических описательных документов и унификация конфигов.