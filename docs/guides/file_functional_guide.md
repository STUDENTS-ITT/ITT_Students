# Подробное руководство по файлам проекта

**Область:** `C:\ITT_Students-VO-FastFreak-homography\ITT_Students-VO-FastFreak-homography`  
**Назначение документа:** описать фактическую структуру рабочей копии, назначение файлов и ключевых каталогов.  
**Актуализировано:** 2026-11-23

## Примечания

- Документ отражает текущее состояние рабочей копии после реструктуризации (итерации A–C по `docs/plans/restructuring_plan.md`).
- Для бинарных/крупных артефактов (`.pdf`, `.png`, `.docx`, `.exe`, наборы CSV/изображений) описание даётся по роли; перечисляются ключевые файлы и группы.
- Каталог `build_align/` — локальные артефакты сборки; в руководство по исходникам не включается.
- `experiments/add-kalman-report/` сохранён как экспериментальный контур/снимок для сравнения и воспроизводимости.

---

## 1. Корень репозитория

| Путь | Тип | Назначение |
|---|---|---|
| `.gitignore` | файл | Правила исключения артефактов сборки, локальных результатов, временных файлов и сред. |
| `AGENTS.md` | файл | Локальные инструкции для агентов/разработчиков: карта репозитория, критические факты по VO, команды запуска, оговорки. |
| `CMakeLists.txt` | файл | Сборка C++ цели `imitator` (C++17): `src/app/imitator/main.cpp`, `src/navigation/averaging.cpp`, `src/data_io/data_reader.cpp`, `src/data_io/data_writer.cpp`; include root — `src`. |
| `README.md` | файл | Пользовательский обзор: зависимости, данные, сборка/прогон фильтра, повторный расчёт VO, основные команды. |
| `requirements.txt` | файл | Python-зависимости для анализа/отчётов/скриптов (`numpy`, `matplotlib`, `pillow`, `fpdf2`). |

## 2. Конфигурация (`configs/`)

| Путь | Назначение |
|---|---|
| `configs/calibration/camera.json` | Параметры калибровки камеры для VO-пайплайна (intrinsics/distortion и связанные параметры). |
| `configs/datasets/HKairport03/StartupNav.ini` | Конфигурация запуска/выставки БИНС для датасета HKairport03. Дублируется в `data/raw/HKairport03/StartupNav.ini` для совместимости с текущими сценариями запуска. |

## 3. Данные (`data/`)

### 3.1. `data/raw/HKairport03/`

Сырые данные датасета HKairport03, разложенные по подсистемам.

| Путь/группа | Назначение |
|---|---|
| `bag_info.json` | Метаданные исходного bag/набора данных. |
| `StartupNav.ini` | Копия конфигурации выставки/запуска для совместимости с запуском из каталога данных. |
| `raw.md` | Описание/заметки по сырым данным. |
| `camera/timestamps.csv`, `camera/timestamps.csv.md` | Таймстемпы кадров камеры и их описание. Сами изображения в git обычно не хранятся. |
| `gps/*.csv`, `gps/*.csv.md` | GPS/RTK-потоки DJI/OSDK (позиция, скорость, здоровье, RTK-статус/yaw/position/velocity и др.) и описания форматов. |
| `imu/*.csv`, `imu/*.csv.md` | IMU-потоки (`dji_osdk_ros_imu.csv`, `livox_imu.csv`) и описания. |
| `other/*.csv`, `other/*.csv.md` | Вспомогательные потоки: attitude, gimbal, height_above_takeoff, local/vo position, velocity, RC, battery, flight status/anomaly и др. |
| `reference/altitude_per_frame.csv` | Высота по кадрам для масштабирования/оценки VO. |
| `reference/gps_reference.csv` | RTK/GPS-референс для сравнения траекторий и метрик. |
| `time_sync/*.csv`, `time_sync/*.csv.md` | Данные синхронизации времени (FC UTC, NMEA, PPS source) и описания. |

### 3.2. `data/processed/HKairport03/`

| Путь | Назначение |
|---|---|
| `data.md` | Описание обработанных/производных данных датасета. |

## 4. Документация (`docs/`)

### 4.1. Руководства

| Путь | Назначение |
|---|---|
| `docs/guides/file_functional_guide.md` | Этот файл: актуальный функциональный гид по структуре и файлам. |
| `docs/guides/Руководство_по_vo_final.docx` | Отдельное руководство по `vo_final` (двоичный документ Word; содержимое не индексируется как текст). |

### 4.2. Планы

| Путь | Назначение |
|---|---|
| `docs/plans/restructuring_plan.md` | План семантической реструктуризации, статус итераций A–C, открытые решения и карта переноса. |
| `docs/plans/alignment_x0_p0_transfer_plan.md` | План переноса/уточнения выставки и инициализации `x0`, `P0`, окна усреднения. |
| `docs/plans/gnss_guard_development_plan.md` | План развития защиты GNSS/SNS, диагностики и критериев приёмки. |

### 4.3. Протоколы и справка

| Путь | Назначение |
|---|---|
| `docs/protocols/Protocol_data.md` | Протокол/описание форматов и потоков данных. |
| `docs/protocols/sensor_frames.md` | Описание систем координат и осей датчиков. |
| `docs/reference/AirSim-1.pdf` | Внешний справочный материал. |
| `docs/reference/Example.txt` | Пример/справочный текстовый артефакт. |

### 4.4. Отчёты

| Путь | Назначение |
|---|---|
| `docs/reports/VO_HKairport03.pdf` | Отчёт по VO на HKairport03. |
| `docs/reports/VO_INS_Fusion_HKairport03.pdf` | Отчёт по комплексированию VO+INS+Kalman на HKairport03. |
| `docs/reports/alignment_theory_Alignment2019.md` | Теоретическая заметка/сопоставление по выставке. |
| `docs/reports/gnss_guard_report.md` | Отчёт по реализации и интеграции защиты СНС. |
| `docs/reports/tracking_timing.txt` | Тайминги/замеры трекинга. |
| `docs/reports/figures/*.png` | Графики и иллюстрации для отчётов (траектории, ошибки, панели fusion, кадры). |

## 5. Экспериментальный контур (`experiments/`)

### 5.1. `experiments/add-kalman-report/`

Сохранённый экспериментальный снимок/контур для отладки и сравнения Kalman/INS-пайплайна.

| Путь/группа | Назначение |
|---|---|
| `CMakeLists.txt`, `README.md`, `requirements.txt`, `.gitignore` | Сборка, инструкции и локальные зависимости экспериментального контура. |
| `data/vo_hk03/params.ini`, `data/vo_hk03_key/params.ini`, `data/vo_hk03_key/StartupNav.ini` | Конфигурации и параметры прогонов. |
| `docs/kalman_filter_report.md`, `docs/Protocol_data.md` | Локальная документация эксперимента. |
| `src/main.cpp` | Точка входа экспериментального прогона. |
| `src/data_io/*` | I/O для экспериментального контура. |
| `src/ins/*` | INS/attitude/filter-компоненты экспериментального контура. |
| `src/math_lib/*` | Математические утилиты экспериментального контура. |
| `src/navigation/*` | Навигационные компоненты, включая `maneuver_tilt.h` и варианты aligner. |
| `src/utils/{constants.h,paths.h,types.h}` | Утилиты/типы/константы внутри экспериментального снимка (историческая структура). |
| `test/test.md` | Тестовая/проверочная заметка. |
| `tools/plot_trajectory.py`, `tools/prepare_vo_inputs.py`, `tools/run_eval.ps1` | Подготовка входов, прогон оценки и визуализация. |
| `Отчёт_о_проблемах_и_работе.md` | Локальный отчёт о проблемах/работе (исторический артефакт). |

## 6. Результаты (`results/`)

| Путь/группа | Назначение |
|---|---|
| `results/fusion/errors.txt` | Ошибки/невязки прогона fusion. |
| `results/fusion/reference.txt` | Референсные значения/траектория для сравнения. |
| `results/fusion/metrics_summary.json` | Сводные метрики. |
| `results/fusion/timing.json` | Тайминги выполнения. |
| `results/fusion/fusion_comparison.png`, `trajectory_comparison.png` | Графики сравнения траекторий/ошибок. |
| `results/HKairport03/homography/visual_odometry/FAST-FREAK/*` | Результаты VO-прогона HKairport03 (траектории CSV, графики PNG, `vo_track_gps.csv`, `console_log.txt`). |

## 7. Исходный код (`src/`)

### 7.1. Точка входа приложения

| Путь | Назначение |
|---|---|
| `src/app/imitator/main.cpp` | CLI/оркестрация прогона БИНС: чтение входов, выставка/усреднение, инициализация состояния, основной цикл `nav::step`, запись выходов. |

### 7.2. Общие определения

| Путь | Назначение |
|---|---|
| `src/common/types.h` | Базовые типы линейной алгебры: `Matrix`, `Vector` (row-major хранение в `std::vector<double>`). |
| `src/common/constants.h` | Общие физические/математические константы и параметры, используемые несколькими подсистемами. |

### 7.3. Ввод/вывод

| Путь | Назначение |
|---|---|
| `src/data_io/data_reader.h/.cpp` | Чтение входных данных (`imu.dat`, `gps.dat`, `angle.dat` и связанных форматов) в структуры проекта. |
| `src/data_io/data_writer.h/.cpp` | Запись траекторий, ошибок и служебных выходов. |
| `src/data_io/file_parser.py` | Python-вспомогатель для разбора/подготовки файлов данных. |
| `src/data_io/io_error.h` | Типы/обработка ошибок I/O. |

### 7.4. INS / фильтр

| Путь | Назначение |
|---|---|
| `src/ins/attitude_calc.h` | Расчёт/обновление ориентации. |
| `src/ins/imu_processor.h` | Предобработка/интерпретация измерений IMU. |
| `src/ins/ins_filter.h` | ESKF/фильтр Калмана: состояние, матрицы, predict/correct, конфигурация. |
| `src/ins/__init__.py` | Маркер пакета для Python-контекста. |

### 7.5. Математическая библиотека

| Путь | Назначение |
|---|---|
| `src/math_lib/gravity.h` | Модель гравитации/`normalGravity`. |
| `src/math_lib/interpolation.h` | Интерполяционные утилиты. |
| `src/math_lib/matrix_ops.h` | Операции линейной алгебры над row-major матрицами/векторами. |
| `src/math_lib/statistics.h` | Статистические утилиты. |
| `src/math_lib/transformations.h` | Координатные/угловые преобразования. |

### 7.6. Навигация

| Путь | Назначение |
|---|---|
| `src/navigation/aligner.h` | Интерфейсы выставки/инициализации `x0`, `P0`, начального состояния. |
| `src/navigation/averaging.h/.cpp` | Окно усреднения для выставки: статистики IMU/SNS, оценки `ba/bg`, `rest`-гейт, расчёт сигм/ковариаций. |
| `src/navigation/gnss_guard.h` | Защита GNSS/SNS-измерений перед коррекцией (валидация, таймаут/STALЕ, причины отбраковки). |
| `src/navigation/gps_processor.h` | Структуры/обработка GNSS-измерений. |
| `src/navigation/position_calc.h` | Пересчёт/интегрирование координат и скоростей. |
| `src/navigation/trajectory.h` | `NavState`, основной шаг навигации `nav::step`, связь ИНС, фильтра и GNSS-коррекции. |
| `src/navigation/legacy/Aligner.cpp`, `aligner.hpp`, `bins_alignment.h` | Устаревшие реализации выставки/выравнивания; сохранены вне активной сборки до отдельного решения об удалении. |

### 7.7. Утилиты

| Путь | Назначение |
|---|---|
| `src/utils/paths.h` | Вспомогательная функция поиска каталога данных/пути относительно рабочей директории и расположения исполняемого файла. |

### 7.8. Видео/VO-домен (`src/video/`)

| Путь/группа | Назначение |
|---|---|
| `src/video/README.md` | Границы VO-домена и точки входа. |
| `src/video/vo_final2.py` | Основной Python-скрипт монокулярной VO (конфигурация и пайплайн в одном файле). |
| `src/video/vo/vo_cpp/CMakeLists.txt` | Сборка C++ VO-подпроекта. |
| `src/video/vo/vo_cpp/apps/vo_final.cpp` | CLI-приложение VO-прогона. |
| `src/video/vo/vo_cpp/apps/run_all_hkairport03.cpp` | Прогон VO-сценария HKairport03. |
| `src/video/vo/vo_cpp/include/vo/*.hpp` | Заголовки VO-модулей: конфиг, калибровка, признаки, matching, tracking, motion, pipeline, viewer и др. |
| `src/video/vo/vo_cpp/include/vo/fusion/kalman_filter.hpp` | Заголовок fusion/kalman-компонента в C++ VO-подпроекте. |
| `src/video/vo/vo_cpp/src/*.cpp` | Реализации VO-модулей. |
| `src/video/vo/vo_cpp/tools/export_for_ins.cpp` | Экспорт VO-данных для INS/интеграции. |
| `src/video/vo/vo_cpp/tools/score_hkairport03.py` | Оценка/скоринг результатов HKairport03. |
| `src/video/vo/vo_cpp/README.md`, `CHANGES.md`, `INS_EXPORT_README.md` | Документация C++ VO-подпроекта. |
| `src/video/vo/vo_cpp/.clangd`, `.gitignore` | Локальные настройки разработки/исключений подпроекта. |

## 8. Тестовые фикстуры (`tests/`)

| Путь | Назначение |
|---|---|
| `tests/fixtures/README.md` | Описание тестовых фикстур/минимальных наборов данных (при их наличии). |

## 9. Инструменты (`tools/`)

### 9.1. Анализ

| Путь | Назначение |
|---|---|
| `tools/analysis/plot_fusion_results.py` | Построение графиков сравнения/ошибок fusion. |
| `tools/analysis/plot_trajectory.py` | Визуализация траекторий. |

### 9.2. Пайплайны

| Путь | Назначение |
|---|---|
| `tools/pipelines/run_fusion_pipeline.py` | Запуск пайплайна комплексирования/подготовки результатов fusion. |

### 9.3. Сценарии

| Путь | Назначение |
|---|---|
| `tools/scenarios/run_all_HKairport03.py` | Подготовка входов HKairport03 и запуск VO-сценариев/бенчмарков. |

### 9.4. Отчётность

| Путь | Назначение |
|---|---|
| `tools/reporting/make_report.py` | Формирование PDF-отчёта по VO. |
| `tools/reporting/make_fusion_report.py` | Формирование PDF-отчёта по fusion. |
| `tools/reporting/make_tracking_takeoff_video.py` | Генерация видео/визуализации трекинга и взлётного участка. |

## 10. Открытые моменты

| Тема | Состояние |
|---|---|
| `StartupNav.ini` | Дублируется в `configs/datasets/HKairport03/` и `data/raw/HKairport03/`; нужен единый источник истины. |
| `src/navigation/legacy/*` | Сохранено вне сборки; удаление/архивация — после подтверждения. |
| `experiments/add-kalman-report/*` | Исторический/экспериментальный контур; при дальнейшей реструктуризации проверить актуальность путей и зависимостей. |
| Регрессия после переноса | Требуются прогоны контрольных датасетов, проверка Python/VO/report пайплайнов и сборка `src/video/vo/vo_cpp` в новой раскладке. |

---

*Документ сгенерирован по фактическому дереву файлов рабочей копии; при добавлении/перемещении файлов обновляйте соответствующие разделы.*
