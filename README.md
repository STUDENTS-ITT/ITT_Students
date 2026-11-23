# VO + INS + Kalman (HKairport03)

Монокулярная видеоодометрия с нижней камеры и комплексирование с БИНС и фильтром Калмана. Сравнение с RTK.

На другом компьютере нужны уже установленные **C++ компилятор**, **CMake** и **Python 3.10+**. Компилятор любой из установленных: MSYS2, MinGW-w64, TDM-GCC, Dev-C++, Clang или Visual Studio. Скрипт берёт тот, что уже в `PATH`, а если там пусто — ищет эти установки сам. OpenCV нужен только если заново считать саму одометрию (`src/video/vo/vo_cpp`), фильтр Калмана его не использует.

## Архитектура проекта

Подробный пофайловый гид: [`docs/guides/file_functional_guide.md`](docs/guides/file_functional_guide.md).

```text
.
├── CMakeLists.txt                  # Сборка C++ цели imitator (core INS/ESKF)
├── README.md
├── AGENTS.md
├── requirements.txt
├── configs/                        # Конфигурации и калибровки
│   ├── calibration/camera.json
│   └── datasets/HKairport03/StartupNav.ini
├── data/                           # Данные датасетов
│   ├── raw/HKairport03/            # Сырые потоки, референсы, таймстемпы
│   └── processed/HKairport03/      # Производные/описания обработанных данных
├── docs/                           # Документация
│   ├── guides/                     # Руководства (в т.ч. file_functional_guide.md)
│   ├── plans/                      # Планы разработки/реструктуризации
│   ├── protocols/                  # Протоколы данных и систем координат
│   ├── reference/                  # Справочные материалы
│   └── reports/                    # Отчёты, PDF, графики
├── experiments/                    # Экспериментальные контуры
│   └── add-kalman-report/          # Снимок контура Kalman/report для сравнения
├── results/                        # Результаты прогонов
│   ├── fusion/                     # Выходы fusion/метрики/графики
│   └── HKairport03/                # Результаты VO по HKairport03
├── src/                            # Основной исходный код
│   ├── app/imitator/main.cpp       # Точка входа C++ прогона БИНС
│   ├── common/                     # Общие типы и константы
│   ├── data_io/                    # Чтение/запись данных, I/O-ошибки
│   ├── ins/                        # IMU, ориентация, ESKF/фильтр
│   ├── math_lib/                   # Математика, гравитация, преобразования
│   ├── navigation/                 # Выставка, усреднение, GNSS guard, траектория
│   │   └── legacy/                 # Устаревший код вне активной сборки
│   ├── utils/                      # Вспомогательные утилиты (paths и т.п.)
│   └── video/                      # VO-домен
│       ├── vo_final2.py            # Основной Python VO
│       └── vo/vo_cpp/              # C++ VO-подпроект (CMake, apps, include, src)
├── tests/
│   └── fixtures/                   # Тестовые фикстуры/описания
└── tools/                          # Скрипты запуска, анализа и отчётности
    ├── analysis/                   # Графики траекторий/fusion
    ├── pipelines/                  # Пайплайны обработки
    ├── scenarios/                  # Сценарии прогонов (HKairport03 и др.)
    └── reporting/                  # Генерация PDF/видео-отчётов
```

| Область | Назначение |
|---|---|
| `src/app/imitator` | CLI-приложение БИНС/ESKF: выставка, инициализация `x0/P0`, основной цикл. |
| `src/common`, `src/utils`, `src/math_lib` | Общие типы/константы, helper-утилиты, математика и преобразования. |
| `src/data_io` | Форматы ввода/вывода `.dat`/CSV и обработка ошибок I/O. |
| `src/ins`, `src/navigation` | Инерциальная навигация, фильтр Калмана, GNSS-защита, расчёт траектории. |
| `src/video` | Видеоодометрия: Python-реализация и C++ подпроект `vo_cpp`. |
| `configs`, `data` | Калибровки, конфиги датасетов, сырые и обработанные данные. |
| `tools`, `experiments`, `results`, `docs` | Запуск/анализ/отчёты, экспериментальные контуры, результаты прогонов и документация. |

## Зависимости Python

Из корня проекта:

```powershell
python -m pip install -r requirements.txt
```

Пакеты: `numpy`, `matplotlib`, `pillow`, `fpdf2`.

## Данные

Ожидаемая раскладка данных (пример HKairport03):

- `data/raw/HKairport03/camera/timestamps.csv` и `data/raw/HKairport03/camera/images/` — кадры (изображения в git не входят; для пересчёта VO их нужно скопировать отдельно)
- `data/raw/HKairport03/imu/dji_osdk_ros_imu.csv`
- `data/raw/HKairport03/gps/dji_osdk_ros_rtk_position.csv`
- `data/raw/HKairport03/other/dji_osdk_ros_attitude.csv`
- `data/raw/HKairport03/reference/gps_reference.csv` — RTK-референс
- `data/raw/HKairport03/reference/altitude_per_frame.csv` — высота по кадрам
- `results/HKairport03/homography/visual_odometry/FAST-FREAK/trajectory.csv` — уже посчитанная одометрия
- `configs/calibration/camera.json` — калибровка для повторного прогона `src/video/vo/vo_cpp`

`gps.dat`, `imu.dat`, `angle.dat` скрипты собирают сами; в репозиторий они обычно не входят.

## Сборка и прогон фильтра

`cmake` должен быть в `PATH`. Компилятор тоже, либо стоять в обычном месте (MSYS2, Dev-C++, MinGW, Visual Studio).

### Экспериментальный контур `add-kalman-report`

```powershell
powershell -ExecutionPolicy Bypass -File experiments\add-kalman-report\tools\run_eval.ps1 -Rebuild
```

Скрипт сам находит Python, в котором есть numpy, и генератор CMake под найденный компилятор: Ninja, MinGW Makefiles или Visual Studio. Собирает `imitator.exe`, готовит входы, считает траекторию и печатает RMS / максимум / ошибку на финише.

Повторный прогон без пересборки:

```powershell
powershell -ExecutionPolicy Bypass -File experiments\add-kalman-report\tools\run_eval.ps1
```

Отчёт PDF:

```powershell
python tools/reporting/make_fusion_report.py
```

Файл: `docs/reports/VO_INS_Fusion_HKairport03.pdf`. На графиках траектория фильтра подписана **VO+INS+Kalman**, пунктир — RTK.

Вручную, если нужно собрать только фильтр в экспериментальном контуре:

```powershell
cmake -S experiments/add-kalman-report -B experiments/add-kalman-report/build -DCMAKE_BUILD_TYPE=Release
cmake --build experiments/add-kalman-report/build --config Release
experiments\add-kalman-report\build\imitator.exe experiments\add-kalman-report\data\vo_hk03 experiments\add-kalman-report\data\vo_hk03\params.ini
```

У Visual Studio исполняемый файл лежит в `experiments\add-kalman-report\build\Release\imitator.exe`.

Перед этим один раз: `python experiments\add-kalman-report\tools\prepare_vo_inputs.py`.

### Корневая цель `imitator`

Корневой `CMakeLists.txt` собирает основную цель `imitator` из `src/app/imitator/main.cpp` и модулей `src/data_io`, `src/navigation/averaging.cpp`.

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target imitator --config Release
```

## Повторный расчёт видеоодометрии

Нужен OpenCV 4.7+ с `opencv_contrib` (модуль FREAK), чтобы CMake нашёл `OpenCV_DIR`.

```powershell
cmake -S src/video/vo/vo_cpp -B src/video/vo/vo_cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build src/video/vo/vo_cpp/build --config Release
src/video/vo/vo_cpp/build/vo_final.exe -i data/raw/HKairport03/camera/images --calibration configs/calibration/camera.json --altitude-file data/raw/HKairport03/reference/altitude_per_frame.csv --out-dir results --no-show-video
```

`vo_final.exe` при генераторе Visual Studio будет в `src/video/vo/vo_cpp/build/Release/`.

Python-вариант той же одометрии:

```powershell
python src/video/vo_final2.py --help
```
