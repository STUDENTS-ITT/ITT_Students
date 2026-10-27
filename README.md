# VO + INS + Kalman (HKairport03)

Монокулярная видеоодометрия с нижней камеры и комплексирование с БИНС и фильтром Калмана. Сравнение с RTK.

На другом компьютере нужны уже установленные **C++ компилятор**, **CMake** и **Python 3.10+**. Компилятор любой из установленных: MSYS2, MinGW-w64, TDM-GCC, Dev-C++, Clang или Visual Studio. Скрипт берёт тот, что уже в `PATH`, а если там пусто — ищет эти установки сам. OpenCV нужен только если заново считать саму одометрию (`vo_cpp`), фильтр Калмана его не использует.

## Зависимости Python

Из корня проекта:

```powershell
python -m pip install -r requirements.txt
```

Пакеты: `numpy`, `matplotlib`, `pillow`, `fpdf2`.

## Данные

Рядом с кодом должны лежать:

- `camera/timestamps.csv` и `camera/images/` — кадры (картинки в git не входят, на флешке их нужно скопировать отдельно, если будете пересчитывать одометрию)
- `imu/dji_osdk_ros_imu.csv`
- `gps/dji_osdk_ros_rtk_position.csv`
- `other/dji_osdk_ros_attitude.csv`
- `results_HKairport03/homography/visual_odometry/FAST-FREAK/trajectory.csv` — уже посчитанная одометрия
- `calibration/camera.json` — для повторного прогона `vo_cpp`

`gps.dat`, `imu.dat`, `angle.dat` скрипт собирает сам, в репозиторий они не входят.

## Сборка и прогон фильтра

`cmake` должен быть в `PATH`. Компилятор тоже, либо стоять в обычном месте (MSYS2, Dev-C++, MinGW, Visual Studio).

```powershell
powershell -ExecutionPolicy Bypass -File add-kalman-report\tools\run_eval.ps1 -Rebuild
```

Скрипт сам находит Python, в котором есть numpy, и генератор CMake под найденный компилятор: Ninja, MinGW Makefiles или Visual Studio. Собирает `imitator.exe`, готовит входы, считает траекторию и печатает RMS / максимум / ошибку на финише.

Повторный прогон без пересборки:

```powershell
powershell -ExecutionPolicy Bypass -File add-kalman-report\tools\run_eval.ps1
```

Отчёт PDF:

```powershell
python report\make_fusion_report.py
```

Файл: `report/VO_INS_Fusion_HKairport03.pdf`. На графиках траектория фильтра подписана **VO+INS+Kalman**, пунктир — RTK.

Вручную, если нужно собрать только фильтр:

Генератор не задан специально: CMake выберет его по компилятору в `PATH`. Для одноконфигурационных генераторов (Ninja, Makefiles) нужен `CMAKE_BUILD_TYPE`. Для Visual Studio конфигурация задаётся на шаге сборки.

```powershell
cmake -S add-kalman-report -B add-kalman-report\build -DCMAKE_BUILD_TYPE=Release
cmake --build add-kalman-report\build --config Release
add-kalman-report\build\imitator.exe add-kalman-report\data\vo_hk03 add-kalman-report\data\vo_hk03\params.ini
```

У Visual Studio исполняемый файл лежит в `add-kalman-report\build\Release\imitator.exe`.

Перед этим один раз: `python add-kalman-report\tools\prepare_vo_inputs.py`.

## Повторный расчёт видеоодометрии

Нужен OpenCV 4.7+ с `opencv_contrib` (модуль FREAK), чтобы CMake нашёл `OpenCV_DIR`.

```powershell
cmake -S vo_cpp -B vo_cpp\build -DCMAKE_BUILD_TYPE=Release
cmake --build vo_cpp\build --config Release
vo_cpp\build\vo_final.exe -i camera/images --calibration calibration/camera.json --altitude-file altitude_per_frame.csv --out-dir results --no-show-video
```

`vo_final.exe` при генераторе Visual Studio будет в `vo_cpp\build\Release\`.

Python-вариант той же одометрии: `python vo_final2.py --help`.
