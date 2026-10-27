# vo_cpp — монокулярная визуальная одометрия на C++

C++17-порт скриптов `vo_final2.py` и `run_all_HKairport03.py` в **упрощённом варианте**:
поддерживается только **связка FAST-FREAK** и модель движения **homography**.

Сценарий: надирная аэросъёмка с БПЛА (DJI M300, камера Hikvision CA-050-11UC,
2448×2048, 10 Гц, f ≈ 1449 px). Метрический масштаб берётся из высоты полёта.

## Что внутри

| Модуль (`src/`, `include/vo/`) | Назначение |
|---|---|
| `config` | структура `Config` с параметрами; пресеты скорости (только для совместимости) |
| `util` | форматирование, CSV, лог в консоль и файл, мелкие математические помощники |
| `calibration` | загрузка калибровки (`.json/.yml/.xml`), приближённая калибровка |
| `sequence` | источники кадров: папка/маска `*.png`, видеофайл |
| `altitude` | высота: константа или CSV по кадрам (интерполяция как `numpy.interp`) |
| `preprocess` | undistort, CLAHE, затирание OSD-надписей |
| `features` | только **FAST-FREAK**, `select_uniform`, маска тени/шасси на малой высоте |
| `matching` | Хемминг/SSD, ratio test, взаимная уникальность |
| `tracking` | пирамидальный Лукас-Канаде (выключен; `--lk rotation|only|always`) |
| `motion` | только модель **homography** (OpenCV `findHomography` + разложение) |
| `selection` | адаптивный выбор кадров (`FrameSelector`) и обрезка на поворотах (`RotationCrop`) |
| `body_yaw` | рыскание корпуса по кадрам (attitude CSV) для arc-интеграции |
| `pipeline` | основной цикл, CSV траектории, GPS-трек, графики (arc + body-yaw) |
| `plot`, `viewer` | графики и окна просмотра на чистом OpenCV |

Приложения (`apps/`):

* `vo_final` — одна траектория по папке кадров (FAST-FREAK + homography);
* `run_all_hkairport03` — подготовка и прогон датасета HKairport03 (FAST-FREAK + homography).

## Зависимости

* компилятор C++17 (GCC ≥ 9, Clang ≥ 10, MSVC 2019+);
* CMake ≥ 3.16;
* **OpenCV ≥ 4.7** (проверено на **5.0.0**) **с `opencv_contrib`** (требуется для FREAK).
  Без `xfeatures2d` программа не соберётся.

### Windows (MSYS2, проверенный путь)

```powershell
# установить MSYS2 (https://www.msys2.org), затем в оболочке MSYS2 UCRT64:
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake `
    mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-opencv `
    mingw-w64-ucrt-x86_64-qt6-base
```

> Пакет `opencv` из MSYS2 собран с Qt6 для `highgui`, но не тянет его как зависимость.
> Без `qt6-base` программы не стартуют (код выхода `0xC0000135`, «DLL не найдена»).

### Linux / macOS

```bash
sudo apt install build-essential cmake ninja-build libopencv-dev   # Debian/Ubuntu
brew install cmake ninja opencv                                    # macOS
```

## Сборка

```powershell
$env:Path = "C:\msys64\ucrt64\bin;" + $env:Path      # только для MSYS2 на Windows
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Опции CMake: `-DVO_BUILD_APPS=OFF` (чтобы не собирать приложения), `-DVO_WARNINGS=OFF` (отключить предупреждения).
Если OpenCV лежит в нестандартном месте — `-DOpenCV_DIR=<путь к каталогу с OpenCVConfig.cmake>`.

## Запуск

Список доступных связок:

```bash
./build/vo_final --list
```

Одна траектория по папке кадров:

```bash
./build/vo_final -i camera/images \
    --calibration calibration/camera.json \
    --altitude-file altitude_per_frame.csv \
    --out-dir results --no-show-video
```

Основные ключи (`--help` — полный список):

| Ключ | Смысл |
|---|---|
| `-i, --input` | папка кадров, маска `dir/*.png` или видеофайл |
| `--altitude M` / `--altitude-file CSV` | высота числом или по кадрам |
| `--motion-model` | игнорируется (используется только homography) |
| `-a, --algorithm` | игнорируется (используется только FAST-FREAK) |
| `--fixed-step N` / `--target-shift PCT` | фиксированный шаг или цель адаптивного выбора кадров |
| `--preset` | игнорируется; параметры RANSAC фиксированы |
| `--lk off\|rotation\|only\|always` | Лукас-Канаде (по умолчанию выключен, только FAST-FREAK) |
| `--no-shadow` | отключить маску собственной тени и шасси |
| `--rotation-crop`, `--rotation-threshold`, `--rotation-cut` | обрезка кадра на поворотах |
| `--show-video`, `--live-plot` | окна просмотра (`q`/`Esc` — выход, пробел — пауза) |
| `--profile` | время по этапам (недоступно) |
| `--max-frames N` | ограничить число кадров |

Весь датасет HKairport03 (подготовка + прогон, выравнивание, графики):

```bash
./build/run_all_hkairport03 --dataset /path/to/HKairport03 --out .
```

Ожидаемая структура датасета: `camera/images/*.png`, `camera/timestamps.csv`, `gps/`, `other/`.
Программа сама создаёт `calibration/camera.json`, `altitude_per_frame.csv`, `gps_reference.csv`
(они в `.gitignore`).

## Бенчмарк HKairport03

**Сборка и прогон** (из корня `vo_cpp`, Release):

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target run_all_hkairport03 -j
./build/run_all_hkairport03 --dataset /path/to/HKairport03 --out build/eval_arc --skip-benchmark
python tools/score_hkairport03.py build/eval_arc/results_HKairport03/homography/visual_odometry/FAST-FREAK/trajectory.csv
```

`run_all_hkairport03` задаёт `project_root` каталога набора (`--dataset`) и пути к `camera/timestamps.csv` и
`other/dji_osdk_ros_attitude.csv`, чтобы body-yaw подхватывался независимо от `--out`.

**Production-пайплайн** (`pipeline.cpp`): **FAST-FREAK + homography**,
фильтр вертикального шага `drop_vertical_step`, порог `min_translate_alt_m=5` (ниже 5 м — курс без метрического шага).
**Body-yaw:** в соответствиях «от снимка» точки старшего кадра разворачиваются на −Δψ корпуса; длинный адаптивный шаг
ограничивается **≤9°** рыскания корпуса за шаг (`cap_step_by_body_yaw`). **RTK/INS в позиции не участвуют** — только
высота из `altitude_per_frame.csv` и сверка с эталоном в `run_all`.
При развороте корпуса **>15°** на интервале **>8** кадров хорда «текущий кадр → снимок» не используется (идёт цепочка homography).
**Дуга на якоре:** если `gap ≥ 8` и суммарный |Δψ корпуса| **≥ 12°**, метрический шаг от якоря интегрируется по дуге
(`integrate_arc_xy`) по приращениям ψ между кадрами; иначе — прямой шаг из гомографии. Курс корпуса в траекторию не пишется.

**Честная оценка vs RTK** (в `run_all_hkairport03`, не глобальный Kabsch по всему треку):
масштаб не подгоняется, трек один раз поворачивается и сдвигается по началу полёта
(первые **10%** пути GPS, до **t=105 с**, до первого разворота). Дальше RMS считается по всему полёту.

**Метрики оставленного прогона** (`build/eval_arc`, `score.py`, SE(2) по 10% RTK, t≤105 с):
RMS **18.7 м**, максимум **49.1 м**, конец **4.0 м**, длина пути **1937 м** против GPS **1915 м** (≈1.01×).
Полный полёт на i7-8750H: **118 с** (5.65 шаг/с) при длительности записи **366 с**.
Отчёт с графиками RPE и роликом взлёта: `../report/VO_HKairport03.pdf`.

## Результаты

```
<out-dir>/visual_odometry/<связка>/
    trajectory.csv          # кадр, X/Y/Z, высота, скорость, курс, шаг, число инлаеров, статус, модель…
    vo_track_gps.csv        # трек в формате GPS (по умолчанию 25 fps, масштаб 1 м/ед.)
    trajectory_<плоскость>.png, trajectory_views.png
    console_log.txt
    matches/                # картинки соответствий (отключаются --no-figures)
```

`run_all_hkairport03` дополнительно пишет `results_HKairport03/` (статистика) и `trajectory_plots/HK_01..03`
(графики VO + GPS).

## Формат входных данных

* **Калибровка** — JSON/YAML с ключами `K` (или `fx, fy, cx, cy` / `focal` / `principal`),
  `distortion`, `imageSize`. Нет файла — используется приближённая калибровка (f = 0.9·max(h, w))
  с предупреждением.
* **Высота по кадрам** — CSV с колонкой номера кадра и колонкой высоты (названия определяются автоматически).

## Особенности

* **Только FAST-FREAK**: остальные связки (Harris, BRISK, ORB, KAZE, SIFT, SURF) удалены.
* **Только homography**: модели `similarity` и `essential` удалены.
* По умолчанию только FAST-FREAK. LK на поворотах даёт большую ошибку и выключен.
* На высоте от 50 м шаг кадров не больше 20. Поворот больше 4° при доле инлаеров ниже 50% в траекторию не пишется: шаг уменьшается.
* На высоте ниже 40 м точки на маске теней (тёмные пятна площади тени аппарата) отбрасываются, как и пары с почти нулевым потоком.
* Сверка с GPS: см. [бенчмарк HKairport03](#бенчмарк-hkairport03) (один поворот и сдвиг по началу полёта, не по всему треку).
* Параметры RANSAC фиксированы; пресеты скорости не работают.
* Графики на чистом OpenCV с ASCII-подписями (нет `.mat`, 3D, URL/yt-dlp).
* Требует реального датасета (`camera/images/`, `camera/timestamps.csv`, `gps/`, `other/`).
