# Копия imitator под формат dataset_1786981733

Алгоритм тот же, что в исходной программе (выставка, счисление БИНС, коррекция по акселерометру и фильтр Калмана 15-го порядка). Изменено только чтение файлов:

- время `ЧЧ:ММ:СС.дробь` (например `00:00:00.006000128`)
- `imu.data`: timestamp, wx, wy, wz, ax, ay, az
- `gps.data`: timestamp, lat, lon, alt, vx, vy, vz
- `angle.data`: timestamp, x, y, z, qw..qz, vx, vy, vz, roll, pitch, yaw

Внутри программы строка ИМУ сразу приводится к прежнему виду:
`time_s  timestamp_ns  wx  wy  wz  ax  ay  az`.

## Данные

Файлы положены в `data/raw/`:

| Файл | Источник |
|------|----------|
| `imu.data` | `dataset_1786981733/imu0/imu.data` |
| `gps.data` | `dataset_1786981733/gps0/gps.data` |
| `angle.data` | `dataset_1786981733/state_ground_truth0/angle.data` |
| `StartupNav.ini` | координаты первого отсчёта GPS, выставка 120 с |

Исходный архив: `c:\Users\ORNK\Downloads\dataset_1786981733\dataset_1786981733`

## Сборка и запуск

Из этой папки (`newformat`):

```powershell
$env:Path = "C:\msys64\ucrt64\bin;" + $env:Path
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cd build
.\imitator.exe
```

Результат пишется в один файл `tools/kalman15_line2.txt` (БИНС, эталон СНС, вектор ошибок Калмана).

Графики:

```powershell
& "C:\Users\ORNK\AppData\Local\Programs\Python\Python311\python.exe" tools/plot_trajectory.py --save-only
& "C:\Users\ORNK\AppData\Local\Programs\Python\Python311\python.exe" tools/plot_kalman_state.py --save-only
```

Файлы: `tools/kalman15_line2_comparison.png`, `tools/kalman15_line2_errors.png`, `tools/kalman15_line2_map.png`, `tools/kalman15_line2_plots.png` (вектор состояния Калмана).
