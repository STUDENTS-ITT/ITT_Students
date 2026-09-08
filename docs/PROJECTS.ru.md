# Из чего состоит fire-dron — проекты и роли

Документ для тех, кто впервые открывает репозиторий: **что за программы участвуют**, **где они лежат**, **кто за что отвечает**, и **как это связано** с демо VINS-Mono на YouTube.

---

## Одной фразой

**fire-dron** — не замена VINS-Mono. Это **надстройка вокруг него**:

- конфиги и патчи под **надирный** полёт MARS;
- второй алгоритм **DSO** и **диспетчер**;
- **слияние с RTK** после прогона;
- скрипты, отчёт, видео, документация.

Сам VINS-Mono и DSO **скачиваются и собираются отдельно** (см. `scripts/install/`).

---

## Карта: что где живёт

```
┌─────────────────────────────────────────────────────────────────┐
│  fire-dron (этот репозиторий, GitHub)                           │
│  config/  scripts/  tools/  docs/  data/  results/             │
└────────────┬───────────────────────────────┬────────────────────┘
             │ копируем config               │ запускаем скрипты
             ▼                               ▼
┌────────────────────────────┐    ┌──────────────────────────────┐
│  ~/catkin_ws               │    │  ~/dso                       │
│  VINS-Mono (ROS)           │    │  DSO (standalone C++)        │
│  feature_tracker           │    │  dso_dataset                 │
│  vins_estimator            │    └──────────────────────────────┘
│  pose_graph                │
└────────────┬───────────────┘
             │ rosbag play
             ▼
┌────────────────────────────┐
│  data/mars/*.bag           │  ← запись полёта (камера + IMU)
│  data/mars/*_gt.tum        │  ← RTK эталон (метры, ENU)
└────────────────────────────┘
```

| Место | Что это | Наш код? |
|-------|---------|----------|
| **fire-dron/** | Скрипты, fusion, отчёт, данные MARS/Поворот | **Да** |
| **~/catkin_ws/** | Сборка VINS-Mono под ROS | Нет (upstream HKUST) |
| **~/dso/** | Сборка DSO | Нет (upstream TUM) |
| **/opt/ros/noetic/** | ROS — шина сообщений, rosbag, rviz | Системный пакет |

---

## ROS Noetic — зачем

**ROS** (Robot Operating System) — не «операционка для робота», а **набор программ**, которые обмениваются сообщениями по **топикам**.

Примеры топиков в нашем пайплайне:

| Топик | Кто пишет | Что внутри |
|-------|-----------|------------|
| `/cam0/image_raw` | rosbag play | Кадры камеры |
| `/imu0` | rosbag play | IMU |
| `/feature_tracker/feature_img` | feature_tracker | Кадр с точками (RViz) |
| `/vins_estimator/odometry` | vins_estimator | **Поза VINS** (окно odom справа в демо) |
| `/vins_estimator/path` | vins_estimator | Траектория для RViz |
| `/pose_graph/match_image` | pose_graph | Loop closure картинка |
| `/mars/rtk_path` | **наш** `rtk_path_publisher.py` | RTK только для **сравнения** в RViz |

**RViz** только **рисует** топики. На алгоритм не влияет.

**rosbag** — файл-запись всех топиков с прошлого полёта. Live-режим = тот же VINS, но картинки и IMU идут из bag, не с реального дрона.

---

## VINS-Mono (HKUST)

- **Репозиторий:** [github.com/HKUST-Aerial-Robotics/VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- **Статья:** монокуляр + IMU, sliding window, loop closure
- **У нас:** `~/catkin_ws/src/VINS-Mono`

### Три ROS-ноды (запускает `mars_nadir.launch`)

| Нода | Роль |
|------|------|
| **feature_tracker** | Ищет точки на кадре (Shi-Tomasi + KLT), рисует `tracked image` |
| **vins_estimator** | Считает позу: IMU + визуал, оптимизация в окне |
| **pose_graph** | Loop closure — «узнаёт» уже пройденное место, правит дрейф |

### Локальная система координат `world`

VINS создаёт **свою** СК: начало = точка инициализации, **(0, 0, 0)**.

При **`system reboot`** (failure detection) эта СК **сбрасывается** — в RViz траектория «телепортируется» к началу. Это **не** движение дрона в реальности и **не** RTK.

На надирном MARS reboot бывает часто (повороты камеры, мало parallax).

### Что мы добавили к VINS

| Файл | Изменение |
|------|-----------|
| `config/mars_nadir.yaml` | Камера MARS, extrinsic надир, `show_track: 1` |
| `tools/patch_nadir_parallax.py` | rot-gate / gyro-gate — меньше ложных keyframe на поворотах |
| Патч в `feature_manager.cpp` | Применяется в catkin, нужен `catkin_make` |

Подробнее: [vins_mono_notes.md](vins_mono_notes.md), [patches_nadir_parallax.md](patches_nadir_parallax.md).

### Чистый VINS vs RTK

| Режим | RTK участвует? |
|-------|----------------|
| Live: `rostopic echo /vins_estimator/odometry` | **Нет** |
| Live: красный `rtk_path` в RViz | **Только линия для глаз** |
| Batch: `fuse_vins_rtk.py` | **Да**, offline после прогона |
| Batch: `vins_fused.tum` | VINS + привязка к RTK |

---

## DSO (TUM)

- **Репозиторий:** [github.com/JakobEngel/dso](https://github.com/JakobEngel/dso)
- **Статья:** прямая одометрия по **яркости** пикселей, **без IMU**
- **У нас:** `~/dso/build/bin/dso_dataset`

| | VINS | DSO |
|---|------|-----|
| Сенсоры | Камера + IMU | Только камера |
| Масштаб | Метрический (IMU) | До Sim(3) — масштаб плывёт |
| Надир MARS | **Основной** (2734 поз) | Слабый (781 поз, ~21%) |
| Поворот_коптер | Нет IMU → не запускается | **Единственный** канал |

В **диспетчере** на MARS DSO не заменяет VINS — только **health-check** (жив ли трек).

Подробнее: [dso_notes.md](dso_notes.md).

---

## MARS-LVIG (датасет)

- **Сайт:** [mars.hku.hk](https://mars.hku.hk/dataset.html)
- **У нас:** `data/mars/`

| Файл | Содержимое |
|------|------------|
| `mars_hkairport03.bag` | ROS-запись: камера + IMU (~6 мин полёта, Гонконг) |
| `mars_hkairport03_gt.tum` | RTK/GNSS эталон: timestamp, x, y, z (метры ENU) |
| `camera/images/` | Кадры для офлайн-видео |
| `dso/` | Уменьшенные кадры + `camera.txt` для DSO |

**RTK (Real-Time Kinematic)** — точные координаты с GNSS-базы, сантиметровый класс. В отчёте — **эталон**, с которым сравниваем VINS.

---

## Поворот_коптер (наш датасет)

- **У нас:** `data/povorot_kopter/`
- Камера смотрит вниз, **IMU нет**
- Только DSO; эталона нет — смотрим **дрейф** (~40% пути)

---

## Инструменты внутри fire-dron

### `scripts/run_vins_mars_live.sh`

Live: VINS + RViz + (опционально) RTK path + окно `rostopic echo`.  
**Алгоритм тот же**, что в batch; добавлена только визуализация.

### `scripts/run_vins_mars.sh`

Batch: VINS без GUI → `results/vins_mars_*.csv`.

### `tools/fuse_vins_rtk.py`

**После** прогона VINS:

1. Выравнивание VINS → RTK (масштаб, yaw, сдвиг)
2. Детекция reboot в CSV
3. На reboot — привязка сегмента к RTK
4. Z частично от RTK / baro

Результат: `vins_fused.tum`. Fused ATE ~2–3 м **не** значит, что VINS сам так точен — там RTK-assist после reboot.

### `tools/dispatcher.py`

Смотрит: сколько поз выдал VINS и DSO, lost или нет, тип сцены (`nadyr` / `city`).

На MARS надир: **выбор VINS**, DSO — health-check.

### `tools/generate_combined_report.py`

PDF `ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf` — метрики, графики, сравнение run8e vs run4.

### `tools/rtk_path_publisher.py`

Только для **RViz**: читает `*_gt.tum`, публикует `/mars/rtk_path`. **Не меняет** VINS.

### `tools/visualize_keypoints.py`

Офлайн-видео `mars_hk_flight.avi` — VINS vs DSO side-by-side, не live.

---

## Два режима работы

```
                    ┌─────────────────────────────────────┐
                    │           LIVE (демо, отладка)       │
                    │  run_vins_mars_live.sh + RViz        │
                    │  • odom = чистый VINS                │
                    │  • rtk_path = только рисунок         │
                    │  • fusion НЕ работает                │
                    └─────────────────────────────────────┘

                    ┌─────────────────────────────────────┐
                    │           BATCH (метрики, отчёт)     │
                    │  run_vins_mars.sh → production_mars  │
                    │  • CSV / TUM на диск                 │
                    │  • fuse_vins_rtk.py                  │
                    │  • evo ATE, PDF, видео               │
                    └─────────────────────────────────────┘
```

**Real-time:** bag проигрывается с заданной скоростью (`-r 0.3` = медленнее реального времени). VINS обрабатывает кадры **так же**, как на борту, только вход из файла.

---

## evo

Утилита Python для метрик траекторий: **ATE**, **RPE**.  
Сравнивает `vins.tum` или `vins_fused.tum` с `mars_hkairport03_gt.tum`.

Устанавливается: `scripts/install/07_install_evo.sh` или `pip install evo`.

---

## Ceres, Pangolin

| Библиотека | Зачем |
|------------|-------|
| **Ceres** | Нелинейная оптимизация в VINS и DSO |
| **Pangolin** | Окно DSO (на MARS часто headless) |

Сборка: `scripts/install/03_install_ceres.sh`, `04_install_pangolin.sh`.

---

## Что смотреть по порядку (новому человеку)

1. **[COMMANDS.ru.md](COMMANDS.ru.md)** — команды copy-paste
2. **Этот файл** — кто есть кто
3. **[rviz_live.md](rviz_live.md)** — live RViz и топики
4. **[architecture.md](architecture.md)** — схема конвейера
5. **[vins_mono_notes.md](vins_mono_notes.md)** / **[dso_notes.md](dso_notes.md)** — алгоритмы глубже
6. **PDF-отчёт** в корне — итоговые цифры run8e

---

## Частые вопросы

**Это тот же VINS, что на YouTube?**  
Да, тот же upstream VINS-Mono + наши конфиги и патчи под надир MARS.

**RTK в live правит odom?**  
Нет. RTK — отдельная линия в RViz или offline fusion.

**Почему траектория прыгает?**  
`system reboot` — новая локальная СК VINS, origin снова (0,0,0).

**DSO лучше VINS на MARS?**  
Нет. На nadир MARS VINS + IMU даёт ~75% покрытия; DSO ~21%.

**Где «настоящий» результат для отчёта?**  
`results/vins_mars_run8e_parallax.csv` (raw VINS),  
`results/eval_mars_run8e_fused/vins_fused.tum` (после fusion),  
`ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf`.

---

## Ссылки upstream

| Проект | URL |
|--------|-----|
| VINS-Mono | https://github.com/HKUST-Aerial-Robotics/VINS-Mono |
| DSO | https://github.com/JakobEngel/dso |
| MARS-LVIG | https://mars.hku.hk/dataset.html |
| ROS Noetic | http://wiki.ros.org/noetic |
