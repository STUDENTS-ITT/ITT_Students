# VO-VINS · VINS-Mono + DSO + RTK fusion

> **Ветка:** `VO-VINS` · репозиторий [STUDENTS-ITT/ITT_Students](https://github.com/STUDENTS-ITT/ITT_Students)  
> Объединённый алгоритм визуальной одометрии для **надирных** полётов БПЛА.

[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange)]()
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Что внутри

| Артефакт | Путь |
|----------|------|
| **PDF-отчёт** | [`ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf`](ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf) |
| **Видео MARS** (~150 с) | [`results/combined_run8e_fused/mars_viz/mars_hk_flight.mp4`](results/combined_run8e_fused/mars_viz/mars_hk_flight.mp4) |
| **Видео Поворот_коптер** | [`results/combined_run8e_fused/povorot_viz/povorot_flight.avi`](results/combined_run8e_fused/povorot_viz/povorot_flight.avi) |
| **Код fusion / диспетчер** | [`tools/`](tools/) |
| **Конфиги VINS/RViz** | [`config/`](config/) |
| **Документация** | [`docs/COMMANDS.ru.md`](docs/COMMANDS.ru.md) · [`docs/PROJECTS.ru.md`](docs/PROJECTS.ru.md) |

### Результаты run8e_fused (production)

| Метрика | Значение |
|---------|----------|
| Покрытие VINS | ~75% bag |
| Raw ATE (автономный VINS) | ~136 м |
| Fused ATE (offline RTK) | ~2.9 м |
| Reboot VINS | 16 |
| Быстродействие | ~54 мс/кадр, wall ~18 мин |

---

## Быстрый старт

### 1. Клонировать ветку

```bash
git clone -b VO-VINS --single-branch https://github.com/STUDENTS-ITT/ITT_Students.git vo-vins
cd vo-vins
```

### 2. Зависимости (Ubuntu 20.04, один раз)

```bash
chmod +x scripts/install/*.sh scripts/*.sh
./scripts/install/01_install_base.sh
./scripts/install/02_install_ros.sh
./scripts/install/03_install_ceres.sh
./scripts/install/04_install_pangolin.sh
./scripts/install/05_build_vins_mono.sh
./scripts/install/06_build_dso.sh
./scripts/install/07_install_evo.sh
pip3 install -r tools/requirements.txt
```

### 3. Rosbag MARS (обязательно для HKairport03)

Bag **не в git** (>1 ГБ). Скачайте с [MARS-LVIG](https://mars.hku.hk/dataset.html) и положите:

```bash
# data/mars/mars_hkairport03.bag
./scripts/download_mars_bag.sh   # проверка наличия
```

### 4. Конфиг VINS в catkin

```bash
export VNAV_ROOT="$(pwd)"
mkdir -p ~/catkin_ws/src/VINS-Mono/config/mars
cp config/mars_nadir.yaml ~/catkin_ws/src/VINS-Mono/config/mars/
cp config/mars_nadir.launch ~/catkin_ws/src/VINS-Mono/vins_estimator/launch/
sed -i "s|/home/USER|$HOME|g" ~/catkin_ws/src/VINS-Mono/config/mars/mars_nadir.yaml
python3 tools/patch_nadir_parallax.py
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j$(nproc)
```

### 5. Live RViz (запись экрана)

```bash
cd vo-vins
./scripts/run_vins_mars_live.sh 0.3 50 --play 90
```

Запись экрана GNOME: **`Ctrl + Shift + Alt + R`** → файл в `~/Videos/`.

### 6. Полный batch-прогон

```bash
./scripts/run_vins_mars.sh 0.3 50 run8e_parallax
./scripts/run_dso_mars.sh 0 run8e_fused
./scripts/production_mars.sh results/vins_mars_run8e_parallax.csv run8e_fused
./scripts/finalize_run8.sh run8e_parallax run8e_fused
```

---

## Архитектура

```
Кадры + IMU ──► VINS-Mono ──┐
                             ├──► Диспетчер ──► fuse_vins_rtk ──► траектория ENU
Кадры ────────► DSO ────────┘         │
                                        └── health-check (надир)
RTK эталон ───────────────────────────────► offline fusion
```

Подробнее: [docs/architecture.md](docs/architecture.md)

---

## Структура репозитория

```
├── ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf
├── config/          # mars_nadir.yaml, mars_rviz.rviz
├── data/
│   ├── mars/        # GT, aux (bag — отдельно)
│   └── povorot_kopter/
├── docs/            # COMMANDS, PROJECTS, RViz
├── results/         # run8e_fused, видео
├── scripts/         # install, run, live RViz
└── tools/           # fusion, dispatcher, отчёт
```

---

## Авторы и ссылки

- [VINS-Mono](https://github.com/hkust-aerial-robotics/VINS-Mono)
- [DSO](https://github.com/JakobEngel/dso)
- [MARS-LVIG Dataset](https://mars.hku.hk/dataset.html)

MIT — см. [LICENSE](LICENSE).
