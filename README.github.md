# VO-VINS · VINS-Mono + DSO (запас) · RTK только сравнение

> **Ветка:** `VO-VINS` · [STUDENTS-ITT/ITT_Students](https://github.com/STUDENTS-ITT/ITT_Students)  
> Основной канал — **VINS-Mono** (Livox IMU). Курс после оценки — **AHRS DJI**, не RTK.  
> DSO — второй вариант одометрии для **города / фасадов**.

[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange)]()
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Что внутри

| Артефакт | Путь |
|----------|------|
| **PDF-отчёт (VINS, 3 сцены)** | [`results/ПРОМЕЖУТОЧНЫЙ_ОТЧЁТ_VINS.pdf`](results/ПРОМЕЖУТОЧНЫЙ_ОТЧЁТ_VINS.pdf) |
| **Графики отчёта** | [`results/eval_vins_report/`](results/eval_vins_report/) |
| **Код** | [`tools/`](tools/) · [`scripts/`](scripts/) · [`config/`](config/) |
| **Документация** | [`docs/architecture.md`](docs/architecture.md) · [`docs/COMMANDS.ru.md`](docs/COMMANDS.ru.md) |

### Правила оценки

- **VINS** — главный навигационный канал (надир / поле / вода / лес).
- **Livox `/livox/imu`** — официальный SLAM-IMU датасета. DJI AHRS — только курс *после* VINS.
- **RTK** — эталон ATE и рисунки. В estimator и fusion XY не идёт.
- **Баро** — Z и приор глубины фич (`init_depth`, `use_baro_ground`).
- **DSO** — город / фасады; на надире MARS не заменяет VINS.
- Нет 15-state EKF и нет замены VINS на гомографию как основной канал.

---

## Быстрый старт

```bash
git clone -b VO-VINS --single-branch https://github.com/STUDENTS-ITT/ITT_Students.git vo-vins
cd vo-vins
chmod +x scripts/install/*.sh scripts/*.sh
./scripts/install/01_install_base.sh
./scripts/install/05_build_vins_mono.sh
# DSO — опционально, для city:
# ./scripts/install/06_build_dso.sh
```

Конфиг Livox + патчи надира:

```bash
export VNAV_ROOT="$(pwd)"
mkdir -p ~/catkin_ws/src/VINS-Mono/config/mars
cp config/mars_livox*.yaml config/mars_nadir.yaml ~/catkin_ws/src/VINS-Mono/config/mars/
cp config/mars_livox*.launch config/mars_nadir.launch \
   ~/catkin_ws/src/VINS-Mono/vins_estimator/launch/
sed -i "s|/home/USER|$HOME|g" ~/catkin_ws/src/VINS-Mono/config/mars/*.yaml
python3 tools/patch_nadir_parallax.py
python3 tools/patch_baro_ground.py
python3 tools/patch_nadir_init_depth.py
python3 tools/patch_nadir_failure.py
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j$(nproc)
```

Прогон сцены (курс AHRS, RTK только ATE):

```bash
./scripts/run_vins_ahrs_scene.sh field   # полный bag
./scripts/run_vins_ahrs_scene.sh water
./scripts/run_vins_ahrs_scene.sh forest
python3 tools/generate_vins_report.py
cp results/ПРОМЕЖУТОЧНЫЙ_ОТЧЁТ_VINS.pdf .
```

Окно 90 с (быстро): `./scripts/eval_three_scenes.sh 90`

---

## Архитектура

```
Камера + Livox IMU + баро ──► VINS-Mono ──► fuse_vins_baro (Z)
                                          └──► fuse_vins_ahrs (курс DJI)
                                                    │
                                                    ▼
                                          evo ATE  vs  RTK (только оценка)

Камера (город) ──► DSO ──► dispatcher scene=city
```

Подробнее: [docs/architecture.md](docs/architecture.md)

---

## Авторы и ссылки

- [VINS-Mono](https://github.com/hkust-aerial-robotics/VINS-Mono)
- [DSO](https://github.com/JakobEngel/dso)
- [MARS-LVIG](https://mars.hku.hk/dataset.html)

MIT — см. [LICENSE](LICENSE).
