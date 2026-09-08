# Live RViz на MARS

Полная пошаговая инструкция: **[COMMANDS.ru.md](COMMANDS.ru.md)**.  
Объяснение всех проектов: **[PROJECTS.ru.md](PROJECTS.ru.md)**.

Визуализация как в [официальном демо VINS-Mono](https://www.youtube.com/watch?v=yRPl4zy9z1c):
tracked image, loop_match_image, 3D path, pose graph.

## Файлы

| Файл | Назначение |
|------|------------|
| `config/mars_rviz.rviz` | Layout RViz (RTK → `/mars/rtk_path`, VIO включён) |
| `config/mars_rviz.launch` | Launch RViz (нужен `VNAV_ROOT`) |
| `scripts/run_vins_mars_live.sh` | VINS + RTK publisher + RViz |
| `tools/rtk_path_publisher.py` | RTK TUM → `nav_msgs/Path` по времени odom |

## Запуск

```bash
cd ~/Downloads/cursor/Projects/fire-dron
./scripts/run_vins_mars_live.sh 0.3 50 --play 90
```

**Не запускайте** `roslaunch mars_nadir.launch` вручную без bag — RViz будет пустым.

### Что нормально

| Симптом | Объяснение |
|---------|------------|
| `throw img, only should happen at the beginning` | VINS отбрасывает кадры на старте — норма |
| **loop_match_image** = No Image | Петли замыкания редки на MARS — норма |
| **tracked image** есть, path растёт | **Всё работает** — это главный индикатор |
| `rostopic: команда не найдена` | ROS не source'нут в этом терминале — используйте только `./scripts/run_vins_mars_live.sh` |

### Если не работает

```bash
# 1. Остановить старые процессы
pkill -f "roscore|vins_estimator|feature_tracker|rviz|rosbag"

# 2. Проверить ROS
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash --extend
rostopic list

# 3. Запуск (bag + RViz + VINS одной командой)
cd ~/Downloads/cursor/Projects/fire-dron
./scripts/run_vins_mars_live.sh 0.3 50 --play 90
```

90 с хватит для демо-записи экрана. Скрипт поднимает VINS, RTK publisher и RViz; bag — флаг `--play`.

## Топики

| Панель RViz | Topic |
|-------------|-------|
| tracked image | `/feature_tracker/feature_img` |
| raw_image | `/cam0/image_raw` |
| loop_match_image | `/pose_graph/match_image` |
| VIO Path | `/vins_estimator/path` |
| rtk_path | `/mars/rtk_path` |
| odom (терминал) | `/vins_estimator/odometry` — **чистый VINS** |

`show_track: 1` в `config/mars_nadir.yaml` обязателен для tracked image.

### Чистый VINS vs RTK

| | Live | Batch |
|---|------|-------|
| `/vins_estimator/odometry` | чистый VINS | — |
| `/mars/rtk_path` | RTK только для RViz | — |
| `vins_fused.tum` | — | VINS+RTK offline |

Окно `rostopic echo` открывает `run_vins_mars_live.sh` (отключить: `--no-echo-odom`). RTK fusion в live **не** участвует.
