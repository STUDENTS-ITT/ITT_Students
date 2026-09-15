# Архитектура fire-dron

Объединённый конвейер визуальной одометрии для надирных полётов БПЛА.
VINS-Mono — основной канал (`field` / `water` / `forest`). DSO — запасной
вариант одометрии для города и фасадов (`scene=city`). После прогона сегменты
VINS сшиваются на reboot, Z берётся из барометра, курс — из AHRS DJI
(`fuse_vins_ahrs.py`). RTK — только эталон ATE, не коррекция.

Официальный IMU MARS-LVIG для VIO — **Livox Avia `/livox/imu`** (жёстко с камерой).
Гимбальный IMU DJI L1 в публичный rosbag не пишется. Уже собранный
`mars_hkairport03.bag` на DJI OSDK — запасной канал.

## Компоненты

```
Кадры + Livox IMU (+ баро)
       │
       ├─► VINS-Mono (~/catkin_ws) ──► vins_mars_*.csv
       │     reboot → last_P (patch_reboot_anchor.py)
       └─► DSO (~/dso)
                    │
                    ▼
            tools/dispatcher.py   field | water | forest
                    │
                    ▼
         fuse_vins_baro.py  ──► Z от баро, если это AGL (не RTK)
                    │
                    ▼
         fuse_vins_ahrs.py  ──► курс AHRS DJI; шаг с текстурных кадров
                    │          (без крейсера 8.5 м/с после clamp)
                    ▼
         evo ATE vs RTK GT  (только оценка, не inject)
```

## VINS-Mono (вне репозитория)

Исходники: `~/catkin_ws/src/VINS-Mono`. Конфиги из `config/`:

- `config/mars_nadir.yaml` + `mars_nadir.launch` — DJI IMU, bag HKairport03
- `config/mars_livox.yaml` + `mars_livox.launch` — Livox IMU, HKisland
- `config/mars_livox_valley.yaml` — AMvalley (Azizken Forest)

Патчи: `tools/patch_nadir_parallax.py`, `tools/patch_reboot_anchor.py`,
`tools/patch_scene_quality.py` (короткий трек, AGL, склон).

## Датасеты

| Каталог / файл | Сцена | IMU |
|----------------|-------|-----|
| `data/mars/mars_hkairport03.bag` | поле (аэродром) | DJI (fallback) |
| `data/mars/mars_hkisland01.bag` | вода (Kai Pei Chau) | Livox |
| `data/mars/mars_amvalley01.bag` | лес (Azizken Forest) | Livox |
| `data/povorot_kopter/` | поворот камеры | нет |

Скачать остров/лес: `scripts/download_mars_seq.sh` с [mars.hku.hk/dataset.html](https://mars.hku.hk/dataset.html).
Конвертация: `tools/official_bag_to_vins.py`.

## Прогон трёх сцен

```bash
./scripts/eval_three_scenes.sh      # окно 90 с
./scripts/eval_three_scenes.sh 0    # полный полёт
```
