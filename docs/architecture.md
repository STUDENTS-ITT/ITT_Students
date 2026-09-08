# Архитектура fire-dron

Объединённый конвейер визуальной одометрии для надирных полётов БПЛА.
VINS-Mono и DSO работают параллельно; диспетчер выбирает канал по типу сцены
и качеству трекинга. Для MARS-LVIG траектория VINS дополнительно сливается
с RTK через `fuse_vins_rtk.py`.

## Компоненты

```
Кадры + IMU/GPS
       │
       ├─► VINS-Mono (~/catkin_ws) ──► vins_mars_*.csv
       │
       └─► DSO (~/dso) ──────────────► dso_mars_*.tum / dso_povorot_*.tum
                    │
                    ▼
            tools/dispatcher.py
                    │
       ┌────────────┴────────────┐
       ▼                         ▼
 fuse_vins_rtk.py          vo_to_gps_dat (внешний)
 (MARS + RTK GT)                  │
       │                          ▼
       └────────► БИНС / отчёт ◄──┘
```

## VINS-Mono (вне репозитория)

Исходники и сборка — в `~/catkin_ws/src/VINS-Mono`. Конфиги проекта копируются
из `config/`:

- `config/mars_nadir.yaml` → `~/catkin_ws/src/VINS-Mono/config/mars/`
- `config/mars_nadir.launch` → `~/catkin_ws/src/VINS-Mono/vins_estimator/launch/`

Патчи к VINS применяются скриптами `tools/patch_*.py` (nadir parallax,
feature bucketing, timing). Подробнее — `docs/patches_nadir_parallax.md`.

## DSO (вне репозитория)

Сборка: `~/dso/build/bin/dso_dataset`. Вход — папки `data/mars/dso/` и
`data/povorot_kopter/` с `images/`, `camera.txt`, `times.txt`.

## Продакшен-прогон (run8e_fused)

1. `scripts/run_vins_mars.sh 0.3 50 run8e_parallax` — VINS на bag MARS
2. `scripts/production_mars.sh` — оценка + RTK fusion
3. `scripts/finalize_run8.sh run8e_parallax run8e_fused` — видео, диспетчер, PDF

## Датасеты

| Каталог | Описание |
|---------|----------|
| `data/mars/` | MARS-LVIG HKairport03: bag, RTK GT, кадры VINS, DSO-версия |
| `data/povorot_kopter/` | «Поворот_коптер»: надирное видео без IMU, только DSO |

Подробнее о датасетах и внешних проектах: [PROJECTS.ru.md](PROJECTS.ru.md).

## Результаты

Минимальный набор для отчёта — `results/combined_run8e_fused/`,
`results/eval_mars_run8e_fused/`, `results/vins_mars_run8e_parallax.csv`.
