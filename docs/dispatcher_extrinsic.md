# Диспетчер VINS+DSO и калибровка extrinsic

## 1. Диспетчер (выбор алгоритма)

### tools/dispatcher.py
Выбирает VINS или DSO по типу сцены и качеству трекинга.

**Логика для надира (профиль fire-dron):**
- VINS инициализировался и держит трек → **выбрать VINS** (основной канал)
- DSO потерял → **fallback на VINS**
- Оба потеряны → **VINS** (приоритет для надира с IMU)

**Логика для города:**
- DSO инициализировался и держит трек → **выбрать DSO** (rich texture)
- VINS потерял → **fallback на DSO**

**Использование:**
```bash
python3 tools/dispatcher.py <vins_csv> <dso_tum> <num_frames> [scene_type]

# Пример: надирный прогон
python3 tools/dispatcher.py \
  results/vins_mars_run2_calib.csv \
  results/dso_mars_run2_calib.tum \
  3657 nadyr

# Выход: JSON с выбранным алгоритмом и причиной
```

**Результат для Поворот_коптер:**
```json
{
  "chosen_algorithm": "vins",
  "reason": "VINS (надир + IMU, основной канал)",
  "vins": {
    "num_poses": 1603,
    "coverage_pct": 312.5,
    "is_initialized": true,
    "is_lost": false
  },
  "dso": {
    "num_poses": 35,
    "coverage_pct": 6.8,
    "is_initialized": true,
    "is_lost": false
  }
}
```

## 2. Калибровка extrinsic камера–IMU

### Проблема
Официальный yaml MARS содержит камера–**LiDAR** экстринсики, не камера–**DJI IMU**.
Это одна из причин остаточной ошибки (ATE 151 м после intrinsics-калибровки).

### Решение (3 варианта в порядке приоритета)

#### 1. CAD-модель DJI M300
- Скачать с DJI SDK или документации
- Извлечь матрицы R (rotation) и t (translation) камера→IMU
- Вставить в `mars_nadir.yaml`, раздел `extrinsicRotation` / `extrinsicTranslation`
- Установить `estimate_extrinsic: 0` (не переоценивать)

**Результат:** Может ещё улучшить ATE на 10–20%.

#### 2. Kalibr-калибровка
Если CAD недоступен, провести собственную калибровку:

```bash
# На аэродроме MARS с шахматной доской
# 1. Снять видео 30–60 сек, камера смотрит на доску под разными углами
# 2. Синхронизировать с IMU логом
# 3. Запустить Kalibr:

rosrun kalibr kalibr_calibrate_cameras \
  --bag <bag_with_imu_and_cam> \
  --models <camera_model> \
  --topics /cam0/image_raw /imu0

# Калибр выдаст yaml с точными intrinsics и extrinsics
```

#### 3. Online-оценка (не рекомендуется для надира)
- Установить `estimate_extrinsic: 1` в mars_nadir.yaml
- Запустить на длинном спокойном участке полёта

**Риск:** На надирном вырожденном движении экстринсики не наблюдаемы.
VINS оценит большие смещения IMU bias вместо того, чтобы исправить экстринсики.

### Рекомендация для fire-dron
```yaml
# mars_nadir.yaml
estimate_extrinsic: 0

# Оставить приближённые значения из CAD до полного Kalibr-теста:
extrinsicRotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
extrinsicTranslation: [0, 0, 0]
```

После Kalibr обновить эти значения и повторить `./run_vins_mars.sh 0.3 50 run3_calib_extrinsic`.

## 3. Интеграция в mo_to_gps_dat.py

Диспетчер выбирает траекторию → мост переводит в gps_vo.dat / angle_vo.dat → БИНС.

```bash
# Пример для MARS с VINS (выбранный диспетчером):
python3 tools/vo_to_gps_dat.py \
  --traj results/vins_mars_run2_calib.tum \
  --ref-gps /path/to/mars_rtk.dat \
  --ref-angle /path/to/mars_angle.dat \
  --out-dir results/ins_input \
  --sigma0 3.0 \
  --sigma-rate 0.05

# Если выбран DSO, добавить масштаб:
python3 tools/vo_to_gps_dat.py \
  --traj results/dso_mars_run2_calib.tum \
  --scale 1.0 \  # или --scale-from-gps 30 (первые 30 сек)
  ...
```

## 4. Статус готовности

- ✅ Диспетчер создан (tools/dispatcher.py)
- ✅ Логика выбора VINS/DSO по сцене
- ✅ Скрипт оценки extrinsic (tools/estimate_extrinsic_camera_imu.py)
- ⚠️ Extrinsic: нужна Kalibr-калибровка или CAD M300
- ⚠️ Интеграция: мост vo_to_gps_dat.py готов, растущая σ в БИНС — по отчёту

## 5. Сеточный отбор точек (feature_tracker)

### tools/patch_feature_bucketing.py

Патч для VINS-Mono фронтенда: вместо глобального `goodFeaturesToTrack` — **по одной точке на блок 32×32** (как DSO), с fallback по максимуму градиента в тёмных ячейках. При превышении `max_cnt` оставляются точки с наибольшим corner eigenvalue.

```bash
python3 tools/patch_feature_bucketing.py --vins ~/catkin_ws/src/VINS-Mono
# пересборка
cd ~/catkin_ws && catkin_make --pkg feature_tracker
# прогон
./scripts/run_vins_mars.sh 0.3 50 run6_bucketing
./scripts/evaluate_mars.sh run6_bucketing
```

Откат: `--revert` (восстанавливает из `.orig`).

На MARS 608×512 покрытие 8×8-сетки улучшается с **25–30% пустых ячеек** (глобальный Shi-Tomasi) до **1.6–4.7%** (полная сетка run6).

**Быстрая проверка** (окно 90 с bag, без полного прогона ~2 мин):

```bash
./scripts/run_vins_mars.sh 1.0 50 run7_q 90
python3 tools/compare_segment.py --gt data/mars/mars_hkairport03_gt.tum --start 50 --duration 90 --fuse \\
    results/vins_mars_run2_calib.csv results/vins_mars_run7_hybrid_q2.csv
```

**Продакшен:** `run2_calib` VINS + RTK fusion → `run4_fused` (~107 м ATE). Гибридный патч (global + сетка на разворотах, `MIN_EIGEN≥12`, без gradient-fallback) — в `feature_tracker.cpp`; на 90 с fused **36 м** vs run2 **40 м**, на 150 с run2 всё ещё лучше (**94 м** vs 102 м).

## 6. Следующие шаги

1. **Найти CAD M300** или провести Kalibr-калибровку
2. **Обновить mars_nadir.yaml** с новыми extrinsics
3. **Перезапустить VINS** с улучшенной калибровкой (run3_calib_extrinsic)
4. **Оценить новый ATE** через `./evaluate_mars.sh run3_calib_extrinsic`
5. **Запустить диспетчер** на обновленных результатах
6. **Интегрировать в БИНС** через vo_to_gps_dat.py с растущей σ
