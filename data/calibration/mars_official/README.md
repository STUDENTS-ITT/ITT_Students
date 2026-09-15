# Официальная калибровка MARS-LVIG

Источник: [Sensor Calibration](https://mars.hku.hk/dataset_sensor_calibration.html)
и выпущенные с датасетом `camera_ext_R` / `camera_ext_t` (камера → LiDAR).

- Intrinsics: шахматная доска (HK большая, Армения маленькая).
- Camera–LiDAR: targetless (как на странице).
- LiDAR–IMU (Avia): производитель, в FAST-LIVO2 `extrinsic_T: [0.04165, 0.02326, -0.0284]`, `extrinsic_R = I`.

VINS: `p_imu = R_ic p_cam + t_ic`, `R_ic = Rcl^T`, `t_ic = -T_il - Rcl^T Pcl`.

HKairport03 в маппинге датасета → **HK_GNSS**. Для `/livox/imu`.

- `config/mars_livox_airport.yaml` — Livox IMU, без поворота в DJI FLU.
- `config/mars_nadir.yaml` — fallback DJI OSDK IMU: `R_ic` повёрнут CAD-ом Livox X↓ = −Z_DJI.

Сборка bag: `python3 tools/mcap_to_vins.py --mcap data/mars/raw/HKairport03.mcap --out data/mars/mars_hkairport03_livox.bag`
