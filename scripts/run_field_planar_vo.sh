#!/usr/bin/env bash
# Field (HKairport03): планарная VO — гомография отделяет наклон от сдвига.
# RTK только для оценки и графиков.
set -euo pipefail
source "$(dirname "$0")/project.sh"
export MPLBACKEND="${MPLBACKEND:-Agg}"

BAG="${1:-$VNAV_ROOT/data/mars/mars_hkairport03_livox.bag}"
GT="${2:-$VNAV_ROOT/data/mars/mars_hkairport03_livox_gt.tum}"
OUT="${3:-$VNAV_ROOT/results/eval_mars_scene_field_vo}"
IMU="$VNAV_ROOT/data/mars/aux/dji_osdk_ros_imu.csv"
BARO="$VNAV_ROOT/data/mars/mars_hkairport03_livox_baro.csv"
YAML="$VNAV_ROOT/config/mars_livox_airport.yaml"
STAMPS="$VNAV_ROOT/data/mars/camera/timestamps.csv"

[[ -f "$BAG" ]] || { echo "нет bag: $BAG"; exit 1; }

python3 "$VNAV_ROOT/tools/nadir_homography_vo.py" \
    --bag "$BAG" \
    --imu "$IMU" \
    --baro "$BARO" \
    --gt "$GT" \
    --yaml "$YAML" \
    --camera-stamps "$STAMPS" \
    --out "$OUT" \
    --title "field planar VO (homography + AHRS yaw)"

python3 "$VNAV_ROOT/tools/plot_xy_vs_rtk.py" \
    --vins "$OUT/homography.tum" \
    --gt "$GT" \
    --out "$OUT/xy_vs_rtk.png" \
    --title "field homography + AHRS yaw"

echo "готово: $OUT (nadir_vo.tum = лучший канал)"
