#!/usr/bin/env bash
# Надирная VO (планиметрия / гомография / essential) на HKairport03.
# VINS на этом bag с DJI IMU ломает форму пути; RTK только для ATE.
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "$0")/project.sh"
RES="$VNAV_ROOT/results/eval_mars_scene_field"
export MPLBACKEND="${MPLBACKEND:-Agg}"
export PATH="$PATH:$HOME/.local/bin"

python3 "$VNAV_ROOT/tools/nadir_homography_vo.py" \
    --images "$VNAV_ROOT/data/mars/dso/images" \
    --stamps "$VNAV_ROOT/data/mars/camera/timestamps.csv" \
    --imu "$VNAV_ROOT/data/mars/aux/dji_osdk_ros_imu.csv" \
    --baro "$VNAV_ROOT/data/mars/aux/dji_osdk_ros_height_above_takeoff.csv" \
    --gt "$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum" \
    --out "$RES"

GT="$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum"
cd "$RES"
if command -v evo_ape >/dev/null; then
    evo_ape tum "$GT" nadir_vo.tum -a --t_max_diff 0.05 --no_warnings \
        --plot_mode xy --save_plot nadir_vo_ape.pdf 2>&1 | tee nadir_vo_ape.txt || true
fi
echo "готово: $RES/nadir_vo_like_report.png"
