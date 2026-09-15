#!/usr/bin/env bash
# VINS-Mono (Livox) + курс AHRS DJI. RTK только ATE.
#   ./scripts/run_vins_ahrs_scene.sh water [rate] [start] [duration]
set -euo pipefail
source "$(dirname "$0")/project.sh"
vnav_setup_ros

SCENE="${1:?water|forest|field}"
RATE="${2:-1.0}"
START="${3:-0}"
DURATION="${4:-}"

case "$SCENE" in
  field)
    export MARS_BAG="$VNAV_ROOT/data/mars/mars_hkairport03_livox.bag"
    export MARS_VINS_YAML="$VNAV_ROOT/config/mars_livox_airport.yaml"
    export MARS_LAUNCH="mars_livox_airport.launch"
    export MARS_DJI_IMU="$VNAV_ROOT/data/mars/aux/dji_osdk_ros_imu.csv"
    GT="$VNAV_ROOT/data/mars/mars_hkairport03_livox_gt.tum"
    BARO="$VNAV_ROOT/data/mars/mars_hkairport03_livox_baro.csv"
    ;;
  water)
    export MARS_BAG="$VNAV_ROOT/data/mars/mars_hkisland01.bag"
    export MARS_VINS_YAML="$VNAV_ROOT/config/mars_livox.yaml"
    export MARS_LAUNCH="mars_livox.launch"
    export MARS_DJI_IMU="$VNAV_ROOT/data/mars/mars_hkisland01_dji_imu.csv"
    GT="$VNAV_ROOT/data/mars/mars_hkisland01_gt.tum"
    BARO="$VNAV_ROOT/data/mars/mars_hkisland01_baro.csv"
    ;;
  forest)
    export MARS_BAG="$VNAV_ROOT/data/mars/mars_amvalley01.bag"
    export MARS_VINS_YAML="$VNAV_ROOT/config/mars_livox_valley.yaml"
    export MARS_LAUNCH="mars_livox_valley.launch"
    export MARS_DJI_IMU="$VNAV_ROOT/data/mars/mars_amvalley01_dji_imu.csv"
    GT="$VNAV_ROOT/data/mars/mars_amvalley01_gt.tum"
    BARO="$VNAV_ROOT/data/mars/mars_amvalley01_baro.csv"
    ;;
  *) echo "сцена: field|water|forest"; exit 1 ;;
esac

TAG="scene_${SCENE}_ahrs"
[[ -f "$MARS_BAG" ]] || { echo "нет $MARS_BAG"; exit 1; }

if [[ -n "$DURATION" ]]; then
  ./scripts/run_vins_mars.sh "$RATE" "$START" "$TAG" "$DURATION"
else
  ./scripts/run_vins_mars.sh "$RATE" "$START" "$TAG"
fi

CSV="$VNAV_ROOT/results/vins_mars_${TAG}.csv"
[[ -f "$CSV" ]] || { echo "VINS не записал $CSV"; exit 1; }
./scripts/production_mars.sh "$CSV" "$TAG" "$GT" "$BARO"

# Field: жёсткая камера — XY от планарной VO (гомография), не VINS 6DOF
if [[ "$SCENE" == "field" ]]; then
  VO_OUT="$VNAV_ROOT/results/eval_mars_${TAG}_vo"
  ./scripts/run_field_planar_vo.sh "$MARS_BAG" "$GT" "$VO_OUT"
  EVAL="$VNAV_ROOT/results/eval_mars_${TAG}"
  cp -f "$VO_OUT/homography.tum" "$EVAL/vins_planar.tum"
  cp -f "$VO_OUT/xy_vs_rtk.png" "$EVAL/xy_planar_vs_rtk.png"
  cp -f "$VO_OUT/vo_three_panels.png" "$EVAL/vo_three_panels.png"
  echo "планарная VO (гомография): $EVAL/vins_planar.tum"
fi

echo "готово: $VNAV_ROOT/results/eval_mars_${TAG}"
