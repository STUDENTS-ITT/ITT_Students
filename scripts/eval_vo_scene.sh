#!/usr/bin/env bash
# Надирная VO (планиметрия / гомография / essential) + видео треков.
# Курс — AHRS DJI, масштаб — баро/крейсер. RTK только для графиков.
set -euo pipefail
source "$(dirname "$0")/project.sh"
vnav_setup_ros

SCENE="${1:?scene: field|water|forest}"
RES="$VNAV_ROOT/results/eval_vo_${SCENE}"
mkdir -p "$RES"

case "$SCENE" in
  field)
    TITLE="HKairport03: VO без коррекции по GPS, курс AHRS DJI"
    YAML="$VNAV_ROOT/config/mars_livox.yaml"
    GT="${VNAV_ROOT}/data/mars/mars_hkairport03_livox_gt.tum"
    [[ -f "$GT" ]] || GT="$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum"
    BARO="$VNAV_ROOT/data/mars/mars_hkairport03_livox_baro.csv"
    IMU="$VNAV_ROOT/data/mars/aux/dji_osdk_ros_imu.csv"
    IMAGES="$VNAV_ROOT/data/mars/dso/images"
    STAMPS="$VNAV_ROOT/data/mars/camera/timestamps.csv"
    BAG="$VNAV_ROOT/data/mars/mars_hkairport03_livox.bag"
    ;;
  water)
    TITLE="HKisland01: VO без коррекции по GPS, курс AHRS DJI"
    YAML="$VNAV_ROOT/config/mars_livox.yaml"
    GT="$VNAV_ROOT/data/mars/mars_hkisland01_gt.tum"
    BARO="$VNAV_ROOT/data/mars/mars_hkisland01_baro.csv"
    IMU="$VNAV_ROOT/data/mars/mars_hkisland01_dji_imu.csv"
    BAG="$VNAV_ROOT/data/mars/mars_hkisland01.bag"
    IMAGES=""
    STAMPS=""
    ;;
  forest)
    TITLE="AMvalley01: VO без коррекции по GPS, курс AHRS DJI"
    YAML="$VNAV_ROOT/config/mars_livox_valley.yaml"
    GT="$VNAV_ROOT/data/mars/mars_amvalley01_gt.tum"
    BARO="$VNAV_ROOT/data/mars/mars_amvalley01_baro.csv"
    IMU="$VNAV_ROOT/data/mars/mars_amvalley01_dji_imu.csv"
    BAG="$VNAV_ROOT/data/mars/mars_amvalley01.bag"
    IMAGES=""
    STAMPS=""
    ;;
  *)
    echo "неизвестная сцена $SCENE"; exit 1
    ;;
esac

VID="$VNAV_ROOT/results/videos/track_${SCENE}.mp4"
mkdir -p "$(dirname "$VID")"

ARGS=(
  --out "$RES"
  --title "$TITLE"
  --yaml "$YAML"
  --video "$VID"
)
[[ -f "$IMU" ]] && ARGS+=(--imu "$IMU")
[[ -f "$BARO" ]] && ARGS+=(--baro "$BARO")
[[ -f "$GT" ]] && ARGS+=(--gt "$GT")

if [[ -n "${IMAGES}" && -d "$IMAGES" && -f "$STAMPS" ]]; then
  ARGS+=(--images "$IMAGES" --stamps "$STAMPS")
else
  [[ -f "$BAG" ]] || { echo "нет bag $BAG — сначала mcap_to_vins.py"; exit 1; }
  ARGS+=(--bag "$BAG")
fi

echo "=== VO $SCENE ==="
python3 "$VNAV_ROOT/tools/nadir_homography_vo.py" "${ARGS[@]}"
echo "готово: $RES  видео: $VID"
