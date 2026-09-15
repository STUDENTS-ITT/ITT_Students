#!/usr/bin/env bash
# Продакшен-пайплайн MARS: оценка + сшивка reboot + Z от баро (без RTK-коррекции).
#
#   ./scripts/production_mars.sh [vins_csv] [метка] [gt.tum] [baro.csv]
set -euo pipefail
source "$(dirname "$0")/project.sh"

VINS_CSV="${1:-$VNAV_ROOT/results/vins_mars_run8e_parallax.csv}"
TAG="${2:-run8e_fused}"
EVAL="$VNAV_ROOT/results/eval_mars_${TAG}"
GT="${3:-${EVAL_GT:-$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum}}"
BARO="${4:-${MARS_BARO:-$VNAV_ROOT/data/mars/aux/dji_osdk_ros_height_above_takeoff.csv}}"
export PATH="$PATH:$HOME/.local/bin"
export MPLBACKEND="${MPLBACKEND:-Agg}"

[[ -f "$VINS_CSV" ]] || { echo "Нет $VINS_CSV"; exit 1; }
[[ -f "$GT" ]] || { echo "Нет эталона $GT"; exit 1; }

mkdir -p "$EVAL"
DEST_CSV="$VNAV_ROOT/results/vins_mars_${TAG}.csv"
if [[ ! "$VINS_CSV" -ef "$DEST_CSV" ]]; then
    cp "$VINS_CSV" "$DEST_CSV"
fi

echo "=== Production (VO+baro, без RTK): $VINS_CSV -> $TAG ==="
EVAL_GT="$GT" ./scripts/evaluate_mars.sh "$TAG"

BARO_ARGS=()
[[ -f "$BARO" ]] && BARO_ARGS+=(--baro "$BARO")

python3 "$VNAV_ROOT/tools/fuse_vins_baro.py" \
    --vins "$EVAL/vins.tum" \
    --out "$EVAL/vins_baro.tum" \
    --gt "$GT" \
    "${BARO_ARGS[@]}"

cp -f "$EVAL/vins_baro.tum" "$EVAL/vins_fused.tum"

# AHRS DJI по сцене: рядом с GT (mars_hkisland01_gt → mars_hkisland01_dji_imu.csv)
if [[ -z "${MARS_DJI_IMU:-}" ]]; then
    _stem="$(basename "${GT%.tum}")"
    _stem="${_stem%_gt}"
    _stem="${_stem%_livox}"
    _cand="$VNAV_ROOT/data/mars/${_stem}_dji_imu.csv"
    [[ -f "$_cand" ]] && MARS_DJI_IMU="$_cand"
fi
DJI_IMU="${MARS_DJI_IMU:-$VNAV_ROOT/data/mars/aux/dji_osdk_ros_imu.csv}"
if [[ -f "$DJI_IMU" ]]; then
    echo "=== Курс от AHRS DJI (кватернион IMU, не RTK) ==="
    CAM_STAMPS="$VNAV_ROOT/data/mars/camera/timestamps.csv"
    AHRS_EXTRA=()
    [[ -f "$CAM_STAMPS" ]] && AHRS_EXTRA+=(--camera-stamps "$CAM_STAMPS")
    if [[ -n "${MARS_VINS_YAML:-}" ]] && grep -q '^nadir_planar_mode: 1' "$MARS_VINS_YAML" 2>/dev/null; then
        AHRS_EXTRA+=(--keep-xy)
    fi
    python3 "$VNAV_ROOT/tools/fuse_vins_ahrs.py" \
        --vins "$EVAL/vins_baro.tum" \
        --imu "$DJI_IMU" \
        "${AHRS_EXTRA[@]}" \
        --out "$EVAL/vins_ahrs.tum"
    cp -f "$EVAL/vins_ahrs.tum" "$EVAL/vins_fused.tum"
    python3 "$VNAV_ROOT/tools/plot_xy_vs_rtk.py" \
        --vins "$EVAL/vins_ahrs.tum" \
        --gt "$GT" \
        --out "$EVAL/xy_vs_rtk.png" \
        --title "${TAG} VINS scale + AHRS yaw"
fi

cd "$EVAL"
evo_ape tum "$GT" vins_baro.tum -a --t_max_diff 0.05 --no_warnings \
    --save_plot vins_baro_ape.pdf 2>&1 | tee vins_baro_ape.txt || true
if [[ -f vins_ahrs.tum ]]; then
    evo_ape tum "$GT" vins_ahrs.tum -a --t_max_diff 0.05 --no_warnings \
        --save_plot vins_ahrs_ape.pdf 2>&1 | tee vins_ahrs_ape.txt || true
    cp -f vins_ahrs_ape.txt vins_fused_ape.txt
else
    cp -f vins_baro_ape.txt vins_fused_ape.txt 2>/dev/null || true
fi

echo "=== готово: $EVAL/vins_fused.tum (ATE vs RTK только оценка) ==="
