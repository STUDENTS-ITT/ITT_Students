#!/usr/bin/env bash
# Прогон трёх сцен MARS: поле (HKairport03), вода (HKisland01), лес (AMvalley01).
# ATE только vs RTK-GT, без RTK-коррекции траектории.
#
#   ./scripts/eval_three_scenes.sh           # окно 90 с
#   ./scripts/eval_three_scenes.sh 0         # полный полёт (долго)
set -euo pipefail
source "$(dirname "$0")/project.sh"
vnav_setup_ros

WINDOW="${1:-90}"
RATE="${RATE:-1.0}"
START="${START:-50}"
RES="$VNAV_ROOT/results"
mkdir -p "$RES"

run_one() {
    local name="$1" bag="$2" yaml="$3" launch="$4" gt="$5" baro="$6" scene="$7" frames="$8" start="$9"
    local tag="scene_${name}"
    local dur_args=()
    if [[ "$WINDOW" != "0" ]]; then
        dur_args=("$WINDOW")
    fi
    echo
    echo "========== $name / $scene =========="
    if [[ ! -f "$bag" ]]; then
        echo "НЕТ bag $bag — пропуск. Скачайте: ./scripts/download_mars_seq.sh и official_bag_to_vins.py"
        echo "{\"scene\":\"$scene\",\"skipped\":true,\"reason\":\"no bag\"}" > "$RES/dispatcher_${tag}.json"
        return 0
    fi
    export MARS_BAG="$bag"
    export MARS_VINS_YAML="$yaml"
    export MARS_LAUNCH="$launch"
    if ! ./scripts/run_vins_mars.sh "$RATE" "$start" "$tag" "${dur_args[@]}"; then
        echo "VINS не сошёлся на $name"
        return 0
    fi
    export EVAL_GT="$gt"
    export MARS_BARO="$baro"
    case "$scene" in
      field)  export MARS_DJI_IMU="$VNAV_ROOT/data/mars/aux/dji_osdk_ros_imu.csv" ;;
      water)  export MARS_DJI_IMU="$VNAV_ROOT/data/mars/mars_hkisland01_dji_imu.csv" ;;
      forest) export MARS_DJI_IMU="$VNAV_ROOT/data/mars/mars_amvalley01_dji_imu.csv" ;;
    esac
    if [[ -f "$gt" ]]; then
        ./scripts/production_mars.sh "$RES/vins_mars_${tag}.csv" "$tag" "$gt" "$baro" || true
    else
        echo "Нет GT $gt — только сшивка без ATE"
        mkdir -p "$RES/eval_mars_${tag}"
        python3 "$VNAV_ROOT/tools/fuse_vins_baro.py" \
            --vins "$RES/vins_mars_${tag}.csv" --out "$RES/eval_mars_${tag}/vins_baro.tum" \
            ${baro:+--baro "$baro"} || true
    fi
    python3 "$VNAV_ROOT/tools/dispatcher.py" \
        "$RES/vins_mars_${tag}.csv" \
        /dev/null \
        "$frames" "$scene" | tee "$RES/dispatcher_${tag}.json"
}

# Поле: официальный Livox IMU, если bag собран; иначе DJI fallback.
AIR_LIVOX="$VNAV_ROOT/data/mars/mars_hkairport03_livox.bag"
AIR_BAG="$VNAV_ROOT/data/mars/mars_hkairport03.bag"
AIR_YAML="$VNAV_ROOT/config/mars_nadir.yaml"
AIR_LAUNCH="mars_nadir.launch"
AIR_GT="$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum"
AIR_BARO="$VNAV_ROOT/data/mars/aux/dji_osdk_ros_height_above_takeoff.csv"
if [[ -f "$AIR_LIVOX" ]]; then
    AIR_BAG="$AIR_LIVOX"
    AIR_YAML="$VNAV_ROOT/config/mars_livox_airport.yaml"
    AIR_LAUNCH="mars_livox_airport.launch"
    [[ -f "${AIR_LIVOX%.bag}_gt.tum" ]] && AIR_GT="${AIR_LIVOX%.bag}_gt.tum"
    [[ -f "${AIR_LIVOX%.bag}_baro.csv" ]] && AIR_BARO="${AIR_LIVOX%.bag}_baro.csv"
fi

# Вода / лес: Livox IMU после конвертации MCAP (или официального bag).
ISL_MCAP="$VNAV_ROOT/data/mars/raw/HKisland01.mcap"
ISL_RAW="$VNAV_ROOT/data/mars/raw/HKisland01.bag"
ISL_BAG="$VNAV_ROOT/data/mars/mars_hkisland01.bag"
ISL_GT="$VNAV_ROOT/data/mars/mars_hkisland01_gt.tum"
ISL_BARO="$VNAV_ROOT/data/mars/mars_hkisland01_baro.csv"

VAL_MCAP="$VNAV_ROOT/data/mars/raw/AMvalley01.mcap"
VAL_RAW="$VNAV_ROOT/data/mars/raw/AMvalley01.bag"
VAL_BAG="$VNAV_ROOT/data/mars/mars_amvalley01.bag"
VAL_GT="$VNAV_ROOT/data/mars/mars_amvalley01_gt.tum"
VAL_BARO="$VNAV_ROOT/data/mars/mars_amvalley01_baro.csv"

SKIP_FIELD="${SKIP_FIELD:-0}"

convert_scene() {
    local mcap="$1" raw="$2" bag="$3" start_win="$4"
    [[ -f "$bag" ]] && return 0
    if [[ -f "$mcap" ]]; then
        echo "MCAP → VINS bag: $mcap"
        python3 "$VNAV_ROOT/tools/mcap_to_vins.py" --mcap "$mcap" --out "$bag"
        return 0
    fi
    if [[ -f "$raw" ]]; then
        if [[ "$WINDOW" == "0" ]]; then
            python3 "$VNAV_ROOT/tools/official_bag_to_vins.py" --bag "$raw" --out "$bag"
        else
            python3 "$VNAV_ROOT/tools/official_bag_to_vins.py" --bag "$raw" --out "$bag" \
                --start "$start_win" --duration $((WINDOW + 40))
        fi
    fi
}
convert_scene "$ISL_MCAP" "$ISL_RAW" "$ISL_BAG" 108
convert_scene "$VAL_MCAP" "$VAL_RAW" "$VAL_BAG" 60

if [[ "$SKIP_FIELD" != "1" ]]; then
    run_one field "$AIR_BAG" "$AIR_YAML" "$AIR_LAUNCH" \
        "$AIR_GT" "$AIR_BARO" field 900 "$START"
fi

# Срез bag начинается за 10 с до eval → play -s 10
ISL_PLAY=118
VAL_PLAY=70
if [[ "$WINDOW" != "0" ]]; then
    ISL_PLAY=10
    VAL_PLAY=10
fi

run_one water "$ISL_BAG" "$VNAV_ROOT/config/mars_livox.yaml" mars_livox.launch \
    "$ISL_GT" "$ISL_BARO" water 750 "$ISL_PLAY"

run_one forest "$VAL_BAG" "$VNAV_ROOT/config/mars_livox_valley.yaml" mars_livox_valley.launch \
    "$VAL_GT" "$VAL_BARO" forest 1200 "$VAL_PLAY"

echo
echo "=== сводка ==="
for f in "$RES"/dispatcher_scene_*.json; do
    [[ -f "$f" ]] && echo "-- $(basename "$f") --" && cat "$f"
done
ls -1 "$RES"/eval_mars_scene_*/vins_baro_ape.txt 2>/dev/null || true
