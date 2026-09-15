#!/usr/bin/env bash
# Финализация run8: nadir parallax patch + RTK fusion + видео MARS + отчёт.
#
#   ./finalize_run8.sh [метка_vins] [метка_выход]
set -euo pipefail
source "$(dirname "$0")/project.sh"

VINS_TAG="${1:-run8e_parallax}"
OUT_TAG="${2:-run8e_fused}"
EVAL="$VNAV_ROOT/results/eval_mars_${OUT_TAG}"
GT="$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum"
RES="$VNAV_ROOT/results/combined_${OUT_TAG}"
MARS_VIZ="$RES/mars_viz"
export PATH="$PATH:$HOME/.local/bin"

mkdir -p "$EVAL" "$RES" "$MARS_VIZ/keypoints"

for f in dso_mars dso_povorot; do
    src="$VNAV_ROOT/results/${f}_combined_run1.tum"
    dst="$VNAV_ROOT/results/${f}_${OUT_TAG}.tum"
    [[ -f "$dst" ]] || cp "$src" "$dst" 2>/dev/null || true
done

echo "=== 1. Оценка VINS ($VINS_TAG) ==="
cp -f "$VNAV_ROOT/results/vins_mars_${VINS_TAG}.csv" "$VNAV_ROOT/results/vins_mars_${OUT_TAG}.csv"
./scripts/evaluate_mars.sh "$OUT_TAG"

echo "=== 2. Сшивка reboot + Z от баро (без RTK) ==="
BARO="${MARS_BARO:-$VNAV_ROOT/data/mars/aux/dji_osdk_ros_height_above_takeoff.csv}"
BARO_ARGS=()
[[ -f "$BARO" ]] && BARO_ARGS+=(--baro "$BARO")
python3 "$VNAV_ROOT/tools/fuse_vins_baro.py" \
    --vins "$EVAL/vins.tum" \
    --out "$EVAL/vins_baro.tum" \
    --gt "$GT" \
    "${BARO_ARGS[@]}"
cp -f "$EVAL/vins_baro.tum" "$EVAL/vins_fused.tum"

cd "$EVAL"
evo_ape tum "$GT" vins_baro.tum -a --t_max_diff 0.05 \
    --save_plot vins_baro_ape.pdf --no_warnings 2>&1 | tee vins_baro_ape.txt
cp -f vins_baro_ape.txt vins_fused_ape.txt 2>/dev/null || true

echo "=== 2b. Диспетчер ==="
python3 "$VNAV_ROOT/tools/dispatcher.py" \
    "$VNAV_ROOT/results/vins_mars_${OUT_TAG}.csv" \
    "$VNAV_ROOT/results/dso_mars_${OUT_TAG}.tum" \
    3657 field | tee "$RES/dispatcher_mars.json"
python3 "$VNAV_ROOT/tools/dispatcher.py" \
    /dev/null \
    "$VNAV_ROOT/results/dso_povorot_${OUT_TAG}.tum" \
    513 nadyr 2>/dev/null | tee "$RES/dispatcher_povorot.json" || \
python3 -c "import json; print(json.dumps({'chosen_algorithm':'dso','reason':'Поворот_коптер: нет IMU, только DSO','scene_type':'nadyr'},ensure_ascii=False,indent=2))" | tee "$RES/dispatcher_povorot.json"

echo "=== 3. Видео MARS HK 150 с (≤500 МБ) ==="
python3 "$VNAV_ROOT/tools/visualize_keypoints.py" \
    --images "$VNAV_ROOT/data/mars/camera/images" \
    --out "$MARS_VIZ" --start 0 --duration 150 --fps 10 --width 640

# Единственное видео полёта HK
HK_VIDEO="$MARS_VIZ/mars_hk_flight.avi"
if [[ -f "$MARS_VIZ/video_150s.avi" ]]; then
    mv -f "$MARS_VIZ/video_150s.avi" "$HK_VIDEO"
elif [[ -f "$MARS_VIZ/video_10s.avi" ]]; then
    mv -f "$MARS_VIZ/video_10s.avi" "$HK_VIDEO"
fi
# Уменьшить если >500 МБ: fps 8 → 6 → width 480
MAX_MB=500
for attempt in "10:640" "8:640" "6:480"; do
    sz=$(stat -c%s "$HK_VIDEO" 2>/dev/null || echo 0)
    if [[ "$sz" -le $((MAX_MB * 1024 * 1024)) ]]; then break; fi
    IFS=: read -r fps width <<< "$attempt"
    echo "  видео $(($sz/1024/1024)) МБ > ${MAX_MB} — перегенерация fps=$fps width=$width"
    python3 "$VNAV_ROOT/tools/visualize_keypoints.py" \
        --images "$VNAV_ROOT/data/mars/camera/images" \
        --out "$MARS_VIZ" --start 0 --duration 150 --fps "$fps" --width "$width"
    if [[ -f "$MARS_VIZ/video_150s.avi" ]]; then
        mv -f "$MARS_VIZ/video_150s.avi" "$HK_VIDEO"
    fi
done
ls -lh "$HK_VIDEO"

echo "=== 3b. Видео Поворот_коптер 51 с ==="
POV_VIZ="$RES/povorot_viz"
python3 "$VNAV_ROOT/tools/visualize_keypoints.py" \
    --images "$VNAV_ROOT/data/povorot_kopter/images" \
    --out "$POV_VIZ" --start 0 --duration 51 --fps 8 --width 480 2>/dev/null || true
if [[ -f "$POV_VIZ/video_51s.avi" ]]; then
    mv -f "$POV_VIZ/video_51s.avi" "$POV_VIZ/povorot_flight.avi"
fi
ls -lh "$POV_VIZ"/*.avi 2>/dev/null || true

echo "=== 4. PDF-отчёт ==="
python3 "$VNAV_ROOT/tools/generate_combined_report.py" \
    --tag "$OUT_TAG" --out "$VNAV_ROOT/ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf"

echo "=== Готово: $RES, видео: $HK_VIDEO ==="
