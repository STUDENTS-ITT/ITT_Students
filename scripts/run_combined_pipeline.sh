#!/usr/bin/env bash
# Объединённый пайплайн: VINS + DSO + диспетчер на MARS и Поворот_коптер.
#
#   ./run_combined_pipeline.sh [метка]
set -euo pipefail
source "$(dirname "$0")/project.sh"

TAG="${1:-combined_run1}"
RES="$VNAV_ROOT/results/combined_${TAG}"
mkdir -p "$RES"

echo "=== 1. Синхронизация mars_nadir.yaml ==="
mkdir -p ~/catkin_ws/src/VINS-Mono/config/mars
cp "$VNAV_ROOT/config/mars_nadir.yaml" ~/catkin_ws/src/VINS-Mono/config/mars/mars_nadir.yaml
sed -i "s|/home/USER|$VNAV_ROOT|g" ~/catkin_ws/src/VINS-Mono/config/mars/mars_nadir.yaml

echo "=== 2. VINS MARS (extrinsic calib) ==="
if [[ ! -f "$VNAV_ROOT/results/vins_mars_${TAG}.csv" ]]; then
    "$VNAV_ROOT/scripts/run_vins_mars.sh" 0.3 50 "$TAG" || true
    if [[ -f "$VNAV_ROOT/results/vins_out/vins_result_no_loop.csv" ]]; then
        cp "$VNAV_ROOT/results/vins_out/vins_result_no_loop.csv" "$VNAV_ROOT/results/vins_mars_${TAG}.csv"
    fi
fi

echo "=== 3. DSO MARS ==="
if [[ ! -s "$VNAV_ROOT/results/dso_mars_${TAG}.tum" ]]; then
    "$VNAV_ROOT/scripts/run_dso_mars.sh" 0 "$TAG" || true
fi

echo "=== 4. DSO Поворот_коптер ==="
if [[ ! -s "$VNAV_ROOT/results/dso_povorot_${TAG}.tum" ]]; then
    "$VNAV_ROOT/scripts/run_dso_povorot.sh" 0 "$TAG" || true
fi

echo "=== 5. Оценка MARS ==="
"$VNAV_ROOT/scripts/evaluate_mars.sh" "$TAG" || true

echo "=== 6. Диспетчер ==="
python3 "$VNAV_ROOT/tools/dispatcher.py" \
    "$VNAV_ROOT/results/vins_mars_${TAG}.csv" \
    "$VNAV_ROOT/results/dso_mars_${TAG}.tum" \
    3657 nadyr | tee "$RES/dispatcher_mars.json"

python3 "$VNAV_ROOT/tools/dispatcher.py" \
    /dev/null \
    "$VNAV_ROOT/results/dso_povorot_${TAG}.tum" \
    513 nadyr | tee "$RES/dispatcher_povorot.json" || \
python3 - <<'PY' | tee "$RES/dispatcher_povorot.json"
import json
print(json.dumps({"chosen_algorithm":"dso","reason":"Поворот_коптер: нет IMU, только DSO","scene_type":"nadyr"}, ensure_ascii=False, indent=2))
PY

echo "=== 7. Визуализация ключевых точек ==="
python3 "$VNAV_ROOT/tools/visualize_keypoints.py" \
    --images "$VNAV_ROOT/data/mars/camera/images" \
    --out "$RES/mars_viz" --start 500 --duration 10 --width 640

python3 "$VNAV_ROOT/tools/visualize_keypoints.py" \
    --images "$VNAV_ROOT/data/povorot_kopter/images" \
    --out "$RES/povorot_viz" --start 50 --duration 10 --width 640

echo "=== 8. PDF-отчёт ==="
python3 "$VNAV_ROOT/tools/generate_combined_report.py" --tag "$TAG" --out "$VNAV_ROOT/ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf"

echo "=== Готово: $RES ==="
