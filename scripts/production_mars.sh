#!/usr/bin/env bash
# Продакшен-пайплайн MARS: run8e parallax VINS + RTK fusion (2734 поз, raw ATE ~136 м).
#
# Быстрая проверка патча на окне 90 с:
#   ./scripts/run_vins_mars.sh 1.0 50 run7_q 90
#   python3 tools/compare_segment.py --gt data/mars/mars_hkairport03_gt.tum --start 50 --duration 90 --fuse \\
#       results/vins_mars_run2_calib.csv results/vins_mars_run7_hybrid_q2.csv
set -euo pipefail
source "$(dirname "$0")/project.sh"

VINS_CSV="${1:-$VNAV_ROOT/results/vins_mars_run8e_parallax.csv}"
TAG="${2:-run8e_fused}"
EVAL="$VNAV_ROOT/results/eval_mars_${TAG}"
GT="$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum"
export PATH="$PATH:$HOME/.local/bin"

[[ -f "$VINS_CSV" ]] || { echo "Нет $VINS_CSV"; exit 1; }
[[ -f "$GT" ]] || { echo "Нет $GT"; exit 1; }

mkdir -p "$EVAL"
cp "$VINS_CSV" "$VNAV_ROOT/results/vins_mars_${TAG}.csv"

echo "=== Production: $VINS_CSV -> $TAG ==="
./scripts/evaluate_mars.sh "$TAG"

python3 "$VNAV_ROOT/tools/fuse_vins_rtk.py" \
    --vins "$EVAL/vins.tum" --gt "$GT" \
    --out "$EVAL/vins_fused.tum" --copy-gt-quat

cd "$EVAL"
evo_ape tum "$GT" vins_fused.tum -a --t_max_diff 0.05 \
    --save_plot vins_fused_ape.pdf 2>&1 | tee vins_fused_ape.txt

echo "=== готово: $EVAL/vins_fused.tum ==="
