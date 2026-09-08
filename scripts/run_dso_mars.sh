#!/usr/bin/env bash
# Прогон DSO на MARS-LVIG HKairport03.
set -euo pipefail
source "$(dirname "$0")/project.sh"

SPEED="${1:-0}"
TAG="${2:-run1}"

DSO_BIN=~/dso/build/bin/dso_dataset
DATA="$VNAV_ROOT/data/mars/dso"
RES="$VNAV_ROOT/results"

[[ -x "$DSO_BIN" ]] || { echo "DSO не собран"; exit 1; }
[[ -d "$DATA/images" ]] || { echo "Нет $DATA/images"; exit 1; }

mkdir -p "$RES/timing"
export DSO_TIMING_LOG="$RES/timing/dso_mars_s${SPEED}_${TAG}.csv"
rm -f "$DSO_TIMING_LOG"

echo "=== DSO / MARS-LVIG: playbackSpeed=$SPEED, метка $TAG ==="
cd "$DATA"
"$DSO_BIN" files="$DATA/images" calib="$DATA/camera.txt" preset=0 mode=1 nogui=1 quiet=1 playbackSpeed="$SPEED" \
    2>&1 | tee "$RES/dso_mars_${TAG}.log"

if [[ -f "$DATA/result.txt" && -s "$DATA/result.txt" ]]; then
    cp "$DATA/result.txt" "$RES/dso_mars_${TAG}.tum"
    echo "готово: $(wc -l < "$DATA/result.txt") поз"
else
    echo "DSO LOST или пустой result.txt"
    : > "$RES/dso_mars_${TAG}.tum"
fi
