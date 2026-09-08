#!/usr/bin/env bash
# Прогон DSO на собственном надирном видео «Поворот_коптер».
#
#   ./run_dso_povorot.sh [playbackSpeed] [метка]
#
# Что с этого набора можно взять и чего нельзя:
#   можно  -- время обработки кадра на реальном надирном видео;
#             качественную оценку сходимости; грубую оценку дрейфа по
#             несовпадению начала и конца (коптер возвращается в исходную точку).
#   нельзя -- численную точность: эталона нет, IMU нет, калибровка приближённая.
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/project.sh"

SPEED="${1:-0}"
TAG="${2:-run1}"

DSO_BIN=~/dso/build/bin/dso_dataset
DATA="$VNAV_ROOT/data/povorot_kopter"
RES="$VNAV_ROOT/results"

[[ -x "$DSO_BIN" ]] || { echo "DSO не собран, запустите ./06_build_dso.sh"; exit 1; }
[[ -d "$DATA/images" ]] || {
    echo "Нет подготовленных кадров в $DATA/images."
    echo "Сначала выполните:"
    echo "  python3 $VNAV_ROOT/tools/frames_to_dso.py \\"
    echo "      --frames '/media/\$USER/DISK/Датасет/Datasets/Поворот_коптер/frames_1' \\"
    echo "      --out $DATA --fps 10 --hfov 73 --scale 0.5"
    exit 1
}

mkdir -p "$RES/timing"
export DSO_TIMING_LOG="$RES/timing/dso_povorot_s${SPEED}_${TAG}.csv"
rm -f "$DSO_TIMING_LOG"

echo "=== DSO / Поворот_коптер: playbackSpeed=$SPEED, метка $TAG ==="
echo "кадров: $(ls -1 "$DATA/images" | wc -l)"

cd "$DATA"
"$DSO_BIN" \
    files="$DATA/images" \
    calib="$DATA/camera.txt" \
    preset=0 \
    mode=1 \
    nogui=1 \
    quiet=1 \
    playbackSpeed="$SPEED" \
    2>&1 | tee "$RES/dso_povorot_${TAG}.log"

if [[ -f "$DATA/result.txt" ]]; then
    cp "$DATA/result.txt" "$RES/dso_povorot_${TAG}.tum"
    echo "готово: $(wc -l < "$DATA/result.txt") поз"

    # Замкнулась ли траектория. Масштаб условный, поэтому величина
    # выражается в долях пройденного пути, а не в метрах.
    python3 - "$RES/dso_povorot_${TAG}.tum" <<'PY'
import sys
import numpy as np
rows = [l.split() for l in open(sys.argv[1]) if l.strip() and not l.startswith('#')]
p = np.array([[float(v) for v in r[1:4]] for r in rows])
path = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
gap = float(np.linalg.norm(p[-1] - p[0]))
print(f"\nПройдено (усл. ед.): {path:.3f}")
print(f"Расхождение начала и конца: {gap:.3f} = {100 * gap / path:.2f} % пути")
print("Коптер возвращался в исходную точку, поэтому эта доля — грубая")
print("оценка накопленного дрейфа. Абсолютных метров тут нет: DSO")
print("выдаёт траекторию с точностью до неизвестного масштаба.")
PY
else
    echo "ОШИБКА: result.txt не создан — DSO потерял трекинг сразу."
    echo "Проверьте camera.txt (калибровка приближённая) и качество кадров."
    exit 1
fi

[[ -f "$DSO_TIMING_LOG" ]] && echo "замеров времени: $(wc -l < "$DSO_TIMING_LOG")"
