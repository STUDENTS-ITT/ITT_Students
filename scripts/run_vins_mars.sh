#!/usr/bin/env bash
# Прогон VINS-Mono на MARS-LVIG (надирная камера).
#
#   ./run_vins_mars.sh [скорость] [старт_сек] [метка] [длительность_сек]
#
# Аргумент "длительность_сек" (опционально) — сколько секунд bag проиграть после старта.
# Для быстрой проверки: ./run_vins_mars.sh 1.0 50 run7_q 90  (~1.5 мин wall time)
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/project.sh"

RATE="${1:-0.3}"
START="${2:-0}"
TAG="${3:-run1}"
DURATION="${4:-}"

WS=~/catkin_ws
RES="$VNAV_ROOT/results"
BAG="$VNAV_ROOT/data/mars/mars_hkairport03.bag"
CFG="$WS/src/VINS-Mono/config/mars/mars_nadir.yaml"

vnav_setup_ros

[[ -f "$BAG" ]] || { echo "Нет $BAG. Соберите его: tools/mars_to_bag.py"; exit 1; }
[[ -f "$CFG" ]] || {
    echo "Нет конфига $CFG."
    echo "Скопируйте: mkdir -p $(dirname "$CFG") && cp $VNAV_ROOT/config/mars_nadir.yaml $CFG"
    echo "И launch:   cp $VNAV_ROOT/config/mars_nadir.launch $WS/src/VINS-Mono/vins_estimator/launch/"
    echo "В конфиге замените USER на $USER в путях output_path и pose_graph_save_path."
    exit 1
}

mkdir -p "$RES/timing" "$RES/vins_out"
sed -i "s|^output_path:.*|output_path: \"$RES/vins_out/\"|" "$CFG"
rm -f "$RES/vins_out/vins_result_no_loop.csv"

export VINS_TIMING_LOG="$RES/timing/vins_front_mars_r${RATE}_${TAG}.csv"
export VINS_BACKEND_LOG="$RES/timing/vins_back_mars_r${RATE}_${TAG}.csv"
rm -f "$VINS_TIMING_LOG" "$VINS_BACKEND_LOG"

if [[ -n "$DURATION" ]]; then
    echo "=== VINS-Mono / MARS-LVIG: скорость $RATE, старт $START с, длительность $DURATION с, метка $TAG ==="
else
    echo "=== VINS-Mono / MARS-LVIG: скорость $RATE, старт $START с, метка $TAG ==="
fi

roscore >/dev/null 2>&1 &
ROSCORE_PID=$!
sleep 3

roslaunch vins_estimator mars_nadir.launch > "$RES/vins_mars_${TAG}.log" 2>&1 &
LAUNCH_PID=$!
sleep 8

PLAY_ARGS=(-r "$RATE" -s "$START" --clock)
if [[ -n "$DURATION" ]]; then
    PLAY_ARGS+=(--duration "$DURATION")
fi
rosbag play "$BAG" "${PLAY_ARGS[@]}"
sleep 5

kill $LAUNCH_PID 2>/dev/null || true
sleep 2
kill $ROSCORE_PID 2>/dev/null || true
pkill -f vins_estimator 2>/dev/null || true
pkill -f feature_tracker 2>/dev/null || true

OUT="$RES/vins_out/vins_result_no_loop.csv"
if [[ -f "$OUT" && -s "$OUT" ]]; then
    cp "$OUT" "$RES/vins_mars_${TAG}.csv"
    echo "готово: $(wc -l < "$OUT") строк траектории"
else
    echo
    echo "Траектория пуста — инициализация не сошлась."
    echo "Это ожидаемая ситуация для надирной камеры над плоской сценой."
    echo "Смотрите $RES/vins_mars_${TAG}.log и раздел 6.3 отчёта."
    echo
    echo "Типовые сообщения и что делать:"
    echo "  'Not enough features or parallax'  -> другой --start, поднять max_cnt"
    echo "  'IMU excitation not enough'        -> стартовать с участка разгона"
    echo "  'misalign visual structure'        -> проверить extrinsicRotation и шкалу времени"
    grep -E "Not enough|excitation|misalign|failure" "$RES/vins_mars_${TAG}.log" | tail -20 || true
    exit 1
fi
