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
BAG="${MARS_BAG:-$VNAV_ROOT/data/mars/mars_hkairport03.bag}"
SRC_YAML="${MARS_VINS_YAML:-$VNAV_ROOT/config/mars_nadir.yaml}"
LAUNCH="${MARS_LAUNCH:-mars_nadir.launch}"
CFG="$WS/src/VINS-Mono/config/mars/$(basename "$SRC_YAML")"

vnav_setup_ros

[[ -f "$BAG" ]] || { echo "Нет $BAG. Соберите его: tools/mars_to_bag.py или official_bag_to_vins.py"; exit 1; }
mkdir -p "$(dirname "$CFG")"
cp -f "$VNAV_ROOT/config/mars_nadir.launch" "$WS/src/VINS-Mono/vins_estimator/launch/"
cp -f "$VNAV_ROOT/config/mars_livox.launch" "$WS/src/VINS-Mono/vins_estimator/launch/" 2>/dev/null || true
cp -f "$VNAV_ROOT/config/mars_livox_valley.launch" "$WS/src/VINS-Mono/vins_estimator/launch/" 2>/dev/null || true
cp -f "$VNAV_ROOT/config/mars_livox_airport.launch" "$WS/src/VINS-Mono/vins_estimator/launch/" 2>/dev/null || true
for y in mars_livox mars_livox_valley mars_livox_airport; do
    [[ -f "$VNAV_ROOT/config/$y.yaml" ]] && cp -f "$VNAV_ROOT/config/$y.yaml" "$WS/src/VINS-Mono/config/mars/"
done
# SRC_YAML копируется последним: свип может подсунуть свой вариант с тем же именем.
cp -f "$SRC_YAML" "$CFG"
sed -i "s|/home/USER|$HOME|g" "$CFG"
sed -i "s|/home/USER|$HOME|g" "$WS/src/VINS-Mono/config/mars/"*.yaml 2>/dev/null || true
if grep -q '^nadir_planar_mode: 1' "$CFG" 2>/dev/null; then
    IMU="${MARS_DJI_IMU:-$VNAV_ROOT/data/mars/aux/dji_osdk_ros_imu.csv}"
    if [[ -f "$IMU" ]]; then
        esc="${IMU//\\/\\\\}"
        esc="${esc//|/\\|}"
        sed -i "s|^ahrs_sidecar:.*|ahrs_sidecar: \"${esc}\"|" "$CFG"
    fi
fi

mkdir -p "$RES/timing" "$RES/vins_out"
sed -i "s|^output_path:.*|output_path: \"$RES/vins_out/\"|" "$CFG"
rm -f "$RES/vins_out/vins_result_no_loop.csv"

export VINS_TIMING_LOG="$RES/timing/vins_front_mars_r${RATE}_${TAG}.csv"
export VINS_BACKEND_LOG="$RES/timing/vins_back_mars_r${RATE}_${TAG}.csv"
rm -f "$VINS_TIMING_LOG" "$VINS_BACKEND_LOG"

echo "=== VINS-Mono / $(basename "$BAG") yaml=$(basename "$SRC_YAML") ==="
if [[ -n "$DURATION" ]]; then
    echo "скорость $RATE, старт $START с, длительность $DURATION с, метка $TAG"
else
    echo "скорость $RATE, старт $START с, метка $TAG"
fi

ROS_PORT="${ROS_PORT:-11311}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
roscore -p "$ROS_PORT" >/dev/null 2>&1 &
ROSCORE_PID=$!
sleep 3

roslaunch vins_estimator "$LAUNCH" > "$RES/vins_mars_${TAG}.log" 2>&1 &
LAUNCH_PID=$!
sleep 8

PLAY_ARGS=(-r "$RATE" -s "$START" --clock)
if [[ -n "$DURATION" ]]; then
    PLAY_ARGS+=(--duration "$DURATION")
fi
rosbag play "$BAG" "${PLAY_ARGS[@]}"
# Оценщик часто отстаёт от ленты: не убивать, пока CSV растёт.
OUT_WAIT="$RES/vins_out/vins_result_no_loop.csv"
prev_n=-1
stable=0
for _ in $(seq 1 72); do
    n=0
    [[ -f "$OUT_WAIT" ]] && n=$(wc -l < "$OUT_WAIT")
    if [[ "$n" -eq "$prev_n" && "$n" -gt 10 ]]; then
        stable=$((stable + 1))
    else
        stable=0
    fi
    prev_n=$n
    if [[ "$stable" -ge 6 ]]; then
        echo "оценщик догнал ($n строк)"
        break
    fi
    sleep 5
done

kill $LAUNCH_PID 2>/dev/null || true
sleep 2
# Только дети этого запуска: чужой roscore на другом порту не трогаем.
if [[ -n "${LAUNCH_PID:-}" ]]; then
    pkill -P "$LAUNCH_PID" 2>/dev/null || true
fi
kill $ROSCORE_PID 2>/dev/null || true

OUT="$RES/vins_out/vins_result_no_loop.csv"
LOOP_OUT="$RES/vins_out/vins_result_loop.csv"
if [[ -f "$OUT" && -s "$OUT" ]]; then
    cp "$OUT" "$RES/vins_mars_${TAG}.csv"
    echo "готово: $(wc -l < "$OUT") строк no_loop"
    if [[ -f "$LOOP_OUT" && -s "$LOOP_OUT" ]]; then
        cp "$LOOP_OUT" "$RES/vins_mars_${TAG}_loop.csv"
        echo "loop: $(wc -l < "$LOOP_OUT") кейфреймов pose_graph"
    else
        echo "loop: пусто (pose_graph не накопил KF)"
    fi
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
