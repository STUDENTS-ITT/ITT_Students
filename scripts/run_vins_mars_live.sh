#!/usr/bin/env bash
# Live-визуализация VINS-Mono на MARS (как в демо VINS-Mono + RViz).
#
# Поднимает: roscore, VINS (mars_nadir), RTK path publisher, RViz.
# Bag проигрывается отдельно (или флаг --play).
#
#   ./run_vins_mars_live.sh [rate] [start_sec] [--play] [duration_sec] [--no-rtk] [--no-echo-odom]
#
#   --no-rtk         не публиковать RTK в RViz (это значение по умолчанию)
#   --rtk            красный /mars/rtk_path только для глаз, не коррекция
#   --no-echo-odom   не открывать терминал rostopic echo /vins_estimator/odometry
#
# Окно odom справа в демо VINS-Mono — это сырой выход estimators, без RTK fusion.
#   ./run_vins_mars_live.sh 0.3 50 --play        # полный bag с 50 с
#   ./run_vins_mars_live.sh 1.0 50 --play 90     # 90 с для быстрой проверки
#
# В RViz: tracked image, raw_image, loop_match_image, VIO path, pose_graph, rtk_path.
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/project.sh"

RATE="0.3"
START="50"
PLAY=0
DURATION=""
RTK=0
ECHO_ODOM=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --play) PLAY=1; shift ;;
        --no-rtk) RTK=0; shift ;;
        --rtk) RTK=1; shift ;;
        --no-echo-odom) ECHO_ODOM=0; shift ;;
        -h|--help)
            sed -n '2,20p' "$0"
            exit 0
            ;;
        *)
            if [[ "$RATE" == "0.3" && "$1" =~ ^[0-9.]+$ ]]; then RATE="$1"; shift
            elif [[ "$START" == "50" && "$1" =~ ^[0-9.]+$ ]]; then START="$1"; shift
            elif [[ -z "$DURATION" && "$1" =~ ^[0-9.]+$ ]]; then DURATION="$1"; shift
            else echo "Неизвестный аргумент: $1"; exit 1
            fi
            ;;
    esac
done

WS=~/catkin_ws
BAG="$VNAV_ROOT/data/mars/mars_hkairport03.bag"
GT="$VNAV_ROOT/data/mars/mars_hkairport03_gt.tum"
CFG="$WS/src/VINS-Mono/config/mars/mars_nadir.yaml"
RVIZ_CFG="$VNAV_ROOT/config/mars_rviz.rviz"
RES="$VNAV_ROOT/results"

vnav_setup_ros

[[ -f "$BAG" ]] || { echo "Нет bag: $BAG"; exit 1; }
[[ -f "$GT" ]] || { echo "Нет эталона $GT (нужен только для RViz --rtk)"; [[ "$RTK" -eq 1 ]] && exit 1; }
[[ -f "$RVIZ_CFG" ]] || { echo "Нет $RVIZ_CFG"; exit 1; }
[[ -f "$CFG" ]] || {
    mkdir -p "$(dirname "$CFG")"
    cp "$VNAV_ROOT/config/mars_nadir.yaml" "$CFG"
    cp "$VNAV_ROOT/config/mars_nadir.launch" "$WS/src/VINS-Mono/vins_estimator/launch/"
    sed -i "s|/home/USER|$HOME|g" "$CFG"
}

# show_track: 1 для /feature_tracker/feature_img
grep -q '^show_track: 1' "$VNAV_ROOT/config/mars_nadir.yaml" || \
    echo "Внимание: show_track не 1 в config/mars_nadir.yaml — tracked image может быть пустым."
cp "$VNAV_ROOT/config/mars_nadir.yaml" "$CFG"
sed -i "s|/home/USER|$HOME|g" "$CFG"
sed -i "s|^output_path:.*|output_path: \"$RES/vins_out/\"|" "$CFG"
mkdir -p "$RES/vins_out"

export VNAV_ROOT
export ROS_PACKAGE_PATH="$WS/src/VINS-Mono/vins_estimator/launch:${ROS_PACKAGE_PATH:-}"

cleanup() {
    echo
    echo "Остановка..."
    jobs -p | xargs -r kill 2>/dev/null || true
    pkill -f "rostopic echo /vins_estimator/odometry" 2>/dev/null || true
    pkill -f "vins_estimator|feature_tracker|pose_graph|rtk_path_publisher|rviz" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_odom_echo() {
    local setup="source /opt/ros/noetic/setup.bash; source $HOME/catkin_ws/devel/setup.bash --extend; export PATH=/opt/ros/noetic/bin:\$PATH"
    local cmd="$setup; echo '=== /vins_estimator/odometry (чистый VINS, без RTK) ==='; rostopic echo /vins_estimator/odometry"
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="VINS odometry (raw)" -- bash -c "$cmd; exec bash" &
    elif command -v x-terminal-emulator >/dev/null 2>&1; then
        x-terminal-emulator -T "VINS odometry" -e bash -c "$cmd; exec bash" &
    else
        bash -c "$cmd" >"$RES/vins_odom_live.log" 2>&1 &
        echo "Odom → $RES/vins_odom_live.log  (tail -f для просмотра)"
    fi
}

echo "=== MARS live RViz: rate=$RATE start=${START}s play=$PLAY rtk=$RTK echo_odom=$ECHO_ODOM ==="
echo "RViz config: $RVIZ_CFG"
echo

if ! rostopic list &>/dev/null; then
    echo "Запуск roscore..."
    roscore >/dev/null 2>&1 &
    for _ in $(seq 1 15); do
        rostopic list &>/dev/null && break
        sleep 1
    done
    rostopic list &>/dev/null || { echo "Ошибка: roscore не поднялся"; exit 1; }
fi

rosparam set use_sim_time true 2>/dev/null || true
roslaunch vins_estimator mars_nadir.launch &
sleep 6

if [[ "$RTK" -eq 1 ]]; then
    python3 "$VNAV_ROOT/tools/rtk_path_publisher.py" --gt "$GT" &
else
    echo "RTK publisher выключен (--no-rtk) — в RViz только VINS."
fi
sleep 1

if [[ "$ECHO_ODOM" -eq 1 ]]; then
    start_odom_echo
    sleep 1
fi

rosrun rviz rviz -d "$RVIZ_CFG" &
sleep 2

if [[ "$PLAY" -eq 1 ]]; then
    PLAY_ARGS=(--clock -r "$RATE" -s "$START")
    [[ -n "$DURATION" ]] && PLAY_ARGS+=(--duration "$DURATION")
    echo "rosbag play ${PLAY_ARGS[*]} $BAG"
    rosbag play "$BAG" "${PLAY_ARGS[@]}"
    echo "Bag закончен. RViz остаётся открытым — Ctrl+C для выхода."
else
    cat <<EOF

Терминал 2 — проиграть bag:

  source $VNAV_ROOT/scripts/project.sh && vnav_setup_ros
  rosbag play --clock -r $RATE -s $START $BAG

$( [[ -n "$DURATION" ]] && echo "  # или с ограничением: ... --duration $DURATION" )

Ctrl+C здесь — остановить VINS и RViz.
EOF
fi

wait
