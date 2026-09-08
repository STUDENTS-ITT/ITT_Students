#!/usr/bin/env bash
# Корень проекта fire-dron (родитель каталога scripts/).
# Переопределение: export VNAV_ROOT=/другой/путь
if [[ -z "${VNAV_ROOT:-}" ]]; then
    VNAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    export VNAV_ROOT
fi

LOCAL_PREFIX="${LOCAL_PREFIX:-$HOME/.local}"
export PATH="$LOCAL_PREFIX/bin:$PATH"
export LD_LIBRARY_PATH="$LOCAL_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="$LOCAL_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export CMAKE_PREFIX_PATH="$LOCAL_PREFIX:${CMAKE_PREFIX_PATH:-}"
export CPLUS_INCLUDE_PATH="$LOCAL_PREFIX/include:${CPLUS_INCLUDE_PATH:-}"

# ROS setup.bash обращается к $ROS_DISTRO до присвоения; с set -u это падает.
vnav_source() {
    local had_u=0
    [[ $- == *u* ]] && had_u=1 && set +u
    # shellcheck disable=SC1090
    source "$@"
    (( had_u )) && set -u
}

# ROS Noetic + catkin_ws. devel/setup.bash без --extend сбрасывает PATH
# и убирает /opt/ros/noetic/bin — тогда rosbag «не найден».
vnav_setup_ros() {
    vnav_source /opt/ros/noetic/setup.bash
    if [[ -f "$HOME/catkin_ws/devel/setup.bash" ]]; then
        vnav_source "$HOME/catkin_ws/devel/setup.bash" --extend
    fi
    # hooks catkin не всегда прописывают ROS_PACKAGE_PATH — добавляем VINS-Mono.
    local vins_src="$HOME/catkin_ws/src/VINS-Mono"
    if [[ -d "$vins_src/vins_estimator" ]]; then
        export ROS_PACKAGE_PATH="${vins_src}/vins_estimator:${vins_src}/feature_tracker:${vins_src}/camera_model:${vins_src}/pose_graph:${ROS_PACKAGE_PATH:-}"
    fi
    export LD_LIBRARY_PATH="$HOME/catkin_ws/devel/lib:${LOCAL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export PATH="/opt/ros/noetic/bin:${PATH}"
}
