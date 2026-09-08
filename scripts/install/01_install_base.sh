#!/usr/bin/env bash
# Системные библиотеки, нужные и VINS-Mono, и DSO.
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/../project.sh"

echo "=== 01: базовые пакеты ==="

if [[ "$(lsb_release -rs)" != "20.04" ]]; then
    echo "ВНИМАНИЕ: обнаружена Ubuntu $(lsb_release -rs), а не 20.04."
    echo "ROS Noetic существует только под 20.04. Продолжать? [y/N]"
    read -r ans
    [[ "$ans" == "y" ]] || exit 1
fi

need_apt=0
for pkg in build-essential cmake libeigen3-dev libopencv-dev python3-pip; do
    dpkg -s "$pkg" >/dev/null 2>&1 || need_apt=1
done

if (( need_apt )); then
    sudo apt update
    sudo apt install -y \
        build-essential cmake git wget curl unzip pkg-config \
        libeigen3-dev \
        libgoogle-glog-dev libgflags-dev \
        libatlas-base-dev libsuitesparse-dev \
        libboost-all-dev \
        libglew-dev libgl1-mesa-dev libegl1-mesa-dev \
        libzip-dev \
        libopencv-dev python3-opencv \
        python3-pip python3-dev python3-numpy \
        ffmpeg \
        linux-tools-common linux-tools-generic
else
    echo "Базовые пакеты уже установлены, пропускаю apt."
fi

# CMake-модуль DSO ищет zipconf.h в /usr/include, а Ubuntu кладёт его
# в архитектурный подкаталог. Без этой ссылки поддержка zip молча отключается.
if [[ -f /usr/include/x86_64-linux-gnu/zipconf.h && ! -f /usr/include/zipconf.h ]]; then
    echo "  копирую zipconf.h в /usr/include"
    sudo cp /usr/include/x86_64-linux-gnu/zipconf.h /usr/include/ 2>/dev/null \
        || echo "  (пропуск: нужен sudo для zipconf.h — DSO может собраться без zip)"
fi

mkdir -p "$VNAV_ROOT"/{data,results,results/timing,src}

echo "=== 01: готово ==="
echo "Eigen: $(pkg-config --modversion eigen3 2>/dev/null || echo '3.3.x (заголовки)')"
echo "OpenCV: $(pkg-config --modversion opencv4 2>/dev/null || echo 'см. /usr/include/opencv4')"
