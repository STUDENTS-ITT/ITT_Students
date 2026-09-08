#!/usr/bin/env bash
# ROS Noetic + рабочее пространство catkin.
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/../project.sh"

echo "=== 02: ROS Noetic ==="

if [[ -d /opt/ros/noetic ]]; then
    echo "ROS уже установлен, пропускаю установку пакетов."
else
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
        > /etc/apt/sources.list.d/ros-latest.list'

    # Ключ репозитория. Основной источник, при недоступности — зеркало.
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | sudo apt-key add - \
        || curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' \
           | sudo apt-key add -

    sudo apt update
    sudo apt install -y ros-noetic-desktop-full
    sudo apt install -y \
        python3-rosdep python3-rosinstall python3-rosinstall-generator \
        python3-wstool python3-catkin-tools \
        ros-noetic-cv-bridge ros-noetic-image-transport \
        ros-noetic-tf ros-noetic-message-filters \
        ros-noetic-rviz

    sudo rosdep init 2>/dev/null || true
    rosdep update || true
fi

if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi
# shellcheck disable=SC1091
vnav_source /opt/ros/noetic/setup.bash

# Рабочее пространство
if [[ ! -d ~/catkin_ws/src ]]; then
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
fi
if ! grep -q "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi

echo "=== 02: готово ==="
rosversion -d
