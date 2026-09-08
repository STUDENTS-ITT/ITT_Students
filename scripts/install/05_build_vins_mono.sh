#!/usr/bin/env bash
# Клонирование, адаптация под OpenCV 4 и сборка VINS-Mono.
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/../project.sh"

WS=~/catkin_ws
PKG="$WS/src/VINS-Mono"

echo "=== 05: VINS-Mono ==="

# shellcheck disable=SC1091
vnav_source /opt/ros/noetic/setup.bash

mkdir -p "$WS/src"
if [[ ! -d "$PKG" ]]; then
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git "$PKG"
fi

echo "--- применяю замены OpenCV 3 -> OpenCV 4 ---"
# VINS-Mono написан под OpenCV 3. В Ubuntu 20.04 стоит OpenCV 4.2, где
# перечисленные константы переименованы, а заголовок opencv/cv.h удалён.
# Замены идемпотентны: повторный запуск ничего не испортит.
mapfile -t FILES < <(find "$PKG" \
    \( -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.hpp' \) -type f)

for f in "${FILES[@]}"; do
    sed -i \
        -e 's|#include <opencv/cv\.h>|#include <opencv2/opencv.hpp>|g' \
        -e 's|#include <opencv/highgui\.h>|#include <opencv2/highgui.hpp>|g' \
        -e 's|#include "opencv/cv\.h"|#include <opencv2/opencv.hpp>|g' \
        -e 's/\bCV_LOAD_IMAGE_GRAYSCALE\b/cv::IMREAD_GRAYSCALE/g' \
        -e 's/\bCV_LOAD_IMAGE_COLOR\b/cv::IMREAD_COLOR/g' \
        -e 's/\bCV_LOAD_IMAGE_UNCHANGED\b/cv::IMREAD_UNCHANGED/g' \
        -e 's/\bCV_GRAY2RGB\b/cv::COLOR_GRAY2RGB/g' \
        -e 's/\bCV_GRAY2BGR\b/cv::COLOR_GRAY2BGR/g' \
        -e 's/\bCV_RGB2GRAY\b/cv::COLOR_RGB2GRAY/g' \
        -e 's/\bCV_BGR2GRAY\b/cv::COLOR_BGR2GRAY/g' \
        -e 's/\bCV_FM_RANSAC\b/cv::FM_RANSAC/g' \
        -e 's/\bCV_RANSAC\b/cv::RANSAC/g' \
        -e 's/\bCV_AA\b/cv::LINE_AA/g' \
        -e 's/\bCV_TERMCRIT_ITER\b/cv::TermCriteria::COUNT/g' \
        -e 's/\bCV_TERMCRIT_EPS\b/cv::TermCriteria::EPS/g' \
        -e 's/\bCV_MINMAX\b/cv::NORM_MINMAX/g' \
        -e 's/\bCV_THRESH_BINARY\b/cv::THRESH_BINARY/g' \
        -e 's/\bCV_ADAPTIVE_THRESH_MEAN_C\b/cv::ADAPTIVE_THRESH_MEAN_C/g' \
        -e 's/\bCV_CALIB_CB_ADAPTIVE_THRESH\b/cv::CALIB_CB_ADAPTIVE_THRESH/g' \
        -e 's/\bCV_CALIB_CB_NORMALIZE_IMAGE\b/cv::CALIB_CB_NORMALIZE_IMAGE/g' \
        -e 's/\bCV_CALIB_CB_FILTER_QUADS\b/cv::CALIB_CB_FILTER_QUADS/g' \
        -e 's/\bCV_CALIB_CB_FAST_CHECK\b/cv::CALIB_CB_FAST_CHECK/g' \
        -e 's/\bCV_SHAPE_CROSS\b/cv::MORPH_CROSS/g' \
        -e 's/\bCV_StsBadArg\b/cv::Error::StsBadArg/g' \
        "$f"
done
echo "    обработано файлов: ${#FILES[@]}"

echo "--- сборка ---"
cd "$WS"
export CPLUS_INCLUDE_PATH="$LOCAL_PREFIX/include:${CPLUS_INCLUDE_PATH:-}"
catkin_make -j"$(nproc)" -DCMAKE_BUILD_TYPE=Release \
    -DCeres_DIR="$LOCAL_PREFIX/lib/cmake/Ceres" \
    -DCMAKE_PREFIX_PATH="$LOCAL_PREFIX"

echo "=== 05: готово ==="
echo "Проверка:"
# shellcheck disable=SC1091
vnav_source "$WS/devel/setup.bash"
ls "$WS/devel/lib/vins_estimator/" "$WS/devel/lib/feature_tracker/" 2>/dev/null || true
