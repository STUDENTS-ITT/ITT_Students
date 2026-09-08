#!/usr/bin/env bash
# Сборка DSO (Direct Sparse Odometry).
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/../project.sh"

DSO_ROOT=~/dso

echo "=== 06: DSO ==="

if [[ ! -d "$DSO_ROOT" ]]; then
    git clone https://github.com/JakobEngel/dso.git "$DSO_ROOT"
fi

cd "$DSO_ROOT"

# OpenCV 4: переименованные константы imread/imdecode.
if [[ -f src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp ]]; then
    sed -i \
        -e 's/\bCV_LOAD_IMAGE_GRAYSCALE\b/cv::IMREAD_GRAYSCALE/g' \
        -e 's/\bCV_LOAD_IMAGE_COLOR\b/cv::IMREAD_COLOR/g' \
        -e 's/\bCV_LOAD_IMAGE_UNCHANGED\b/cv::IMREAD_UNCHANGED/g' \
        src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
fi

# GCC 9 строже к включениям, чем компилятор времён выхода DSO.
for f in src/util/settings.h src/util/NumType.h; do
    [[ -f "$f" ]] || continue
    grep -q "#include <limits>" "$f" || sed -i '1a #include <limits>' "$f"
    grep -q "#include <cstdint>" "$f" || sed -i '1a #include <cstdint>' "$f"
done

# Release обязателен: в Debug DSO медленнее на порядок, и замеры времени
# потеряют всякий смысл.
export CMAKE_BUILD_TYPE=Release

rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$LOCAL_PREFIX"
make -j"$(nproc)"

echo "=== 06: готово ==="
if [[ -x "$DSO_ROOT/build/bin/dso_dataset" ]]; then
    echo "Исполняемый файл: $DSO_ROOT/build/bin/dso_dataset"
else
    echo "ОШИБКА: dso_dataset не собрался, смотрите вывод выше."
    exit 1
fi
