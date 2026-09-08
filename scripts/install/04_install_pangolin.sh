#!/usr/bin/env bash
# Pangolin v0.6 — визуализация для DSO.
#
# Версия принципиальна. В Pangolin 0.7+ произошёл рефакторинг (в частности
# components/pango_core/include/signals/signals.hpp), после которого DSO
# не собирается. Тег v0.6 — последний совместимый.
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/../project.sh"
SRC="$VNAV_ROOT/src"

echo "=== 04: Pangolin v0.6 ==="

if [[ -f "$LOCAL_PREFIX/lib/libpangolin.so" || -f /usr/local/lib/libpangolin.so ]]; then
    echo "Pangolin уже установлен. Пропускаю."
    exit 0
fi

mkdir -p "$SRC" && cd "$SRC"

if [[ ! -d Pangolin ]]; then
    git clone https://github.com/stevenlovegrove/Pangolin.git
fi

cd Pangolin
git fetch --tags
git checkout v0.6

# Pangolin v0.6 объявляет C++11, но использует конструкции C++14.
sed -i 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 14)/' CMakeLists.txt || true
grep -q "CMAKE_CXX_STANDARD" CMakeLists.txt \
    || sed -i '1a set(CMAKE_CXX_STANDARD 14)' CMakeLists.txt

# Отсутствующий include в v0.6 на GCC 9.
grep -q "#include <limits>" include/pangolin/gl/colour.h \
    || sed -i '/#pragma once/a #include <limits>' include/pangolin/gl/colour.h

rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX="$LOCAL_PREFIX"
make -j"$(nproc)"
make install
[[ -w /etc/ld.so.conf.d ]] && sudo ldconfig 2>/dev/null || true

echo "=== 04: готово ==="
