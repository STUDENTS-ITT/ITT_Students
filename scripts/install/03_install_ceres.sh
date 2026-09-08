#!/usr/bin/env bash
# Ceres Solver 1.14.0 из исходников.
#
# Версия принципиальна. В Ceres 2.2 удалён класс ceres::LocalParameterization,
# на котором построен vins_estimator/src/factor/pose_local_parameterization.h.
# С Ceres >= 2.2 VINS-Mono не собирается.
set -euo pipefail

CERES_VERSION="${CERES_VERSION:-1.14.0}"
# shellcheck disable=SC1091
source "$(dirname "$0")/../project.sh"
SRC="$VNAV_ROOT/src"

echo "=== 03: Ceres ${CERES_VERSION} ==="

# Пакетная версия конфликтует с собранной из исходников.
if dpkg -l | grep -q libceres-dev; then
    echo "Удаляю libceres-dev из apt, чтобы не было конфликта версий."
    sudo apt remove -y libceres-dev 2>/dev/null \
        || echo "  (пропуск: libceres-dev не удалён — нужен sudo или пакет не установлен)"
fi

if [[ -f "$LOCAL_PREFIX/lib/libceres.a" || -f "$LOCAL_PREFIX/lib/libceres.so" \
      || -f /usr/local/lib/libceres.a || -f /usr/local/lib/libceres.so ]]; then
    echo "Ceres уже установлен. Пропускаю."
    exit 0
fi

mkdir -p "$SRC" && cd "$SRC"

if [[ ! -d ceres-solver ]]; then
    git clone https://ceres-solver.googlesource.com/ceres-solver \
        || git clone https://github.com/ceres-solver/ceres-solver.git
fi

cd ceres-solver
git fetch --tags
git checkout "${CERES_VERSION}"

rm -rf build && mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DCMAKE_CXX_STANDARD=14 \
    -DCMAKE_INSTALL_PREFIX="$LOCAL_PREFIX"

make -j"$(nproc)"
make install
[[ -w /etc/ld.so.conf.d ]] && sudo ldconfig 2>/dev/null || true

echo "=== 03: готово ==="
echo "Если сборка упала — попробуйте Ceres 2.1.0, в нём LocalParameterization ещё есть:"
echo "  CERES_VERSION=2.1.0 ./03_install_ceres.sh"
