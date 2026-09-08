#!/usr/bin/env bash
# Оценка VINS-Mono и DSO на MARS-LVIG / HKairport03 (RTK эталон).
#
#   ./evaluate_mars.sh [метка]
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/project.sh"

TAG="${1:-run1}"
RES="$VNAV_ROOT/results"
DATA="$VNAV_ROOT/data"
EVAL="$RES/eval_mars_${TAG}"
export PATH="$PATH:$HOME/.local/bin"

mkdir -p "$EVAL"
cd "$EVAL"

GT="$DATA/mars/mars_hkairport03_gt.tum"
[[ -f "$GT" ]] || { echo "Нет эталона $GT. Запустите mars_to_bag.py"; exit 1; }

VINS_CSV="$RES/vins_mars_${TAG}.csv"
DSO_TUM="$RES/dso_mars_${TAG}.tum"

echo "=== Оценка MARS-LVIG / $TAG ==="

if [[ -f "$VINS_CSV" ]]; then
    echo
    echo "--- VINS-Mono ---"
    VINS_TUM="${VINS_CSV%.csv}.tum"
    rm -f "$VINS_TUM"
    sed 's/,$//' "$VINS_CSV" > vins_euroc.csv
    evo_traj euroc vins_euroc.csv --save_as_tum >/dev/null
    [[ -f vins_euroc.tum ]] && mv vins_euroc.tum "$VINS_TUM"
    [[ -f "$VINS_TUM" ]] || { echo "evo не смог прочитать $VINS_CSV"; exit 1; }
    cp "$VINS_TUM" vins.tum

    echo "ATE (SE3):"
    evo_ape tum "$GT" vins.tum -a --t_max_diff 0.05 \
        --save_results vins_ape.zip --plot_mode xy --save_plot vins_ape.pdf | tee vins_ape.txt

    echo "RPE на сегментах 10 м:"
    evo_rpe tum "$GT" vins.tum -a -r trans_part --delta 10 --delta_unit m --t_max_diff 0.05 \
        --save_results vins_rpe.zip | tee vins_rpe.txt
else
    echo "Файла $VINS_CSV нет, пропускаю VINS-Mono."
fi

if [[ -f "$DSO_TUM" && -s "$DSO_TUM" ]]; then
    echo
    echo "--- DSO ---"
    awk '{$1=$1; print}' "$DSO_TUM" > dso.tum

    echo "ATE (Sim3):"
    evo_ape tum "$GT" dso.tum -as --t_max_diff 0.05 \
        --save_results dso_ape.zip --plot_mode xy --save_plot dso_ape.pdf | tee dso_ape.txt

    echo "RPE на сегментах 10 м:"
    evo_rpe tum "$GT" dso.tum -as -r trans_part --delta 10 --delta_unit m --t_max_diff 0.05 \
        --save_results dso_rpe.zip | tee dso_rpe.txt
else
    echo "Файла $DSO_TUM нет или он пуст — DSO не оценивается."
fi

if [[ -f vins.tum && -f dso.tum ]]; then
    echo
    echo "--- совмещённый график ---"
    evo_traj tum vins.tum dso.tum --ref="$GT" -a --plot_mode xy \
        --save_plot trajectories.pdf >/dev/null
elif [[ -f vins.tum ]]; then
    evo_traj tum vins.tum --ref="$GT" -a --plot_mode xy \
        --save_plot trajectories.pdf >/dev/null
fi

echo
echo "=== Результаты в $EVAL ==="
ls -1 "$EVAL"
