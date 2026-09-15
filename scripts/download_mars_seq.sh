#!/usr/bin/env bash
# Скачать последовательности MARS-LVIG (официальные rosbag).
#
#   ./scripts/download_mars_seq.sh              # HKisland01 + AMvalley01
#   ./scripts/download_mars_seq.sh HKisland01
#
# Источник: https://mars.hku.hk/dataset.html
# Зеркало:  https://drive.google.com/drive/folders/1aG21le4QZl9LhSLE1O0vuVDQ8YgAwdas
set -euo pipefail
source "$(dirname "$0")/project.sh"

DEST="$VNAV_ROOT/data/mars/raw"
mkdir -p "$DEST"

# Ссылки с mars.hku.hk/dataset.html часто 404. Живые файлы — в папке
# https://drive.google.com/drive/folders/1aG21le4QZl9LhSLE1O0vuVDQ8YgAwdas
FOLDER_ID="1aG21le4QZl9LhSLE1O0vuVDQ8YgAwdas"
declare -A FILES=(
  [HKisland01]="1nhX7hGyjCaoIfqc2b3PhAaHHu2Vv8wOQ"
  [AMvalley01]="1NTecR3tb2-NYZDPH_p94bFy3lYmsQ53b"
  [HKairport03]="1CIyTQiMcyflYCaNJmZySBn7WlXBr8NLp"
)
declare -A HF_MCAP=(
  [HKisland01]="https://huggingface.co/datasets/DapengFeng/MCAP/resolve/main/mars_lvig/HKisland01/HKisland01_0.mcap"
  [AMvalley01]="https://huggingface.co/datasets/DapengFeng/MCAP/resolve/main/mars_lvig/AMvalley01/AMvalley01_0.mcap"
  [HKairport03]="https://huggingface.co/datasets/DapengFeng/MCAP/resolve/main/mars_lvig/HKairport03/HKairport03_0.mcap"
)

if [[ $# -eq 0 ]]; then
    NAMES=(HKisland01 AMvalley01)
else
    NAMES=("$@")
fi

if ! command -v gdown >/dev/null 2>&1; then
    python3 -m pip install --user -q gdown
fi

failed=0
for name in "${NAMES[@]}"; do
    id="${FILES[$name]:-}"
    [[ -n "$id" ]] || { echo "Неизвестная последовательность: $name"; failed=1; continue; }
    out="$DEST/${name}.bag"
    if [[ -f "$out" && -s "$out" ]]; then
        echo "уже есть $out ($(du -h "$out" | cut -f1))"
        continue
    fi
    echo "=== качаю $name → $out ==="
    if gdown --fuzzy "https://drive.google.com/file/d/${id}/view?usp=drive_link" -O "$out"; then
        echo "готово: $(du -h "$out" | cut -f1)"
        continue
    fi
    echo "Drive недоступен (квота/404). Пробую HuggingFace MCAP…"
    rm -f "$out"
    hf="${HF_MCAP[$name]:-}"
    if [[ -n "$hf" ]]; then
        mcap="$DEST/${name}.mcap"
        if wget -c -O "$mcap" "$hf"; then
            echo "готово MCAP: $(du -h "$mcap" | cut -f1)"
            echo "Дальше: rosbags-convert --src $mcap --dst $DEST/${name}_ros2  (или открыть в браузере Drive)"
            continue
        fi
        echo "HuggingFace тоже не скачался: $hf"
    fi
    echo "Откройте папку в браузере:"
    echo "  https://drive.google.com/drive/folders/${FOLDER_ID}"
    echo "  https://mars.hku.hk/dataset.html"
    failed=1
done
if [[ "$failed" -ne 0 ]]; then
    echo
    echo "Не все bag скачались. Поле (HKairport03) уже в репо; вода/лес — вручную с сайта HKU."
    exit 1
fi

echo
echo "Конвертация в VINS-bag:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  python3 $VNAV_ROOT/tools/official_bag_to_vins.py --bag $DEST/HKisland01.bag --out $VNAV_ROOT/data/mars/mars_hkisland01.bag"
echo "  python3 $VNAV_ROOT/tools/official_bag_to_vins.py --bag $DEST/AMvalley01.bag --out $VNAV_ROOT/data/mars/mars_amvalley01.bag"
