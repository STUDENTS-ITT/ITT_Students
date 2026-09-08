#!/usr/bin/env bash
# evo — стандартный инструмент оценки траекторий (ATE, RPE, выравнивание).
set -euo pipefail

echo "=== 07: evo и Python-зависимости ==="

python3 -m pip install --user --upgrade pip
python3 -m pip install --user \
    "evo>=1.20" \
    numpy scipy matplotlib pandas \
    opencv-python-headless \
    tqdm

if ! grep -q 'export PATH=$PATH:$HOME/.local/bin' ~/.bashrc; then
    echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc
fi
export PATH="$PATH:$HOME/.local/bin"

echo "=== 07: готово ==="
evo_ape --help >/dev/null && echo "evo работает"
