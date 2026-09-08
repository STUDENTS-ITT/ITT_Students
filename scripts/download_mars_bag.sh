#!/usr/bin/env bash
# Скачать rosbag MARS HKairport03 (не входит в git — лимит 100 МБ GitHub).
#
# Официальный датасет: https://mars.hku.hk/dataset.html
# Нужен файл HKairport03 (или аналог с /cam0/image_raw + /imu0).
#
# Положите bag сюда:
#   data/mars/mars_hkairport03.bag
set -euo pipefail
DEST="$(dirname "$0")/../data/mars/mars_hkairport03.bag"
echo "Ожидаемый путь: $DEST"
echo
echo "1. Зарегистрируйтесь на https://mars.hku.hk/dataset.html"
echo "2. Скачайте HKairport03 rosbag"
echo "3. Переименуйте/скопируйте в: $DEST"
echo
if [[ -f "$DEST" ]]; then
  echo "OK: bag найден ($(du -h "$DEST" | cut -f1))"
else
  echo "Файл пока отсутствует."
  exit 1
fi
