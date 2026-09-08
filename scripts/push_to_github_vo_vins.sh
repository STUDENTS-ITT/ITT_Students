#!/usr/bin/env bash
set -euo pipefail
SRC="/home/vasiliy/Downloads/cursor/Projects/fire-dron"
WORK="/tmp/vo-vins-repo"
REPO="https://github.com/STUDENTS-ITT/ITT_Students.git"

rm -rf "$WORK"
git clone "$REPO" "$WORK"
cd "$WORK"
git checkout -b VO-VINS
find . -mindepth 1 -maxdepth 1 ! -name .git -exec rm -rf {} +

rsync -a --exclude='.git' \
  --exclude='data/mars/camera/images' \
  --exclude='data/mars/dso/images' \
  --exclude='data/mars/mars_hkairport03.bag' \
  --exclude='results/vins_out' \
  --exclude='results/timing' \
  --exclude='results/*.log' \
  --exclude='results/combined_run8e_fused/mars_viz/mars_hk_flight.avi' \
  --exclude='results/combined_run8e_fused/assets/video_posters/_pdf_*' \
  --exclude='__pycache__' \
  --exclude='.venv' \
  "$SRC/" "$WORK/"

cp "$SRC/README.github.md" "$WORK/README.md"

# .gitignore для ветки VO-VINS
cat > "$WORK/.gitignore" <<'GI'
__pycache__/
*.py[cod]
.venv/
venv/
build/ devel/ .catkin_workspace logs/
.idea/ .vscode/ *.swp *~ .DS_Store

# Слишком большие (скачать отдельно)
data/mars/mars_hkairport03.bag
data/mars/camera/images/
data/mars/dso/images/
results/combined_run8e_fused/mars_viz/mars_hk_flight.avi

# Промежуточные логи
results/*.log
results/vins_out/
results/timing/
GI

git add -A
git status --short | head -40
echo "---"
du -sh "$WORK"
find "$WORK" -type f -size +90M 2>/dev/null | while read f; do ls -lh "$f"; done

git config user.email "vasiliy@users.noreply.github.com" 2>/dev/null || true
git config user.name "Vasiliy" 2>/dev/null || true

git commit -m "$(cat <<'EOF'
Add VO-VINS: VINS-Mono + DSO + RTK fusion pipeline

Combined nadir VIO for MARS-LVIG and Поворот_коптер datasets.
Includes PDF report, demo videos (MP4/AVI), configs, fusion tools,
dispatcher, and full documentation for reproduction.
EOF
)"

git push -u origin VO-VINS
echo "DONE: https://github.com/STUDENTS-ITT/ITT_Students/tree/VO-VINS"
