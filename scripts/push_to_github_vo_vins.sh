#!/usr/bin/env bash
# Синхронизация fire-dron → ветка VO-VINS репозитория ITT_Students.
# Не трогает git config. Большие bag/mcap/видео в git не кладём.
set -euo pipefail
SRC="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${VO_VINS_WORK:-/tmp/vo-vins-repo}"
REPO="${VO_VINS_REPO:-https://github.com/STUDENTS-ITT/ITT_Students.git}"

rm -rf "$WORK"
git clone --branch VO-VINS --single-branch "$REPO" "$WORK"
cd "$WORK"
find . -mindepth 1 -maxdepth 1 ! -name .git -exec rm -rf {} +

rsync -a \
  --exclude='.git' \
  --exclude='data/mars/raw/' \
  --exclude='data/mars/*.bag' \
  --exclude='data/mars/*_dji_imu.csv' \
  --exclude='data/mars/camera/images/' \
  --exclude='data/mars/dso/images/' \
  --exclude='data/povorot_kopter/images/' \
  --exclude='results/vins_out/' \
  --exclude='results/timing/' \
  --exclude='results/*.log' \
  --exclude='results/eval_vo_*' \
  --exclude='results/videos/' \
  --exclude='results/eval_vo_*' \
  --exclude='results/eval_mars_scene_field_lawn_*' \
  --exclude='results/eval_mars_scene_field_*_vo' \
  --exclude='results/vins_mars_scene_field_lawn_*' \
  --exclude='results/vins_mars_scene_field_planar*' \
  --exclude='results/combined_run8e_fused/mars_viz/*.avi' \
  --exclude='__pycache__' \
  --exclude='.venv' \
  --exclude='*.webm' \
  --exclude='*.mcap' \
  "$SRC/" "$WORK/"

cp "$SRC/README.github.md" "$WORK/README.md"

cat > "$WORK/.gitignore" <<'GI'
__pycache__/
*.py[cod]
.venv/
venv/
build/ devel/ .catkin_workspace logs/
.idea/ .vscode/ *.swp *~ .DS_Store
*.bag
*.mcap
data/mars/raw/
data/mars/*_dji_imu.csv
data/mars/camera/images/
data/mars/dso/images/
data/povorot_kopter/images/
results/*.log
results/vins_out/
results/timing/
results/eval_vo_*/
results/videos/*.mp4
results/videos/*.avi
GI

git add -A
echo "--- status ---"
git status --short | head -60 || true
echo "--- size ---"
du -sh "$WORK"
find "$WORK" -type f -size +90M 2>/dev/null | while read -r f; do ls -lh "$f"; done || true

export GIT_AUTHOR_NAME="${GIT_AUTHOR_NAME:-VO-VINS}"
export GIT_AUTHOR_EMAIL="${GIT_AUTHOR_EMAIL:-vo-vins@students-itt.local}"
export GIT_COMMITTER_NAME="$GIT_AUTHOR_NAME"
export GIT_COMMITTER_EMAIL="$GIT_AUTHOR_EMAIL"

# git 2.25 не понимает --trailer от обёртки Cursor — только /usr/bin/git -F
MSG=/tmp/vo-vins-commit-msg.txt
cat > "$MSG" <<'EOF'
VINS: full field/water/forest runs, interim PDF report, planar field mode.

- generate_vins_report.py: timing table + traj/error for 3 scenes
- production: VINS + baro Z + AHRS yaw (--keep-xy on field planar)
- RTK eval-only; removed local logs, old lawn/planar experiment artifacts
EOF
/usr/bin/git commit -F "$MSG"

echo "Коммит готов. Push:"
echo "  cd $WORK && git push origin VO-VINS"
if [[ -n "${GITHUB_TOKEN:-}" ]]; then
  git push "https://x-access-token:${GITHUB_TOKEN}@github.com/STUDENTS-ITT/ITT_Students.git" VO-VINS
else
  git push origin VO-VINS
fi
