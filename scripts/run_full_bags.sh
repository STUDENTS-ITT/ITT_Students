#!/usr/bin/env bash
# Три сцены с начала bag до конца. Пишет WALL <сцена> <сек>.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p "$ROOT/results/timing"
LOG="$ROOT/results/timing/full_bag_wall.log"
{
  echo "start $(date -Is)"
  for scene in field water forest; do
    echo "=== $scene $(date -Is) ==="
    t0=$(date +%s)
    ./scripts/run_vins_ahrs_scene.sh "$scene" 1.0 0
    t1=$(date +%s)
    echo "WALL $scene $((t1 - t0))"
  done
  echo "done $(date -Is)"
} 2>&1 | tee "$LOG"
