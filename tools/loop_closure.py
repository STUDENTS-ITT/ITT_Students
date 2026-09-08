#!/usr/bin/env python3
"""Лёгкая подсказка loop closure: детекция возврата к ранее пройденной зоне.

Не полный pose graph — только флаг + небольшая XY-коррекция при замыкании петли.
Точка интеграции: fuse_vins_rtk.py (--loop-closure) или пост-обработка TUM.

    python3 tools/loop_closure.py --tum results/eval_mars_run8e_fused/vins_fused.tum \\
        --out results/eval_mars_run8e_fused/vins_lc.tum
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np


def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    t_list, p_list = [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 4:
                continue
            try:
                t_list.append(float(parts[0]))
                p_list.append([float(parts[i]) for i in range(1, 4)])
            except ValueError:
                continue
    if not t_list:
        raise SystemExit(f"Пустой TUM: {path}")
    return np.array(t_list), np.array(p_list)


def write_tum(path: Path, t: np.ndarray, p: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# loop-closure hint applied (minimal XY correction)\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.9f} {p[i,0]:.6f} {p[i,1]:.6f} {p[i,2]:.6f}\n")


def detect_loop_closures(
    t: np.ndarray,
    p: np.ndarray,
    radius_m: float = 15.0,
    min_time_gap: float = 120.0,
    min_path_m: float = 200.0,
) -> list[dict]:
    """Найти пары (ранний, поздний) индексов при возврате в ту же зону."""
    closures: list[dict] = []
    n = len(p)
    cum_path = np.zeros(n)
    for i in range(1, n):
        cum_path[i] = cum_path[i - 1] + float(np.linalg.norm(p[i, :2] - p[i - 1, :2]))

    for j in range(n - 1, 0, -1):
        for i in range(0, j - 1):
            if t[j] - t[i] < min_time_gap:
                continue
            if cum_path[j] - cum_path[i] < min_path_m:
                continue
            dist = float(np.linalg.norm(p[j, :2] - p[i, :2]))
            if dist <= radius_m:
                closures.append({
                    "i": i, "j": j,
                    "t_i": float(t[i]), "t_j": float(t[j]),
                    "dist_m": dist,
                    "path_m": float(cum_path[j] - cum_path[i]),
                })
                break  # один closure на позднюю точку
    return closures


def apply_loop_correction(
    p: np.ndarray,
    closures: list[dict],
    alpha: float = 0.15,
) -> tuple[np.ndarray, dict]:
    """Небольшая XY-коррекция после точки замыкания (не глобальный pose graph)."""
    out = p.copy()
    stats = {"closures": len(closures), "corrections": []}
    for c in closures:
        i, j = c["i"], c["j"]
        delta = out[i, :2] - out[j, :2]
        n_apply = j - i
        if n_apply < 2:
            continue
        # Плавное смещение от j к концу траектории
        for k in range(j, len(out)):
            w = alpha * (k - j + 1) / max(len(out) - j, 1)
            out[k, :2] += w * delta
        stats["corrections"].append({
            "t_j": c["t_j"],
            "shift_xy_m": [float(delta[0]), float(delta[1])],
            "alpha": alpha,
        })
    return out, stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tum", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--radius", type=float, default=15.0, help="радиус замыкания, м")
    ap.add_argument("--min-gap", type=float, default=60.0, help="мин. Δt между точками, с")
    ap.add_argument("--alpha", type=float, default=0.15, help="сила коррекции 0..1")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    t, p = read_tum(args.tum)
    closures = detect_loop_closures(t, p, radius_m=args.radius, min_time_gap=args.min_gap)
    print(f"Loop closures detected: {len(closures)}")
    for c in closures:
        print(f"  t={c['t_i']:.1f}→{c['t_j']:.1f} s, dist={c['dist_m']:.1f} m, path={c['path_m']:.0f} m")

    if args.dry_run:
        return 0

    p_out, stats = apply_loop_correction(p, closures, alpha=args.alpha)
    write_tum(args.out, t, p_out)
    print(f"Saved: {args.out} ({stats['closures']} hints, {len(stats['corrections'])} corrections)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
