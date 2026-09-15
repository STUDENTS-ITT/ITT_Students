#!/usr/bin/env python3
"""Быстрая оценка VINS на временном окне (без полного прогона).

    python3 tools/compare_segment.py \\
        --gt data/mars/mars_hkairport03_gt.tum \\
        --start 50 --duration 90 \\
        results/vins_mars_run2_calib.csv \\
        results/vins_mars_run6_bucketing.csv
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from fuse_vins_baro import fuse_vins_baro, load_baro_height  # noqa: E402
from fuse_vins_rtk import compute_ate, read_tum, write_tum  # noqa: E402


BAG_T0 = 1671607365.0  # mars_hkairport03.bag start (Unix)


def read_vins_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip().rstrip(",")
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0])
                if t > 1e15:
                    t *= 1e-9
                rows.append((t, float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                continue
    if not rows:
        raise SystemExit(f"Пустой CSV: {path}")
    arr = np.array(rows)
    return arr[:, 0], arr[:, 1:4]


def slice_traj(t: np.ndarray, p: np.ndarray, t_min: float, t_max: float):
    m = (t >= t_min) & (t <= t_max)
    return t[m], p[m]


def csv_to_tum(path: Path, out: Path) -> None:
    t, p = read_vins_csv(path)
    write_tum(out, t, p)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("vins_csv", type=Path, nargs="+")
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--start", type=float, default=50.0, help="сек от начала bag")
    ap.add_argument("--duration", type=float, default=90.0, help="длина окна, сек")
    ap.add_argument("--fuse", action="store_true", help="сшивка reboot + баро Z перед ATE (без RTK)")
    ap.add_argument("--baro", type=Path, default=None)
    args = ap.parse_args()

    t_min = BAG_T0 + args.start
    t_max = t_min + args.duration
    t_gt, p_gt, _ = read_tum(args.gt)
    m_gt = (t_gt >= t_min) & (t_gt <= t_max)
    t_gt_s, p_gt_s = t_gt[m_gt], p_gt[m_gt]

    print(f"Окно: bag [{args.start:.0f}, {args.start + args.duration:.0f}] с  "
          f"({len(t_gt_s)} GT точек)\n")
    print(f"{'run':<28} {'poses':>6} {'raw RMSE':>10} {'baro RMSE':>12} {'reboot~':>8}")
    print("-" * 70)

    for csv in args.vins_csv:
        tag = csv.stem.replace("vins_mars_", "")
        t_v, p_v = read_vins_csv(csv)
        t_v, p_v = slice_traj(t_v, p_v, t_min, t_max)
        if len(t_v) < 5:
            print(f"{tag:<28} {len(t_v):>6} {'—':>10} {'—':>12} {'—':>8}")
            continue

        ate_raw = compute_ate(t_v, p_v, t_gt_s, p_gt_s)
        fused_rmse = float("nan")
        if args.fuse:
            baro_path = args.baro
            if baro_path is None:
                default_baro = ROOT / "data/mars/aux/dji_osdk_ros_height_above_takeoff.csv"
                baro_path = default_baro if default_baro.exists() else None
            baro_z = load_baro_height(baro_path, t_v) if baro_path else None
            p_f, _stats = fuse_vins_baro(t_v, p_v, baro_z)
            ate_f = compute_ate(t_v, p_f, t_gt_s, p_gt_s)
            fused_rmse = ate_f["rmse"]

        # грубая оценка reboot по скачкам >50 м/с
        dt = np.diff(t_v)
        spd = np.linalg.norm(np.diff(p_v, axis=0), axis=1) / np.maximum(dt, 1e-3)
        reboots = int((spd > 50).sum())

        print(f"{tag:<28} {len(t_v):>6} {ate_raw['rmse']:>9.1f}m {fused_rmse:>11.1f}m {reboots:>8}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
