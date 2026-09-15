#!/usr/bin/env python3
"""Сшивка VINS после reboot + коррекция Z по барометру. Без RTK.

XY: приращения VINS; на скачке origin сегмент стартует с последней позы.
Z: height_above_takeoff, выровненный к Z первой позы VINS.

RTK/GT не участвует в коррекции. Если передан --gt, печатается ATE
(оценка, не inject).

    python3 tools/fuse_vins_baro.py \\
        --vins results/eval_mars_run8e_fused/vins.tum \\
        --baro data/mars/aux/dji_osdk_ros_height_above_takeoff.csv \\
        --out results/eval_mars_run8e_fused/vins_baro.tum

--vins принимает TUM или CSV VINS (запятые, timestamp в нс или сек).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

# Переиспользуем I/O и детектор скачков из архивного RTK-скрипта.
from fuse_vins_rtk import detect_jumps, read_tum, compute_ate  # noqa: E402
from agl_quality import classify_baro  # noqa: E402


def load_vins_poses(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """TUM или CSV VINS: запятые, опциональный заголовок, timestamp нс→сек."""
    t, p, q = read_tum(path)
    if len(t) and float(t[0]) > 1e15:
        t = t * 1e-9
    return t, p, q


def load_baro_height(path: Path, t_vins: np.ndarray) -> np.ndarray | None:
    if not path.exists():
        return None
    t_list, z_list = [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        for row in reader:
            try:
                if "bag_time_ns" in row and row["bag_time_ns"]:
                    t_s = float(row["bag_time_ns"]) * 1e-9
                elif "stamp" in row and row["stamp"]:
                    t_s = float(row["stamp"])
                elif "timestamp" in row and row["timestamp"]:
                    t_s = float(row["timestamp"])
                    if t_s > 1e15:
                        t_s *= 1e-9
                else:
                    continue
                z = float(row.get("data") or row.get("height") or row.get("z") or "")
            except (KeyError, ValueError, TypeError):
                continue
            t_list.append(t_s)
            z_list.append(z)
    if len(t_list) < 5:
        print(f"  ВНИМАНИЕ: баро {path} слишком короткий ({len(t_list)}), Z остаётся VINS",
              file=sys.stderr)
        return None
    _ = fieldnames
    z = np.interp(t_vins, np.array(t_list), np.array(z_list))
    kind = classify_baro(z)
    if kind != "USABLE":
        peak = float(np.nanmax(np.abs(z)))
        print(f"  ВНИМАНИЕ: баро {path} не AGL ({kind}, max |h| = {peak:.2f} м) — "
              f"Z остаётся из VINS", file=sys.stderr)
        return None
    return z


def stitch_vins_deltas(t: np.ndarray, p: np.ndarray, max_speed: float = 50.0) -> tuple[np.ndarray, list[int]]:
    """Непрерывная траектория: сегменты стыкуются по last pose, не по origin."""
    jumps = detect_jumps(t, p, max_speed)
    n = len(p)
    out = np.zeros_like(p)
    if n == 0:
        return out, jumps
    out[0] = p[0]
    jump_set = set(jumps)
    for i in range(1, n):
        if i in jump_set:
            out[i] = out[i - 1]
            continue
        out[i] = out[i - 1] + (p[i] - p[i - 1])
    return out, jumps


def fuse_vins_baro(
    t_vins: np.ndarray,
    p_vins: np.ndarray,
    baro_z: np.ndarray | None,
    max_speed: float = 50.0,
) -> tuple[np.ndarray, dict]:
    out, jumps = stitch_vins_deltas(t_vins, p_vins, max_speed)
    baro_used = False
    if baro_z is not None and len(baro_z) == len(out):
        z0 = float(out[0, 2])
        out[:, 2] = (baro_z - baro_z[0]) + z0
        baro_used = True
    stats = {
        "jumps": len(jumps),
        "poses": len(out),
        "baro_used": baro_used,
        "path_m": float(np.sum(np.linalg.norm(np.diff(out, axis=0), axis=1))) if len(out) > 1 else 0.0,
    }
    return out, stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vins", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--baro", type=Path, default=None)
    ap.add_argument("--gt", type=Path, default=None, help="только для печати ATE, не коррекция")
    ap.add_argument("--max-speed", type=float, default=50.0)
    args = ap.parse_args()

    t_v, p_v, q_v = load_vins_poses(args.vins)

    baro_path = args.baro
    if baro_path is None:
        default_baro = Path(__file__).resolve().parents[1] / "data/mars/aux/dji_osdk_ros_height_above_takeoff.csv"
        baro_path = default_baro if default_baro.exists() else None

    baro_z = None
    if baro_path:
        baro_z = load_baro_height(baro_path, t_v)
        if baro_z is None:
            print(f"  ВНИМАНИЕ: не прочитал баро {baro_path} — Z из VINS", file=sys.stderr)
    else:
        print("  ВНИМАНИЕ: баро не задан — Z из VINS", file=sys.stderr)

    p_out, stats = fuse_vins_baro(t_v, p_v, baro_z, max_speed=args.max_speed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8") as f:
        f.write("# VINS deltas + baro Z, no RTK correction\n")
        for i in range(len(t_v)):
            if q_v is not None:
                f.write(f"{t_v[i]:.9f} {p_out[i,0]:.6f} {p_out[i,1]:.6f} {p_out[i,2]:.6f} "
                        f"{q_v[i,0]:.9f} {q_v[i,1]:.9f} {q_v[i,2]:.9f} {q_v[i,3]:.9f}\n")
            else:
                f.write(f"{t_v[i]:.9f} {p_out[i,0]:.6f} {p_out[i,1]:.6f} {p_out[i,2]:.6f}\n")

    print(f"Saved: {args.out}")
    print(f"  reboots={stats['jumps']}, poses={stats['poses']}, "
          f"baro Z: {stats['baro_used']}, path={stats['path_m']:.1f} m")

    if args.gt and args.gt.exists():
        t_g, p_g, _ = read_tum(args.gt)
        ate_v = compute_ate(t_v, p_v, t_g, p_g)
        ate_f = compute_ate(t_v, p_out, t_g, p_g)
        print(f"  VINS  ATE RMSE: {ate_v['rmse']:.1f} m (n={ate_v['n']})")
        print(f"  Baro  ATE RMSE: {ate_f['rmse']:.1f} m (n={ate_f['n']})  [оценка, без RTK-assist]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
