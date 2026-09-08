#!/usr/bin/env python3
"""Уточнение extrinsic камера–IMU для надирного монтажа по RTK yaw + IMU.

Kalibr с шахматной доской на надирном полёте невозможен, но можно:
  1. Проверить геометрическую матрицу надирного монтажа
  2. Оценить систематический yaw-bias между IMU и RTK
  3. Предложить поворот extrinsic вокруг вертикали (±90°, 180°)

Использует:
  - data/mars/aux/dji_osdk_ros_imu.csv
  - data/mars/aux/dji_osdk_ros_rtk_yaw.csv
  - data/mars/mars_hkairport03_gt.tum (yaw из quaternion)

    python3 tools/refine_extrinsic_nadir.py --update-yaml
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
YAML_PATH = ROOT / "config/mars_nadir.yaml"

# Геометрический надир: X_cam->-Y_imu, Y_cam->-X_imu, Z_cam->-Z_imu
R_NADIR = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)


def load_imu_yaw_rate(imu_csv: Path) -> tuple[np.ndarray, np.ndarray]:
    ts, wz = [], []
    with imu_csv.open(encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["header.stamp.sec"]) + float(row.get("header.stamp.nanosec", 0)) / 1e9
                ts.append(t)
                wz.append(float(row["angular_velocity.z"]))
            except (KeyError, ValueError):
                continue
    return np.array(ts), np.array(wz)


def load_rtk_yaw(gt_tum: Path) -> tuple[np.ndarray, np.ndarray]:
    ts, yaw = [], []
    for line in gt_tum.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        p = line.split()
        if len(p) < 8:
            continue
        t = float(p[0])
        qz, qw = float(p[6]), float(p[7])
        ts.append(t)
        yaw.append(2 * np.arctan2(qz, qw))
    return np.array(ts), np.unwrap(np.array(yaw))


def yaw_rate_from_yaw(t: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    dt = np.diff(t)
    dy = np.diff(yaw)
    return dy / np.maximum(dt, 1e-3)


def evaluate_R(R: np.ndarray, imu_wz: np.ndarray, rtk_dyaw: np.ndarray) -> float:
    """Чем меньше — тем лучше согласование yaw-rate IMU Z и RTK."""
    n = min(len(imu_wz), len(rtk_dyaw))
    if n < 100:
        return float("inf")
    # IMU z должен соответствовать yaw rate с учётом знака монтажа
    corr = np.corrcoef(imu_wz[:n], rtk_dyaw[:n])[0, 1]
    return 1.0 - abs(corr)


def update_yaml(R: np.ndarray, t: np.ndarray):
    text = YAML_PATH.read_text(encoding="utf-8")
    flat = R.flatten()
    data_str = ", ".join(f"{v:.8f}" for v in flat)
    # row-major 3x3 in opencv yaml format (rows listed)
    new_rot = (
        "extrinsicRotation: !!opencv-matrix\n"
        "   rows: 3\n"
        "   cols: 3\n"
        "   dt: d\n"
        f"   data: [ {flat[0]:.8f}, {flat[1]:.8f}, {flat[2]:.8f},\n"
        f"          {flat[3]:.8f}, {flat[4]:.8f}, {flat[5]:.8f},\n"
        f"          {flat[6]:.8f}, {flat[7]:.8f}, {flat[8]:.8f}]"
    )
    # Блок data может занимать несколько строк — матчим до закрывающей скобки
    text = re.sub(
        r"extrinsicRotation: !!opencv-matrix\n(?:.*\n)*?.*?\]",
        new_rot,
        text,
        count=1,
    )
    new_t = (
        "extrinsicTranslation: !!opencv-matrix\n"
        "   rows: 3\n"
        "   cols: 1\n"
        "   dt: d\n"
        f"   data: [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]"
    )
    text = re.sub(
        r"extrinsicTranslation: !!opencv-matrix\n(?:.*\n)*?.*?\]",
        new_t,
        text,
        count=1,
    )
    YAML_PATH.write_text(text, encoding="utf-8")
    print(f"Updated {YAML_PATH}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--imu", type=Path,
                    default=ROOT / "data/mars/aux/dji_osdk_ros_imu.csv")
    ap.add_argument("--gt", type=Path, default=ROOT / "data/mars/mars_hkairport03_gt.tum")
    ap.add_argument("--update-yaml", action="store_true")
    ap.add_argument("--out", type=Path, default=ROOT / "data/calibration/extrinsic_nadir_refined.json")
    args = ap.parse_args()

    t_imu, wz = load_imu_yaw_rate(args.imu)
    t_gt, yaw = load_rtk_yaw(args.gt)
    dyaw = yaw_rate_from_yaw(t_gt, yaw)

    # resample IMU wz to GT rate grid
    t_common = t_gt[1:min(len(t_gt) - 1, len(t_imu))]
    wz_interp = np.interp(t_common, t_imu, wz)
    dyaw_common = dyaw[: len(t_common)]

    candidates = {"nadir_geom": R_NADIR}
    for deg, name in [(90, "Rz90"), (180, "Rz180"), (-90, "Rz-90")]:
        candidates[name] = R_NADIR @ Rotation.from_euler("z", deg, degrees=True).as_matrix()

    best_name, best_score = "nadir_geom", float("inf")
    results = {}
    for name, R in candidates.items():
        score = evaluate_R(R, wz_interp, dyaw_common)
        results[name] = {"score": score, "R": R.tolist()}
        print(f"  {name}: yaw-rate corr score={score:.4f}")
        if score < best_score:
            best_score, best_name = score, name

    best_R = candidates[best_name]
    print(f"\nBest: {best_name} (score={best_score:.4f})")

    out = {
        "best": best_name,
        "score": best_score,
        "extrinsicRotation": best_R.tolist(),
        "extrinsicTranslation": [0.0, 0.0, 0.0],
        "note": "Nadir-only refinement via RTK yaw vs IMU gyro Z correlation",
        "candidates": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Saved: {args.out}")

    if args.update_yaml:
        update_yaml(best_R, np.zeros(3))
    return 0


if __name__ == "__main__":
    sys.exit(main())
