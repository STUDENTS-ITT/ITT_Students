#!/usr/bin/env python3
"""Прописать в config/ официальные camera–LiDAR MARS-LVIG + T_il Avia.

    python3 tools/apply_mars_official_extrinsic.py
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data/calibration/mars_official"

# IMU(Livox) → LiDAR, производитель Avia / FAST-LIVO2 config/MARS_LVIG.yaml
T_IL = np.array([0.04165, 0.02326, -0.0284])
R_IL = np.eye(3)

# Livox X (ось камеры/лидара вниз) → DJI FLU: X вперёд, Y влево, Z вверх.
R_LIVOX_TO_DJI = np.array([[0.0, 0.0, 1.0],
                           [0.0, 1.0, 0.0],
                           [-1.0, 0.0, 0.0]])

# Выпуск датасета (UAVScenes / MARS yaml). HKairport03 → HK_GNSS.
CALIB = {
    "HK_GNSS": {
        "K": [1444.43, 1444.34, 1179.50, 1044.90],
        "dist": [-0.0560, 0.1180, 0.00122, 0.00064, -0.0627],
        "Rcl": [0.00363212, -0.999819, -0.0213618,
                -0.0111679, 0.0214512, -0.999591,
                0.999879, 0.00375613, -0.0111134],
        "Pcl": [-0.0021928, 0.0470312, -0.0513126],
        # FAST-LIVO2 img_time_offset = -0.1; в VINS знак td обратный.
        # Свип 90 с на HKairport03: +0.1 → ATE 55.6 м, 0 → 109.9 м, -0.1 → 87.2 м.
        "td": 0.1,
        "full_wh": (2448, 2048),
    },
    "HKisland": {
        "K": [1444.43, 1444.34, 1177.8, 1043.6],
        "dist": [-0.053, 0.121, 0.00127, 0.00043, -0.06495],
        "Rcl": [0.00352762, -0.999765, -0.0213775,
                -0.0111803, 0.0213369, -0.99971,
                0.999931, 0.00376561, -0.0111025],
        "Pcl": [-0.0025563, 0.0470454, -0.0513375],
        "td": 0.0,
        "full_wh": (2448, 2048),
    },
    "AMvalley": {
        "K": [1453.88, 1452.85, 1182.53, 1045.82],
        "dist": [-0.052, 0.1168, 0.0015, 0.00013, -0.068564],
        "Rcl": [0.00298068, -0.999735, -0.0231428,
                -0.00504595, 0.023132, -0.99974,
                0.999985, 0.00309701, -0.00497598],
        "Pcl": [-0.0025563, 0.0567484, -0.0512149],
        "td": 0.1,  # тот же знак, что у HK_GNSS; на AMvalley не проверялось
        "full_wh": (2448, 2048),
    },
}

SCALE_WH = (608, 512)


def ric_tic(Rcl: np.ndarray, Pcl: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """p_l = p_imu + T_il (R_il=I); p_c = Rcl p_l + Pcl → p_imu = Rcl^T p_c - T_il - Rcl^T Pcl."""
    ric = Rcl.T
    tic = -T_IL - Rcl.T @ Pcl
    return ric, tic


def fmt_R(R: np.ndarray) -> str:
    a = R.ravel()
    return (f"[ {a[0]:.8f}, {a[1]:.8f}, {a[2]:.8f},\n"
            f"          {a[3]:.8f}, {a[4]:.8f}, {a[5]:.8f},\n"
            f"          {a[6]:.8f}, {a[7]:.8f}, {a[8]:.8f}]")


def fmt_t(t: np.ndarray) -> str:
    return f"[{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]"


def patch_yaml(path: Path, fx, fy, cx, cy, k1, k2, p1, p2, R, t, td) -> None:
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"^   fx: .*$", f"   fx: {fx:.4f}", text, flags=re.M)
    text = re.sub(r"^   fy: .*$", f"   fy: {fy:.4f}", text, flags=re.M)
    text = re.sub(r"^   cx: .*$", f"   cx: {cx:.4f}", text, flags=re.M)
    text = re.sub(r"^   cy: .*$", f"   cy: {cy:.4f}", text, flags=re.M)
    text = re.sub(r"^   k1: .*$", f"   k1: {k1:.8f}", text, flags=re.M)
    text = re.sub(r"^   k2: .*$", f"   k2: {k2:.8f}", text, flags=re.M)
    text = re.sub(r"^   p1: .*$", f"   p1: {p1:.8f}", text, flags=re.M)
    text = re.sub(r"^   p2: .*$", f"   p2: {p2:.8f}", text, flags=re.M)
    text = re.sub(
        r"extrinsicRotation: !!opencv-matrix\n(?:.*\n)*?.*?\]",
        "extrinsicRotation: !!opencv-matrix\n"
        "   rows: 3\n   cols: 3\n   dt: d\n"
        f"   data: {fmt_R(R)}",
        text, count=1,
    )
    text = re.sub(
        r"extrinsicTranslation: !!opencv-matrix\n(?:.*\n)*?.*?\]",
        "extrinsicTranslation: !!opencv-matrix\n"
        "   rows: 3\n   cols: 1\n   dt: d\n"
        f"   data: {fmt_t(t)}",
        text, count=1,
    )
    text = re.sub(r"^td: .*$", f"td: {td}", text, flags=re.M)
    path.write_text(text, encoding="utf-8")
    print(f"updated {path}")


def scale_intr(c: dict) -> tuple:
    fx, fy, cx, cy = c["K"]
    w, h = c["full_wh"]
    wo, ho = SCALE_WH
    sx, sy = wo / w, ho / h
    d = c["dist"]
    return fx * sx, fy * sy, cx * sx, cy * sy, d[0], d[1], d[2], d[3]


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    dump = {}
    for name, c in CALIB.items():
        Rcl = np.array(c["Rcl"], float).reshape(3, 3)
        Pcl = np.array(c["Pcl"], float)
        ric, tic = ric_tic(Rcl, Pcl)
        dump[name] = {
            "Rcl": Rcl.tolist(), "Pcl": Pcl.tolist(),
            "T_il": T_IL.tolist(),
            "R_ic_livox": ric.tolist(), "t_ic_livox": tic.tolist(),
            "R_ic_dji": (R_LIVOX_TO_DJI @ ric).tolist(),
            "t_ic_dji": (R_LIVOX_TO_DJI @ tic).tolist(),
            "td": c["td"], "K_full": c["K"], "dist": c["dist"],
        }
        print(name, "R_ic_livox optical", ric[:, 2], "t", tic)

    (OUT / "vins_from_official.json").write_text(
        json.dumps(dump, indent=2), encoding="utf-8")

    fx, fy, cx, cy, k1, k2, p1, p2 = scale_intr(CALIB["HK_GNSS"])
    ric, tic = ric_tic(
        np.array(CALIB["HK_GNSS"]["Rcl"]).reshape(3, 3),
        np.array(CALIB["HK_GNSS"]["Pcl"]),
    )
    patch_yaml(ROOT / "config/mars_nadir.yaml",
               fx, fy, cx, cy, k1, k2, p1, p2,
               R_LIVOX_TO_DJI @ ric, R_LIVOX_TO_DJI @ tic,
               CALIB["HK_GNSS"]["td"])

    airport = ROOT / "config/mars_livox_airport.yaml"
    if airport.is_file():
        patch_yaml(airport, fx, fy, cx, cy, k1, k2, p1, p2, ric, tic,
                   CALIB["HK_GNSS"]["td"])

    fx, fy, cx, cy, k1, k2, p1, p2 = scale_intr(CALIB["HKisland"])
    ric, tic = ric_tic(
        np.array(CALIB["HKisland"]["Rcl"]).reshape(3, 3),
        np.array(CALIB["HKisland"]["Pcl"]),
    )
    patch_yaml(ROOT / "config/mars_livox.yaml",
               fx, fy, cx, cy, k1, k2, p1, p2, ric, tic, CALIB["HKisland"]["td"])

    fx, fy, cx, cy, k1, k2, p1, p2 = scale_intr(CALIB["AMvalley"])
    ric, tic = ric_tic(
        np.array(CALIB["AMvalley"]["Rcl"]).reshape(3, 3),
        np.array(CALIB["AMvalley"]["Pcl"]),
    )
    patch_yaml(ROOT / "config/mars_livox_valley.yaml",
               fx, fy, cx, cy, k1, k2, p1, p2, ric, tic, CALIB["AMvalley"]["td"])
    print("wrote", OUT / "vins_from_official.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
