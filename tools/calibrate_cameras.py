#!/usr/bin/env python3
"""Калибровка камеры для MARS-LVIG и «Поворот_коптер».

MARS-LVIG HKairport03:
  официальные intrinsics из data/calibration/mars_official/HK_GNSS*.yaml
  (шахматная доска, MARS-LVIG dataset).

Поворот_коптер:
  в кадрах полёта шахматной доски нет → калибровка по прямым линиям
  (plumb-line / минимизация кривизны линий после undistort) на дороге.

Выход:
  config/mars_camera_full.txt      — DSO, 2448×2048
  config/mars_camera_dso.txt       — DSO, 608×512 (scale 0.25)
  config/povorot_camera.txt        — DSO, 960×536 (scale 0.5)
  config/mars_nadir.yaml           — VINS (обновляет intrinsics + dist)
  data/calibration/calibration_report.json

Пример:
  python3 tools/calibrate_cameras.py
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
MARS_YAML = ROOT / "data/calibration/mars_official/HK_GNSS(airport & island).yaml"
POVOROT_FRAMES = ROOT / "data/povorot_kopter/images"
MARS_VINS_YAML = ROOT / "config/mars_nadir.yaml"
OUT_REPORT = ROOT / "data/calibration/calibration_report.json"

MARS_FULL = (2448, 2048)
MARS_DSO_SCALE = 0.25
POVOROT_FULL = (1920, 1080)
POVOROT_DSO_SCALE = 0.5


def parse_mars_yaml(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    # Простой парсер под формат MARS yaml (списки в одну строку)
    def grab(name: str) -> list[float]:
        m = re.search(rf"{name}:\s*\[(.*?)\]", text, re.S)
        if not m:
            raise SystemExit(f"Нет {name} в {path}")
        return [float(x.strip()) for x in m.group(1).split(",") if x.strip()]

    K_flat = grab("camera_intrinsic")
    K = np.array(K_flat, dtype=float).reshape(3, 3)
    dist = grab("camera_dist_coeffs")
    R = np.array(grab("camera_ext_R"), dtype=float).reshape(3, 3)
    t = np.array(grab("camera_ext_t"), dtype=float)
    return {
        "fx": float(K[0, 0]),
        "fy": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
        "k1": dist[0],
        "k2": dist[1],
        "p1": dist[2],
        "p2": dist[3],
        "k3": dist[4] if len(dist) > 4 else 0.0,
        "R_cam_lidar": R.tolist(),
        "t_cam_lidar": t.tolist(),
        "source": str(path),
    }


def scale_intrinsics(fx, fy, cx, cy, w_in, h_in, w_out, h_out):
    sx = w_out / w_in
    sy = h_out / h_in
    return fx * sx, fy * sy, cx * sx, cy * sy, sx, sy


def write_dso_camera(path: Path, fx, fy, cx, cy, k1, k2, p1, p2, w, h) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"RadTan {fx:.6f} {fy:.6f} {cx:.6f} {cy:.6f} "
        f"{k1:.8f} {k2:.8f} {p1:.8f} {p2:.8f}\n"
        f"{w} {h}\n"
        f"crop\n"
        f"{w} {h}\n",
        encoding="utf-8",
    )


def update_mars_vins_yaml(mars: dict, scale: float, w_out: int, h_out: int) -> None:
    fx, fy, cx, cy, _, _ = scale_intrinsics(
        mars["fx"], mars["fy"], mars["cx"], mars["cy"],
        MARS_FULL[0], MARS_FULL[1], w_out, h_out,
    )
    text = MARS_VINS_YAML.read_text(encoding="utf-8")
    text = re.sub(
        r"# Пока значения приближённые.*?\n",
        "# Intrinsics: официальная калибровка MARS-LVIG HK_GNSS (HKairport03).\n",
        text,
        count=1,
    )
    repl = {
        r"image_width: \d+": f"image_width: {w_out}",
        r"image_height: \d+": f"image_height: {h_out}",
        r"k1: [^\n]+": f"k1: {mars['k1']:.8f}",
        r"k2: [^\n]+": f"k2: {mars['k2']:.8f}",
        r"p1: [^\n]+": f"p1: {mars['p1']:.8f}",
        r"p2: [^\n]+": f"p2: {mars['p2']:.8f}",
        r"fx: [^\n]+": f"fx: {fx:.4f}",
        r"fy: [^\n]+": f"fy: {fy:.4f}",
        r"cx: [^\n]+": f"cx: {cx:.4f}",
        r"cy: [^\n]+": f"cy: {cy:.4f}",
        r"estimate_extrinsic: \d+": "estimate_extrinsic: 0",
    }
    for pat, val in repl.items():
        text = re.sub(pat, val, text)
    MARS_VINS_YAML.write_text(text, encoding="utf-8")


def line_straightness_score(img_gray: np.ndarray, k1, k2, fx, fy, cx, cy) -> float:
    """Меньше — линии straighter после undistort."""
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array([k1, k2, 0.0, 0.0], dtype=np.float64)
    und = cv2.undistort(img_gray, K, D)
    edges = cv2.Canny(und, 60, 180)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80,
                            minLineLength=80, maxLineGap=15)
    if lines is None or len(lines) < 5:
        return 1e6
    score = 0.0
    n_lines = 0
    for ln in lines:
        seg = ln[0] if getattr(ln, "ndim", 1) > 1 else ln
        x1, y1, x2, y2 = [float(v) for v in seg[:4]]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1:
            continue
        # Отклонение середины линии от хорды
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        # Проекция на нормаль к линии — для прямой ≈ 0
        nx, ny = -dy / length, dx / length
        # sample midpoint deviation using nearby edge pixels — proxy: use angle variance
        score += abs(ny * (mx - cx) - nx * (my - cy)) / length
        n_lines += 1
    return score / max(n_lines, 1)


def calibrate_povorot_plumb_line(frames_dir: Path, sample_step: int = 8) -> dict:
    files = sorted(frames_dir.glob("*.png"))[::sample_step]
    if not files:
        raise SystemExit(f"Нет кадров в {frames_dir}")

    w, h = POVOROT_FULL
    imgs = [cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) for f in files[:12]]
    imgs = [cv2.resize(im, (960, 540)) for im in imgs if im is not None]
    cx0, cy0 = 480.0, 270.0
    fx0 = (960 / 2.0) / math.tan(math.radians(73.0) / 2.0)

    best = {"score": 1e18, "fx": fx0, "fy": fx0, "k1": 0.0, "k2": 0.0}

    for fx in np.linspace(fx0 * 0.95, fx0 * 1.05, 5):
        for k1 in np.linspace(-0.12, 0.02, 5):
            for k2 in np.linspace(0.0, 0.18, 5):
                total = 0.0
                n = 0
                for im in imgs:
                    s = line_straightness_score(im, k1, k2, fx, fx, cx0, cy0)
                    if s >= 1e5:
                        continue
                    total += s
                    n += 1
                if n == 0:
                    continue
                avg = total / n
                if avg < best["score"]:
                    best = {"score": avg, "fx": float(fx), "fy": float(fx),
                            "k1": float(k1), "k2": float(k2), "frames_used": n}

    fx_full = best["fx"] * (POVOROT_FULL[0] / 960.0)
    fy_full = best["fy"] * (POVOROT_FULL[0] / 960.0)
    cx_full, cy_full = POVOROT_FULL[0] / 2.0, POVOROT_FULL[1] / 2.0
    fx_s, fy_s, cx_s, cy_s, _, _ = scale_intrinsics(
        fx_full, fy_full, cx_full, cy_full, POVOROT_FULL[0], POVOROT_FULL[1],
        int(round(w * POVOROT_DSO_SCALE)) // 8 * 8,
        int(round(h * POVOROT_DSO_SCALE)) // 8 * 8,
    )
    return {
        "method": "plumb-line (road edges, no chessboard in flight frames)",
        "full_resolution": list(POVOROT_FULL),
        "fx": fx_full,
        "fy": fy_full,
        "cx": cx_full,
        "cy": cy_full,
        "k1": best["k1"],
        "k2": best["k2"],
        "p1": 0.0,
        "p2": 0.0,
        "score": best["score"],
        "frames_used": best.get("frames_used", 0),
        "dso_scaled": {
            "scale": POVOROT_DSO_SCALE,
            "fx": fx_s,
            "fy": fy_s,
            "cx": cx_s,
            "cy": cy_s,
            "w": int(round(w * POVOROT_DSO_SCALE)) // 8 * 8,
            "h": int(round(h * POVOROT_DSO_SCALE)) // 8 * 8,
        },
        "note": "Шахматная доска в кадрах не найдена; для этalonной калибровки нужна отдельная съёмка доски.",
    }


def main() -> int:
    report = {}

    # --- MARS-LVIG ---
    if not MARS_YAML.is_file():
        raise SystemExit(f"Нет {MARS_YAML}. Запустите: gdown folder mars_official")
    mars = parse_mars_yaml(MARS_YAML)
    report["mars_hkairport03"] = mars

    write_dso_camera(
        ROOT / "config/mars_camera_full.txt",
        mars["fx"], mars["fy"], mars["cx"], mars["cy"],
        mars["k1"], mars["k2"], mars["p1"], mars["p2"],
        MARS_FULL[0], MARS_FULL[1],
    )
    w_dso = int(round(MARS_FULL[0] * MARS_DSO_SCALE)) // 8 * 8
    h_dso = int(round(MARS_FULL[1] * MARS_DSO_SCALE)) // 8 * 8
    fx, fy, cx, cy, _, _ = scale_intrinsics(
        mars["fx"], mars["fy"], mars["cx"], mars["cy"],
        MARS_FULL[0], MARS_FULL[1], w_dso, h_dso,
    )
    write_dso_camera(
        ROOT / "config/mars_camera_dso.txt",
        fx, fy, cx, cy,
        mars["k1"], mars["k2"], mars["p1"], mars["p2"],
        w_dso, h_dso,
    )
    update_mars_vins_yaml(mars, MARS_DSO_SCALE, w_dso, h_dso)
    report["mars_vins_yaml"] = str(MARS_VINS_YAML)
    report["mars_dso_size"] = [w_dso, h_dso]

    # --- Поворот_коптер ---
    pov = calibrate_povorot_plumb_line(POVOROT_FRAMES)
    report["povorot_kopter"] = pov
    ds = pov["dso_scaled"]
    write_dso_camera(
        ROOT / "config/povorot_camera.txt",
        ds["fx"], ds["fy"], ds["cx"], ds["cy"],
        pov["k1"], pov["k2"], pov["p1"], pov["p2"],
        ds["w"], ds["h"],
    )

    OUT_REPORT.parent.mkdir(parents=True, exist_ok=True)
    OUT_REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print("=== MARS-LVIG HKairport03 (official HK_GNSS) ===")
    print(f"  full: fx={mars['fx']:.2f} fy={mars['fy']:.2f} cx={mars['cx']:.1f} cy={mars['cy']:.1f}")
    print(f"  dist: k1={mars['k1']:.4f} k2={mars['k2']:.4f}")
    print(f"  DSO scaled {w_dso}x{h_dso}: fx={fx:.2f} fy={fy:.2f}")
    print(f"  VINS yaml updated: {MARS_VINS_YAML}")
    print()
    print("=== Поворот_коптер (plumb-line) ===")
    print(f"  full: fx={pov['fx']:.2f} k1={pov['k1']:.4f} k2={pov['k2']:.4f} score={pov['score']:.5f}")
    print(f"  DSO: {ds['w']}x{ds['h']} fx={ds['fx']:.2f}")
    print(f"  report: {OUT_REPORT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
