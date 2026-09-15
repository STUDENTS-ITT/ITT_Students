#!/usr/bin/env python3
"""Sweep fuse clamp/takeoff variants — без жёстких порогов по времени.

    python3 tools/sweep_fuse_clamp.py
"""
from __future__ import annotations

import csv
import sys
from collections import deque
from dataclasses import dataclass
from itertools import product
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from fuse_vins_ahrs import (  # noqa: E402
    CLAMP_LIMIT,
    GOOD_SPEED_HISTORY,
    GOOD_SPEED_N,
    climb_path_threshold,
    estimate_speed,
    is_clamp_step,
    is_good_raw,
    load_dji_imu,
    load_tum,
    rebuild_xy,
)

VINS = ROOT / "results/eval_mars_scene_field_ahrs/vins_baro.tum"
IMU = ROOT / "data/mars/aux/dji_osdk_ros_imu.csv"
GT = ROOT / "data/mars/mars_hkairport03_livox_gt.tum"
OUT_DIR = ROOT / "results/eval_mars_scene_field_ahrs/sweep_clamp"

CLAMP_MODES = ("speed_dt", "clamp_raw", "rolling_median", "max_speed_clamp", "good_or_15", "blend")
TAKEOFF_MODES = ("zero_clamp", "allow_nonclamp", "allow_half", "allow_full")
ROLLING_N = GOOD_SPEED_HISTORY


def associate(t_a, p_a, t_b, p_b, max_dt=0.05):
    idx = np.searchsorted(t_b, t_a)
    idx = np.clip(idx, 1, len(t_b) - 1)
    pick = np.where(np.abs(t_b[idx] - t_a) < np.abs(t_b[idx - 1] - t_a), idx, idx - 1)
    ok = np.abs(t_b[pick] - t_a) < max_dt
    return p_a[ok], p_b[pick[ok]], t_a[ok]


def umeyama_2d(src: np.ndarray, dst: np.ndarray, with_scale: bool):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s, d = src - mu_s, dst - mu_d
    cov = d.T @ s / len(s)
    u, sv, vt = np.linalg.svd(cov)
    w = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        w[1, 1] = -1
    r = u @ w @ vt
    scale = float(np.trace(np.diag(sv) @ w) / (s**2).sum() * len(s)) if with_scale else 1.0
    t = mu_d - scale * r @ mu_s
    return r, t, scale


def path_len(p: np.ndarray) -> float:
    if len(p) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def bbox_extent(p: np.ndarray) -> tuple[float, float]:
    return float(p[:, 0].max() - p[:, 0].min()), float(p[:, 1].max() - p[:, 1].min())


def substitute_clamp(
    mode: str,
    speed: float,
    dt_i: float,
    last_good: float,
    good_history: deque[float],
) -> float:
    spd_dt = speed * dt_i if dt_i > 1e-4 else last_good
    if mode == "speed_dt":
        return spd_dt if speed > 0.0 else 0.0
    if mode == "clamp_raw":
        return CLAMP_LIMIT
    if mode == "rolling_median":
        if len(good_history) >= 3:
            return float(np.median(np.array(good_history)))
        return CLAMP_LIMIT
    if mode == "max_speed_clamp":
        return max(spd_dt if speed > 0.0 else 0.0, CLAMP_LIMIT)
    if mode == "good_or_15":
        if len(good_history) >= 3:
            return float(np.median(np.array(good_history)))
        return CLAMP_LIMIT
    if mode == "blend":
        med = float(np.median(np.array(good_history))) if len(good_history) >= 3 else CLAMP_LIMIT
        return min(CLAMP_LIMIT, max(spd_dt if speed > 0.0 else 0.0, med * 0.85))
    raise ValueError(mode)


def rebuild_xy_variant(
    p_xy: np.ndarray,
    yaw: np.ndarray,
    dt: np.ndarray,
    clamp_mode: str,
    takeoff_mode: str,
) -> np.ndarray:
    raw = np.linalg.norm(np.diff(p_xy, axis=0), axis=1)
    dti = dt[: len(raw)]
    clamp_flags = np.abs(raw - CLAMP_LIMIT) < 0.05
    clamp_frac = float(np.mean(clamp_flags)) if len(raw) else 0.0
    speed = 0.0

    out = np.zeros_like(p_xy)
    out[0] = p_xy[0]
    start = out[0].copy()
    last_good = 0.0
    good_history: deque[float] = deque(maxlen=ROLLING_N)
    good_speeds: deque[float] = deque(maxlen=ROLLING_N)
    path_len = 0.0
    peak_path = 0.0
    max_extent = 0.0
    returned = False
    best_home = 1e9
    best_home_xy = start.copy()
    landing_xy = start.copy()
    freeze_idx = -1
    home_ring: deque[np.ndarray] = deque(maxlen=40)

    for i in range(len(p_xy) - 1):
        step = float(raw[i])
        peak_path = max(peak_path, path_len)
        climb = path_len < climb_path_threshold(peak_path)

        dist_home = float(np.hypot(out[i, 0] - start[0], out[i, 1] - start[1]))
        max_extent = max(max_extent, dist_home)

        far = max(0.12 * max_extent, 25.0)
        near_r = max(0.06 * max_extent, 10.0)
        depart_r = max(0.02 * max_extent, 4.0)

        if max_extent > far and dist_home < near_r:
            returned = True
            home_ring.append(out[i].copy())
            if dist_home < best_home:
                best_home = dist_home
                best_home_xy = out[i].copy()

        clamp_step = is_clamp_step(step)

        if freeze_idx < 0 and returned and best_home < near_r:
            leaving = dist_home > best_home + depart_r
            if leaving:
                freeze_idx = i + 1
                if len(home_ring) >= 5:
                    landing_xy = np.median(np.array(home_ring), axis=0)
                else:
                    landing_xy = best_home_xy.copy()

        if freeze_idx >= 0:
            out[i + 1] = landing_xy
            continue

        if len(good_speeds) >= GOOD_SPEED_N:
            speed = min(float(np.median(np.array(good_speeds))), 10.0)
        else:
            speed, _ = estimate_speed(good_speeds, clamp_frac)

        if step < 0.15:
            step = 0.0
        elif clamp_step:
            if returned:
                step = 0.0
            elif climb:
                if takeoff_mode == "zero_clamp":
                    step = 0.0
                elif takeoff_mode == "allow_nonclamp":
                    step = substitute_clamp(clamp_mode, speed, float(dt[i]), last_good, good_history)
                elif takeoff_mode == "allow_half":
                    step = 0.5 * substitute_clamp(
                        clamp_mode, speed, float(dt[i]), last_good, good_history
                    )
                elif takeoff_mode == "allow_full":
                    step = substitute_clamp(clamp_mode, speed, float(dt[i]), last_good, good_history)
            elif speed > 0.0:
                step = substitute_clamp(clamp_mode, speed, float(dt[i]), last_good, good_history)
            else:
                step = 0.0
        else:
            last_good = step
            if is_good_raw(step, clamp_step) and not climb:
                good_history.append(step)
                dt_i = float(dt[i])
                if dt_i > 1e-4:
                    good_speeds.append(step / dt_i)

        out[i + 1] = out[i] + step * np.array([np.cos(yaw[i]), np.sin(yaw[i])])
        path_len += step

    return out


@dataclass
class Metrics:
    name: str
    clamp_mode: str
    takeoff_mode: str
    end_dist: float
    path_ratio: float
    bbox_x_ratio: float
    bbox_y_ratio: float
    centroid_dx: float
    centroid_dy: float
    se2_rms: float
    sim2_scale: float

    @property
    def composite(self) -> float:
        end_pen = max(0.0, self.end_dist - 10.0) * 3.0
        return end_pen + abs(self.centroid_dx) * 0.4 + self.se2_rms * 0.3 + abs(self.bbox_x_ratio - 1.0) * 20.0


def eval_variant(xy: np.ndarray, t_v: np.ndarray, gt_path: Path, name: str, clamp_mode: str, takeoff_mode: str) -> Metrics:
    t_g, p_g = load_tum(gt_path)[:, 0], load_tum(gt_path)[:, 1:4]
    pv = np.column_stack([xy[:, 0], xy[:, 1], np.zeros(len(xy))])
    pv_a, pg_a, _ = associate(t_v, pv, t_g, p_g)

    r_e, t_e, _ = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=False)
    xy_se2 = (r_e @ pv_a[:, :2].T).T + t_e
    err_se2 = np.linalg.norm(xy_se2 - pg_a[:, :2], axis=1)
    _, _, scale = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=True)

    bx_v, by_v = bbox_extent(pv_a[:, :2])
    bx_g, by_g = bbox_extent(pg_a[:, :2])
    cent = pg_a[:, :2].mean(0) - pv_a[:, :2].mean(0)

    return Metrics(
        name=name,
        clamp_mode=clamp_mode,
        takeoff_mode=takeoff_mode,
        end_dist=float(np.linalg.norm(pv_a[-1, :2] - pg_a[-1, :2])),
        path_ratio=path_len(pv_a[:, :2]) / max(path_len(pg_a[:, :2]), 1e-6),
        bbox_x_ratio=bx_v / max(bx_g, 1e-6),
        bbox_y_ratio=by_v / max(by_g, 1e-6),
        centroid_dx=float(cent[0]),
        centroid_dy=float(cent[1]),
        se2_rms=float(np.sqrt((err_se2**2).mean())),
        sim2_scale=float(scale),
    )


def main() -> int:
    src = load_tum(VINS)
    t_v = src[:, 0]
    t_i, yaw_i, _ = load_dji_imu(IMU)
    yaw = np.interp(t_v, t_i, yaw_i)
    dt = np.diff(t_v, append=t_v[-1] + 0.1)

    results: list[Metrics] = []
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    t_rel = t_v - t_v[0]
    prod_xy = rebuild_xy(src[:, 1:3], yaw, dt, t_rel=t_rel)
    results.append(
        eval_variant(prod_xy, t_v, GT, "production", "speed_dt", "zero_clamp")
    )

    for clamp_mode, takeoff_mode in product(CLAMP_MODES, TAKEOFF_MODES):
        name = f"{clamp_mode}+{takeoff_mode}"
        xy = rebuild_xy_variant(src[:, 1:3], yaw, dt, clamp_mode, takeoff_mode)
        results.append(eval_variant(xy, t_v, GT, name, clamp_mode, takeoff_mode))

    results.sort(key=lambda m: m.composite)

    csv_path = OUT_DIR / "sweep_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].__dict__.keys()))
        w.writeheader()
        for m in results:
            w.writerow(m.__dict__)

    print(f"Sweep (no time thresholds): {len(results)} variants")
    for rank, m in enumerate(results[:8], 1):
        tag = " WINNER" if rank == 1 else ""
        print(
            f"{rank:2d} {m.name:32s} end={m.end_dist:5.1f} bboxX={m.bbox_x_ratio:.3f} "
            f"dX={m.centroid_dx:+5.0f} SE2={m.se2_rms:.1f}{tag}"
        )
    w = results[0]
    print(f"WINNER: {w.name} end={w.end_dist:.1f} dX={w.centroid_dx:+.0f} SE2={w.se2_rms:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
