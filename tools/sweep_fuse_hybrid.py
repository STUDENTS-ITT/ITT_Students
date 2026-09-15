#!/usr/bin/env python3
"""Sweep HYBRID clamp modes + late-freeze variants for fuse_vins_ahrs.

Score: end_m + 0.5*se2_median + 50*abs(path_ratio - 0.94)

    python3 tools/sweep_fuse_hybrid.py
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
    SHADOW_CLAMP_FRAC,
    SHADOW_CLAMP_WINDOW,
    SHADOW_DYAW,
    SHADOW_WZ,
    VINS_YAW_MAX_STEP,
    YAW_MIS_CLIMB,
    climb_path_threshold,
    estimate_speed,
    is_clamp_step,
    is_good_raw,
    load_dji_imu,
    load_tum,
    rebuild_xy,
    shadow_proxy,
    vins_step_bearing,
    yaw_mismatch,
)

VINS = ROOT / "results/eval_mars_scene_field_ahrs/vins_baro.tum"
IMU = ROOT / "data/mars/aux/dji_osdk_ros_imu.csv"
GT = ROOT / "data/mars/mars_hkairport03_livox_gt.tum"
OUT_DIR = ROOT / "results/eval_mars_scene_field_ahrs/sweep_hybrid"

# clamp substitution during cruise (non-climb, non-returned)
CLAMP_MODES = (
    "speed_dt",
    "clamp_raw",
    "blend",
    "hybrid_speed_takeoff_clamp_half",
    "conditional_frac045",
)

TAKEOFF_MODES = ("zero_clamp", "allow_half", "allow_full")
LATE_FREEZE_MODES = ("near_r", "half_extent", "quarter_extent")


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
    clamp_frac: float,
    climb: bool,
    takeoff_mode: str,
) -> float:
    spd_dt = speed * dt_i if dt_i > 1e-4 else last_good

    def _base(mode_inner: str) -> float:
        if mode_inner == "speed_dt":
            return spd_dt if speed > 0.0 else 0.0
        if mode_inner == "clamp_raw":
            return CLAMP_LIMIT
        if mode_inner == "blend":
            med = float(np.median(np.array(good_history))) if len(good_history) >= 3 else CLAMP_LIMIT
            return min(CLAMP_LIMIT, max(spd_dt if speed > 0.0 else 0.0, med * 0.85))
        raise ValueError(mode_inner)

    if mode == "hybrid_speed_takeoff_clamp_half":
        if climb:
            if takeoff_mode == "zero_clamp":
                return 0.0
            if takeoff_mode == "allow_half":
                return 0.5 * CLAMP_LIMIT
            return CLAMP_LIMIT
        return _base("speed_dt")

    if mode == "conditional_frac045":
        return _base("speed_dt" if clamp_frac > 0.45 else "clamp_raw")

    return _base(mode)


def late_freeze_ok(mode: str, late_best: float, near_r: float, max_extent: float) -> bool:
    if mode == "near_r":
        return late_best < near_r
    if mode == "half_extent":
        return late_best < 0.55 * max_extent
    if mode == "quarter_extent":
        return late_best < 0.25 * max_extent
    raise ValueError(mode)


def rebuild_xy_variant(
    p_xy: np.ndarray,
    yaw: np.ndarray,
    dt: np.ndarray,
    wz: np.ndarray,
    clamp_mode: str,
    takeoff_mode: str,
    late_freeze_mode: str,
) -> np.ndarray:
    raw = np.linalg.norm(np.diff(p_xy, axis=0), axis=1)
    clamp_mask = np.array([is_clamp_step(float(s)) for s in raw])
    clamp_frac = float(np.mean(clamp_mask)) if len(raw) else 0.0

    out = np.zeros_like(p_xy)
    out[0] = p_xy[0]
    start = out[0].copy()
    last_good = 0.0
    path_len_acc = 0.0
    peak_path = 0.0
    max_extent = 0.0
    returned = False
    best_home = 1e9
    best_home_xy = start.copy()
    landing_xy = start.copy()
    freeze_idx = -1
    home_ring: deque[np.ndarray] = deque(maxlen=40)
    good_speeds: deque[float] = deque(maxlen=GOOD_SPEED_HISTORY)
    good_history: deque[float] = deque(maxlen=GOOD_SPEED_HISTORY)
    clamp_hist: deque[bool] = deque(maxlen=SHADOW_CLAMP_WINDOW)
    prev_yaw = float(yaw[0])
    raw_path_total = float(raw.sum())
    late_best = 1e9
    late_xy = start.copy()

    speed, _ = estimate_speed(good_speeds, clamp_frac)

    for i in range(len(p_xy) - 1):
        peak_path = max(peak_path, path_len_acc)
        climb = path_len_acc < climb_path_threshold(peak_path)

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

        step = float(raw[i])
        clamp_step = is_clamp_step(step)
        clamp_hist.append(clamp_step)
        clamp_frac_win = float(np.mean(clamp_hist)) if clamp_hist else 0.0
        dyaw = abs(float(yaw[i]) - prev_yaw)
        unreliable = shadow_proxy(dist_home, max_extent, clamp_frac_win, float(wz[i]), dyaw)

        if freeze_idx < 0 and returned and dist_home < near_r and len(home_ring) >= 5:
            freeze_idx = i + 1
            landing_xy = np.median(np.array(home_ring), axis=0)
        elif freeze_idx < 0 and returned and best_home < near_r:
            leaving = dist_home > best_home + depart_r
            if leaving:
                freeze_idx = i + 1
                if len(home_ring) >= 5:
                    landing_xy = np.median(np.array(home_ring), axis=0)
                else:
                    landing_xy = best_home_xy.copy()

        remain_raw = float(raw[i:].sum()) if i < len(raw) else 0.0
        late = remain_raw < 0.25 * raw_path_total and max_extent > far
        if late:
            if dist_home < late_best:
                late_best = dist_home
                late_xy = out[i].copy()
            elif (
                freeze_idx < 0
                and late_freeze_ok(late_freeze_mode, late_best, near_r, max_extent)
                and dist_home > late_best + max(depart_r, 15.0)
            ):
                freeze_idx = i + 1
                landing_xy = late_xy
                best_home = late_best

        if freeze_idx >= 0:
            out[i + 1] = landing_xy
            continue

        if len(good_speeds) >= GOOD_SPEED_N:
            speed = min(float(np.median(np.array(good_speeds))), 10.0)

        heading = float(yaw[i])
        if (
            climb
            and not clamp_step
            and is_good_raw(step, clamp_step)
            and step < VINS_YAW_MAX_STEP
        ):
            vb = vins_step_bearing(p_xy, i, heading)
            if yaw_mismatch(heading, vb) > YAW_MIS_CLIMB:
                heading = vb

        if step < 0.15:
            step = 0.0
        elif clamp_step:
            if returned:
                step = 0.0
            elif climb:
                if takeoff_mode == "zero_clamp" and clamp_mode != "hybrid_speed_takeoff_clamp_half":
                    step = 0.0
                else:
                    step = substitute_clamp(
                        clamp_mode,
                        speed,
                        float(dt[i]),
                        last_good,
                        good_history,
                        clamp_frac,
                        climb=True,
                        takeoff_mode=takeoff_mode,
                    )
            elif speed > 0.0:
                step = substitute_clamp(
                    clamp_mode,
                    speed,
                    float(dt[i]),
                    last_good,
                    good_history,
                    clamp_frac,
                    climb=False,
                    takeoff_mode=takeoff_mode,
                )
            else:
                step = 0.0
        elif unreliable and climb:
            step = 0.0
        else:
            last_good = step
            if is_good_raw(step, clamp_step) and not climb:
                good_history.append(step)
                dt_i = float(dt[i])
                if dt_i > 1e-4:
                    good_speeds.append(step / dt_i)

        out[i + 1] = out[i] + step * np.array([np.cos(heading), np.sin(heading)])
        path_len_acc += step
        prev_yaw = float(yaw[i])

    return out


@dataclass
class Metrics:
    name: str
    clamp_mode: str
    takeoff_mode: str
    late_freeze_mode: str
    end_dist: float
    path_ratio: float
    bbox_x_ratio: float
    bbox_y_ratio: float
    se2_rms: float
    se2_median: float
    sim2_scale: float

    @property
    def score(self) -> float:
        return self.end_dist + 0.5 * self.se2_median + 50.0 * abs(self.path_ratio - 0.94)


def eval_variant(
    xy: np.ndarray,
    t_v: np.ndarray,
    gt_path: Path,
    name: str,
    clamp_mode: str,
    takeoff_mode: str,
    late_freeze_mode: str,
) -> Metrics:
    t_g, p_g = load_tum(gt_path)[:, 0], load_tum(gt_path)[:, 1:4]
    pv = np.column_stack([xy[:, 0], xy[:, 1], np.zeros(len(xy))])
    pv_a, pg_a, _ = associate(t_v, pv, t_g, p_g)

    r_e, t_e, _ = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=False)
    xy_se2 = (r_e @ pv_a[:, :2].T).T + t_e
    err_se2 = np.linalg.norm(xy_se2 - pg_a[:, :2], axis=1)
    _, _, scale = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=True)

    bx_v, _ = bbox_extent(pv_a[:, :2])
    bx_g, _ = bbox_extent(pg_a[:, :2])

    return Metrics(
        name=name,
        clamp_mode=clamp_mode,
        takeoff_mode=takeoff_mode,
        late_freeze_mode=late_freeze_mode,
        end_dist=float(np.linalg.norm(pv_a[-1, :2] - pg_a[-1, :2])),
        path_ratio=path_len(pv_a[:, :2]) / max(path_len(pg_a[:, :2]), 1e-6),
        bbox_x_ratio=bx_v / max(bx_g, 1e-6),
        bbox_y_ratio=0.0,
        se2_rms=float(np.sqrt((err_se2**2).mean())),
        se2_median=float(np.median(err_se2)),
        sim2_scale=float(scale),
    )


def main() -> int:
    src = load_tum(VINS)
    t_v = src[:, 0]
    t_i, yaw_i, wz_i = load_dji_imu(IMU)
    yaw = np.interp(t_v, t_i, yaw_i)
    wz = np.clip(np.interp(t_v, t_i, wz_i), -0.5, 0.5)
    dt = np.diff(t_v, append=t_v[-1] + 0.1)
    t_rel = t_v - t_v[0]

    results: list[Metrics] = []
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    prod_xy = rebuild_xy(src[:, 1:3], yaw, dt, t_rel=t_rel, wz=wz)
    results.append(
        eval_variant(prod_xy, t_v, GT, "production", "speed_dt", "zero_clamp", "near_r")
    )

    for clamp_mode, takeoff_mode, late_mode in product(CLAMP_MODES, TAKEOFF_MODES, LATE_FREEZE_MODES):
        if clamp_mode == "hybrid_speed_takeoff_clamp_half" and takeoff_mode == "zero_clamp":
            continue
        name = f"{clamp_mode}+{takeoff_mode}+{late_mode}"
        xy = rebuild_xy_variant(
            src[:, 1:3], yaw, dt, wz, clamp_mode, takeoff_mode, late_mode
        )
        results.append(eval_variant(xy, t_v, GT, name, clamp_mode, takeoff_mode, late_mode))

    results.sort(key=lambda m: m.score)

    csv_path = OUT_DIR / "sweep_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "name",
                "clamp_mode",
                "takeoff_mode",
                "late_freeze_mode",
                "end_dist",
                "path_ratio",
                "bbox_x_ratio",
                "se2_rms",
                "se2_median",
                "sim2_scale",
                "score",
            ],
        )
        w.writeheader()
        for m in results:
            w.writerow(
                {
                    "name": m.name,
                    "clamp_mode": m.clamp_mode,
                    "takeoff_mode": m.takeoff_mode,
                    "late_freeze_mode": m.late_freeze_mode,
                    "end_dist": m.end_dist,
                    "path_ratio": m.path_ratio,
                    "bbox_x_ratio": m.bbox_x_ratio,
                    "se2_rms": m.se2_rms,
                    "se2_median": m.se2_median,
                    "sim2_scale": m.sim2_scale,
                    "score": m.score,
                }
            )

    print(f"Sweep HYBRID: {len(results)} variants, score = end + 0.5*se2_med + 50*|path-0.94|")
    print(f"Target: end~5m SE2~70m path~0.94 bboxX~0.96")
    print()
    for rank, m in enumerate(results[:10], 1):
        tag = " <--" if rank <= 3 else ""
        print(
            f"{rank:2d} {m.name:55s} score={m.score:6.1f} "
            f"end={m.end_dist:5.1f} path={m.path_ratio:.3f} bboxX={m.bbox_x_ratio:.3f} "
            f"SE2rms={m.se2_rms:.1f} SE2med={m.se2_median:.1f}{tag}"
        )
    print(f"\nCSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
