#!/usr/bin/env python3
"""Sweep clamp step-length variants for fuse_vins_ahrs (field scene).

Score: 0.4*se2_median + 0.3*end_m + 0.3*50*abs(path_ratio - 0.94)

Keeps production landing_anchor + allow_half climb; only clamp cruise step differs.

    python3 tools/sweep_fuse_clamp_step.py
"""
from __future__ import annotations

import csv
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from fuse_vins_ahrs import (  # noqa: E402
    CLAMP_LIMIT,
    GOOD_SPEED_HISTORY,
    GOOD_SPEED_N,
    SHADOW_CLAMP_WINDOW,
    VINS_YAW_MAX_STEP,
    YAW_MIS_CLIMB,
    climb_path_threshold,
    is_clamp_step,
    is_good_raw,
    landing_anchor,
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
OUT_DIR = ROOT / "results/eval_mars_scene_field_ahrs/sweep_clamp_step"


def is_clamp_step_strict(step: float) -> bool:
    return abs(step - CLAMP_LIMIT) < 0.05


def estimate_speed_median_only(good_speeds: deque[float], _clamp_frac: float = 0.0) -> tuple[float, str]:
    if len(good_speeds) >= GOOD_SPEED_N:
        speed = min(float(np.median(np.array(good_speeds))), 10.0)
        return speed, f"median good ({speed:.1f} m/s)"
    return 0.0, "stop: no good speeds yet"


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


def clamp_cruise_step(
    mode: str,
    raw_step: float,
    speed: float,
    dt_i: float,
    last_good: float,
    leg_speed: float,
) -> float:
    spd_dt = speed * dt_i if dt_i > 1e-4 else last_good
    if mode == "production":
        return spd_dt if speed > 0.0 else 0.0
    if mode == "raw_on_clamp":
        return raw_step
    if mode == "clamp_15_fixed":
        return CLAMP_LIMIT
    if mode == "max_speed_raw":
        if speed <= 0.0:
            return raw_step
        return max(spd_dt, raw_step)
    if mode == "min_speed_clamp":
        if speed <= 0.0:
            return 0.0
        return min(spd_dt, CLAMP_LIMIT)
    if mode == "speed_dt_median_only":
        return spd_dt if speed > 0.0 else 0.0
    if mode == "per_leg_speed":
        ls = leg_speed if leg_speed > 0.0 else speed
        if ls <= 0.0:
            return 0.0
        return ls * dt_i if dt_i > 1e-4 else last_good
    raise ValueError(mode)


def rebuild_xy_variant(
    p_xy: np.ndarray,
    yaw: np.ndarray,
    dt: np.ndarray,
    wz: np.ndarray,
    mode: str,
    clamp_fn: Callable[[float], bool] = is_clamp_step,
    speed_fn: Callable[[deque[float], float], tuple[float, str]] | None = None,
) -> np.ndarray:
    raw = np.linalg.norm(np.diff(p_xy, axis=0), axis=1)
    clamp_mask = np.array([clamp_fn(float(s)) for s in raw])
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
    clamp_hist: deque[bool] = deque(maxlen=SHADOW_CLAMP_WINDOW)
    prev_yaw = float(yaw[0])
    raw_path_total = float(raw.sum())
    late_best = 1e9
    late_xy = start.copy()

    # per-leg speed: good steps on previous straight segment
    leg_speeds: deque[float] = deque(maxlen=GOOD_SPEED_HISTORY)
    leg_speed = 0.0
    in_straight = False
    prev_wz = 0.0

    if speed_fn is None:
        from fuse_vins_ahrs import estimate_speed

        def _default_speed(gs: deque[float], cf: float) -> tuple[float, str]:
            return estimate_speed(gs, cf)

        speed_fn = _default_speed

    speed, _ = speed_fn(good_speeds, clamp_frac)

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
        clamp_step = clamp_fn(step)
        clamp_hist.append(clamp_step)
        clamp_frac_win = float(np.mean(clamp_hist)) if clamp_hist else 0.0
        dyaw = abs(float(yaw[i]) - prev_yaw)
        unreliable = shadow_proxy(dist_home, max_extent, clamp_frac_win, float(wz[i]), dyaw)

        if freeze_idx < 0 and returned and dist_home < near_r and len(home_ring) >= 5:
            freeze_idx = i + 1
            landing_xy = landing_anchor(
                np.median(np.array(home_ring), axis=0), start, dist_home, max_extent
            )
        elif freeze_idx < 0 and returned and best_home < near_r:
            leaving = dist_home > best_home + depart_r
            if leaving:
                freeze_idx = i + 1
                if len(home_ring) >= 5:
                    home = np.median(np.array(home_ring), axis=0)
                else:
                    home = best_home_xy.copy()
                landing_xy = landing_anchor(home, start, best_home, max_extent)

        remain_raw = float(raw[i:].sum()) if i < len(raw) else 0.0
        late = remain_raw < 0.25 * raw_path_total and max_extent > far
        if late:
            if dist_home < late_best:
                late_best = dist_home
                late_xy = out[i].copy()
            elif (
                freeze_idx < 0
                and late_best < 0.55 * max_extent
                and dist_home > late_best + max(depart_r, 15.0)
            ):
                freeze_idx = i + 1
                landing_xy = landing_anchor(late_xy, start, late_best, max_extent)
                best_home = late_best

        if freeze_idx >= 0:
            out[i + 1] = landing_xy
            continue

        if len(good_speeds) >= GOOD_SPEED_N:
            speed = min(float(np.median(np.array(good_speeds))), 10.0)

        # per-leg: detect straight segment transitions
        wz_i = float(wz[i])
        straight_now = abs(wz_i) < 0.04 and not climb
        if mode == "per_leg_speed":
            if in_straight and not straight_now and len(leg_speeds) >= 3:
                leg_speed = min(float(np.median(np.array(leg_speeds))), 10.0)
                leg_speeds.clear()
            in_straight = straight_now

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
                step = 0.5 * CLAMP_LIMIT  # allow_half (production)
            else:
                step = clamp_cruise_step(
                    mode, float(raw[i]), speed, float(dt[i]), last_good, leg_speed
                )
        elif unreliable and climb:
            step = 0.0
        else:
            last_good = step
            if is_good_raw(step, clamp_step) and not climb:
                dt_i = float(dt[i])
                if dt_i > 1e-4:
                    spd = step / dt_i
                    good_speeds.append(spd)
                    if mode == "per_leg_speed" and in_straight:
                        leg_speeds.append(spd)

        out[i + 1] = out[i] + step * np.array([np.cos(heading), np.sin(heading)])
        path_len_acc += step
        prev_yaw = float(yaw[i])
        prev_wz = wz_i

    return out


@dataclass
class Metrics:
    name: str
    end_dist: float
    path_ratio: float
    se2_median: float
    se2_rms: float
    sim2_scale: float

    @property
    def score(self) -> float:
        return (
            0.4 * self.se2_median
            + 0.3 * self.end_dist
            + 0.3 * 50.0 * abs(self.path_ratio - 0.94)
        )


def eval_variant(xy: np.ndarray, t_v: np.ndarray, gt_path: Path, name: str) -> Metrics:
    t_g, p_g = load_tum(gt_path)[:, 0], load_tum(gt_path)[:, 1:4]
    pv = np.column_stack([xy[:, 0], xy[:, 1], np.zeros(len(xy))])
    pv_a, pg_a, _ = associate(t_v, pv, t_g, p_g)

    r_e, t_e, _ = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=False)
    xy_se2 = (r_e @ pv_a[:, :2].T).T + t_e
    err_se2 = np.linalg.norm(xy_se2 - pg_a[:, :2], axis=1)
    _, _, scale = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=True)

    return Metrics(
        name=name,
        end_dist=float(np.linalg.norm(pv_a[-1, :2] - pg_a[-1, :2])),
        path_ratio=path_len(pv_a[:, :2]) / max(path_len(pg_a[:, :2]), 1e-6),
        se2_median=float(np.median(err_se2)),
        se2_rms=float(np.sqrt((err_se2**2).mean())),
        sim2_scale=float(scale),
    )


VARIANTS: list[tuple[str, dict]] = [
    ("production", {"mode": "production"}),
    ("1_raw_on_clamp", {"mode": "raw_on_clamp"}),
    ("2_clamp_15_fixed", {"mode": "clamp_15_fixed"}),
    ("3a_max_speed_raw", {"mode": "max_speed_raw"}),
    ("3b_min_speed_clamp", {"mode": "min_speed_clamp"}),
    ("4_speed_dt_median_only", {"mode": "speed_dt_median_only", "speed_fn": estimate_speed_median_only}),
    ("5_is_clamp_strict", {"mode": "production", "clamp_fn": is_clamp_step_strict}),
    ("6_per_leg_speed", {"mode": "per_leg_speed"}),
]


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
    results.append(eval_variant(prod_xy, t_v, GT, "production"))

    for name, kwargs in VARIANTS[1:]:
        xy = rebuild_xy_variant(src[:, 1:3], yaw, dt, wz, **kwargs)
        results.append(eval_variant(xy, t_v, GT, name))

    results.sort(key=lambda m: m.score)

    csv_path = OUT_DIR / "sweep_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=["name", "score", "se2_median", "end_dist", "path_ratio", "se2_rms", "sim2_scale"],
        )
        w.writeheader()
        for m in results:
            w.writerow(
                {
                    "name": m.name,
                    "score": m.score,
                    "se2_median": m.se2_median,
                    "end_dist": m.end_dist,
                    "path_ratio": m.path_ratio,
                    "se2_rms": m.se2_rms,
                    "sim2_scale": m.sim2_scale,
                }
            )

    prod = next(m for m in results if m.name == "production")
    print(f"Score = 0.4*SE2med + 0.3*end + 0.3*50*|path-0.94|")
    print(f"Production baseline: SE2med={prod.se2_median:.1f} end={prod.end_dist:.1f} path={prod.path_ratio:.3f} score={prod.score:.1f}")
    print()
    for rank, m in enumerate(results[:5], 1):
        tag = " <--" if rank <= 3 else ""
        print(
            f"{rank:2d} {m.name:28s} score={m.score:6.1f} "
            f"SE2med={m.se2_median:6.1f} end={m.end_dist:5.1f} path={m.path_ratio:.3f}{tag}"
        )
    print(f"\nCSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
