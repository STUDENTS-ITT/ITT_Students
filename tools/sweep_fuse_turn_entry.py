#!/usr/bin/env python3
"""Sweep turn-entry clamp step reduction for fuse_vins_ahrs (field scene).

Production base: CLAMP_SPEED_SCALE=0.90 + leg-local speed (LEG_WZ_*).
On cruise clamp steps when |wz| > wz_thr: step *= turn_scale.

Grid:
  wz_thr     in [0.06, 0.08, 0.10, 0.12]
  turn_scale in [0.3, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

Score: 0.4*se2_median + 0.25*end_m + 0.2*50*|path_ratio-0.94| + 0.15*max(0,end_m-15)

    python3 tools/sweep_fuse_turn_entry.py
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
    CLAMP_SPEED_SCALE,
    GOOD_SPEED_HISTORY,
    GOOD_SPEED_N,
    LEG_SPEED_MIN,
    LEG_WZ_RESET,
    LEG_WZ_STRAIGHT,
    SHADOW_CLAMP_WINDOW,
    VINS_YAW_MAX_STEP,
    YAW_MIS_CLIMB,
    climb_path_threshold,
    estimate_speed,
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
OUT_DIR = ROOT / "results/eval_mars_scene_field_ahrs/sweep_turn_entry"

WZ_THRS = [0.06, 0.08, 0.10, 0.12]
TURN_SCALES = [0.3, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
BASELINE_SE2_MED = 98.9
END_CAP_M = 25.0
SE2_IMPROVE_THRESH = 8.0


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


def rebuild_xy_turn_entry(
    p_xy: np.ndarray,
    yaw: np.ndarray,
    dt: np.ndarray,
    wz: np.ndarray,
    *,
    wz_thr: float = 0.10,
    turn_scale: float = 1.0,
) -> np.ndarray:
    """Production fuse + optional turn-entry dampening on clamp cruise steps."""
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
    clamp_hist: deque[bool] = deque(maxlen=SHADOW_CLAMP_WINDOW)
    prev_yaw = float(yaw[0])
    raw_path_total = float(raw.sum())
    late_best = 1e9
    late_xy = start.copy()
    leg_speeds: deque[float] = deque(maxlen=20)

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
        wz_i = float(wz[i])
        unreliable = shadow_proxy(dist_home, max_extent, clamp_frac_win, wz_i, dyaw)

        if abs(wz_i) >= LEG_WZ_RESET:
            leg_speeds.clear()

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
                step = 0.5 * CLAMP_LIMIT
            elif speed > 0.0:
                spd = speed
                if abs(wz_i) < LEG_WZ_STRAIGHT and len(leg_speeds) >= LEG_SPEED_MIN:
                    spd = min(float(np.median(np.array(leg_speeds))), 10.0)
                dt_i = float(dt[i])
                step = CLAMP_SPEED_SCALE * spd * dt_i if dt_i > 1e-4 else last_good
                if turn_scale < 1.0 and abs(wz_i) > wz_thr:
                    step *= turn_scale
            else:
                step = 0.0
        elif unreliable and climb:
            step = 0.0
        else:
            last_good = step
            if is_good_raw(step, clamp_step) and not climb:
                dt_i = float(dt[i])
                if dt_i > 1e-4:
                    gs = step / dt_i
                    good_speeds.append(gs)
                    if abs(wz_i) < LEG_WZ_STRAIGHT:
                        leg_speeds.append(gs)

        out[i + 1] = out[i] + step * np.array([np.cos(heading), np.sin(heading)])
        path_len_acc += step
        prev_yaw = float(yaw[i])

    return out


@dataclass
class Metrics:
    name: str
    end_dist: float
    path_ratio: float
    se2_median: float
    se2_rms: float
    sim2_scale: float
    wz_thr: float = 0.0
    turn_scale: float = 1.0

    @property
    def score(self) -> float:
        return (
            0.4 * self.se2_median
            + 0.25 * self.end_dist
            + 0.2 * 50.0 * abs(self.path_ratio - 0.94)
            + 0.15 * max(0.0, self.end_dist - 15.0)
        )


def eval_variant(
    xy: np.ndarray,
    t_v: np.ndarray,
    gt_path: Path,
    name: str,
    wz_thr: float = 0.0,
    turn_scale: float = 1.0,
) -> Metrics:
    t_g, p_g = load_tum(gt_path)[:, 0], load_tum(gt_path)[:, 1:4]
    pv = np.column_stack([xy[:, 0], xy[:, 1], np.zeros(len(xy))])
    pv_a, pg_a, _ = associate(t_v, pv, t_g, p_g)

    r_e, t_e, _ = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=False)
    xy_se2 = (r_e @ pv_a[:, :2].T).T + t_e
    err_se2 = np.linalg.norm(xy_se2 - pg_a[:, :2], axis=1)
    _, _, sim_scale = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=True)

    return Metrics(
        name=name,
        end_dist=float(np.linalg.norm(pv_a[-1, :2] - pg_a[-1, :2])),
        path_ratio=path_len(pv_a[:, :2]) / max(path_len(pg_a[:, :2]), 1e-6),
        se2_median=float(np.median(err_se2)),
        se2_rms=float(np.sqrt((err_se2**2).mean())),
        sim2_scale=float(sim_scale),
        wz_thr=wz_thr,
        turn_scale=turn_scale,
    )


def print_patch(winner: Metrics) -> None:
    print(
        f"""
--- fuse_vins_ahrs.py patch (wz_thr={winner.wz_thr:.2f}, turn_scale={winner.turn_scale:.1f}) ---
Add constants near top (after CLAMP_SPEED_SCALE):
TURN_ENTRY_WZ = {winner.wz_thr:.2f}
TURN_ENTRY_SCALE = {winner.turn_scale:.1f}

In rebuild_xy() cruise clamp block, after:
                step = CLAMP_SPEED_SCALE * spd * dt_i if dt_i > 1e-4 else last_good
add:
                if abs(float(wz[i])) > TURN_ENTRY_WZ:
                    step *= TURN_ENTRY_SCALE
"""
    )


def main() -> int:
    src = load_tum(VINS)
    t_v = src[:, 0]
    t_i, yaw_i, wz_i = load_dji_imu(IMU)
    yaw = np.interp(t_v, t_i, yaw_i)
    wz = np.clip(np.interp(t_v, t_i, wz_i), -0.5, 0.5)
    dt = np.diff(t_v, append=t_v[-1] + 0.1)
    t_rel = t_v - t_v[0]
    p_xy = src[:, 1:3]

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    results: list[Metrics] = []

    prod_xy = rebuild_xy(p_xy, yaw, dt, t_rel=t_rel, wz=wz)
    results.append(eval_variant(prod_xy, t_v, GT, "production", wz_thr=0.0, turn_scale=1.0))

    for thr, ts in product(WZ_THRS, TURN_SCALES):
        name = f"thr{thr:.2f}_ts{ts:.1f}"
        xy = rebuild_xy_turn_entry(p_xy, yaw, dt, wz, wz_thr=thr, turn_scale=ts)
        results.append(eval_variant(xy, t_v, GT, name, wz_thr=thr, turn_scale=ts))

    results.sort(key=lambda m: m.score)

    csv_path = OUT_DIR / "sweep_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "name",
                "score",
                "se2_median",
                "end_dist",
                "path_ratio",
                "se2_rms",
                "sim2_scale",
                "wz_thr",
                "turn_scale",
            ],
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
                    "wz_thr": m.wz_thr,
                    "turn_scale": m.turn_scale,
                }
            )

    prod = next(m for m in results if m.name == "production")

    print()
    print("Score = 0.4*SE2med + 0.25*end + 0.2*50*|path-0.94| + 0.15*max(0,end-15)")
    print(
        f"Production: SE2med={prod.se2_median:.1f} end={prod.end_dist:.1f} "
        f"path={prod.path_ratio:.3f} score={prod.score:.1f}"
    )
    print(f"Baseline SE2 med target: {BASELINE_SE2_MED:.1f} m")
    print()
    print("Top 5:")
    for rank, m in enumerate(results[:5], 1):
        tag = " <-- WINNER" if rank == 1 else ""
        print(
            f"{rank:2d} {m.name:24s} score={m.score:6.1f} "
            f"SE2med={m.se2_median:6.1f} end={m.end_dist:5.1f} "
            f"path={m.path_ratio:.3f} thr={m.wz_thr:.2f} ts={m.turn_scale:.1f}{tag}"
        )
    print(f"\nCSV: {csv_path}")

    winner = results[0]
    se2_improve = BASELINE_SE2_MED - winner.se2_median
    if se2_improve > SE2_IMPROVE_THRESH and winner.end_dist < END_CAP_M:
        print(
            f"\n*** Winner improves SE2 med by {se2_improve:.1f} m vs {BASELINE_SE2_MED:.1f} m "
            f"baseline, end={winner.end_dist:.1f} m < {END_CAP_M:.0f} m ***"
        )
        print_patch(winner)
    else:
        print(
            f"\nNo patch: SE2 improve={se2_improve:.1f} m (need >{SE2_IMPROVE_THRESH:.0f}), "
            f"winner end={winner.end_dist:.1f} m (need <{END_CAP_M:.0f})"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
