#!/usr/bin/env python3
"""Sweep clamp scale + turn dampening for fuse_vins_ahrs (field scene).

Variants:
  1. clamp step = scale * speed * dt, scale in [0.70..1.0]
  2. turn dampening: |wz| > turn_thr → clamp step *= turn_scale
  3. combined best scale + turn
  4. split clamp: 1.2 < step < 1.45 → step*0.85; step >= 1.45 → speed*dt

Keeps production landing_anchor, allow_half climb, is_clamp step>1.2, late freeze 0.55.

Score: 0.35*se2_median + 0.25*end_m + 0.25*50*|path_ratio-0.94| + 0.15*bboxX_penalty
  bboxX_penalty = abs(bboxX/0.96 - 1) * 50

    python3 tools/sweep_fuse_scale_turn.py
"""
from __future__ import annotations

import csv
import sys
from collections import deque
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Literal

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
OUT_DIR = ROOT / "results/eval_mars_scene_field_ahrs/sweep_scale_turn"

SCALES = [0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0]
TURN_THRS = [0.06, 0.08, 0.10]
TURN_SCALES = [0.5, 0.7, 1.0]
BASELINE_SE2_MED = 103.0


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


ClampMode = Literal["production", "scale", "turn", "combined", "split"]


def cruise_clamp_step(
    mode: ClampMode,
    raw_step: float,
    speed: float,
    dt_i: float,
    last_good: float,
    wz_i: float,
    *,
    scale: float = 1.0,
    turn_thr: float = 0.10,
    turn_scale: float = 1.0,
) -> float:
    spd_dt = speed * dt_i if dt_i > 1e-4 else last_good
    if speed <= 0.0:
        return 0.0

    if mode == "split":
        if 1.2 < raw_step < 1.45:
            step = raw_step * 0.85
        elif raw_step >= 1.45:
            step = spd_dt
        else:
            step = spd_dt
    elif mode in ("scale", "combined"):
        step = scale * spd_dt
    else:
        step = spd_dt

    if mode in ("turn", "combined") and abs(wz_i) > turn_thr:
        step *= turn_scale
    elif mode == "turn" and abs(wz_i) > turn_thr:
        step *= turn_scale

    return step


def rebuild_xy_variant(
    p_xy: np.ndarray,
    yaw: np.ndarray,
    dt: np.ndarray,
    wz: np.ndarray,
    mode: ClampMode,
    *,
    scale: float = 1.0,
    turn_thr: float = 0.10,
    turn_scale: float = 1.0,
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

        wz_i = float(wz[i])
        if step < 0.15:
            step = 0.0
        elif clamp_step:
            if returned:
                step = 0.0
            elif climb:
                step = 0.5 * CLAMP_LIMIT  # allow_half
            else:
                step = cruise_clamp_step(
                    mode,
                    float(raw[i]),
                    speed,
                    float(dt[i]),
                    last_good,
                    wz_i,
                    scale=scale,
                    turn_thr=turn_thr,
                    turn_scale=turn_scale,
                )
        elif unreliable and climb:
            step = 0.0
        else:
            last_good = step
            if is_good_raw(step, clamp_step) and not climb:
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
    variant: str
    end_dist: float
    path_ratio: float
    bbox_x_ratio: float
    se2_median: float
    se2_rms: float
    sim2_scale: float
    scale: float = 1.0
    turn_thr: float = 0.0
    turn_scale: float = 1.0

    @property
    def bbox_x_penalty(self) -> float:
        return abs(self.bbox_x_ratio / 0.96 - 1.0) * 50.0

    @property
    def score(self) -> float:
        return (
            0.35 * self.se2_median
            + 0.25 * self.end_dist
            + 0.25 * 50.0 * abs(self.path_ratio - 0.94)
            + 0.15 * self.bbox_x_penalty
        )


def eval_variant(
    xy: np.ndarray,
    t_v: np.ndarray,
    gt_path: Path,
    name: str,
    variant: str,
    scale: float = 1.0,
    turn_thr: float = 0.0,
    turn_scale: float = 1.0,
) -> Metrics:
    t_g, p_g = load_tum(gt_path)[:, 0], load_tum(gt_path)[:, 1:4]
    pv = np.column_stack([xy[:, 0], xy[:, 1], np.zeros(len(xy))])
    pv_a, pg_a, _ = associate(t_v, pv, t_g, p_g)

    r_e, t_e, _ = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=False)
    xy_se2 = (r_e @ pv_a[:, :2].T).T + t_e
    err_se2 = np.linalg.norm(xy_se2 - pg_a[:, :2], axis=1)
    _, _, sim_scale = umeyama_2d(pv_a[:, :2], pg_a[:, :2], with_scale=True)

    bx_v, _ = bbox_extent(pv_a[:, :2])
    bx_g, _ = bbox_extent(pg_a[:, :2])

    return Metrics(
        name=name,
        variant=variant,
        end_dist=float(np.linalg.norm(pv_a[-1, :2] - pg_a[-1, :2])),
        path_ratio=path_len(pv_a[:, :2]) / max(path_len(pg_a[:, :2]), 1e-6),
        bbox_x_ratio=bx_v / max(bx_g, 1e-6),
        se2_median=float(np.median(err_se2)),
        se2_rms=float(np.sqrt((err_se2**2).mean())),
        sim2_scale=float(sim_scale),
        scale=scale,
        turn_thr=turn_thr,
        turn_scale=turn_scale,
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

    # production baseline
    prod_xy = rebuild_xy(p_xy, yaw, dt, t_rel=t_rel, wz=wz)
    results.append(eval_variant(prod_xy, t_v, GT, "production", "production"))

    # 1. scale sweep
    scale_results: list[Metrics] = []
    for sc in SCALES:
        name = f"scale_{sc:.2f}"
        xy = rebuild_xy_variant(p_xy, yaw, dt, wz, "scale", scale=sc)
        m = eval_variant(xy, t_v, GT, name, "scale", scale=sc)
        scale_results.append(m)
        if sc < 1.0:
            results.append(m)

    best_scale = min(scale_results, key=lambda m: m.score)
    print(f"Best scale: {best_scale.scale:.2f} (SE2med={best_scale.se2_median:.1f}, score={best_scale.score:.1f})")

    # 2. turn dampening sweep
    turn_results: list[Metrics] = []
    for thr, ts in product(TURN_THRS, TURN_SCALES):
        name = f"turn_thr{thr:.2f}_ts{ts:.1f}"
        xy = rebuild_xy_variant(p_xy, yaw, dt, wz, "turn", turn_thr=thr, turn_scale=ts)
        m = eval_variant(xy, t_v, GT, name, "turn", turn_thr=thr, turn_scale=ts)
        turn_results.append(m)
        if ts < 1.0:
            results.append(m)

    active_turn = [m for m in turn_results if m.turn_scale < 1.0]
    best_turn = min(active_turn, key=lambda m: m.score)
    print(
        f"Best turn: thr={best_turn.turn_thr:.2f} ts={best_turn.turn_scale:.1f} "
        f"(SE2med={best_turn.se2_median:.1f}, score={best_turn.score:.1f})"
    )

    # 3. combined best scale + turn
    comb_name = f"combined_s{best_scale.scale:.2f}_t{best_turn.turn_thr:.2f}_ts{best_turn.turn_scale:.1f}"
    comb_xy = rebuild_xy_variant(
        p_xy,
        yaw,
        dt,
        wz,
        "combined",
        scale=best_scale.scale,
        turn_thr=best_turn.turn_thr,
        turn_scale=best_turn.turn_scale,
    )
    results.append(
        eval_variant(
            comb_xy,
            t_v,
            GT,
            comb_name,
            "combined",
            scale=best_scale.scale,
            turn_thr=best_turn.turn_thr,
            turn_scale=best_turn.turn_scale,
        )
    )

    # 4. split clamp
    split_xy = rebuild_xy_variant(p_xy, yaw, dt, wz, "split")
    results.append(eval_variant(split_xy, t_v, GT, "split_clamp", "split"))

    results.sort(key=lambda m: m.score)

    csv_path = OUT_DIR / "sweep_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "name",
                "variant",
                "score",
                "se2_median",
                "end_dist",
                "path_ratio",
                "bbox_x_ratio",
                "bbox_x_penalty",
                "se2_rms",
                "sim2_scale",
                "scale",
                "turn_thr",
                "turn_scale",
            ],
        )
        w.writeheader()
        for m in results:
            w.writerow(
                {
                    "name": m.name,
                    "variant": m.variant,
                    "score": m.score,
                    "se2_median": m.se2_median,
                    "end_dist": m.end_dist,
                    "path_ratio": m.path_ratio,
                    "bbox_x_ratio": m.bbox_x_ratio,
                    "bbox_x_penalty": m.bbox_x_penalty,
                    "se2_rms": m.se2_rms,
                    "sim2_scale": m.sim2_scale,
                    "scale": m.scale,
                    "turn_thr": m.turn_thr,
                    "turn_scale": m.turn_scale,
                }
            )

    prod = next(m for m in results if m.name == "production")
    winner = results[0]

    print()
    print("Score = 0.35*SE2med + 0.25*end + 0.25*50*|path-0.94| + 0.15*bboxX_pen")
    print(
        f"Production: SE2med={prod.se2_median:.1f} end={prod.end_dist:.1f} "
        f"path={prod.path_ratio:.3f} bboxX={prod.bbox_x_ratio:.3f} score={prod.score:.1f}"
    )
    print(f"Baseline SE2 med target: {BASELINE_SE2_MED:.0f} m")
    print()
    print("Top 5:")
    for rank, m in enumerate(results[:5], 1):
        tag = " <-- WINNER" if rank == 1 else ""
        print(
            f"{rank:2d} {m.name:40s} score={m.score:6.1f} "
            f"SE2med={m.se2_median:6.1f} end={m.end_dist:5.1f} "
            f"path={m.path_ratio:.3f} bboxX={m.bbox_x_ratio:.3f}{tag}"
        )
    print(f"\nCSV: {csv_path}")

    se2_improve = BASELINE_SE2_MED - winner.se2_median
    if se2_improve > 10.0:
        print(f"\n*** Winner improves SE2 med by {se2_improve:.1f} m vs {BASELINE_SE2_MED:.0f} m baseline ***")
        print_patch(winner)
    else:
        print(f"\nWinner SE2 med improvement vs baseline: {se2_improve:.1f} m (need >10 m for patch)")

    return 0


def print_patch(winner: Metrics) -> None:
    """Print suggested fuse_vins_ahrs.py patch for the winning variant."""
    if winner.variant == "scale":
        print(
            f"""
--- fuse_vins_ahrs.py patch (scale={winner.scale:.2f}) ---
In rebuild_xy(), replace cruise clamp step:
                step = speed * float(dt[i]) if dt[i] > 1e-4 else last_good
with:
                step = {winner.scale:.2f} * speed * float(dt[i]) if dt[i] > 1e-4 else last_good
"""
        )
    elif winner.variant == "turn":
        print(
            f"""
--- fuse_vins_ahrs.py patch (turn_thr={winner.turn_thr:.2f}, turn_scale={winner.turn_scale:.1f}) ---
Add constants near top:
TURN_DAMP_WZ = {winner.turn_thr:.2f}
TURN_DAMP_SCALE = {winner.turn_scale:.1f}

In rebuild_xy() cruise clamp block, after computing step = speed * dt:
                if abs(float(wz[i])) > TURN_DAMP_WZ:
                    step *= TURN_DAMP_SCALE
"""
        )
    elif winner.variant == "combined":
        print(
            f"""
--- fuse_vins_ahrs.py patch (scale={winner.scale:.2f}, turn_thr={winner.turn_thr:.2f}, turn_scale={winner.turn_scale:.1f}) ---
Add constants:
CLAMP_SPEED_SCALE = {winner.scale:.2f}
TURN_DAMP_WZ = {winner.turn_thr:.2f}
TURN_DAMP_SCALE = {winner.turn_scale:.1f}

In rebuild_xy() cruise clamp block:
                step = CLAMP_SPEED_SCALE * speed * float(dt[i]) if dt[i] > 1e-4 else last_good
                if abs(float(wz[i])) > TURN_DAMP_WZ:
                    step *= TURN_DAMP_SCALE
"""
        )
    elif winner.variant == "split":
        print(
            """
--- fuse_vins_ahrs.py patch (split clamp) ---
In rebuild_xy() cruise clamp block, replace speed*dt with:
                raw_i = float(raw[i])
                if 1.2 < raw_i < 1.45:
                    step = raw_i * 0.85
                elif raw_i >= 1.45:
                    step = speed * float(dt[i]) if dt[i] > 1e-4 else last_good
                else:
                    step = speed * float(dt[i]) if dt[i] > 1e-4 else last_good
"""
        )
    else:
        print("Winner is production — no patch needed.")


if __name__ == "__main__":
    raise SystemExit(main())
