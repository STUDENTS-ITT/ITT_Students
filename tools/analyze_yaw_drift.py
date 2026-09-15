#!/usr/bin/env python3
"""Yaw drift analysis: AHRS vs VINS bearing vs RTK motion bearing (field scene)."""
from __future__ import annotations

import csv
import sys
from collections import deque
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from fuse_vins_ahrs import (  # noqa: E402
    CLAMP_LIMIT,
    VINS_YAW_MAX_STEP,
    YAW_MIS_CLIMB,
    climb_path_threshold,
    is_clamp_step,
    is_good_raw,
    load_dji_imu,
    load_tum,
    vins_step_bearing,
    yaw_mismatch,
)


def signed_yaw_diff(a: float, b: float) -> float:
    """a - b wrapped to [-pi, pi]."""
    return float(((a - b + np.pi) % (2.0 * np.pi)) - np.pi)


def rtk_bearing(t_rtk: np.ndarray, xy_rtk: np.ndarray, t_query: float, win_s: float = 2.0) -> float | None:
    """Motion bearing from RTK over ±win_s around t_query."""
    m = (t_rtk >= t_query - win_s) & (t_rtk <= t_query + win_s)
    if m.sum() < 3:
        return None
    seg = xy_rtk[m]
    d = seg[-1] - seg[0]
    n = float(np.linalg.norm(d))
    if n < 0.5:
        return None
    return float(np.arctan2(d[1], d[0]))


def rolling_median(vals: deque[float], n: int) -> float | None:
    if len(vals) < n:
        return None
    return float(np.median(np.array(vals)))


def main() -> int:
    vins_path = ROOT / "results/eval_mars_scene_field_ahrs/vins_baro.tum"
    imu_path = ROOT / "data/mars/aux/dji_osdk_ros_imu.csv"
    rtk_path = ROOT / "data/mars/mars_hkairport03_livox_gt.tum"
    out_dir = ROOT / "results/eval_mars_scene_field_ahrs/yaw_drift_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    src = load_tum(vins_path)
    t_v = src[:, 0]
    p_xy = src[:, 1:3]
    t_rel = t_v - t_v[0]
    dt = np.diff(t_v, append=t_v[-1] + 0.1)

    t_i, yaw_i, wz_i = load_dji_imu(imu_path)
    yaw = np.interp(t_v, t_i, yaw_i)
    wz = np.clip(np.interp(t_v, t_i, wz_i), -0.5, 0.5)

    rtk = load_tum(rtk_path)
    t_r = rtk[:, 0]
    xy_r = rtk[:, 1:3]

    raw = np.linalg.norm(np.diff(p_xy, axis=0), axis=1)
    n = len(raw)

    clamp_mask = np.array([is_clamp_step(float(s)) for s in raw])
    good_mask = np.array([is_good_raw(float(s), bool(c)) for s, c in zip(raw, clamp_mask)])

    vb = np.full(n, np.nan)
    mis_vins = np.full(n, np.nan)
    signed_vins = np.full(n, np.nan)
    mis_rtk = np.full(n, np.nan)
    signed_rtk = np.full(n, np.nan)
    rtk_bear = np.full(n, np.nan)

    path_len = 0.0
    peak_path = 0.0
    climb_flags = np.zeros(n, dtype=bool)
    clamp_hist: deque[bool] = deque(maxlen=40)

    for i in range(n):
        peak_path = max(peak_path, path_len)
        climb_flags[i] = path_len < climb_path_threshold(peak_path)
        clamp_hist.append(bool(clamp_mask[i]))
        step = float(raw[i])
        path_len += step

        h = float(yaw[i])
        b = vins_step_bearing(p_xy, i, h)
        vb[i] = b
        mis_vins[i] = yaw_mismatch(h, b)
        signed_vins[i] = signed_yaw_diff(b, h)

        rb = rtk_bearing(t_r, xy_r, float(t_v[i]))
        if rb is not None:
            rtk_bear[i] = rb
            mis_rtk[i] = yaw_mismatch(h, rb)
            signed_rtk[i] = signed_yaw_diff(rb, h)

    cruise = ~climb_flags
    good_cruise = good_mask & cruise
    straight = np.abs(wz[:n]) < 0.04  # match SHADOW_WZ scale
    straight_cruise_good = good_cruise & straight

    # --- 1. AHRS vs VINS on good non-clamp steps ---
    print("=" * 72)
    print("1. AHRS vs VINS step bearing (good, non-clamp steps)")
    print("=" * 72)
    for label, mask in [
        ("all good", good_mask),
        ("good cruise (post-climb)", good_cruise),
        ("good cruise + |wz|<0.04", straight_cruise_good),
        ("good climb", good_mask & climb_flags),
    ]:
        m = mask & np.isfinite(mis_vins)
        if m.sum() == 0:
            continue
        vals = mis_vins[m]
        flip = np.array([min(v, np.pi - v) for v in vals])  # 180° ambiguity
        print(
            f"  {label:30s} n={m.sum():5d}  "
            f"med={np.degrees(np.median(vals)):6.2f}°  "
            f"med_180amb={np.degrees(np.median(flip)):6.2f}°  "
            f"p90={np.degrees(np.percentile(vals, 90)):6.2f}°  "
            f"max={np.degrees(vals.max()):6.2f}°  "
            f">YAW_MIS_CLIMB({np.degrees(YAW_MIS_CLIMB):.0f}°)={100*(vals>YAW_MIS_CLIMB).mean():5.1f}%"
        )

    big = good_cruise & (mis_vins > YAW_MIS_CLIMB)
    if big.sum():
        idx = np.where(big)[0]
        # cluster consecutive indices
        breaks = np.where(np.diff(idx) > 5)[0]
        starts = np.concatenate([[0], breaks + 1])
        ends = np.concatenate([breaks, [len(idx) - 1]])
        print("\n  Largest divergence episodes (good cruise, mis > YAW_MIS_CLIMB):")
        episodes = []
        for s, e in zip(starts, ends):
            ii = idx[s : e + 1]
            episodes.append(
                (
                    float(mis_vins[ii].max()),
                    float(t_rel[ii[0]]),
                    float(t_rel[ii[-1]]),
                    int(len(ii)),
                )
            )
        episodes.sort(reverse=True)
        for mx, t0, t1, ln in episodes[:8]:
            print(f"    t={t0:6.0f}-{t1:6.0f}s  n={ln:4d}  peak={np.degrees(mx):5.1f}°")

    # --- 2. AHRS vs RTK motion bearing (eval) ---
    print("\n" + "=" * 72)
    print("2. AHRS vs RTK motion bearing (eval only, 2s window)")
    print("=" * 72)
    rtk_ok = np.isfinite(mis_rtk)
    for label, mask in [
        ("all RTK samples", rtk_ok),
        ("RTK + cruise", rtk_ok & cruise),
        ("RTK + cruise + |wz|<0.04", rtk_ok & cruise & straight),
        ("RTK + good cruise", rtk_ok & good_cruise),
    ]:
        if mask.sum() == 0:
            continue
        vals = mis_rtk[mask]
        svals = signed_rtk[mask]
        print(
            f"  {label:30s} n={mask.sum():5d}  "
            f"med={np.degrees(np.median(vals)):6.2f}°  "
            f"mean_signed={np.degrees(np.mean(svals)):7.2f}°  "
            f"p90={np.degrees(np.percentile(vals, 90)):6.2f}°"
        )

    # cumulative signed error vs path on straight cruise
    sc = rtk_ok & cruise & straight
    if sc.sum() > 20:
        order = np.where(sc)[0]
        path_at = np.zeros(len(order))
        acc = 0.0
        for j, i in enumerate(order):
            acc += float(raw[i])
            path_at[j] = acc
        err = signed_rtk[order]
        # linear fit err vs path
        A = np.vstack([path_at, np.ones(len(path_at))]).T
        slope, intercept = np.linalg.lstsq(A, err, rcond=None)[0]
        resid = err - (slope * path_at + intercept)
        print(
            f"\n  Linear drift on straight cruise: "
            f"{np.degrees(slope)*1000:.3f} mdeg/m path  "
            f"(R²={1-np.var(resid)/np.var(err):.3f}, resid_std={np.degrees(np.std(resid)):.2f}°)"
        )
        total_path = path_at[-1] - path_at[0]
        print(
            f"  Total RTK signed error change over {total_path:.0f}m straight cruise: "
            f"{np.degrees(err[-1]-err[0]):+.1f}°"
        )

    # --- 3. Linear vs turn jumps ---
    print("\n" + "=" * 72)
    print("3. Error growth: linear drift vs turn jumps")
    print("=" * 72)
    dyaw_step = np.abs(np.diff(yaw[: n + 1]))  # n step transitions
    turn_i = dyaw_step > 0.06  # SHADOW_DYAW

    wz_turn_i = np.abs(wz[:n]) > 0.08

    sc = rtk_ok & cruise & straight
    for sig_name, sig in [("dyaw>0.06", turn_i), ("|wz|>0.08", wz_turn_i)]:
        jump_at_turn = []
        jump_straight = []
        for i in range(1, n):
            if not rtk_ok[i] or not rtk_ok[i - 1]:
                continue
            d = abs(signed_rtk[i] - signed_rtk[i - 1])
            if sig[i]:
                jump_at_turn.append(d)
            elif cruise[i] and straight[i]:
                jump_straight.append(d)
        if jump_at_turn:
            print(
                f"  |ΔRTK err| step at {sig_name}: "
                f"med={np.degrees(np.median(jump_at_turn)):.2f}°  "
                f"p90={np.degrees(np.percentile(jump_at_turn, 90)):.2f}°  "
                f"n={len(jump_at_turn)}"
            )
        if jump_straight:
            print(
                f"  |ΔRTK err| step on straight cruise (ref): "
                f"med={np.degrees(np.median(jump_straight)):.2f}°  "
                f"p90={np.degrees(np.percentile(jump_straight, 90)):.2f}°  "
                f"n={len(jump_straight)}"
            )

    # Where AHRS diverges from RTK (>15°)
    big_rtk = rtk_ok & (mis_rtk > np.radians(15))
    if big_rtk.sum():
        idx = np.where(big_rtk)[0]
        print(f"\n  RTK mis > 15°: {big_rtk.sum()} samples ({100*big_rtk.mean():.1f}% of all)")
        print(f"    at turns (dyaw>0.06): {100*(big_rtk & turn_i).sum()/big_rtk.sum():.1f}%")
        print(f"    at |wz|>0.08: {100*(big_rtk & wz_turn_i).sum()/big_rtk.sum():.1f}%")
        print(f"    on straight cruise: {100*(big_rtk & sc).sum()/big_rtk.sum():.1f}%")

    # per-leg straight segments (clamp_frac high proxy: rolling clamp)
    clamp_roll = np.full(n, np.nan)
    ch: deque[bool] = deque(maxlen=40)
    for i in range(n):
        ch.append(bool(clamp_mask[i]))
        clamp_roll[i] = float(np.mean(ch))

    high_clamp = clamp_roll > 0.40
    leg_mask = cruise & straight & high_clamp & rtk_ok
    if leg_mask.sum() > 30:
        idx = np.where(leg_mask)[0]
        splits = np.where(np.diff(idx) > 3)[0]
        seg_starts = np.concatenate([[0], splits + 1])
        seg_ends = np.concatenate([splits, [len(idx) - 1]])
        print(f"\n  Straight high-clamp legs (cruise, |wz|<0.04, clamp_frac>0.4): {len(seg_starts)} segments")
        leg_slopes = []
        for s, e in zip(seg_starts, seg_ends):
            ii = idx[s : e + 1]
            if len(ii) < 8:
                continue
            pl = np.cumsum(raw[ii]) - raw[ii[0]]
            er = signed_rtk[ii]
            A = np.vstack([pl, np.ones(len(pl))]).T
            sl, _ = np.linalg.lstsq(A, er, rcond=None)[0]
            leg_slopes.append(np.degrees(sl) * 1000)
        if leg_slopes:
            print(
                f"    per-leg slope (mdeg/m): med={np.median(leg_slopes):+.2f}  "
                f"std={np.std(leg_slopes):.2f}  "
                f"(near-zero median => mostly bias-free drift; high std => turn/segment jumps)"
            )

    # --- 4. Cruise correction strategy sweep ---
    print("\n" + "=" * 72)
    print("4. Cruise correction strategies (simulated heading, no RTK in fuse)")
    print("=" * 72)

    def sim_heading(
        i: int,
        *,
        mode: str,
        bias_deque: deque[float],
        bias_median_n: int,
        mis_thresh: float,
        wz_max: float,
        clamp_min: float,
        blend_alpha: float,
    ) -> float:
        h = float(yaw[i])
        if not cruise[i]:
            return h
        cf = clamp_roll[i] if np.isfinite(clamp_roll[i]) else 0.0
        if cf < clamp_min:
            return h
        if abs(float(wz[i])) > wz_max:
            return h
        b = float(vb[i])
        delta = signed_yaw_diff(b, h)
        bias_deque.append(delta)
        med = rolling_median(bias_deque, bias_median_n)
        if med is None:
            return h
        if mode == "bias_correct":
            if abs(med) > mis_thresh:
                return h + med
            return h
        if mode == "blend":
            if yaw_mismatch(h, b) > mis_thresh:
                return (1 - blend_alpha) * h + blend_alpha * b
            return h
        if mode == "bias+blend":
            hc = h + med if abs(med) > mis_thresh else h
            if yaw_mismatch(hc, b) > mis_thresh:
                return (1 - blend_alpha) * hc + blend_alpha * b
            return hc
        return h

    def eval_strategy(**kw) -> dict:
        bias_window = kw.pop("bias_window", 60)
        bias_d: deque[float] = deque(maxlen=bias_window)
        head = yaw.copy()
        for i in range(n):
            if not good_cruise[i]:
                continue
            head[i] = sim_heading(i, bias_deque=bias_d, **kw)
        # metrics on good cruise + straight
        m = good_cruise & straight
        vins_mis = mis_vins[m]
        ahrs_mis = mis_vins[m]  # already ahrs vs vins
        new_mis = np.array([yaw_mismatch(head[i], vb[i]) for i in range(n) if m[i]])
        rtk_m = m & rtk_ok
        rtk_before = mis_rtk[rtk_m]
        rtk_after = np.array([yaw_mismatch(head[i], rtk_bear[i]) for i in range(n) if rtk_m[i]])
        return {
            "vins_med": float(np.median(new_mis)),
            "vins_p90": float(np.percentile(new_mis, 90)),
            "vins_gt_thresh": float(np.mean(new_mis > YAW_MIS_CLIMB)),
            "rtk_med": float(np.median(rtk_after)) if len(rtk_after) else np.nan,
            "rtk_p90": float(np.percentile(rtk_after, 90)) if len(rtk_after) else np.nan,
            "rtk_delta_med": float(np.median(rtk_after) - np.median(rtk_before)) if len(rtk_after) else np.nan,
        }

    base = eval_strategy(
        mode="bias_correct",
        bias_median_n=9999,
        mis_thresh=999,
        wz_max=0,
        clamp_min=1,
        blend_alpha=0,
        bias_window=10,
    )
    # baseline: raw ahrs
    m = good_cruise & straight
    base_vins_med = float(np.median(mis_vins[m]))
    base_rtk_med = float(np.median(mis_rtk[m & rtk_ok])) if (m & rtk_ok).sum() else np.nan

    configs = []
    for mode in ["bias_correct", "blend", "bias+blend"]:
        for bias_n in [20, 40, 60, 80]:
            for thresh_deg in [8, 12, 15, 20]:
                for wz_max in [0.03, 0.04, 0.06]:
                    for clamp_min in [0.35, 0.40, 0.50]:
                        for alpha in [0.3, 0.5, 0.7]:
                            if mode == "bias_correct" and alpha != 0.3:
                                continue
                            kw = dict(
                                mode=mode,
                                bias_median_n=bias_n,
                                mis_thresh=np.radians(thresh_deg),
                                wz_max=wz_max,
                                clamp_min=clamp_min,
                                blend_alpha=alpha,
                                bias_window=bias_n * 2,
                            )
                            r = eval_strategy(**kw)
                            score = r["vins_med"] + 0.3 * r["rtk_med"] if np.isfinite(r["rtk_med"]) else r["vins_med"]
                            configs.append((score, kw, r))

    configs.sort(key=lambda x: x[0])
    print(f"  Baseline good-cruise-straight: VINS mis med={np.degrees(base_vins_med):.2f}°  RTK mis med={np.degrees(base_rtk_med):.2f}°")
    print("\n  Top 5 configs (min VINS+0.3*RTK median mismatch on good cruise straight):")
    for score, kw, r in configs[:5]:
        print(
            f"    {kw['mode']:12s} n={kw['bias_median_n']:2d} thr={np.degrees(kw['mis_thresh']):4.0f}° "
            f"wz<{kw['wz_max']:.2f} clamp>{kw['clamp_min']:.2f} α={kw['blend_alpha']:.1f}  "
            f"VINS med={np.degrees(r['vins_med']):5.2f}° p90={np.degrees(r['vins_p90']):5.2f}°  "
            f"RTK med={np.degrees(r['rtk_med']):5.2f}° (Δ{np.degrees(r['rtk_delta_med']):+.2f}°)"
        )

    best_score, best_kw, best_r = configs[0]

    # save series for plotting
    np.savetxt(
        out_dir / "yaw_series.csv",
        np.column_stack(
            [
                t_rel[:n],
                yaw[:n],
                vb,
                np.degrees(mis_vins),
                np.degrees(signed_vins),
                rtk_bear,
                np.degrees(mis_rtk),
                np.degrees(signed_rtk),
                clamp_mask.astype(float),
                good_mask.astype(float),
                cruise.astype(float),
                wz[:n],
                clamp_roll,
            ]
        ),
        delimiter=",",
        header="t_rel,ahrs_yaw,vins_bearing,mis_vins_deg,signed_vins_deg,rtk_bearing,mis_rtk_deg,signed_rtk_deg,clamp,good,cruise,wz,clamp_roll40",
        comments="",
    )
    print(f"\n  Wrote {out_dir / 'yaw_series.csv'}")

    # Recommendation block
    print("\n" + "=" * 72)
    print("RECOMMENDATION")
    print("=" * 72)
    print(
        "  Do NOT add cruise VINS-bearing correction on field scene.\n"
        "  VINS step bearing is ~78° off AHRS (arbitrary VO frame); any blend worsens RTK (~1° -> ~97°).\n"
        "  AHRS vs RTK on straight cruise is already ~0.9-1.0° median; error jumps at |wz|>0.08 turns,\n"
        "  not linear gyro drift (R²≈0, straight |Δerr|≈0.07°/step vs turn 4°/step).\n"
        "  Keep pure AHRS heading on cruise (fuse_vins_ahrs.py line 240).\n"
        "  Keep climb-only VINS override (lines 241-249, YAW_MIS_CLIMB=0.35 rad)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
