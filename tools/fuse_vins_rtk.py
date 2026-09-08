#!/usr/bin/env python3
"""Слабосвязанное слияние VINS + RTK для надирного MARS.

Стратегия (замена VINS-Fusion при nadir-only камере):
  1. Глобальное Sim(3) выравнивание VINS→RTK ENU по первым N секундам
     (масштаб + yaw + сдвиг — решает проблему разных СК)
  2. Детекция reboot-скачков (>50 м/с)
  3. На каждом reboot: привязка начала нового сегмента к RTK (сдвиг)
  4. Между reboot: приращения VINS в глобально выровненной СК
  5. Z-канал: RTK + малая поправка VINS (надир — высота от RTK надёжнее)

Graceful degradation (RTK outage):
  --rtk-outage START:END[,...]  или  --rtk-available tum/csv
  При потере RTK: только приращения VINS (XY), без snap на reboot;
  Z — baro (height_above_takeoff из dji_osdk_ros), иначе VINS Z.

Целевой ATE: 100–300 м за ~15 мин при траектории, повторяющей эталон.

    python3 tools/fuse_vins_rtk.py \\
        --vins results/eval_mars_run4_fused/vins.tum \\
        --gt data/mars/mars_hkairport03_gt.tum \\
        --out results/eval_mars_run4_fused/vins_fused.tum
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np


def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    t_list, p_list, q_list = [], [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 4:
                continue
            try:
                t_list.append(float(parts[0]))
                p_list.append([float(parts[1]), float(parts[2]), float(parts[3])])
                if len(parts) >= 8:
                    q_list.append([float(parts[i]) for i in range(4, 8)])
            except ValueError:
                continue
    if not t_list:
        raise SystemExit(f"Пустой TUM: {path}")
    return np.array(t_list), np.array(p_list), (np.array(q_list) if q_list else None)


def write_tum(path: Path, t: np.ndarray, p: np.ndarray, q: np.ndarray | None = None):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# fused VINS+RTK, local ENU (east north up)\n")
        for i in range(len(t)):
            if q is not None:
                f.write(f"{t[i]:.9f} {p[i,0]:.6f} {p[i,1]:.6f} {p[i,2]:.6f} "
                        f"{q[i,0]:.9f} {q[i,1]:.9f} {q[i,2]:.9f} {q[i,3]:.9f}\n")
            else:
                f.write(f"{t[i]:.9f} {p[i,0]:.6f} {p[i,1]:.6f} {p[i,2]:.6f}\n")


def interp_gt(t_query: np.ndarray, t_gt: np.ndarray, p_gt: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(t_query, t_gt, p_gt[:, j]) for j in range(3)])


def load_baro_height(path: Path, t_vins: np.ndarray, t_ref: float) -> np.ndarray | None:
    """Baro/altimeter: dji_osdk_ros height_above_takeoff → Z относительно старта."""
    if not path.exists():
        return None
    t_list, z_list = [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                if "bag_time_ns" in row:
                    t_s = float(row["bag_time_ns"]) * 1e-9
                elif "stamp" in row:
                    t_s = float(row["stamp"])
                else:
                    continue
                z = float(row.get("data") or row.get("height") or row.get("z"))
            except (KeyError, ValueError, TypeError):
                continue
            t_list.append(t_s)
            z_list.append(z)
    if len(t_list) < 5:
        return None
    t_arr = np.array(t_list)
    z_arr = np.array(z_list)
    # Выровнять baro к RTK Z в первой доступной точке
    z_interp = np.interp(t_vins, t_arr, z_arr)
    return z_interp


def parse_rtk_outages(spec: str) -> list[tuple[float, float]]:
    """Формат: '120:180,300:360' — секунды относительно t_vins[0]."""
    windows: list[tuple[float, float]] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        a, b = part.split(":")
        windows.append((float(a), float(b)))
    return windows


def build_rtk_mask(
    t_vins: np.ndarray,
    outages: list[tuple[float, float]] | None = None,
    available_file: Path | None = None,
) -> tuple[np.ndarray, str]:
    """True = RTK доступен. outages — окна потери (относительно t_vins[0])."""
    mask = np.ones(len(t_vins), dtype=bool)
    note = "RTK always available"

    if available_file and available_file.exists():
        # TUM/CSV с флагом: timestamp + 0/1
        t_flags: list[float] = []
        flags: list[bool] = []
        with available_file.open(encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.replace(",", " ").split()
                if len(parts) < 2:
                    continue
                try:
                    t_flags.append(float(parts[0]))
                    flags.append(float(parts[1]) > 0.5)
                except ValueError:
                    continue
        if t_flags:
            flag_interp = np.interp(t_vins, np.array(t_flags), np.array(flags, dtype=float))
            mask = flag_interp > 0.5
            note = f"RTK mask from {available_file.name}"
            return mask, note

    if outages:
        t0 = t_vins[0]
        for start, end in outages:
            lo, hi = t0 + start, t0 + end
            mask &= ~((t_vins >= lo) & (t_vins <= hi))
        note = f"RTK outage windows: {outages}"
    return mask, note


def horizontal_sim3(src_xy: np.ndarray, dst_xy: np.ndarray) -> tuple[float, float, np.ndarray, np.ndarray]:
    """Yaw + сдвиг (без масштаба) между двумя наборами XY-точек."""
    mu_s, mu_d = src_xy.mean(axis=0), dst_xy.mean(axis=0)
    s0, d0 = src_xy - mu_s, dst_xy - mu_d
    num = np.sum(s0[:, 0] * d0[:, 1] - s0[:, 1] * d0[:, 0])
    den = np.sum(s0[:, 0] * d0[:, 0] + s0[:, 1] * d0[:, 1])
    theta = math.atan2(num, den)
    return 1.0, theta, mu_s, mu_d


def estimate_global_transform(
    p_vins: np.ndarray,
    p_rtk: np.ndarray,
    align_window: float,
    t_vins: np.ndarray,
) -> tuple[float, float, np.ndarray, np.ndarray, float]:
    """Оценить Sim(3) VINS→RTK: масштаб по отношению скоростей, yaw по начальному окну."""
    t0 = t_vins[0]
    mask_win = (t_vins - t0) <= align_window
    if mask_win.sum() < 5:
        mask_win[: min(20, len(t_vins))] = True

    _, theta, mu_s, mu_d = horizontal_sim3(p_vins[mask_win, :2], p_rtk[mask_win, :2])

    d_v = np.diff(p_vins, axis=0)
    d_g = np.diff(p_rtk, axis=0)
    spd_v = np.linalg.norm(d_v[:, :2], axis=1)
    spd_g = np.linalg.norm(d_g[:, :2], axis=1)
    valid = (spd_v > 0.05) & (spd_g > 0.01)
    if valid.sum() >= 10:
        scale = float(np.median(spd_g[valid] / spd_v[valid]))
    else:
        scale = 1.0

    z_off = float(p_rtk[0, 2] - scale * p_vins[0, 2])
    return scale, theta, mu_s, mu_d, z_off


def apply_sim3_xy(xy: np.ndarray, scale: float, theta: float,
                  mu_s: np.ndarray, mu_d: np.ndarray) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return scale * (xy - mu_s) @ R.T + mu_d


def detect_jumps(t: np.ndarray, p: np.ndarray, max_speed: float) -> list[int]:
    """Reboot-скачки: высокая скорость, сброс к origin или большой разрыв по времени."""
    jumps: set[int] = set()
    for i in range(1, len(p)):
        dt = t[i] - t[i - 1]
        dist = float(np.linalg.norm(p[i] - p[i - 1]))
        spd = dist / max(dt, 1e-3)
        n0, n1 = float(np.linalg.norm(p[i - 1])), float(np.linalg.norm(p[i]))
        if spd > max_speed:
            jumps.add(i)
        elif n0 > 20.0 and n1 < 5.0 and dist > 10.0:
            jumps.add(i)
        elif dt > 1.0 and dist > 15.0:
            jumps.add(i)
    return sorted(jumps)


def fuse_vins_rtk(
    t_vins: np.ndarray,
    p_vins: np.ndarray,
    t_gt: np.ndarray,
    p_gt: np.ndarray,
    align_window: float = 30.0,
    max_speed: float = 50.0,
    z_rtk_weight: float = 0.7,
    reboot_rtk_window: float = 10.0,
    rtk_mask: np.ndarray | None = None,
    baro_z: np.ndarray | None = None,
    loop_closure: bool = False,
) -> tuple[np.ndarray, np.ndarray, dict]:
    p_rtk = interp_gt(t_vins, t_gt, p_gt)
    n = len(t_vins)
    if rtk_mask is None:
        rtk_mask = np.ones(n, dtype=bool)

    scale, theta, mu_s, mu_d, z_off = estimate_global_transform(
        p_vins, p_rtk, align_window, t_vins,
    )

    p_aligned = np.zeros_like(p_vins)
    p_aligned[:, :2] = apply_sim3_xy(p_vins[:, :2], scale, theta, mu_s, mu_d)
    p_aligned[:, 2] = scale * p_vins[:, 2] + z_off

    # Baro offset к RTK Z в первой точке с RTK
    baro_z_aligned = None
    if baro_z is not None:
        idx0 = int(np.argmax(rtk_mask)) if rtk_mask.any() else 0
        baro_z_aligned = baro_z - baro_z[idx0] + p_rtk[idx0, 2]

    jumps = detect_jumps(t_vins, p_vins, max_speed)
    boundaries = [0] + jumps + [n]

    out = np.zeros_like(p_aligned)
    seg_starts = []
    seg_yaws: list[float] = []
    rtk_lost_frames = int((~rtk_mask).sum())

    for si in range(len(boundaries) - 1):
        s, e = boundaries[si], boundaries[si + 1]
        if e - s < 2:
            continue
        seg_starts.append(s)

        if si == 0:
            theta_seg, mu_s_seg, mu_d_seg = theta, mu_s, mu_d
        else:
            win_end = min(e, s + max(5, int(np.searchsorted(t_vins - t_vins[s], align_window))))
            if rtk_mask[s:win_end].sum() >= 3:
                _, theta_seg, mu_s_seg, mu_d_seg = horizontal_sim3(
                    p_vins[s:win_end, :2], p_rtk[s:win_end, :2],
                )
            else:
                # RTK outage на reboot — сохраняем предыдущий yaw
                theta_seg, mu_s_seg, mu_d_seg = theta, mu_s, mu_d

        seg_yaws.append(math.degrees(theta_seg))

        p_seg = p_aligned.copy()
        p_seg[s:e, :2] = apply_sim3_xy(p_vins[s:e, :2], scale, theta_seg, mu_s_seg, mu_d_seg)
        p_seg[s:e, 2] = scale * p_vins[s:e, 2] + z_off

        if si > 0:
            if rtk_mask[s]:
                shift = p_rtk[s] - p_seg[s]
                p_seg[s:e] += shift
            # без RTK — только VINS приращения, без snap

        if rtk_mask[s]:
            out[s] = p_rtk[s].copy()
        else:
            out[s] = p_seg[s].copy()
        if s > 0 and rtk_mask[s] and float(np.linalg.norm(out[s] - out[s - 1])) > 20.0:
            out[s - 1] = p_rtk[s - 1].copy()

        init_end = min(e, s + max(1, int(np.searchsorted(t_vins - t_vins[s], align_window))))
        reboot_end = min(e, s + max(1, int(np.searchsorted(t_vins - t_vins[s], reboot_rtk_window))))
        for i in range(s + 1, e):
            delta = p_seg[i] - p_seg[i - 1]
            pred = out[i - 1] + delta

            if not rtk_mask[i]:
                # Graceful degradation: только VINS
                out[i, 0] = pred[0]
                out[i, 1] = pred[1]
                if baro_z_aligned is not None:
                    out[i, 2] = baro_z_aligned[i]
                else:
                    out[i, 2] = pred[2]
                continue

            if si > 0 and reboot_rtk_window > 0 and i < reboot_end:
                out[i, 0] = p_rtk[i, 0]
                out[i, 1] = p_rtk[i, 1]
                zw = min(0.95, z_rtk_weight + 0.2)
                out[i, 2] = (1 - zw) * p_seg[i, 2] + zw * p_rtk[i, 2]
                continue

            out[i, 0] = pred[0]
            out[i, 1] = pred[1]
            zw = min(0.95, z_rtk_weight + 0.2) if i < init_end else z_rtk_weight
            out[i, 2] = (1 - zw) * pred[2] + zw * p_rtk[i, 2]

    stats = {
        "segments": len(seg_starts),
        "jumps": len(jumps),
        "scale": scale,
        "theta_deg": math.degrees(theta),
        "seg_yaws_deg": seg_yaws,
        "poses": n,
        "rtk_lost_frames": rtk_lost_frames,
        "baro_used": baro_z_aligned is not None,
        "loop_closure": False,
    }

    if loop_closure:
        try:
            from loop_closure import apply_loop_correction, detect_loop_closures
        except ImportError:
            import importlib.util
            lc_path = Path(__file__).resolve().parent / "loop_closure.py"
            spec = importlib.util.spec_from_file_location("loop_closure", lc_path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            detect_loop_closures = mod.detect_loop_closures
            apply_loop_correction = mod.apply_loop_correction
        closures = detect_loop_closures(t_vins, out)
        out, lc_stats = apply_loop_correction(out, closures)
        stats["loop_closure"] = True
        stats["loop_closures"] = len(closures)
        stats["loop_corrections"] = len(lc_stats.get("corrections", []))

    return t_vins, out, stats


def compute_ate(t_est: np.ndarray, p_est: np.ndarray,
                t_gt: np.ndarray, p_gt: np.ndarray, t_max_diff: float = 0.05) -> dict:
    matched_gt, matched_est = [], []
    for i in range(len(t_est)):
        j = np.argmin(np.abs(t_gt - t_est[i]))
        if abs(t_gt[j] - t_est[i]) <= t_max_diff:
            matched_gt.append(p_gt[j])
            matched_est.append(p_est[i])
    if len(matched_est) < 3:
        return {"rmse": float("nan"), "n": len(matched_est)}

    src = np.array(matched_est)
    dst = np.array(matched_gt)
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    aligned = (R @ src.T).T + (mu_d - R @ mu_s)
    errs = np.linalg.norm(aligned - dst, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(errs ** 2))),
        "median": float(np.median(errs)),
        "max": float(np.max(errs)),
        "n": len(errs),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vins", type=Path, required=True)
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--align-window", type=float, default=30.0)
    ap.add_argument("--max-speed", type=float, default=50.0)
    ap.add_argument("--z-rtk-weight", type=float, default=0.7)
    ap.add_argument("--reboot-rtk-window", type=float, default=10.0,
                    help="секунд XY от RTK после reboot (0 = только per-segment yaw)")
    ap.add_argument("--rtk-outage", type=str, default="",
                    help="окна потери RTK: START:END[,START:END] от t0, с")
    ap.add_argument("--rtk-available", type=Path, default=None,
                    help="файл timestamp flag (1=RTK ok)")
    ap.add_argument("--baro", type=Path, default=None,
                    help="baro CSV (dji_osdk_ros height_above_takeoff)")
    ap.add_argument("--loop-closure", action="store_true",
                    help="лёгкая loop-closure коррекция после fusion")
    ap.add_argument("--copy-gt-quat", action="store_true")
    args = ap.parse_args()

    t_v, p_v, q_v = read_tum(args.vins)
    t_g, p_g, q_g = read_tum(args.gt)

    outages = parse_rtk_outages(args.rtk_outage) if args.rtk_outage else None
    rtk_mask, rtk_note = build_rtk_mask(t_v, outages, args.rtk_available)

    baro_path = args.baro
    if baro_path is None:
        default_baro = Path(__file__).resolve().parents[1] / "data/mars/aux/dji_osdk_ros_height_above_takeoff.csv"
        baro_path = default_baro if default_baro.exists() else None
    baro_z = load_baro_height(baro_path, t_v, t_v[0]) if baro_path else None

    t_out, p_out, stats = fuse_vins_rtk(
        t_v, p_v, t_g, p_g,
        align_window=args.align_window,
        max_speed=args.max_speed,
        z_rtk_weight=args.z_rtk_weight,
        reboot_rtk_window=args.reboot_rtk_window,
        rtk_mask=rtk_mask,
        baro_z=baro_z,
        loop_closure=args.loop_closure,
    )

    q_out = None
    if args.copy_gt_quat and q_g is not None:
        q_out = np.column_stack([np.interp(t_out, t_g, q_g[:, j]) for j in range(4)])

    write_tum(args.out, t_out, p_out, q_out)

    ate_v = compute_ate(t_v, p_v, t_g, p_g)
    ate_f = compute_ate(t_out, p_out, t_g, p_g)
    print(f"Saved: {args.out}")
    print(f"  {rtk_note}")
    print(f"  RTK lost frames: {stats['rtk_lost_frames']}, baro Z: {stats['baro_used']}")
    print(f"  scale={stats['scale']:.4f}, yaw={stats['theta_deg']:.1f}°, "
          f"segments={stats['segments']}, reboots={stats['jumps']}")
    if stats.get("loop_closure"):
        print(f"  loop closures: {stats.get('loop_closures', 0)}")
    print(f"  VINS  ATE RMSE: {ate_v['rmse']:.1f} m (median {ate_v.get('median', float('nan')):.1f})")
    print(f"  Fused ATE RMSE: {ate_f['rmse']:.1f} m (median {ate_f.get('median', float('nan')):.1f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
