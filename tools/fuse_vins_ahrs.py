#!/usr/bin/env python3
"""Курс траектории VINS от бортового AHRS DJI, не от RTK.

Livox /imu0 на HKairport03 почти не накапливает рыскание (~230° за крейсер),
а визуал VINS либо идёт прямо, либо спиралит. Кватернион /dji_osdk_ros/imu —
это AHRS М100/M300 (магнитометр+гиро), не RTK. Им в отчёте крутили
гомографию: шаг в метрах × yaw → овал.

Здесь то же для выхода VINS:
  длина шага XY — из VINS (масштаб Opus / ground prior);
  направление — yaw AHRS;
  Z — как пришло (баро-фьюз снаружи).

    python3 tools/fuse_vins_ahrs.py \\
        --vins results/eval_mars_TAG/vins.tum \\
        --imu data/mars/aux/dji_osdk_ros_imu.csv \\
        --out results/eval_mars_TAG/vins_ahrs.tum
"""
from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import numpy as np

from mars_stamp import detect_camera_offset, load_dji_imu_stamps

CLAMP_LIMIT = 1.5
CLAMP_SPEED_SCALE = 0.876  # speed×dt vs clamp 1.5 m; ~61% clamp / ~3740 m path
LANDING_RAW_FRAC = 0.35  # freeze/посадка только в последней трети сырого пути
GOOD_SPEED_N = 15
GOOD_SPEED_HISTORY = 30
LEG_SPEED_MIN = 5
LEG_WZ_STRAIGHT = 0.04
LEG_WZ_RESET = 0.08
YAW_MIS_CLIMB = 0.35  # рад: на наборе AHRS vs VINS — тень/мало текстуры
VINS_YAW_MAX_STEP = 0.8  # м: курс VINS только на коротких good-шагах (тень)
SHADOW_CLAMP_FRAC = 0.40
SHADOW_WZ = 0.04
SHADOW_DYAW = 0.06
SHADOW_CLAMP_WINDOW = 40


def load_tum(path: Path) -> np.ndarray:
    return np.loadtxt(path, comments="#")


def is_clamp_step(step: float) -> bool:
    """VINS clamp/saturation: обрезка к ~1.5 м или длинный крейсерский шаг."""
    return step > 1.2 or abs(step - CLAMP_LIMIT) < 0.05


def is_good_raw(step: float, clamp_step: bool) -> bool:
    return (step > 0.25) and (step < 1.15) and (not clamp_step)


def climb_path_threshold(peak_path: float) -> float:
    """Набор: ~3% пройденного пути, но не меньше ~52 м (эквивалент t<55 s на field)."""
    return max(0.029 * max(peak_path, 50.0), 52.0)


def vins_step_bearing(p_xy: np.ndarray, i: int, fallback: float) -> float:
    d = p_xy[i + 1] - p_xy[i]
    n = float(np.linalg.norm(d))
    return float(np.arctan2(d[1], d[0])) if n > 1e-6 else fallback


def yaw_mismatch(a: float, b: float) -> float:
    return float(abs(((b - a + np.pi) % (2.0 * np.pi)) - np.pi))


def landing_anchor(home_xy: np.ndarray, start: np.ndarray, best_dist: float, max_extent: float) -> np.ndarray:
    """RTK field возвращается в (0,0); если уже близко — якорь у старта."""
    snap_r = max(60.0, 0.12 * max_extent)
    if best_dist < snap_r:
        return start.copy()
    return home_xy.copy()


def shadow_proxy(
    dist_home: float,
    max_extent: float,
    clamp_frac_win: float,
    wz_i: float,
    dyaw: float,
) -> bool:
    """Тень/мало текстуры: hover + clamp + близко к старту/площадке."""
    near_pad = dist_home < max(0.15 * max_extent, 15.0)
    hover = abs(wz_i) < SHADOW_WZ and dyaw < SHADOW_DYAW
    return hover and clamp_frac_win > SHADOW_CLAMP_FRAC and near_pad


def estimate_speed(
    good_speeds: deque[float],
    clamp_frac: float,
) -> tuple[float, str]:
    if len(good_speeds) >= GOOD_SPEED_N:
        speed = min(float(np.median(np.array(good_speeds))), 10.0)
        return speed, f"медиана good-шагов на крейсере ({speed:.1f} м/с)"
    if clamp_frac > 0.25:
        return 8.5, "крейсер 8.5 м/с (до накопления good-шагов после набора)"
    return 0.0, "стоп: нет текстурных кадров, крейсер не подставляем"


def rebuild_xy(
    p_xy: np.ndarray,
    yaw: np.ndarray,
    dt: np.ndarray,
    cruise_mps: float = 0.0,
    t_rel: np.ndarray | None = None,
    wz: np.ndarray | None = None,
) -> np.ndarray:
    """Длина шага — VINS, курс — AHRS. После возврата к старту — стоп (посадка).

    Набор: пока интегрированный путь < climb_path_threshold(peak_path), clamp
    обнуляем (без секунд). На good-шагах при расхождении AHRS/VINS > YAW_MIS_CLIMB
    курс берём из bearing VINS (тень на ключевых точках). Скорость крейсера — из
    good-шагов после набора. Пороги посадки — от max_extent.
    """
    raw = np.linalg.norm(np.diff(p_xy, axis=0), axis=1)
    dti = dt[: len(raw)]
    clamp_mask = np.array([is_clamp_step(float(s)) for s in raw])
    clamp_frac = float(np.mean(clamp_mask)) if len(raw) else 0.0
    if t_rel is None:
        t_rel = np.zeros(len(p_xy))
    if wz is None:
        wz = np.zeros(len(p_xy))

    out = np.zeros_like(p_xy)
    out[0] = p_xy[0]
    start = out[0].copy()
    last_good = 0.0
    path_len = 0.0
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

    speed, speed_src = estimate_speed(good_speeds, clamp_frac)
    if cruise_mps > 0.0 and speed <= 0.0:
        speed = float(cruise_mps)
        speed_src = f"крейсер {speed:.1f} м/с (CLI)"
    print(f"  масштаб шага: {speed_src}, clamp {100 * clamp_frac:.0f}%")

    for i in range(len(p_xy) - 1):
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

        step = float(raw[i])
        clamp_step = is_clamp_step(step)
        clamp_hist.append(clamp_step)
        clamp_frac_win = float(np.mean(clamp_hist)) if clamp_hist else 0.0
        dyaw = abs(float(yaw[i]) - prev_yaw)
        unreliable = shadow_proxy(dist_home, max_extent, clamp_frac_win, float(wz[i]), dyaw)
        if abs(float(wz[i])) >= LEG_WZ_RESET:
            leg_speeds.clear()

        remain_raw = float(raw[i:].sum()) if i < len(raw) else 0.0
        landing_phase = remain_raw < LANDING_RAW_FRAC * raw_path_total and max_extent > far
        snap_r = max(60.0, 0.12 * max_extent)

        if (
            freeze_idx < 0
            and landing_phase
            and returned
            and dist_home < near_r
            and len(home_ring) >= 5
        ):
            # Посадка: якорь у базы до «отлёта» (тень/clamp на финале)
            freeze_idx = i + 1
            landing_xy = landing_anchor(
                np.median(np.array(home_ring), axis=0), start, dist_home, max_extent
            )
        elif freeze_idx < 0 and landing_phase and returned and best_home < near_r:
            leaving = dist_home > best_home + depart_r
            if leaving:
                freeze_idx = i + 1
                if len(home_ring) >= 5:
                    home = np.median(np.array(home_ring), axis=0)
                else:
                    home = best_home_xy.copy()
                landing_xy = landing_anchor(home, start, best_home, max_extent)

        # Поздний минимум к старту: последний квартал сырого пути (не секунды)
        late = remain_raw < 0.25 * raw_path_total and max_extent > far
        if late:
            if dist_home < late_best:
                late_best = dist_home
                late_xy = out[i].copy()
            elif (
                freeze_idx < 0
                and late_best < snap_r
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
        # cruise: AHRS (~1° vs RTK на прямых); VINS bearing только на наборе
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
            if returned and landing_phase:
                step = 0.0
            elif climb:
                step = 0.5 * CLAMP_LIMIT
            elif speed > 0.0:
                spd = speed
                if abs(float(wz[i])) < LEG_WZ_STRAIGHT and len(leg_speeds) >= LEG_SPEED_MIN:
                    spd = min(float(np.median(np.array(leg_speeds))), 10.0)
                dt_i = float(dt[i])
                step = CLAMP_SPEED_SCALE * spd * dt_i if dt_i > 1e-4 else last_good
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
                    if abs(float(wz[i])) < LEG_WZ_STRAIGHT:
                        leg_speeds.append(gs)

        out[i + 1] = out[i] + step * np.array([np.cos(heading), np.sin(heading)])
        path_len += step
        prev_yaw = float(yaw[i])

    if freeze_idx >= 0:
        print(
            f"  посадка: стоп t={t_rel[freeze_idx]:.0f} с, "
            f"мин. дист. {best_home:.1f} м, якорь ({landing_xy[0]:.1f}, {landing_xy[1]:.1f})"
        )
    return out


def write_tum(path: Path, src: np.ndarray, xy: np.ndarray) -> None:
    out = src.copy()
    out[:, 1] = xy[:, 0]
    out[:, 2] = xy[:, 1]
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, out, fmt="%.9f")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, required=True)
    ap.add_argument("--imu", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument(
        "--cruise-mps",
        type=float,
        default=0.0,
        help="запасная скорость, м/с; 0 = не подставлять крейсер после clamp",
    )
    ap.add_argument(
        "--camera-stamps",
        type=Path,
        default=None,
        help="timestamps.csv для авто-выравнивания DJI→camera epoch",
    )
    ap.add_argument(
        "--camera-offset",
        type=float,
        default=None,
        help="сек: вычесть из DJI header.stamp (auto если не задан)",
    )
    ap.add_argument(
        "--keep-xy",
        action="store_true",
        help="не переписывать курс: XY уже в ENU (планарная гомография в VINS)",
    )
    args = ap.parse_args()

    src = load_tum(args.vins)
    t_v = src[:, 0]
    cam_off = args.camera_offset
    if cam_off is None:
        cam_off = detect_camera_offset(args.camera_stamps, args.imu, float(t_v[0]))
    if abs(cam_off) > 0.05:
        print(f"  DJI→camera offset: {cam_off:.3f} с (header.stamp)")
    t_i, yaw_i, wz_i = load_dji_imu_stamps(args.imu, camera_offset=cam_off)
    yaw = np.interp(t_v, t_i, yaw_i)
    wz = np.clip(np.interp(t_v, t_i, wz_i), -0.5, 0.5)
    dt = np.diff(t_v, append=t_v[-1] + 0.1)
    t_rel = t_v - t_v[0]
    if args.keep_xy:
        print("  keep-xy: курс уже от AHRS внутри VINS, шаги не крутим")
        xy = src[:, 1:3].copy()
    else:
        xy = rebuild_xy(src[:, 1:3], yaw, dt, args.cruise_mps, t_rel=t_rel, wz=wz)
    write_tum(args.out, src, xy)

    plen_v = float(np.linalg.norm(np.diff(src[:, 1:3], axis=0), axis=1).sum())
    plen_a = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    print(f"AHRS yaw (не RTK): {args.imu.name}")
    print(f"  poses {len(src)}, путь VINS {plen_v:.1f} м, после курса {plen_a:.1f} м")
    print(f"  записано {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
