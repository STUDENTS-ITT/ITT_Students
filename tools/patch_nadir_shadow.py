#!/usr/bin/env python3
"""VINS-Mono: shadow guard — zero XY на наборе/посадке при hover+VO blow-up.

Тень: track высокий, imu Δp≈0.05, VO 4–36 м. Нельзя резать по imu везде (imu≈0.05
на всём полёте). Shadow только в pad_phase:
  - набор: z_agl < 25 м или баро AGL < 30 м
  - посадка: horiz < 12% peak и track < 80

    python3 tools/patch_nadir_shadow.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_nadir_shadow.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

MARKER = "[VNAV-NADIR-SHADOW]"

H_OLD = """    // [VNAV-NADIR-MARG] VO blow-up → drop stale margin next frame
    bool nadir_vo_unstable_;

    // [VNAV-REBOOT-ANCHOR] last good pose survives clearState"""

H_NEW = """    // [VNAV-NADIR-MARG] VO blow-up → drop stale margin next frame
    bool nadir_vo_unstable_;
    // """ + MARKER + """ peak horizontal extent for landing pad detection
    double peak_horiz_extent_;

    // [VNAV-REBOOT-ANCHOR] last good pose survives clearState"""

CLEAR_OLD = """    skip_visual_frame_ = false;
    nadir_vo_unstable_ = false;
    imu_course_init = false;"""

CLEAR_NEW = """    skip_visual_frame_ = false;
    nadir_vo_unstable_ = false;
    peak_horiz_extent_ = 0.0;
    imu_course_init = false;"""

FAIL_OLD = """        else
        {
            const double max_step = std::max(1.5, 3.0 * std::max(imu_n, 0.05));
            if (vis_step > max_step && vis_step > 1e-6)
            {
                const double k = max_step / vis_step;
                Ps[WINDOW_SIZE] = last_P + dP * k;
                Vs[WINDOW_SIZE] *= k;
                // [VNAV-NADIR-HOLD] large VO jump — reset feature depths to INIT_DEPTH
                if (vis_step > 3.0 * max_step)
                {
                    for (auto &it : f_manager.feature)
                        if (it.estimated_depth > 0.0)
                            it.estimated_depth = INIT_DEPTH;
                    nadir_vo_unstable_ = true;
                }
                ROS_WARN("step clamped %.2f m -> %.2f m (imu Δp=%.2f, track=%d)",
                         vis_step, max_step, imu_n, f_manager.last_track_num);
                tmp_P = Ps[WINDOW_SIZE];
            }
        }"""

FAIL_NEW = """        else
        {
            const double max_step = std::max(1.5, 3.0 * std::max(imu_n, 0.05));
            const double imu_ref = std::max(imu_n, 0.05);
            const double horiz = std::hypot(Ps[WINDOW_SIZE].x() - Ps[0].x(),
                                            Ps[WINDOW_SIZE].y() - Ps[0].y());
            peak_horiz_extent_ = std::max(peak_horiz_extent_, horiz);
            const double z_agl = Ps[WINDOW_SIZE].z() - Ps[0].z();
            const bool takeoff_pad = (z_agl < 25.0) ||
                (USE_BARO_GROUND && cur_baro_h > 0.0 && cur_baro_h < 30.0);
            const bool landing_pad = (peak_horiz_extent_ > 40.0 &&
                horiz < std::max(0.12 * peak_horiz_extent_, 10.0) &&
                f_manager.last_track_num < 80);
            const bool pad_phase = takeoff_pad || landing_pad;
            const bool hover_vo = (vis_step > max_step && vis_step > 1e-6 &&
                                   imu_n < 0.10 && vis_step / imu_ref > 12.0);
            // """ + MARKER + """ тень на pad: hover + VO blow-up, не clamp 1.5 m
            if (pad_phase && hover_vo)
            {
                Ps[WINDOW_SIZE].x() = last_P.x();
                Ps[WINDOW_SIZE].y() = last_P.y();
                Vs[WINDOW_SIZE].x() = 0.0;
                Vs[WINDOW_SIZE].y() = 0.0;
                for (auto &it : f_manager.feature)
                    if (it.estimated_depth > 0.0)
                        it.estimated_depth = INIT_DEPTH;
                nadir_vo_unstable_ = true;
                ROS_WARN_THROTTLE(1.0, "%s zero XY pad=%d vis=%.2f imu=%.2f track=%d z=%.1f h=%.1f",
                                  "[VNAV-NADIR-SHADOW]", pad_phase ? 1 : 0,
                                  vis_step, imu_n, f_manager.last_track_num,
                                  z_agl, horiz);
                tmp_P = Ps[WINDOW_SIZE];
            }
            else if (vis_step > max_step && vis_step > 1e-6)
            {
                const double k = max_step / vis_step;
                Ps[WINDOW_SIZE] = last_P + dP * k;
                Vs[WINDOW_SIZE] *= k;
                // [VNAV-NADIR-HOLD] large VO jump — reset feature depths to INIT_DEPTH
                if (vis_step > 3.0 * max_step)
                {
                    for (auto &it : f_manager.feature)
                        if (it.estimated_depth > 0.0)
                            it.estimated_depth = INIT_DEPTH;
                    nadir_vo_unstable_ = true;
                }
                ROS_WARN("step clamped %.2f m -> %.2f m (imu Δp=%.2f, track=%d)",
                         vis_step, max_step, imu_n, f_manager.last_track_num);
                tmp_P = Ps[WINDOW_SIZE];
            }
        }"""

SOLVE_OLD = """            if (f_manager.last_track_num < 20)
            {
                skip_visual_frame_ = true;
                Ps[WINDOW_SIZE].x() = last_P.x();
                Ps[WINDOW_SIZE].y() = last_P.y();
                vector2double();
                ROS_WARN_THROTTLE(2.0, "%s IMU-only frame (track=%d)",
                                  "[VNAV-NADIR-HOLD]", f_manager.last_track_num);
            }"""

SOLVE_NEW = """            {
                const double vis_n = (Ps[WINDOW_SIZE] - last_P).norm();
                double imu_dn = 0.2;
                if (pre_integrations[WINDOW_SIZE] != nullptr)
                    imu_dn = pre_integrations[WINDOW_SIZE]->delta_p.norm();
                const double imu_dref = std::max(imu_dn, 0.05);
                const double max_pre = std::max(1.5, 3.0 * std::max(imu_dn, 0.05));
                const double horiz_n = std::hypot(Ps[WINDOW_SIZE].x() - Ps[0].x(),
                                                   Ps[WINDOW_SIZE].y() - Ps[0].y());
                const double z_agl_n = Ps[WINDOW_SIZE].z() - Ps[0].z();
                const bool takeoff_pad = (z_agl_n < 25.0) ||
                    (USE_BARO_GROUND && cur_baro_h > 0.0 && cur_baro_h < 30.0);
                const bool landing_pad = (peak_horiz_extent_ > 40.0 &&
                    horiz_n < std::max(0.12 * peak_horiz_extent_, 10.0) &&
                    f_manager.last_track_num < 80);
                const bool pad_phase = takeoff_pad || landing_pad;
                const bool hover_vo = (vis_n > max_pre && imu_dn < 0.10 &&
                                       vis_n / imu_dref > 12.0);
                if (pad_phase && hover_vo)
                {
                    skip_visual_frame_ = true;
                    Ps[WINDOW_SIZE].x() = last_P.x();
                    Ps[WINDOW_SIZE].y() = last_P.y();
                    vector2double();
                    ROS_WARN_THROTTLE(1.0, "%s IMU-only pre-opt (vis=%.2f imu=%.2f track=%d)",
                                      "[VNAV-NADIR-SHADOW]", vis_n, imu_dn,
                                      f_manager.last_track_num);
                }
                else if (f_manager.last_track_num < 20)
                {
                    skip_visual_frame_ = true;
                    Ps[WINDOW_SIZE].x() = last_P.x();
                    Ps[WINDOW_SIZE].y() = last_P.y();
                    vector2double();
                    ROS_WARN_THROTTLE(2.0, "%s IMU-only frame (track=%d)",
                                      "[VNAV-NADIR-HOLD]", f_manager.last_track_num);
                }
            }"""


def patch_file(path: Path, edits: list[tuple[str, str]], revert: bool) -> None:
    bak = path.with_suffix(path.suffix + ".shadow_orig")
    text = path.read_text(encoding="utf-8", errors="replace")
    if revert:
        if not bak.is_file():
            raise SystemExit(f"нет бэкапа {bak}")
        shutil.copy2(bak, path)
        print(f"restored {path}")
        return
    if MARKER in text and "peak_horiz_extent_" in text:
        print(f"уже патчен {path}")
        return
    if MARKER in text:
        raise SystemExit(f"{path}: старый shadow-патч — сначала --revert")
    if not bak.is_file():
        shutil.copy2(path, bak)
    for old, new in edits:
        if old not in text:
            raise SystemExit(f"Не найден якорь в {path}:\n{old[:200]}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()
    root = args.vins
    if args.revert:
        patch_file(root / "vins_estimator/src/estimator.cpp", [], True)
        h_path = root / "vins_estimator/src/estimator.h"
        if h_path.is_file() and MARKER in h_path.read_text():
            hbak = h_path.with_suffix(".h.shadow_orig")
            if hbak.is_file():
                shutil.copy2(hbak, h_path)
                print(f"restored {h_path}")
        return 0

    h_path = root / "vins_estimator/src/estimator.h"
    hbak = h_path.with_suffix(".h.shadow_orig")
    if not hbak.is_file():
        shutil.copy2(h_path, hbak)
    h_text = h_path.read_text(encoding="utf-8", errors="replace")
    if "peak_horiz_extent_" not in h_text:
        if H_OLD not in h_text:
            raise SystemExit(f"Не найден якорь в {h_path}")
        h_path.write_text(h_text.replace(H_OLD, H_NEW, 1), encoding="utf-8")
        print(f"patched {h_path}")

    est = root / "vins_estimator/src/estimator.cpp"
    est_text = est.read_text(encoding="utf-8", errors="replace")
    if MARKER in est_text:
        patch_file(est, [], True)
        est_text = est.read_text(encoding="utf-8", errors="replace")
    if CLEAR_OLD in est_text:
        est_text = est_text.replace(CLEAR_OLD, CLEAR_NEW, 1)
    patch_file(est, [(FAIL_OLD, FAIL_NEW), (SOLVE_OLD, SOLVE_NEW)], False)
    if CLEAR_NEW in est.read_text() or True:
        pass
    # ensure clearState patched
    est_text = est.read_text(encoding="utf-8", errors="replace")
    if "peak_horiz_extent_ = 0.0" not in est_text and CLEAR_OLD in est_text:
        est.write_text(est_text.replace(CLEAR_OLD, CLEAR_NEW, 1), encoding="utf-8")
        print("patched clearState peak_horiz_extent_")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
