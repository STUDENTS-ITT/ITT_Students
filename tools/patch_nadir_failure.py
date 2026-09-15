#!/usr/bin/env python3
"""Патч VINS-Mono: ошибки из отчёта HKairport03, аналоги в estimator.

В VO-отчёте: (1) зеркало OpenCV-Y, (2) единичный масштаб, (3) STEP_LENGTH_LIMIT
вместо сброса. В VINS то же проявляется как failureDetection:

  - |Δz| > 1 м → reboot  (взлёт/посадка, §5 отчёта)
  - |ΔP| > 5 м → reboot  («выброс длины шага», §6)

Исправление: обрезать шаг, направление сохранить, reboot только по IMU-bias.
Cruise IMU-bias clamp: tools/patch_nadir_cruise_bias.py.

    python3 tools/patch_nadir_failure.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_nadir_failure.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-NADIR-STEP]"

OLD_FAIL = """    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true; 
    }"""

NEW_FAIL = """    Vector3d tmp_P = Ps[WINDOW_SIZE];
    // """ + MARKER + """ STEP_LENGTH_LIMIT: clamp, do not reboot (HKairport report §6)
    {
        Vector3d dP = tmp_P - last_P;
        const double vis_step = dP.norm();
        double imu_n = 0.2;
        if (pre_integrations[WINDOW_SIZE] != nullptr)
            imu_n = pre_integrations[WINDOW_SIZE]->delta_p.norm();
        const double max_step = std::max(1.5, 3.0 * std::max(imu_n, 0.05));
        if (vis_step > max_step && vis_step > 1e-6)
        {
            Ps[WINDOW_SIZE] = last_P + dP * (max_step / vis_step);
            ROS_WARN("step clamped %.2f m -> %.2f m (imu Δp=%.2f, no reboot)",
                     vis_step, max_step, imu_n);
            tmp_P = Ps[WINDOW_SIZE];
        }
        const double dz = tmp_P.z() - last_P.z();
        const double max_dz = USE_BARO_GROUND
            ? std::max(3.0, 3.0 * std::max(imu_n, 0.05))
            : std::max(0.5, 3.0 * std::max(imu_n, 0.05));
        if (std::abs(dz) > max_dz)
        {
            Ps[WINDOW_SIZE].z() = last_P.z() + (dz > 0.0 ? max_dz : -max_dz);
            ROS_WARN("z step clamped %.2f m (takeoff/nadir, no reboot)", dz);
        }
    }"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()
    path = args.vins / "vins_estimator/src/estimator.cpp"
    if not path.is_file():
        raise SystemExit(f"Нет {path}")
    bak = path.with_suffix(".cpp.failure_orig")
    if args.revert:
        if bak.is_file():
            shutil.copy2(bak, path)
            print(f"restored {path}")
            return 0
        raise SystemExit(f"нет бэкапа {bak}")
    text = path.read_text(encoding="utf-8", errors="replace")
    if MARKER in text:
        print(f"уже патчен {path}")
        return 0
    if OLD_FAIL not in text:
        raise SystemExit("Не найден блок big translation / big z в failureDetection")
    if not bak.is_file():
        shutil.copy2(path, bak)
    path.write_text(text.replace(OLD_FAIL, NEW_FAIL, 1), encoding="utf-8")
    print(f"patched {path} (nadir step clamp)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
