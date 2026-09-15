#!/usr/bin/env python3
"""Патч VINS-Mono: не сбрасывать трек на крейсере из-за IMU-bias.

На крейсере (USE_BARO_GROUND, track >= 20) большие Bas/Bgs — не признак
сбоя, а накопленный дрейф. Вместо reboot обрезаем bias по норме.

    python3 tools/patch_nadir_cruise_bias.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_nadir_cruise_bias.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-NADIR-CRUISE]"

OLD_BIAS = """    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }"""

NEW_BIAS = f"""    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {{
        if (USE_BARO_GROUND && f_manager.last_track_num >= 20)
        {{
            const double norm_ba = Bas[WINDOW_SIZE].norm();
            Bas[WINDOW_SIZE] *= 2.5 / norm_ba;
            ROS_WARN("{MARKER} acc bias clamped %.3f -> 2.5 (tracks=%d, no reboot)",
                     norm_ba, f_manager.last_track_num);
        }}
        else
        {{
            ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
            return true;
        }}
    }}
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {{
        if (USE_BARO_GROUND && f_manager.last_track_num >= 20)
        {{
            const double norm_bg = Bgs[WINDOW_SIZE].norm();
            Bgs[WINDOW_SIZE] *= 1.0 / norm_bg;
            ROS_WARN("{MARKER} gyr bias clamped %.3f -> 1.0 (tracks=%d, no reboot)",
                     norm_bg, f_manager.last_track_num);
        }}
        else
        {{
            ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
            return true;
        }}
    }}"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()
    path = args.vins / "vins_estimator/src/estimator.cpp"
    if not path.is_file():
        raise SystemExit(f"Нет {path}")
    bak = path.with_suffix(".cpp.cruise_bias_orig")
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
    if OLD_BIAS not in text:
        raise SystemExit("Не найден блок Bas/Bgs в failureDetection")
    if not bak.is_file():
        shutil.copy2(path, bak)
    path.write_text(text.replace(OLD_BIAS, NEW_BIAS, 1), encoding="utf-8")
    print(f"patched {path} (nadir cruise bias clamp)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
