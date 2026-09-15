#!/usr/bin/env python3
"""Патч VINS-Mono: после system reboot якорить новое окно на last_P / yaw(last_R).

По умолчанию clearState() обнуляет failure_occur, и ветка double2vector()
с last_P0 на reboot никогда не срабатывает — траектория прыгает в (0,0,0).

    python3 tools/patch_reboot_anchor.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_reboot_anchor.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-REBOOT-ANCHOR]"

H_NEEDLE = "    bool failure_occur;"
H_PATCH = """    bool failure_occur;

    // """ + MARKER + """ last good pose survives clearState
    bool reboot_pending;
    Vector3d reboot_anchor_P;
    Matrix3d reboot_anchor_R;"""

CTOR_NEEDLE = """Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}"""

CTOR_PATCH = """Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
    // """ + MARKER + """
    reboot_pending = false;
    reboot_anchor_P.setZero();
    reboot_anchor_R.setIdentity();
}"""

FAIL_NEEDLE = """        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }"""

FAIL_PATCH = """        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            // """ + MARKER + """ keep last accepted pose
            reboot_anchor_P = last_P;
            reboot_anchor_R = last_R;
            reboot_pending = true;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }"""

INIT_NEEDLE = """                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];"""

INIT_PATCH = """                ROS_INFO("Initialization finish!");
                // """ + MARKER + """ map new origin onto last good pose
                if (reboot_pending)
                {
                    Vector3d ypr_saved = Utility::R2ypr(reboot_anchor_R);
                    Vector3d ypr_new = Utility::R2ypr(Rs[0]);
                    double y_diff = ypr_saved.x() - ypr_new.x();
                    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
                    Vector3d P0 = Ps[0];
                    for (int i = 0; i <= WINDOW_SIZE; i++)
                    {
                        Rs[i] = rot_diff * Rs[i];
                        Ps[i] = rot_diff * (Ps[i] - P0) + reboot_anchor_P;
                        Vs[i] = rot_diff * Vs[i];
                    }
                    g = rot_diff * g;
                    reboot_pending = false;
                    ROS_WARN("reboot anchored to last pose (%.2f %.2f %.2f)",
                             reboot_anchor_P.x(), reboot_anchor_P.y(), reboot_anchor_P.z());
                }
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];"""


def patch_text(text: str, needle: str, repl: str, path: Path) -> str:
    if MARKER in text and needle not in text:
        return text
    if needle not in text:
        raise SystemExit(f"Не найден якорь патча в {path}")
    return text.replace(needle, repl, 1)


def apply(vins: Path) -> None:
    h_path = vins / "vins_estimator/src/estimator.h"
    c_path = vins / "vins_estimator/src/estimator.cpp"
    for path, needle, repl in (
        (h_path, H_NEEDLE, H_PATCH),
        (c_path, CTOR_NEEDLE, CTOR_PATCH),
        (c_path, FAIL_NEEDLE, FAIL_PATCH),
        (c_path, INIT_NEEDLE, INIT_PATCH),
    ):
        bak = path.with_suffix(path.suffix + ".reboot_orig")
        if not bak.is_file():
            shutil.copy2(path, bak)
        text = path.read_text(encoding="utf-8", errors="replace")
        if MARKER in text and needle not in text:
            continue
        path.write_text(patch_text(text, needle, repl, path), encoding="utf-8")
    print(f"patched {h_path.name} + {c_path.name} (reboot-anchor)")


def revert(vins: Path) -> None:
    for rel in (
        "vins_estimator/src/estimator.h",
        "vins_estimator/src/estimator.cpp",
    ):
        path = vins / rel
        bak = path.with_suffix(path.suffix + ".reboot_orig")
        if bak.is_file():
            shutil.copy2(bak, path)
            print(f"restored {path}")
        else:
            print(f"нет бэкапа {bak}", file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()
    if not args.vins.is_dir():
        raise SystemExit(f"Нет VINS: {args.vins}")
    if args.revert:
        revert(args.vins)
        return 0
    apply(args.vins)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
