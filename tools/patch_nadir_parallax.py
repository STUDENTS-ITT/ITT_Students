#!/usr/bin/env python3
"""Патч nadir-параллакса VINS-Mono (run8e).

Минимальный рабочий вариант: run2-параллакс + блок кейфрейма при |dR|>0.07 рад.
Adaptive comp/raw (run8b–d) на 90 с хуже run2; полный прогон — run8e.

    python3 patch_nadir_parallax.py --vins ~/catkin_ws/src/VINS-Mono --mode rot-gate
    python3 patch_nadir_parallax.py --vins ... --revert
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-NADIR-PARALLAX]"
GYRO_MARKER = "[VNAV-NADIR-GYRO-GATE]"
ROT_GATE = """
        const double avg_parallax = parallax_sum / parallax_num;
        // """ + MARKER + """ rot-gate: не кейфрейм на развороте
        const int r_i = frame_count - 2;
        const int r_j = frame_count - 1;
        const Matrix3d dR = Rs[r_j].transpose() * Rs[r_i];
        const double cos_a = std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) * 0.5));
        const double rot_rad = std::acos(cos_a);
        if (rot_rad > 0.07)
            return false;
        // """ + GYRO_MARKER + """: блок кейфрейма при высокой угл. скорости (≈>0.5 рад/с)
        if (frame_count >= 3) {
            const int r_k = frame_count - 3;
            const Matrix3d dR2 = Rs[r_j].transpose() * Rs[r_k];
            const double cos_a2 = std::max(-1.0, std::min(1.0, (dR2.trace() - 1.0) * 0.5));
            const double rot2_rad = std::acos(cos_a2);
            // 2 интервала ≈ 0.2 с при 10 Гц → порог 0.10 рад ≈ 0.5 рад/с
            if (rot2_rad > 0.10)
                return false;
        }
        return avg_parallax >= MIN_PARALLAX;"""

OLD_RETURN = """        return parallax_sum / parallax_num >= MIN_PARALLAX;"""


def apply_rot_gate(text: str) -> str:
    if GYRO_MARKER in text:
        return text
    if MARKER in text and "return avg_parallax >= MIN_PARALLAX;" in text:
        # Уже есть rot-gate без gyro — заменить хвост
        old = text[text.index(MARKER):]
        if "return avg_parallax >= MIN_PARALLAX;" in old:
            start = text.rindex("const double avg_parallax")
            end = text.index("return avg_parallax >= MIN_PARALLAX;", start) + len("return avg_parallax >= MIN_PARALLAX;")
            return text[:start] + ROT_GATE.strip() + text[end:]
    if MARKER in text:
        return text
    if OLD_RETURN not in text:
        raise SystemExit("Не найден return parallax_sum в addFeatureCheckParallax")
    return text.replace(OLD_RETURN, ROT_GATE)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, required=True)
    ap.add_argument("--mode", choices=["rot-gate"], default="rot-gate")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()
    path = args.vins / "vins_estimator/src/feature_manager.cpp"
    bak = path.with_suffix(".cpp.orig")
    if args.revert and bak.is_file():
        shutil.copy2(bak, path)
        print(f"restored {path}")
        return 0
    if not bak.is_file():
        shutil.copy2(path, bak)
    path.write_text(apply_rot_gate(path.read_text()))
    print(f"patched {path} (rot-gate)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
