#!/usr/bin/env python3
"""Патч VINS-Mono: INIT_DEPTH из yaml + отсев вырожденной триангуляции.

На надире MARS земля ~80–130 м, штатный INIT_DEPTH=5 м даёт масштаб в 10–20 раз
меньше RTK. Если SVD-глубина не в окрестности init_depth — берём init_depth
(плоскость + высота баро/типичный клиренс).

    python3 tools/patch_nadir_init_depth.py --vins ~/catkin_ws/src/VINS-Mono
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-NADIR-INIT-DEPTH]"

OLD_PARAM = "    INIT_DEPTH = 5.0;"
NEW_PARAM = """    INIT_DEPTH = 5.0;
    // """ + MARKER + """
    if (!fsSettings["init_depth"].empty())
        INIT_DEPTH = (double)fsSettings["init_depth"];
    ROS_INFO("INIT_DEPTH: %f", INIT_DEPTH);"""

OLD_TRI = """        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }"""

NEW_TRI = """        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
        // """ + MARKER + """ degenerate planar triangulation
        else if (INIT_DEPTH > 10.0 &&
                 (it_per_id.estimated_depth < 0.2 * INIT_DEPTH ||
                  it_per_id.estimated_depth > 5.0 * INIT_DEPTH))
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }"""


def patch_one(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    if MARKER in text and old not in text:
        print(f"уже патчен {path}")
        return
    if old not in text:
        raise SystemExit(f"Не найден фрагмент в {path}")
    bak = path.with_suffix(path.suffix + ".initdepth_orig")
    if not bak.is_file():
        shutil.copy2(path, bak)
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    print(f"patched {path}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    args = ap.parse_args()
    param = args.vins / "vins_estimator/src/parameters.cpp"
    feat = args.vins / "vins_estimator/src/feature_manager.cpp"
    if not param.is_file() or not feat.is_file():
        raise SystemExit(f"Нет исходников VINS в {args.vins}")
    patch_one(param, OLD_PARAM, NEW_PARAM)
    patch_one(feat, OLD_TRI, NEW_TRI)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
