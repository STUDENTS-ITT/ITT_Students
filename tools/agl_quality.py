#!/usr/bin/env python3
"""Классификация баро как AGL. Без имён сцен и без RTK.

MARS height_above_takeoff часто ~0 при реальных 80 м — это не «на земле».
"""
from __future__ import annotations

import numpy as np

DEAD_PEAK_M = 2.0
USABLE_PEAK_M = 15.0
USABLE_P80_M = 10.0


def classify_baro(z) -> str:
    """DEAD | NOT_AGL | STALE | USABLE."""
    z = np.asarray(z, dtype=np.float64)
    z = z[np.isfinite(z)]
    if len(z) < 5:
        return "DEAD"
    peak = float(np.max(np.abs(z)))
    p80 = float(np.percentile(np.abs(z), 80))
    climb = float(np.max(z) - np.min(z))
    if peak < DEAD_PEAK_M:
        return "DEAD"
    if peak < USABLE_PEAK_M or p80 < USABLE_P80_M:
        return "NOT_AGL"
    if float(np.std(z)) < 0.5 and climb < 5.0:
        return "STALE"
    return "USABLE"


def baro_is_agl(z) -> bool:
    return classify_baro(z) == "USABLE"
