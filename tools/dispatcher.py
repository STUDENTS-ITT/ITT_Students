#!/usr/bin/env python3
"""
Диспетчер VINS-Mono и DSO для fire-dron.

Выбирает алгоритм по типу сцены и качеству трекинга:
- Надир / поле / вода + IMU → VINS-Mono (основной канал, всегда)
- Лес: VINS основной; DSO только если VINS потерял трек
- Город, фасады, улица — профиль DSO (другой вариант одометрии)
- DSO на надир/поле/воду — health-check, не навигационный fallback
- Вода без фич → imu_only + баро Z
RTK в диспетчер не входит (только ATE снаружи).

Входы: траектории VINS и DSO
Выходы: выбранная траектория + метрики качества + health status
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np

NADIR_LIKE = {"nadyr", "field", "water"}


def _nadir_like(scene_type: str) -> bool:
    return scene_type in NADIR_LIKE


@dataclass
class AlgorithmQuality:
    """Метрика качества алгоритма."""
    algorithm: Literal["vins", "dso"]
    num_poses: int
    num_frames: int
    coverage_pct: float  # процент кадров, для которых есть поза
    is_lost: bool
    is_initialized: bool
    note: str


@dataclass
class HealthStatus:
    """Статус здоровья навигационного канала."""
    status: Literal["ok", "low_confidence", "imu_only", "lost"]
    vins_reboots: int
    dso_lost: bool
    recommendation: str


def analyze_vins(vins_csv: Path, num_frames: int) -> AlgorithmQuality:
    """Проанализировать выход VINS-Mono."""
    if not vins_csv.exists() or vins_csv.stat().st_size == 0:
        return AlgorithmQuality(
            algorithm="vins",
            num_poses=0,
            num_frames=num_frames,
            coverage_pct=0.0,
            is_lost=True,
            is_initialized=False,
            note="Файл не найден"
        )
    
    try:
        data = np.genfromtxt(vins_csv, delimiter=',', skip_header=1)
        if data.size == 0:
            raise ValueError("пустой CSV")
        if data.ndim == 1:
            data = data.reshape(1, -1)
        num_poses = len(data)
        coverage_pct = 100.0 * num_poses / num_frames if num_frames > 0 else 0
        
        is_initialized = num_poses > 100
        is_lost = num_poses == 0
        
        return AlgorithmQuality(
            algorithm="vins",
            num_poses=num_poses,
            num_frames=num_frames,
            coverage_pct=coverage_pct,
            is_lost=is_lost,
            is_initialized=is_initialized,
            note=f"{num_poses} поз, покрытие {coverage_pct:.1f}%"
        )
    except Exception as e:
        return AlgorithmQuality(
            algorithm="vins",
            num_poses=0,
            num_frames=num_frames,
            coverage_pct=0.0,
            is_lost=True,
            is_initialized=False,
            note=f"Ошибка чтения: {e}"
        )


def analyze_dso(dso_tum: Path, num_frames: int) -> AlgorithmQuality:
    """Проанализировать выход DSO."""
    if not dso_tum.exists() or dso_tum.stat().st_size == 0:
        return AlgorithmQuality(
            algorithm="dso",
            num_poses=0,
            num_frames=num_frames,
            coverage_pct=0.0,
            is_lost=True,
            is_initialized=False,
            note="Файл не найден или пуст"
        )
    
    try:
        data = np.genfromtxt(dso_tum)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        num_poses = len(data)
        coverage_pct = 100.0 * num_poses / num_frames if num_frames > 0 else 0
        
        is_initialized = num_poses > 5
        is_lost = num_poses == 0
        
        return AlgorithmQuality(
            algorithm="dso",
            num_poses=num_poses,
            num_frames=num_frames,
            coverage_pct=coverage_pct,
            is_lost=is_lost,
            is_initialized=is_initialized,
            note=f"{num_poses} поз, покрытие {coverage_pct:.1f}%"
        )
    except Exception as e:
        return AlgorithmQuality(
            algorithm="dso",
            num_poses=0,
            num_frames=num_frames,
            coverage_pct=0.0,
            is_lost=True,
            is_initialized=False,
            note=f"Ошибка чтения: {e}"
        )


def detect_vins_reboots(vins_csv: Path, max_speed: float = 50.0) -> int:
    """Детекция reboot VINS по скачкам позиции в CSV."""
    if not vins_csv.exists() or vins_csv.stat().st_size == 0:
        return 0
    try:
        data = np.genfromtxt(vins_csv, delimiter=',', skip_header=1)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if len(data) < 2:
            return 0
        # cols: timestamp, x, y, z, ...
        t = data[:, 0]
        p = data[:, 1:4]
        reboots = 0
        for i in range(1, len(p)):
            dt = t[i] - t[i - 1]
            dist = float(np.linalg.norm(p[i] - p[i - 1]))
            spd = dist / max(dt, 1e-3)
            n0, n1 = float(np.linalg.norm(p[i - 1])), float(np.linalg.norm(p[i]))
            if spd > max_speed or (n0 > 20.0 and n1 < 5.0 and dist > 10.0):
                reboots += 1
        return reboots
    except Exception:
        return 0


def assess_health(
    vins: AlgorithmQuality,
    dso: AlgorithmQuality,
    scene_type: str,
    vins_reboots: int,
) -> HealthStatus:
    """Health-check: DSO как индикатор, не fallback на надир."""
    dso_lost = dso.is_lost or not dso.is_initialized

    if not vins.is_initialized or vins.is_lost:
        if scene_type == "water":
            return HealthStatus(
                status="imu_only",
                vins_reboots=vins_reboots,
                dso_lost=dso_lost,
                recommendation="Вода: мало фич — только IMU/баро Z, DSO не fallback",
            )
        if scene_type == "forest" and dso.is_initialized and not dso.is_lost:
            return HealthStatus(
                status="low_confidence",
                vins_reboots=vins_reboots,
                dso_lost=False,
                recommendation="Лес: VINS потерян — DSO по текстуре крон",
            )
        if _nadir_like(scene_type) and dso.is_initialized and not dso.is_lost:
            return HealthStatus(
                status="low_confidence",
                vins_reboots=vins_reboots,
                dso_lost=False,
                recommendation="Нет IMU/VINS — DSO единственный канал; ограниченная точность",
            )
        if _nadir_like(scene_type):
            return HealthStatus(
                status="imu_only",
                vins_reboots=vins_reboots,
                dso_lost=dso_lost,
                recommendation="VINS не инициализирован — только БИНС/IMU, DSO не использовать на надир/поле",
            )
        return HealthStatus(
            status="lost",
            vins_reboots=vins_reboots,
            dso_lost=dso_lost,
            recommendation="Оба канала потеряны — только БИНС",
        )

    if vins_reboots > 0 and dso_lost and _nadir_like(scene_type):
        return HealthStatus(
            status="low_confidence",
            vins_reboots=vins_reboots,
            dso_lost=True,
            recommendation=(
                f"VINS reboot ×{vins_reboots} + DSO LOST — низкая достоверность; "
                "рекомендуется imu_only + баро Z"
            ),
        )

    if vins_reboots >= 5:
        return HealthStatus(
            status="low_confidence",
            vins_reboots=vins_reboots,
            dso_lost=dso_lost,
            recommendation=f"Много reboot VINS ({vins_reboots}) — повышенная σ в vo_to_gps_dat",
        )

    return HealthStatus(
        status="ok",
        vins_reboots=vins_reboots,
        dso_lost=dso_lost,
        recommendation="Нормальный режим VINS+IMU",
    )


def choose_algorithm(
    vins: AlgorithmQuality,
    dso: AlgorithmQuality,
    scene_type: str = "nadyr",
    health: HealthStatus | None = None,
) -> tuple[AlgorithmQuality, str]:
    """
    Выбрать алгоритм по качеству и типу сцены.
    На надир: DSO НЕ используется как fallback для навигации.
    """
    
    if scene_type in ("nadyr", "field"):
        if vins.is_initialized and not vins.is_lost:
            reason = "VINS (поле/надир + IMU, основной канал)"
            if health and health.status == "low_confidence":
                reason += f"; health={health.status}, reboots={health.vins_reboots}"
            return vins, reason
        if not vins.is_initialized and dso.is_initialized and not dso.is_lost:
            return dso, "DSO (нет IMU/VINS, единственный канал; не для навигации надир-MARS)"
        if health and health.status in ("imu_only", "lost"):
            return vins, f"VINS недоступен → {health.recommendation}"
        return vins, "VINS (надир/поле, DSO не fallback — только health-check)"

    if scene_type == "water":
        if vins.is_initialized and not vins.is_lost:
            return vins, "VINS (вода: пока есть фичи берега/волн)"
        return vins, "VINS недоступен над водой → imu_only + баро Z, DSO не fallback"

    if scene_type == "forest":
        if vins.is_initialized and not vins.is_lost:
            return vins, "VINS (лес + IMU)"
        if dso.is_initialized and not dso.is_lost:
            return dso, "DSO (VINS потерял трек — запас по текстуре крон)"
        return vins, "Оба слабые в лесу → IMU/баро"

    # Любая сцена с 3D-текстурой: если VINS молчит, а DSO жив — DSO.
    if (
        scene_type not in NADIR_LIKE
        and (not vins.is_initialized or vins.is_lost)
        and dso.is_initialized
        and not dso.is_lost
    ):
        return dso, "DSO (VINS потерян, есть текстура)"

    if scene_type == "city":
        if dso.is_initialized and not dso.is_lost:
            return dso, "DSO (город, богатая текстура)"
        elif vins.is_initialized and not vins.is_lost:
            return vins, "VINS (DSO потерял, fallback к VINS)"
        else:
            return dso, "DSO (оба потеряны, но город - DSO профиль)"

    # mixed и неизвестные типы — по покрытию
    if vins.coverage_pct >= 80 or (vins.is_initialized and not dso.is_initialized):
        return vins, "VINS (лучшее покрытие или DSO не инициализировался)"
    if dso.coverage_pct >= 60 and not dso.is_lost:
        return dso, "DSO (хорошее покрытие)"
    if vins.num_poses >= dso.num_poses:
        return vins, "VINS (больше поз)"
    return dso, "DSO (больше поз)"


def main():
    """CLI: выбор VINS/DSO по качеству трекинга; JSON на stdout."""
    if len(sys.argv) < 4:
        print("Использование: dispatcher.py <vins_csv> <dso_tum> <num_frames> [scene_type]")
        print("  vins_csv: путь к results/vins_mars_run2_calib.csv")
        print("  dso_tum: путь к results/dso_mars_run2_calib.tum")
        print("  num_frames: число кадров в датасете (например, 3657)")
        print("  scene_type: nadyr|field|water|forest|city|mixed")
        sys.exit(1)
    
    vins_path = Path(sys.argv[1])
    dso_path = Path(sys.argv[2])
    num_frames = int(sys.argv[3])
    scene_type = sys.argv[4] if len(sys.argv) > 4 else "nadyr"
    
    vins = analyze_vins(vins_path, num_frames)
    dso = analyze_dso(dso_path, num_frames)
    vins_reboots = detect_vins_reboots(vins_path) if vins_path.exists() else 0
    health = assess_health(vins, dso, scene_type, vins_reboots)
    
    chosen, reason = choose_algorithm(vins, dso, scene_type, health)
    
    result = {
        "chosen_algorithm": chosen.algorithm,
        "reason": reason,
        "health": {
            "status": health.status,
            "vins_reboots": health.vins_reboots,
            "dso_lost": health.dso_lost,
            "recommendation": health.recommendation,
        },
        "vins": {
            "num_poses": vins.num_poses,
            "coverage_pct": vins.coverage_pct,
            "is_initialized": vins.is_initialized,
            "is_lost": vins.is_lost,
            "note": vins.note
        },
        "dso": {
            "num_poses": dso.num_poses,
            "coverage_pct": dso.coverage_pct,
            "is_initialized": dso.is_initialized,
            "is_lost": dso.is_lost,
            "note": dso.note
        },
        "scene_type": scene_type
    }
    
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
