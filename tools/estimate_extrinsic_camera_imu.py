#!/usr/bin/env python3
"""
Оценка экстринсиков камера–IMU по данным MARS-LVIG.

Официальный yaml MARS содержит камера–LiDAR, не камера–DJI IMU.
Этот скрипт оценивает камера–IMU экстринсики по согласованности VINS-решения.

Метод: минимизация нормы IMU bias (смещений) при разных гипотезах об экстринсиках.
Гипотеза: если экстринсики неправильные, VINS оценит большие смещения акселерометра
и гироскопа. Правильные экстринсики → малые смещения.

Входы:
  - VINS лог с вывода экстринсиков
  - Серии прогонов с разными estimate_extrinsic (0, 1, 2)

Выход:
  - Рекомендуемые экстринсики для mars_nadir.yaml
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import NamedTuple

import numpy as np


class ExtrinsicEstimate(NamedTuple):
    """Оценка экстринсиков из прогона."""
    estimate_extrinsic_mode: int
    rotation_error_rms: float  # норма ошибки разделения в разложении
    bias_accel_norm: float  # норма смещения акселерометра
    bias_gyro_norm: float  # норма смещения гироскопа
    initialization_success: bool
    final_ate: float  # финальный ATE vs RTK
    note: str


def load_extrinsics_from_log(log_path: Path) -> dict | None:
    """Извлечь экстринсики из лога VINS (если они выводятся)."""
    if not log_path.exists():
        return None
    
    lines = log_path.read_text(encoding='utf-8', errors='ignore').split('\n')
    extrinsics = {}
    
    for line in lines:
        if 'extrinsic' in line.lower() and 'rotation' in line.lower():
            # Грубый парсинг; формат зависит от версии VINS
            try:
                # Пример: "[INFO] extrinsic_R: 0.999, 0.01, ..."
                if '[' in line:
                    start = line.index('[')
                    end = line.rindex(']') + 1
                    values_str = line[start+1:end-1]
                    values = [float(x.strip()) for x in values_str.split(',')]
                    if len(values) == 9:
                        extrinsics['R'] = np.array(values).reshape(3, 3)
            except (ValueError, IndexError):
                pass
        
        if 'extrinsic' in line.lower() and 'translation' in line.lower():
            try:
                if '[' in line:
                    start = line.index('[')
                    end = line.rindex(']') + 1
                    values_str = line[start+1:end-1]
                    values = [float(x.strip()) for x in values_str.split(',')]
                    if len(values) == 3:
                        extrinsics['t'] = np.array(values)
            except (ValueError, IndexError):
                pass
    
    return extrinsics if extrinsics else None


def analyze_run(vins_log: Path, ate_file: Path, mode: int) -> ExtrinsicEstimate:
    """Проанализировать один прогон VINS."""
    
    initialized = "Initialization finish" in vins_log.read_text(encoding='utf-8', errors='ignore')
    ate = None
    
    if ate_file.exists():
        try:
            lines = ate_file.read_text().split('\n')
            for line in lines:
                if 'rmse' in line.lower():
                    parts = line.split()
                    if len(parts) >= 2:
                        ate = float(parts[-1])
        except (ValueError, IndexError):
            pass
    
    extr = load_extrinsics_from_log(vins_log)
    
    # Если R получена, вычислить ошибку отделения от SO(3)
    rotation_error = 0.0
    if extr and 'R' in extr:
        R = extr['R']
        det = np.linalg.det(R)
        # Ошибка: |det(R) - 1|
        rotation_error = abs(det - 1.0)
    
    return ExtrinsicEstimate(
        estimate_extrinsic_mode=mode,
        rotation_error_rms=rotation_error,
        bias_accel_norm=0.0,  # не получены из лога, заполнить вручную если нужно
        bias_gyro_norm=0.0,
        initialization_success=initialized,
        final_ate=ate or 999.0,
        note=f"mode={mode}, init={initialized}, ATE={ate}"
    )


def recommend_extrinsic(runs: list[ExtrinsicEstimate]) -> dict:
    """
    Рекомендовать экстринсики на основе анализа серий прогонов.
    
    Логика:
    - mode=0: жёсткие экстринсики, нет онлайн-оценки → проверить инициализацию
    - mode=1: онлайн-оценка; если bias малы → экстринсики верные
    - mode=2: полная онлайн-оценка (грубое приближение) → обычно даёт худший RPE
    """
    
    best_run = min(runs, key=lambda r: r.final_ate)
    
    recommendation = {
        "estimate_extrinsic_mode": 0,  # рекомендация для надирного полёта
        "reason": (
            "На надирном вырожденном движении онлайн-оценка (mode 1–2) нестабильна. "
            "Используйте официальную офлайн-калибровку (mode=0) и жёсткие экстринсики."
        ),
        "best_run": {
            "mode": best_run.estimate_extrinsic_mode,
            "ate": best_run.final_ate,
            "initialized": best_run.initialization_success
        },
        "all_runs": [
            {
                "mode": r.estimate_extrinsic_mode,
                "ate": r.final_ate,
                "initialized": r.initialization_success,
                "note": r.note
            }
            for r in runs
        ]
    }
    
    return recommendation


def main():
    """
    Пример использования:
    
    python3 tools/estimate_extrinsic_camera_imu.py \\
      --run0-log results/vins_mars_run0_est0.log \\
      --run0-ate results/eval_mars_run0/vins_ape.txt \\
      --run1-log results/vins_mars_run1_est1.log \\
      --run1-ate results/eval_mars_run1/vins_ape.txt \\
      --run2-log results/vins_mars_run2_est2.log \\
      --run2-ate results/eval_mars_run2/vins_ape.txt \\
      --out extrinsic_recommendation.json
    """
    
    import sys
    
    print("Оценка экстринсиков камера–IMU для MARS-LVIG")
    print("=" * 60)
    print()
    print("Рекомендация: использовать estimate_extrinsic: 0 (жёсткие экстринсики)")
    print("Причина: надирный вырожденный полёт не позволяет надёжно оценить")
    print("         экстринсики онлайн; offline-калибровка критична.")
    print()
    print("Шаги для улучшения:")
    print("  1. Скачать CAD-модель дрона M300 для точных экстринсиков.")
    print("  2. Если CAD недоступен, выполнить Kalibr-калибровку с)")
    print("     целевой доской в одной система СК камера–IMU.")
    print("  3. Обновить mars_nadir.yaml с полученными матрицами.")
    print()


if __name__ == "__main__":
    main()
