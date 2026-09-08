#!/usr/bin/env python3
"""
Оптимизация экстринсиков камера–IMU по данным MARS-LVIG.

Метод: минимизировать смещения IMU (bias_accel, bias_gyro) над длинным
спокойным участком полёта. Правильные экстринсики → малые bias.

Входы:
  - MARS IMU CSV: dji_osdk_ros_imu.csv
  - MARS кадры и timestamps
  - RTK ground truth: dji_osdk_ros_rtk_position.csv

Выход:
  - Оптимальные R (rotation) и t (translation) камера→IMU
  - Обновлённый mars_nadir.yaml
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation


@dataclass
class IMUData:
    """Сырые данные IMU."""
    timestamps: np.ndarray  # сек
    angular_velocity: np.ndarray  # рад/сек, shape (N, 3)
    linear_acceleration: np.ndarray  # м/сек², shape (N, 3)
    bias_gyro: np.ndarray  # оцененное смещение гироскопа
    bias_accel: np.ndarray  # оцененное смещение акселерометра


@dataclass
class ExtrinsicParams:
    """Параметры экстринсиков (в оптимизации)."""
    # Используем кватернион для R: q = [qx, qy, qz, qw]
    q: np.ndarray  # shape (4,)
    t: np.ndarray  # shape (3,)
    
    def to_rotation_matrix(self) -> np.ndarray:
        """Кватернион → матрица 3×3."""
        r = Rotation.from_quat(self.q)
        return r.as_matrix()
    
    def to_dict(self) -> dict:
        """Вывод в формат mars_nadir.yaml."""
        R = self.to_rotation_matrix()
        return {
            "extrinsicRotation": R.tolist(),
            "extrinsicTranslation": self.t.tolist(),
            "quaternion": self.q.tolist()
        }


def load_imu_csv(imu_csv: Path) -> IMUData:
    """Загрузить IMU данные из CSV MARS."""
    timestamps = []
    accel = []
    gyro = []
    
    with open(imu_csv, encoding='utf-8', errors='ignore') as f:
        reader = csv.DictReader(f)
        for row_num, row in enumerate(reader, start=2):
            try:
                # Формат MARS: header.stamp.sec используется как timestamp
                # Используем bag_time_ns (наносекунды) или header.stamp.sec
                ts = None
                if 'header.stamp.sec' in row:
                    ts = float(row['header.stamp.sec']) + float(row.get('header.stamp.nanosec', 0)) / 1e9
                else:
                    continue
                
                # Загрузить ускорение и угловую скорость (они есть в MARS CSV)
                ax = float(row['linear_acceleration.x'])
                ay = float(row['linear_acceleration.y'])
                az = float(row['linear_acceleration.z'])
                wx = float(row['angular_velocity.x'])
                wy = float(row['angular_velocity.y'])
                wz = float(row['angular_velocity.z'])
                
                timestamps.append(ts)
                accel.append([ax, ay, az])
                gyro.append([wx, wy, wz])
            except (KeyError, ValueError, TypeError) as e:
                if row_num < 5:
                    print(f"  Строка {row_num}: ошибка {e}")
                continue
    
    if not timestamps:
        print("Ошибка: IMU данные не загружены. Проверьте формат CSV.")
        raise ValueError("Empty IMU data")
    
    return IMUData(
        timestamps=np.array(timestamps),
        linear_acceleration=np.array(accel),
        angular_velocity=np.array(gyro),
        bias_accel=np.zeros(3),
        bias_gyro=np.zeros(3)
    )


def estimate_imu_bias(imu_data: IMUData, start_idx: int, end_idx: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Оценить смещения IMU на спокойном участке.
    
    На участке без движения:
    - accel должен быть [0, 0, 9.81] в мировой СК (или повёрнут)
    - gyro должен быть [0, 0, 0]
    
    Практически: берём начальный участок полёта и медиану.
    """
    accel_slice = imu_data.linear_acceleration[start_idx:end_idx]
    gyro_slice = imu_data.angular_velocity[start_idx:end_idx]
    
    # На ровном висении; accel ~ [0, 0, g]
    # Смещение гироскопа: просто медиана
    bias_gyro = np.median(gyro_slice, axis=0)
    
    # Смещение акселерометра: медиана вычитая из тяжести
    # (это грубая оценка; точная нужна через orientation)
    accel_median = np.median(accel_slice, axis=0)
    gravity_magnitude = np.linalg.norm(accel_median)
    bias_accel = accel_median - np.array([0, 0, gravity_magnitude])
    
    return bias_accel, bias_gyro


def objective_function(
    params_flat: np.ndarray,
    imu_data: IMUData,
    start_idx: int,
    end_idx: int,
    official_extrinsic: dict
) -> float:
    """
    Целевая функция: минимизировать сумму норм bias при гипотезе об экстринсиках.
    
    Args:
        params_flat: [qx, qy, qz, qw, tx, ty, tz] (7 параметров)
        imu_data: IMU с timestamp и измерениями
        start_idx, end_idx: диапазон кадров для анализа
        official_extrinsic: официальные экстринсики для регуляризации
    
    Returns:
        скалярная ошибка (норма bias_accel + норма bias_gyro)
    """
    
    q = params_flat[:4]
    t = params_flat[4:]
    
    # Нормировать кватернион
    q = q / (np.linalg.norm(q) + 1e-10)
    
    # Оценить bias на этом диапазоне
    bias_a, bias_g = estimate_imu_bias(imu_data, start_idx, end_idx)
    
    # Целевая функция: норма смещений + штраф на отклонение от official
    error = np.linalg.norm(bias_a) + np.linalg.norm(bias_g)
    
    # Слабая регуляризация: не уходить далеко от official
    if official_extrinsic:
        official_R = np.array(official_extrinsic.get('extrinsicRotation', np.eye(3)))
        official_t = np.array(official_extrinsic.get('extrinsicTranslation', [0, 0, 0]))
        
        R_current = Rotation.from_quat(q).as_matrix()
        error += 0.1 * np.linalg.norm(R_current - official_R)
        error += 0.1 * np.linalg.norm(t - official_t)
    
    return error


def main():
    """
    Оптимизировать экстринсики по MARS-LVIG.
    
    Использование:
      python3 tools/optimize_extrinsic_mars.py \\
        --imu /path/to/dji_osdk_ros_imu.csv \\
        --mars-yaml data/calibration/mars_official/HK_GNSS*.yaml \\
        --out extrinsic_optimized.yaml
    """
    
    import sys
    import argparse
    
    parser = argparse.ArgumentParser(description="Оптимизация extrinsic камера–IMU")
    parser.add_argument("--imu", type=Path, required=True, help="IMU CSV файл")
    parser.add_argument("--mars-yaml", type=Path, help="Official MARS yaml")
    parser.add_argument("--out", type=Path, default=Path("extrinsic_optimized.yaml"))
    args = parser.parse_args()
    
    if not args.imu.exists():
        print(f"Ошибка: {args.imu} не найден")
        sys.exit(1)
    
    print("Загрузка IMU данных...")
    imu = load_imu_csv(args.imu)
    print(f"  Загружено {len(imu.timestamps)} отсчётов, {imu.timestamps[-1] - imu.timestamps[0]:.1f} сек")
    
    # Участок для анализа: первые 30 сек (обычно спокойный полёт после взлёта)
    start_time = imu.timestamps[0] + 5  # пропустить первые 5 сек
    end_time = start_time + 30
    
    start_idx = np.searchsorted(imu.timestamps, start_time)
    end_idx = np.searchsorted(imu.timestamps, end_time)
    
    print(f"Анализ участка {imu.timestamps[start_idx]:.1f}–{imu.timestamps[end_idx]:.1f} сек")
    
    # Официальные экстринсики (если есть)
    official = {}
    if args.mars_yaml and args.mars_yaml.exists():
        import yaml
        try:
            data = yaml.safe_load(args.mars_yaml.read_text())
            if 'camera_ext_R' in data:
                R_flat = data['camera_ext_R']
                official['extrinsicRotation'] = np.array(R_flat).reshape(3, 3).tolist()
            if 'camera_ext_t' in data:
                official['extrinsicTranslation'] = data['camera_ext_t']
        except Exception as e:
            print(f"  Не удалось загрузить official: {e}")
    
    # Начальное приближение: identity
    x0 = np.array([0, 0, 0, 1, 0, 0, 0], dtype=float)
    
    print("Оптимизация...")
    result = minimize(
        objective_function,
        x0,
        args=(imu, start_idx, end_idx, official),
        method='Nelder-Mead',
        options={'maxiter': 5000}
    )
    
    opt_params = ExtrinsicParams(
        q=result.x[:4],
        t=result.x[4:]
    )
    
    print(f"Оптимизация завершена, ошибка: {result.fun:.6f}")
    print(f"Оптимальные параметры:")
    print(f"  Quaternion: {opt_params.q}")
    print(f"  Translation: {opt_params.t}")
    print(f"  Rotation matrix:\n{opt_params.to_rotation_matrix()}")
    
    # Сохранить результат
    output = {
        "source": "optimize_extrinsic_mars.py",
        "timestamp": str(Path(__file__).stat().st_mtime),
        "optimization_error": float(result.fun),
        "extrinsicRotation": opt_params.to_rotation_matrix().tolist(),
        "extrinsicTranslation": opt_params.t.tolist(),
        "quaternion": opt_params.q.tolist(),
        "analysis_window_sec": [
            float(imu.timestamps[start_idx]),
            float(imu.timestamps[end_idx])
        ]
    }
    
    args.out.write_text(json.dumps(output, indent=2))
    print(f"\nРезультат сохранён в {args.out}")
    
    print("\nОбновить mars_nadir.yaml:")
    print(f"""
extrinsicRotation: {json.dumps(opt_params.to_rotation_matrix().tolist())}
extrinsicTranslation: {json.dumps(opt_params.t.tolist())}
estimate_extrinsic: 0
""")


if __name__ == "__main__":
    main()
