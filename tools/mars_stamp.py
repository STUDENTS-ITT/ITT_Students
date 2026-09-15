#!/usr/bin/env python3
"""Единые метки времени MARS: header.stamp и выравнивание DJI → camera epoch."""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np


def row_stamp_s(row: dict) -> float | None:
    """Секунды: предпочитаем header.stamp, иначе bag_time_ns / stamp_ns."""
    sec = row.get("header.stamp.sec")
    if sec is not None and str(sec).strip() != "":
        nsec = row.get("header.stamp.nanosec") or row.get("header.stamp.nsec") or 0
        return float(sec) + float(nsec) * 1e-9
    for key in ("stamp_ns", "bag_time_ns", "stamp", "timestamp"):
        raw = row.get(key)
        if raw is None or str(raw).strip() == "":
            continue
        t = float(raw)
        return t * 1e-9 if t > 1e15 else t
    return None


def load_csv_series(path: Path, value_key: str = "data") -> tuple[np.ndarray, np.ndarray]:
    t, v = [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            ts = row_stamp_s(row)
            if ts is None:
                continue
            try:
                val = float(row.get(value_key) or row.get("height") or row.get("z") or "")
            except (TypeError, ValueError):
                continue
            t.append(ts)
            v.append(val)
    return np.asarray(t, dtype=np.float64), np.asarray(v, dtype=np.float64)


def load_dji_imu_stamps(path: Path, camera_offset: float = 0.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """t (camera epoch), yaw (rad, unwrapped), wz."""
    t, yaw, wz = [], [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            ts = row_stamp_s(row)
            if ts is None:
                continue
            try:
                x = float(row["orientation.x"])
                y = float(row["orientation.y"])
                z = float(row["orientation.z"])
                w = float(row["orientation.w"])
                wz.append(float(row["angular_velocity.z"]))
            except (KeyError, TypeError, ValueError):
                continue
            t.append(ts - camera_offset)
            yaw.append(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    if len(t) < 10:
        raise SystemExit(f"мало кватернионов AHRS в {path}")
    return (
        np.asarray(t, dtype=np.float64),
        np.unwrap(np.asarray(yaw, dtype=np.float64)),
        np.asarray(wz, dtype=np.float64),
    )


def detect_camera_offset(
    camera_stamps: Path | None,
    dji_imu: Path | None,
    vins_t0: float | None = None,
) -> float:
    """DJI epoch − camera epoch (сек). Вычитаем из DJI при интерполяции на VINS/camera."""
    cam_t0 = None
    if camera_stamps is not None and camera_stamps.is_file():
        with camera_stamps.open(encoding="utf-8", errors="replace") as f:
            row = next(csv.DictReader(f), None)
            if row:
                raw = row.get("stamp_ns") or row.get("timestamp")
                if raw:
                    cam_t0 = float(raw) * 1e-9 if float(raw) > 1e15 else float(raw)
    if cam_t0 is None and vins_t0 is not None:
        cam_t0 = float(vins_t0)

    dji_t0 = None
    if dji_imu is not None and dji_imu.is_file():
        with dji_imu.open(encoding="utf-8", errors="replace") as f:
            for row in csv.DictReader(f):
                ts = row_stamp_s(row)
                if ts is not None:
                    dji_t0 = ts
                    break
    if cam_t0 is None or dji_t0 is None:
        return 0.0
    off = float(dji_t0 - cam_t0)
    if abs(off) > 0.05:
        return off
    return 0.0
