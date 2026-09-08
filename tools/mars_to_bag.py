#!/usr/bin/env python3
"""MARS-LVIG (CSV + кадры) -> rosbag для VINS-Mono + эталон RTK в формате TUM.

Запускать в Ubuntu с активированным ROS Noetic:

    source /opt/ros/noetic/setup.bash
    python3 mars_to_bag.py --dataset /media/$USER/DISK/Датасет \\
        --frames  .../camera/images --stamps .../camera/timestamps.csv \\
        --imu dji --scale 0.25 --out data/mars/mars_hkairport03.bag

Что важно знать про этот датасет
--------------------------------
IMU два, и они разные:

  dji   -- /dji_osdk_ros/imu, 400 Гц, оси FLU, ускорения уже в м/с².
  livox -- /livox/imu, 208 Гц, жёстко связан с камерой, но акселерометр
           записан в единицах g и требует умножения на 9.81. Забыть про
           это -- гарантированный разлёт траектории.

Разрешение 2448x2048 в 14 раз больше, чем у EuRoC. На полном разрешении
VINS-Mono безнадёжно отстанет от 10 Гц, поэтому по умолчанию кадр
уменьшается вчетверо. Коэффициент обязательно указывается в отчёте: он
напрямую влияет на измеренное время обработки кадра. Внутренние параметры
камеры при этом тоже делятся на 4, коэффициенты дисторсии не меняются.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("Нужен opencv: python3 -m pip install --user opencv-python-headless")

try:
    import rosbag
    import rospy
    from sensor_msgs.msg import Image, Imu
except ImportError:
    sys.exit("Не найден ROS. Выполните: source /opt/ros/noetic/setup.bash")

R_EARTH = 6371e3
G = 9.81

IMU_SPECS = {
    "dji": {
        "file": "imu/dji_osdk_ros_imu.csv",
        "accel_scale": 1.0,          # уже м/с²
        "rate_hint": 400,
    },
    "livox": {
        "file": "imu/livox_imu.csv",
        "accel_scale": G,            # записано в g
        "rate_hint": 208,
    },
}


def read_csv(path: Path) -> tuple[list[str], np.ndarray]:
    """Простое чтение числового CSV с заголовком; нечисловые колонки -> nan."""
    with path.open(encoding="utf-8", errors="replace") as f:
        header = f.readline().strip().split(",")
        rows = []
        for line in f:
            parts = line.rstrip("\n").split(",")
            if len(parts) != len(header):
                continue
            row = []
            for v in parts:
                try:
                    row.append(float(v))
                except ValueError:
                    row.append(math.nan)
            rows.append(row)
    return header, np.array(rows, dtype=float)


def col(header: list[str], data: np.ndarray, name: str) -> np.ndarray:
    if name not in header:
        raise SystemExit(f"Нет колонки {name!r}. Есть: {header[:12]} ...")
    return data[:, header.index(name)]


def stamp_seconds(header: list[str], data: np.ndarray) -> np.ndarray:
    """Метка датчика (шкала UTC GPS после синхронизации PPS), секунды."""
    if "header.stamp.sec" in header:
        return (col(header, data, "header.stamp.sec")
                + col(header, data, "header.stamp.nanosec") * 1e-9)
    if "stamp_ns" in header:
        return col(header, data, "stamp_ns") * 1e-9
    return col(header, data, "bag_time_ns") * 1e-9


def write_imu(bag, dataset: Path, kind: str) -> tuple[float, float]:
    spec = IMU_SPECS[kind]
    path = dataset / spec["file"]
    if not path.is_file():
        raise SystemExit(f"Нет файла IMU: {path}")

    header, data = read_csv(path)
    t = stamp_seconds(header, data)
    wx = col(header, data, "angular_velocity.x")
    wy = col(header, data, "angular_velocity.y")
    wz = col(header, data, "angular_velocity.z")
    ax = col(header, data, "linear_acceleration.x") * spec["accel_scale"]
    ay = col(header, data, "linear_acceleration.y") * spec["accel_scale"]
    az = col(header, data, "linear_acceleration.z") * spec["accel_scale"]

    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0.0
    print(f"IMU {kind}: {len(t)} отсчётов, {rate:.1f} Гц, "
          f"масштаб акселерометра x{spec['accel_scale']}")
    mag = float(np.median(np.sqrt(ax ** 2 + ay ** 2 + az ** 2)))
    print(f"  медиана |a| = {mag:.3f} м/с²  (должно быть около 9.8)")
    if not 8.5 < mag < 11.0:
        print("  ВНИМАНИЕ: модуль ускорения далёк от g. Проверьте единицы "
              "и ключ --imu.", file=sys.stderr)

    for i in range(len(t)):
        m = Imu()
        m.header.stamp = rospy.Time.from_sec(float(t[i]))
        m.header.frame_id = "imu"
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = \
            float(wx[i]), float(wy[i]), float(wz[i])
        m.linear_acceleration.x = float(ax[i])
        m.linear_acceleration.y = float(ay[i])
        m.linear_acceleration.z = float(az[i])
        m.orientation_covariance[0] = -1.0     # ориентация не задана
        bag.write("/imu0", m, m.header.stamp)

    return float(t[0]), float(t[-1])


def read_frame_stamps(path: Path, n_frames: int) -> np.ndarray:
    """Метки времени кадров, секунды. Ищем колонку с наносекундами."""
    header, data = read_csv(path)
    for name in ("stamp_ns", "header.stamp.sec", "bag_time_ns"):
        if name == "header.stamp.sec" and name in header:
            return (col(header, data, "header.stamp.sec")
                    + col(header, data, "header.stamp.nanosec") * 1e-9)
        if name in header:
            return col(header, data, name) * 1e-9
    # Запасной вариант: первая числовая колонка похожа на наносекунды.
    first = data[:, 0]
    if np.nanmedian(first) > 1e17:
        return first * 1e-9
    raise SystemExit(f"Не понял формат меток времени в {path}: {header[:8]}")


def write_camera(bag, frames_dir: Path, stamps_csv: Path, scale: float) -> int:
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    files = sorted(p for p in frames_dir.iterdir()
                   if p.is_file() and p.suffix.lower() in exts)
    if not files:
        raise SystemExit(f"В {frames_dir} нет кадров")

    stamps = read_frame_stamps(stamps_csv, len(files))
    if len(stamps) < len(files):
        raise SystemExit(f"Меток {len(stamps)}, а кадров {len(files)} — не совпадает")
    stamps = stamps[: len(files)]

    probe = cv2.imread(str(files[0]), cv2.IMREAD_GRAYSCALE)
    h_in, w_in = probe.shape[:2]
    w_out = int(round(w_in * scale)) // 8 * 8
    h_out = int(round(h_in * scale)) // 8 * 8

    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    print(f"Камера: {len(files)} кадров, {rate:.2f} Гц")
    print(f"  {w_in}x{h_in} -> {w_out}x{h_out} (масштаб {scale})")

    for i, src in enumerate(files):
        im = cv2.imread(str(src), cv2.IMREAD_GRAYSCALE)
        if im is None:
            print(f"  пропускаю {src.name}", file=sys.stderr)
            continue
        if (w_out, h_out) != (w_in, h_in):
            im = cv2.resize(im, (w_out, h_out), interpolation=cv2.INTER_AREA)

        m = Image()
        m.header.stamp = rospy.Time.from_sec(float(stamps[i]))
        m.header.frame_id = "cam0"
        m.height, m.width = h_out, w_out
        m.encoding = "mono8"
        m.is_bigendian = 0
        m.step = w_out
        m.data = im.tobytes()
        bag.write("/cam0/image_raw", m, m.header.stamp)

        if (i + 1) % 200 == 0:
            print(f"  {i + 1}/{len(files)}")

    return len(files)


def write_groundtruth(dataset: Path, out: Path) -> None:
    """RTK -> локальный ENU -> формат TUM для evo."""
    pos_path = dataset / "gps/dji_osdk_ros_rtk_position.csv"
    if not pos_path.is_file():
        print("RTK не найден, эталон не создан", file=sys.stderr)
        return

    header, data = read_csv(pos_path)
    t = stamp_seconds(header, data)
    lat = np.deg2rad(col(header, data, "latitude"))
    lon = np.deg2rad(col(header, data, "longitude"))
    alt = col(header, data, "altitude")

    ok = np.isfinite(lat) & np.isfinite(lon) & np.isfinite(alt)
    t, lat, lon, alt = t[ok], lat[ok], lon[ok], alt[ok]
    if len(t) == 0:
        print("RTK пуст", file=sys.stderr)
        return

    lat0, lon0, alt0 = lat[0], lon[0], alt[0]
    rh = R_EARTH + alt0
    east = (lon - lon0) * rh * math.cos(lat0)
    north = (lat - lat0) * rh
    up = alt - alt0

    # Курс RTK, если есть: даёт ориентацию вокруг вертикали.
    yaw = np.zeros(len(t))
    yaw_path = dataset / "gps/dji_osdk_ros_rtk_yaw.csv"
    if yaw_path.is_file():
        yh, yd = read_csv(yaw_path)
        if "data" in yh:
            ty = col(yh, yd, "bag_time_ns") * 1e-9
            yaw = np.deg2rad(np.interp(t, ty, col(yh, yd, "data")))

    with out.open("w", encoding="utf-8") as f:
        f.write("# timestamp tx ty tz qx qy qz qw  (локальный ENU, начало = первый отсчёт RTK)\n")
        for i in range(len(t)):
            qz, qw = math.sin(yaw[i] / 2), math.cos(yaw[i] / 2)
            f.write(f"{t[i]:.9f} {east[i]:.6f} {north[i]:.6f} {up[i]:.6f} "
                    f"0.0 0.0 {qz:.9f} {qw:.9f}\n")

    dist = float(np.sum(np.linalg.norm(
        np.diff(np.column_stack([east, north, up]), axis=0), axis=1)))
    print(f"Эталон RTK: {len(t)} поз, пройдено {dist:.1f} м -> {out}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", type=Path, required=True, help="корень E:\\Датасет")
    ap.add_argument("--frames", type=Path, help="папка с кадрами камеры")
    ap.add_argument("--stamps", type=Path, help="camera/timestamps.csv")
    ap.add_argument("--imu", choices=sorted(IMU_SPECS), default="dji")
    ap.add_argument("--scale", type=float, default=0.25,
                    help="уменьшение кадра, по умолчанию 1/4")
    ap.add_argument("--out", type=Path, required=True, help="итоговый .bag")
    ap.add_argument("--gt", type=Path, help="куда писать эталон TUM "
                                            "(по умолчанию рядом с bag)")
    args = ap.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    gt_path = args.gt or args.out.with_name(args.out.stem + "_gt.tum")

    print(f"=== Сборка {args.out} ===")
    with rosbag.Bag(str(args.out), "w") as bag:
        t0, t1 = write_imu(bag, args.dataset, args.imu)
        print(f"  интервал IMU: {t1 - t0:.1f} с")

        if args.frames and args.stamps:
            n = write_camera(bag, args.frames, args.stamps, args.scale)
            print(f"  кадров записано: {n}")
        else:
            print("  кадры не заданы, bag будет только с IMU "
                  "(VINS-Mono так не запустится)", file=sys.stderr)

    write_groundtruth(args.dataset, gt_path)

    size_gb = args.out.stat().st_size / 1e9
    print(f"\nГотово: {args.out} ({size_gb:.2f} ГБ)")
    print("\nПроверка содержимого:")
    print(f"  rosbag info {args.out}")
    print("\nЗапуск (не забудьте свой конфиг под надирную камеру):")
    print("  roslaunch vins_estimator mars_nadir.launch")
    print(f"  rosbag play {args.out} -r 0.3")
    return 0


if __name__ == "__main__":
    sys.exit(main())
