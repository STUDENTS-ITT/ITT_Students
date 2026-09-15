#!/usr/bin/env python3
"""Официальный rosbag MARS-LVIG → bag для VINS-Mono.

Вход (Table 3 датасета):
  /left_camera/image/compressed  или  /left_camera/image
  /livox/imu                     (BMI088, может быть в g)
  /dji_osdk_ros/rtk_position     (эталон, не в estimator)
  /dji_osdk_ros/height_above_takeoff  (баро, если есть)

Выход:
  /cam0/image_raw  mono8, масштаб --scale (по умолчанию 0.25)
  /imu0            м/с²
  рядом: *_gt.tum (RTK ENU) и *_baro.csv

    source /opt/ros/noetic/setup.bash
    python3 tools/official_bag_to_vins.py \\
        --bag data/mars/raw/HKisland01.bag \\
        --out data/mars/mars_hkisland01.bag
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
    sys.exit("Нужен opencv-python")

try:
    import rosbag
    import rospy
    from sensor_msgs.msg import CompressedImage, Image, Imu, NavSatFix
    from std_msgs.msg import Float32
except ImportError:
    sys.exit("Нужен ROS: source /opt/ros/noetic/setup.bash")

R_EARTH = 6371e3
G = 9.81

CAM_TOPICS = (
    "/left_camera/image/compressed",
    "/left_camera/image",
    "/camera/image/compressed",
    "/cam0/image_raw",
)
IMU_TOPICS = ("/livox/imu", "/imu0")
RTK_TOPICS = (
    "/dji_osdk_ros/rtk_position",
    "/dji_osdk_ros/gps_position",
)
BARO_TOPICS = (
    "/dji_osdk_ros/height_above_takeoff",
    "/dji_osdk_ros/height",
)


def list_topics(bag: rosbag.Bag) -> set[str]:
    return set(bag.get_type_and_topic_info().topics.keys())


def pick(available: set[str], candidates: tuple[str, ...]) -> str | None:
    for name in candidates:
        if name in available:
            return name
    return None


def decode_image(msg) -> np.ndarray | None:
    if hasattr(msg, "encoding"):
        # sensor_msgs/Image
        h, w = msg.height, msg.width
        if msg.encoding in ("mono8", "8UC1"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        if msg.encoding in ("rgb8", "bgr8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY if msg.encoding == "bgr8" else cv2.COLOR_RGB2GRAY)
        if msg.encoding == "bgra8":
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
            return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)


def stamp_of(msg) -> rospy.Time:
    return msg.header.stamp if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0 else rospy.Time(0)


def write_gt_tum(samples: list[tuple[float, float, float, float]], path: Path) -> None:
    if len(samples) < 5:
        print("RTK слишком короткий — эталон не записан", file=sys.stderr)
        return
    t, lat, lon, alt = (np.array(x) for x in zip(*samples))
    lat_r, lon_r = np.deg2rad(lat), np.deg2rad(lon)
    lat0, lon0, alt0 = lat_r[0], lon_r[0], alt[0]
    rh = R_EARTH + alt0
    east = (lon_r - lon0) * rh * math.cos(lat0)
    north = (lat_r - lat0) * rh
    up = alt - alt0
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# timestamp tx ty tz qx qy qz qw  (локальный ENU, RTK только эталон)\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.9f} {east[i]:.6f} {north[i]:.6f} {up[i]:.6f} 0 0 0 1\n")
    dist = float(np.sum(np.linalg.norm(np.diff(np.column_stack([east, north, up]), axis=0), axis=1)))
    print(f"Эталон RTK: {len(t)} поз, {dist:.1f} м → {path}")


def write_baro_csv(samples: list[tuple[float, float]], path: Path) -> None:
    if len(samples) < 5:
        print("Баро нет или короткое — CSV не записан", file=sys.stderr)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("bag_time_ns,data\n")
        for t_s, z in samples:
            f.write(f"{int(round(t_s * 1e9))},{z:.9f}\n")
    print(f"Баро: {len(samples)} отсчётов → {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scale", type=float, default=0.25)
    ap.add_argument("--gt", type=Path, default=None)
    ap.add_argument("--baro-csv", type=Path, default=None)
    ap.add_argument("--max-frames", type=int, default=0, help="0 = все кадры")
    ap.add_argument("--start", type=float, default=0.0, help="сек от начала bag")
    ap.add_argument("--duration", type=float, default=0.0, help="0 = до конца")
    args = ap.parse_args()
    if not args.bag.is_file():
        raise SystemExit(f"Нет bag: {args.bag}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    gt_path = args.gt or args.out.with_name(args.out.stem + "_gt.tum")
    baro_path = args.baro_csv or args.out.with_name(args.out.stem + "_baro.csv")

    with rosbag.Bag(str(args.bag), "r") as src:
        topics = list_topics(src)
        shown = ", ".join(sorted(topics)[:20])
        print("Топики:", shown, "..." if len(topics) > 20 else "")
        cam_topic = pick(topics, CAM_TOPICS)
        imu_topic = pick(topics, IMU_TOPICS)
        rtk_topic = pick(topics, RTK_TOPICS)
        baro_topic = pick(topics, BARO_TOPICS)
        if not cam_topic or not imu_topic:
            raise SystemExit(f"Нет камеры/IMU. Камера={cam_topic}, IMU={imu_topic}. Есть: {sorted(topics)}")
        if not rtk_topic:
            print("ВНИМАНИЕ: нет RTK/GPS топика — эталон GT не будет записан", file=sys.stderr)
        if not baro_topic:
            print("ВНИМАНИЕ: нет баро топика — CSV высоты не будет записан", file=sys.stderr)
        print(f"камера {cam_topic}, IMU {imu_topic}, RTK {rtk_topic}, баро {baro_topic}")

        # Первый проход IMU: масштаб g vs м/с²
        acc_mags = []
        for _, msg, _ in src.read_messages(topics=[imu_topic]):
            ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            acc_mags.append(math.sqrt(ax * ax + ay * ay + az * az))
            if len(acc_mags) >= 400:
                break
        med = float(np.median(acc_mags)) if acc_mags else 0.0
        accel_scale = G if 0.7 < med < 1.4 else 1.0
        print(f"IMU медиана |a|={med:.3f} → масштаб x{accel_scale}")

        t0 = float(src.get_start_time())
        t1 = float(src.get_end_time())
        t_lo = t0 + max(0.0, args.start)
        t_hi = t1 if args.duration <= 0 else min(t1, t_lo + args.duration)
        start_t = rospy.Time.from_sec(t_lo)
        end_t = rospy.Time.from_sec(t_hi)
        print(f"окно bag [{t_lo - t0:.1f}, {t_hi - t0:.1f}] с  (абсолют {t_lo:.3f}–{t_hi:.3f})")

        n_imu = n_cam = 0
        w_out = h_out = None
        rtk_samples: list[tuple[float, float, float, float]] = []
        baro_samples: list[tuple[float, float]] = []
        wanted = [imu_topic, cam_topic]
        if rtk_topic:
            wanted.append(rtk_topic)
        if baro_topic:
            wanted.append(baro_topic)

        with rosbag.Bag(str(args.out), "w") as dst:
            for topic, msg, t_bag in src.read_messages(
                    topics=wanted, start_time=start_t, end_time=end_t):
                if topic == imu_topic:
                    m = Imu()
                    m.header.stamp = stamp_of(msg)
                    if m.header.stamp.to_sec() <= 0:
                        m.header.stamp = t_bag
                    m.header.frame_id = "imu"
                    m.angular_velocity = msg.angular_velocity
                    m.linear_acceleration.x = msg.linear_acceleration.x * accel_scale
                    m.linear_acceleration.y = msg.linear_acceleration.y * accel_scale
                    m.linear_acceleration.z = msg.linear_acceleration.z * accel_scale
                    m.orientation_covariance[0] = -1.0
                    dst.write("/imu0", m, m.header.stamp)
                    n_imu += 1
                    continue

                if topic == cam_topic:
                    if args.max_frames and n_cam >= args.max_frames:
                        continue
                    img = decode_image(msg)
                    if img is None:
                        continue
                    if w_out is None:
                        h_in, w_in = img.shape[:2]
                        w_out = int(round(w_in * args.scale)) // 8 * 8
                        h_out = int(round(h_in * args.scale)) // 8 * 8
                        print(f"кадр {w_in}x{h_in} → {w_out}x{h_out}")
                    if img.shape[1] != w_out or img.shape[0] != h_out:
                        img = cv2.resize(img, (w_out, h_out), interpolation=cv2.INTER_AREA)
                    m = Image()
                    m.header.stamp = stamp_of(msg)
                    if m.header.stamp.to_sec() <= 0:
                        m.header.stamp = t_bag
                    m.header.frame_id = "cam0"
                    m.height, m.width = h_out, w_out
                    m.encoding = "mono8"
                    m.is_bigendian = 0
                    m.step = w_out
                    m.data = img.tobytes()
                    dst.write("/cam0/image_raw", m, m.header.stamp)
                    n_cam += 1
                    if n_cam % 200 == 0:
                        print(f"  кадров {n_cam}")
                    continue

                if rtk_topic and topic == rtk_topic:
                    try:
                        lat = float(msg.latitude)
                        lon = float(msg.longitude)
                        alt = float(msg.altitude)
                    except AttributeError:
                        continue
                    ts = stamp_of(msg)
                    t_s = ts.to_sec() if ts.to_sec() > 0 else t_bag.to_sec()
                    if math.isfinite(lat) and math.isfinite(lon):
                        rtk_samples.append((t_s, lat, lon, alt))
                    continue

                if baro_topic and topic == baro_topic:
                    z = float(getattr(msg, "data", msg))
                    ts = stamp_of(msg) if hasattr(msg, "header") else t_bag
                    t_s = ts.to_sec() if hasattr(ts, "to_sec") and ts.to_sec() > 0 else t_bag.to_sec()
                    baro_samples.append((t_s, z))

    write_gt_tum(rtk_samples, gt_path)
    write_baro_csv(baro_samples, baro_path)
    print(f"Готово: {args.out}  IMU={n_imu}  кадры={n_cam}  ({args.out.stat().st_size / 1e9:.2f} ГБ)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
