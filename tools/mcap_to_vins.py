#!/usr/bin/env python3
"""MCAP MARS-LVIG (ROS2) → rosbag для VINS-Mono.

Читает только камеру, /livox/imu, RTK и баро — LiDAR не копируется.

    source /opt/ros/noetic/setup.bash
    python3 tools/mcap_to_vins.py \\
        --mcap data/mars/raw/HKairport03.mcap \\
        --out data/mars/mars_hkairport03_livox.bag

Окно относительно начала записи: --start 62 --duration 300
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from ruamel.yaml import YAML

try:
    import cv2
except ImportError:
    sys.exit("Нужен opencv-python")

try:
    import rosbag
    import rospy
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import Image, Imu
except ImportError:
    sys.exit("Нужен ROS: source /opt/ros/noetic/setup.bash")

try:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    sys.exit("Нужен пакет rosbags: python3 -m pip install --user rosbags")

# reuse GT/baro writers
sys.path.insert(0, str(Path(__file__).resolve().parent))
from official_bag_to_vins import G, write_baro_csv, write_gt_tum  # noqa: E402

CAM_TOPICS = (
    "/left_camera/image/compressed",
    "/left_camera/image",
    "/camera/image/compressed",
    "/cam0/image_raw",
)
IMU_TOPICS = ("/livox/imu",)
AHRS_TOPICS = ("/dji_osdk_ros/imu",)
RTK_TOPICS = (
    "/dji_osdk_ros/rtk_position",
    "/dji_osdk_ros/gps_position",
)
BARO_TOPICS = (
    "/dji_osdk_ros/height_above_takeoff",
    "/dji_osdk_ros/height",
)

HF_META = (
    "https://huggingface.co/datasets/DapengFeng/MCAP/resolve/main/"
    "mars_lvig/{name}/metadata.yaml"
)


def ensure_ros2_dir(src: Path) -> Path:
    """rosbags AnyReader хочет каталог с metadata.yaml, не голый .mcap."""
    if src.is_dir():
        if not (src / "metadata.yaml").is_file():
            raise SystemExit(f"В {src} нет metadata.yaml")
        return src
    if src.suffix != ".mcap" or not src.is_file():
        raise SystemExit(f"Нужен .mcap или rosbag2-каталог, а не {src}")

    bagdir = src.parent / src.stem
    bagdir.mkdir(exist_ok=True)
    meta = bagdir / "metadata.yaml"
    if not meta.is_file():
        url = HF_META.format(name=src.stem)
        print(f"качаю metadata.yaml: {url}")
        import urllib.request
        try:
            urllib.request.urlretrieve(url, meta)
        except Exception as exc:
            raise SystemExit(
                f"Нет {meta} и не скачалось с HuggingFace: {exc}"
            ) from exc

    info = YAML(typ="safe").load(meta.read_text(encoding="utf-8"))
    rel = info["rosbag2_bagfile_information"]["relative_file_paths"][0]
    target = bagdir / Path(rel).name
    if target.exists() or target.is_symlink():
        if target.resolve() != src.resolve():
            print(f"уже есть {target} → {target.resolve()}")
    else:
        target.symlink_to(src.resolve())
        print(f"symlink {target} → {src}")
    return bagdir


def pick(available: set[str], candidates: tuple[str, ...]) -> str | None:
    for name in candidates:
        if name in available:
            return name
    return None


def as_bytes(data) -> bytes:
    if isinstance(data, (bytes, bytearray, memoryview)):
        return bytes(data)
    return np.asarray(data).tobytes()


def stamp_sec(msg, fallback_ns: int) -> float:
    h = getattr(msg, "header", None)
    if h is not None:
        st = getattr(h, "stamp", None)
        if st is not None:
            sec = int(getattr(st, "sec", 0) or 0)
            nsec = int(getattr(st, "nanosec", getattr(st, "nsec", 0)) or 0)
            if sec > 0 or nsec > 0:
                return sec + nsec * 1e-9
    return fallback_ns * 1e-9


def decode_image(msg, msgtype: str) -> np.ndarray | None:
    mtype = msgtype.split("/")[-1]
    if "Compressed" in mtype or hasattr(msg, "format"):
        buf = np.frombuffer(as_bytes(msg.data), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
    h, w = int(msg.height), int(msg.width)
    enc = str(getattr(msg, "encoding", "mono8"))
    raw = np.frombuffer(as_bytes(msg.data), dtype=np.uint8)
    if enc in ("mono8", "8UC1"):
        return raw.reshape(h, w)
    if enc in ("rgb8", "bgr8"):
        img = raw.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY if enc == "bgr8" else cv2.COLOR_RGB2GRAY)
    if enc == "bgra8":
        return cv2.cvtColor(raw.reshape(h, w, 4), cv2.COLOR_BGRA2GRAY)
    return None


def vec3(obj) -> tuple[float, float, float]:
    return float(obj.x), float(obj.y), float(obj.z)


def quat4(obj) -> tuple[float, float, float, float]:
    return float(obj.x), float(obj.y), float(obj.z), float(obj.w)


def write_dji_imu_csv(samples: list[tuple], path: Path) -> None:
    """Компактный CSV кватерниона AHRS DJI — курс, не RTK."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write(
            "bag_time_ns,orientation.x,orientation.y,orientation.z,"
            "orientation.w,angular_velocity.z,linear_acceleration.z\n"
        )
        for t_s, qx, qy, qz, qw, wz, az in samples:
            f.write(
                f"{int(round(t_s * 1e9))},{qx:.9f},{qy:.9f},{qz:.9f},"
                f"{qw:.9f},{wz:.9f},{az:.9f}\n"
            )
    print(f"AHRS DJI: {len(samples)}  → {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mcap", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--scale", type=float, default=0.25)
    ap.add_argument("--gt", type=Path, default=None)
    ap.add_argument("--baro-csv", type=Path, default=None)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = до конца")
    ap.add_argument(
        "--imu-csv",
        type=Path,
        default=None,
        help="CSV кватерниона /dji_osdk_ros/imu (AHRS, не RTK)",
    )
    ap.add_argument("--list", action="store_true", help="только список топиков")
    args = ap.parse_args()
    if not args.mcap.exists():
        raise SystemExit(f"Нет MCAP: {args.mcap}")
    bagdir = ensure_ros2_dir(args.mcap)
    if not args.list and not args.out:
        raise SystemExit("нужен --out (или --list)")

    gt_path = baro_path = None
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        gt_path = args.gt or args.out.with_name(args.out.stem + "_gt.tum")
        baro_path = args.baro_csv or args.out.with_name(args.out.stem + "_baro.csv")

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bagdir], default_typestore=typestore) as reader:
        topics = {c.topic for c in reader.connections}
        if args.list:
            counts = {}
            for c in reader.connections:
                counts[c.topic] = counts.get(c.topic, 0) + c.msgcount
            for name, n in sorted(counts.items()):
                print(f"{n:8d}  {name}")
            return 0

        cam_topic = pick(topics, CAM_TOPICS)
        imu_topic = pick(topics, IMU_TOPICS)
        ahrs_topic = pick(topics, AHRS_TOPICS)
        rtk_topic = pick(topics, RTK_TOPICS)
        baro_topic = pick(topics, BARO_TOPICS)
        if not cam_topic or not imu_topic:
            raise SystemExit(
                f"Нет камеры/Livox IMU. cam={cam_topic} imu={imu_topic}. Есть: {sorted(topics)}")
        print(f"камера {cam_topic}, IMU {imu_topic}, AHRS {ahrs_topic}, "
              f"RTK {rtk_topic}, баро {baro_topic}")

        t0_ns = int(reader.start_time)
        t1_ns = int(reader.end_time)
        lo_ns = t0_ns + int(max(0.0, args.start) * 1e9)
        hi_ns = t1_ns if args.duration <= 0 else min(t1_ns, lo_ns + int(args.duration * 1e9))
        print(f"окно [{(lo_ns - t0_ns) * 1e-9:.1f}, {(hi_ns - t0_ns) * 1e-9:.1f}] с  "
              f"(длительность записи {(t1_ns - t0_ns) * 1e-9:.1f} с)")

        wanted = {cam_topic, imu_topic}
        if ahrs_topic:
            wanted.add(ahrs_topic)
        if rtk_topic:
            wanted.add(rtk_topic)
        if baro_topic:
            wanted.add(baro_topic)
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            raise SystemExit("нет соединений для выбранных топиков")
        imu_conns = [c for c in reader.connections if c.topic == imu_topic]

        acc_mags = []
        for conn, ts, raw in reader.messages(connections=imu_conns):
            msg = reader.deserialize(raw, conn.msgtype)
            ax, ay, az = vec3(msg.linear_acceleration)
            acc_mags.append(math.sqrt(ax * ax + ay * ay + az * az))
            if len(acc_mags) >= 400:
                break
        med = float(np.median(acc_mags)) if acc_mags else 0.0
        accel_scale = G if 0.7 < med < 1.4 else 1.0
        print(f"IMU медиана |a|={med:.3f} → масштаб x{accel_scale}")

        n_imu = n_cam = n_baro = n_ahrs = 0
        w_out = h_out = None
        rtk_samples: list[tuple[float, float, float, float]] = []
        baro_samples: list[tuple[float, float]] = []
        ahrs_samples: list[tuple] = []
        imu_csv = args.imu_csv
        if imu_csv is None and args.out is not None:
            imu_csv = args.out.with_name(args.out.stem + "_dji_imu.csv")

        with rosbag.Bag(str(args.out), "w") as dst:
            for conn, ts, raw in reader.messages(
                    connections=conns, start=lo_ns, stop=hi_ns):
                msg = reader.deserialize(raw, conn.msgtype)
                t_s = stamp_sec(msg, ts)

                if conn.topic == imu_topic:
                    ax, ay, az = vec3(msg.linear_acceleration)
                    wx, wy, wz = vec3(msg.angular_velocity)
                    m = Imu()
                    m.header.stamp = rospy.Time.from_sec(t_s)
                    m.header.frame_id = "imu"
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = wx, wy, wz
                    m.linear_acceleration.x = ax * accel_scale
                    m.linear_acceleration.y = ay * accel_scale
                    m.linear_acceleration.z = az * accel_scale
                    m.orientation_covariance[0] = -1.0
                    dst.write("/imu0", m, m.header.stamp)
                    n_imu += 1
                    continue

                if conn.topic == cam_topic:
                    if args.max_frames and n_cam >= args.max_frames:
                        continue
                    img = decode_image(msg, conn.msgtype)
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
                    m.header.stamp = rospy.Time.from_sec(t_s)
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

                if rtk_topic and conn.topic == rtk_topic:
                    lat = float(msg.latitude)
                    lon = float(msg.longitude)
                    alt = float(msg.altitude)
                    if math.isfinite(lat) and math.isfinite(lon):
                        rtk_samples.append((t_s, lat, lon, alt))
                    continue

                if ahrs_topic and conn.topic == ahrs_topic:
                    try:
                        qx, qy, qz, qw = quat4(msg.orientation)
                        _wx, _wy, wz = vec3(msg.angular_velocity)
                        _ax, _ay, az = vec3(msg.linear_acceleration)
                    except AttributeError:
                        continue
                    if abs(qw) + abs(qx) + abs(qy) + abs(qz) > 0.1:
                        ahrs_samples.append((t_s, qx, qy, qz, qw, wz, az))
                        n_ahrs += 1
                    continue

                if baro_topic and conn.topic == baro_topic:
                    z = float(msg.data)
                    baro_samples.append((t_s, z))
                    # /baro нужен estimator'у как опора масштаба (patch_baro_ground.py)
                    m = PointStamped()
                    m.header.stamp = rospy.Time.from_sec(t_s)
                    m.header.frame_id = "baro"
                    m.point.z = z
                    dst.write("/baro", m, m.header.stamp)
                    n_baro += 1

    write_gt_tum(rtk_samples, gt_path)
    write_baro_csv(baro_samples, baro_path)
    if imu_csv is not None and ahrs_samples:
        write_dji_imu_csv(ahrs_samples, imu_csv)
    print(f"Готово: {args.out}  IMU={n_imu}  кадры={n_cam}  баро={n_baro}  "
          f"AHRS={n_ahrs}  ({args.out.stat().st_size / 1e9:.2f} ГБ)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
