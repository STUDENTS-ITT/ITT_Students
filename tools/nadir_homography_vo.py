#!/usr/bin/env python3
"""Надирная VO для поля (HKairport): планиметрия, гомография, essential.

VINS-Mono на надире с DJI IMU даёт reboot и ломает форму пути. На плоской
земле масштаб берётся из высоты (баро, иначе крейсерская / оценка IMU),
без подмешивания RTK в траекторию. RTK — только ATE и картинка сравнения.

    python3 tools/nadir_homography_vo.py \\
        --images data/mars/dso/images \\
        --stamps data/mars/camera/timestamps.csv \\
        --imu data/mars/aux/dji_osdk_ros_imu.csv \\
        --baro data/mars/aux/dji_osdk_ros_height_above_takeoff.csv \\
        --gt data/mars/mars_hkairport03_gt.tum \\
        --out results/eval_mars_scene_field
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from mars_stamp import detect_camera_offset, load_csv_series, load_dji_imu_stamps, row_stamp_s  # noqa: E402
from plot_vo_like_report import plot_figure2, plot_figure3  # noqa: E402

# Камера → IMU (mars_nadir.yaml): X_cam вправо = −Y_imu, Y_cam вниз кадра = −X_imu.
R_IC = np.array(
    [[0.0, -1.0, 0.0],
     [-1.0, 0.0, 0.0],
     [0.0, 0.0, -1.0]],
    dtype=np.float64,
)

K_DEFAULT = np.array(
    [[358.7473, 0.0, 292.9477],
     [0.0, 361.0850, 261.2250],
     [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
DIST_DEFAULT = np.array([-0.056, 0.118, 0.00122, 0.00064], dtype=np.float64)


def load_stamps(path: Path) -> np.ndarray:
    t = []
    with path.open(encoding="utf-8", errors="replace") as f:
        r = csv.DictReader(f)
        for row in r:
            v = float(row.get("stamp_ns") or row.get("timestamp") or next(iter(row.values())))
            if v > 1e15:
                v *= 1e-9
            t.append(v)
    return np.asarray(t, dtype=np.float64)


def load_imu(path: Path, camera_offset: float = 0.0) -> dict[str, np.ndarray]:
    t, qx, qy, qz, qw, wz, az = [], [], [], [], [], [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                tt = row_stamp_s(row)
                if tt is None:
                    continue
                t.append(tt - camera_offset)
                qx.append(float(row["orientation.x"]))
                qy.append(float(row["orientation.y"]))
                qz.append(float(row["orientation.z"]))
                qw.append(float(row["orientation.w"]))
                wz.append(float(row.get("angular_velocity.z") or 0.0))
                az.append(float(row.get("linear_acceleration.z") or 0.0))
            except (KeyError, TypeError, ValueError):
                continue
    return {
        "t": np.asarray(t),
        "qx": np.asarray(qx), "qy": np.asarray(qy),
        "qz": np.asarray(qz), "qw": np.asarray(qw),
        "wz": np.asarray(wz), "az": np.asarray(az),
    }


def quat_yaw_enu(qx, qy, qz, qw) -> float:
    """Рыскание тела FLU в ENU (рад), 0 ≈ восток.

    Знак минус: у DJI OSDK yaw кватерниона зеркален курсу (совпадает с
    rtk_yaw после инверсии: −77°→77°, −172°→172°).
    """
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return -math.atan2(siny, cosy)


def interp_col(t_q: np.ndarray, t: np.ndarray, y: np.ndarray) -> np.ndarray:
    return np.interp(t_q, t, y)


def estimate_cruise_alt(imu: dict, t_img: np.ndarray, g: float = 9.79362) -> np.ndarray:
    """Высота без RTK: набор ~55 с до крейсера 80 м (типовая съёмка aэродрома).

    Интеграл DJI az−g на этом bag не даёт AGL (az уже ≈ g). Баро-топик
    в CSV тоже не AGL. Набор — линейный рамп, крейсер — константа.
    """
    _ = imu, g
    t0 = float(t_img[0])
    t_rel = t_img - t0
    climb_s, h0, cruise = 55.0, 8.0, 80.0
    alt = h0 + (cruise - h0) * np.clip(t_rel / climb_s, 0.0, 1.0)
    return alt


def usable_baro(t_b: np.ndarray, z_b: np.ndarray) -> bool:
    if len(z_b) < 50:
        return False
    return float(np.percentile(np.abs(z_b), 80)) > 20.0


def list_images(folder: Path) -> list[Path]:
    files = sorted(p for p in folder.iterdir() if p.suffix.lower() in {".png", ".jpg", ".jpeg"})
    if not files:
        raise SystemExit(f"Нет кадров в {folder}")
    return files


def undistort_pts(pts: np.ndarray, k: np.ndarray, dist: np.ndarray) -> np.ndarray:
    p = pts.reshape(-1, 1, 2).astype(np.float64)
    u = cv2.undistortPoints(p, k, dist, P=k)
    return u.reshape(-1, 2)


def load_k_yaml(path: Path) -> tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise SystemExit(f"не читается yaml {path}")
    proj = fs.getNode("projection_parameters")
    dist = fs.getNode("distortion_parameters")
    fx, fy = float(proj.getNode("fx").real()), float(proj.getNode("fy").real())
    cx, cy = float(proj.getNode("cx").real()), float(proj.getNode("cy").real())
    k1 = float(dist.getNode("k1").real())
    k2 = float(dist.getNode("k2").real())
    p1 = float(dist.getNode("p1").real())
    p2 = float(dist.getNode("p2").real())
    fs.release()
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    return k, np.array([k1, k2, p1, p2], dtype=np.float64)


def iter_bag_frames(bag_path: Path):
    try:
        import rosbag
    except ImportError as exc:
        raise SystemExit("нужен ROS: source /opt/ros/noetic/setup.bash") from exc
    with rosbag.Bag(str(bag_path)) as bag:
        for _topic, msg, _t in bag.read_messages(topics=["/cam0/image_raw"]):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            ts = msg.header.stamp.to_sec()
            yield ts, img


def track(prev_gray: np.ndarray, gray: np.ndarray, k: np.ndarray, dist: np.ndarray):
    pts = cv2.goodFeaturesToTrack(prev_gray, maxCorners=800, qualityLevel=0.01,
                                  minDistance=12, blockSize=7)
    if pts is None or len(pts) < 20:
        return None
    nxt, st, _ = cv2.calcOpticalFlowPyrLK(
        prev_gray, gray, pts, None,
        winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    if nxt is None:
        return None
    good0 = pts[st.ravel() == 1].reshape(-1, 2)
    good1 = nxt[st.ravel() == 1].reshape(-1, 2)
    if len(good0) < 20:
        return None
    u0 = undistort_pts(good0, k, dist)
    u1 = undistort_pts(good1, k, dist)
    return u0, u1, good0, good1


class TrackVideo:
    """H.264 ролик треков точек, не больше max_mb."""

    def __init__(self, path: Path, fps: float = 10.0, max_mb: float = 480.0):
        self.path = path
        self.fps = fps
        self.max_mb = max_mb
        self.proc = None
        self.size = None
        self.frames = 0

    def write(self, gray: np.ndarray, pix0, pix1, label: str) -> None:
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if pix0 is not None and pix1 is not None:
            for p0, p1 in zip(pix0, pix1):
                a = (int(p0[0]), int(p0[1]))
                b = (int(p1[0]), int(p1[1]))
                cv2.line(vis, a, b, (0, 220, 255), 1, cv2.LINE_AA)
                cv2.circle(vis, b, 2, (0, 255, 80), -1, lineType=cv2.LINE_AA)
        n = 0 if pix1 is None else len(pix1)
        cv2.putText(vis, f"{label}  pts={n}", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        h, w = vis.shape[:2]
        if h % 2 or w % 2:
            vis = cv2.resize(vis, (w - w % 2, h - h % 2))
            h, w = vis.shape[:2]
        if self.proc is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.size = (w, h)
            cmd = [
                "ffmpeg", "-y", "-loglevel", "error",
                "-f", "rawvideo", "-vcodec", "rawvideo",
                "-s", f"{w}x{h}", "-pix_fmt", "bgr24", "-r", str(self.fps),
                "-i", "-", "-an", "-c:v", "libx264", "-preset", "fast",
                "-crf", "26", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                str(self.path),
            ]
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        assert self.proc.stdin is not None
        if (w, h) != self.size:
            vis = cv2.resize(vis, self.size)
        self.proc.stdin.write(vis.tobytes())
        self.frames += 1

    def close(self) -> None:
        if self.proc is None:
            return
        if self.proc.stdin:
            self.proc.stdin.close()
        self.proc.wait()
        if self.path.is_file() and self.path.stat().st_size > self.max_mb * 1e6:
            tmp = self.path.with_suffix(".small.mp4")
            subprocess.check_call([
                "ffmpeg", "-y", "-loglevel", "error", "-i", str(self.path),
                "-c:v", "libx264", "-preset", "fast", "-crf", "32",
                "-pix_fmt", "yuv420p", str(tmp),
            ])
            tmp.replace(self.path)
        mb = self.path.stat().st_size / 1e6 if self.path.is_file() else 0.0
        print(f"видео {self.path.name}: {self.frames} кадр, {mb:.1f} МБ")


def cam_to_imu_planar(c_cam: np.ndarray) -> np.ndarray:
    """OpenCV (x вправо, y вниз) → FLU. Y вниз — это НАЗАД, не вперёд (отчёт §6 зеркало)."""
    c = np.array([float(c_cam[0]), float(c_cam[1]), 0.0], dtype=np.float64)
    t_imu = R_IC @ c
    t_imu[2] = 0.0
    return t_imu


def step_limit_m(u0: np.ndarray, u1: np.ndarray, alt: float, fx: float) -> float:
    """Независимая оценка шага p·H/f; обрезка если модель дала >3× (STEP_LENGTH_LIMIT)."""
    p = float(np.median(np.linalg.norm(u1 - u0, axis=1)))
    return max(3.0 * p * alt / max(fx, 1.0), 0.05)


def step_planimetry(u0: np.ndarray, u1: np.ndarray, alt: float, fx: float, fy: float):
    d = u1 - u0
    dx = float(np.median(d[:, 0]))
    dy = float(np.median(d[:, 1]))
    right_m = dx * alt / fx
    fwd_m = -dy * alt / fy
    return np.array([fwd_m, -right_m, 0.0])


def _pick_homography(H: np.ndarray, k: np.ndarray):
    nsol, Rs, ts, ns = cv2.decomposeHomographyMat(H, k)
    best = None
    best_score = -1e9
    z_axis = np.array([0.0, 0.0, 1.0])
    for i in range(nsol):
        R = Rs[i]
        t = ts[i].reshape(3)
        n = ns[i].reshape(3)
        n = n / (np.linalg.norm(n) + 1e-12)
        if n[2] < 0:
            n = -n
            t = -t
        # малый поворот, нормаль ≈ оптическая ось, t_z небольшой (полёт горизонтальный)
        ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5)))
        score = float(n @ z_axis) - 0.15 * abs(t[2]) - 0.5 * ang
        if score > best_score:
            best_score = score
            best = (R, t, n)
    return best


def step_homography(u0: np.ndarray, u1: np.ndarray, alt: float, k: np.ndarray):
    H, mask = cv2.findHomography(u0, u1, cv2.RANSAC, 2.5)
    if H is None:
        return None
    picked = _pick_homography(H, k)
    if picked is None:
        return None
    R, t_over_d, _n = picked
    t_cam = t_over_d.reshape(3) * alt
    c_cam = -R.T @ t_cam
    t_imu = cam_to_imu_planar(c_cam)
    ninl = int(mask.sum()) if mask is not None else 0
    return t_imu, ninl


def step_essential(u0: np.ndarray, u1: np.ndarray, alt: float, k: np.ndarray, fx: float, fy: float):
    """Масштаб не единица: база recoverPose=1, земля на глубине d → 1 ед. = H/d м (отчёт §6)."""
    _ = fy
    E, mask = cv2.findEssentialMat(u0, u1, k, method=cv2.RANSAC, prob=0.999, threshold=1.5)
    if E is None:
        return None
    _, R, t, mask2 = cv2.recoverPose(E, u0, u1, k)
    t = t.reshape(3)
    if np.linalg.norm(t) < 1e-9:
        return None
    p0 = k @ np.hstack([np.eye(3), np.zeros((3, 1))])
    p1 = k @ np.hstack([R, t.reshape(3, 1)])
    pts4 = cv2.triangulatePoints(p0, p1, u0.T.astype(np.float64), u1.T.astype(np.float64))
    pts = (pts4[:3] / np.maximum(pts4[3], 1e-9)).T
    z = pts[:, 2]
    z = z[np.isfinite(z) & (z > 1e-3)]
    if len(z) < 8:
        return None
    d = float(np.median(z))
    scale = alt / d  # H/d метров на условную единицу
    t_cam = t * scale
    c_cam = -R.T @ t_cam
    t_imu = cam_to_imu_planar(c_cam)
    ninl = int(mask2.sum()) if mask2 is not None else 0
    return t_imu, ninl


def rotate_xy(vec: np.ndarray, yaw_from_north: float) -> np.ndarray:
    """Тело FLU (x вперёд, y влево) → ENU. yaw — курс от севера, как DJI/rtk_yaw."""
    x, y = float(vec[0]), float(vec[1])
    psi = yaw_from_north
    east = x * math.sin(psi) - y * math.cos(psi)
    north = x * math.cos(psi) + y * math.sin(psi)
    return np.array([east, north, 0.0])


def read_tum(path: Path):
    t, p = [], []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            a = line.split()
            t.append(float(a[0]))
            p.append([float(a[1]), float(a[2]), float(a[3])])
    return np.asarray(t), np.asarray(p)


def write_tum(path: Path, t: np.ndarray, p: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# nadir planar VO, local ENU (east north up), Z=alt\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.9f} {p[i, 0]:.6f} {p[i, 1]:.6f} {p[i, 2]:.6f} 0 0 0 1\n")


def se2_align_constant_yaw(est: np.ndarray, gt: np.ndarray, skip_s: float, t: np.ndarray) -> np.ndarray:
    """Один общий yaw+сдвиг (без масштаба), по крейсерскому участку. Не по точкам RTK."""
    mask = t >= t[0] + skip_s
    src = est[mask, :2]
    dst = gt[mask, :2]
    if len(src) < 20:
        src, dst = est[:, :2], gt[:, :2]
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d)
    u, _, vt = np.linalg.svd(H)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    out = est.copy()
    out[:, :2] = (r @ est[:, :2].T).T + (mu_d - r @ mu_s)
    return out


def metrics(est: np.ndarray, gt_i: np.ndarray) -> dict:
    pe = np.sum(np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1))
    pg = np.sum(np.linalg.norm(np.diff(gt_i[:, :2], axis=0), axis=1))
    err = np.linalg.norm(est[:, :2] - gt_i[:, :2], axis=1)
    fin = np.linalg.norm(est[-1, :2] - est[0, :2])
    return {
        "rmse": float(np.sqrt(np.mean(err ** 2))),
        "path_est": pe,
        "path_gt": pg,
        "path_ratio": pe / pg if pg > 1e-6 else float("nan"),
        "end_from_start": fin,
        "n": len(err),
    }


def plot_report(out_dir: Path, t: np.ndarray, gt: np.ndarray, trajs: dict, stats: dict,
                title: str = "VO vs RTK", stem: str = "nadir_vo_like_report"):
    plot_figure3(out_dir / f"{stem}.png", gt, trajs, stats)
    plot_figure2(out_dir / f"{stem}_overlay.png", t, gt, trajs, stats, title)


def load_frames(args) -> tuple[list[float], list]:
    """Кадры из папки или rosbag. Второй список — Path или уже прочитанный gray."""
    if args.bag is not None:
        t_img, frames = [], []
        for ts, img in iter_bag_frames(args.bag):
            t_img.append(ts)
            frames.append(img)
            if args.max_frames and len(frames) >= args.max_frames:
                break
        if args.stride > 1:
            t_img = t_img[:: args.stride]
            frames = frames[:: args.stride]
        if len(frames) < 2:
            raise SystemExit(f"мало кадров в {args.bag}")
        return t_img, frames
    if args.images is None or args.stamps is None:
        raise SystemExit("нужен --images/--stamps или --bag")
    files = list_images(args.images)
    t_img = list(load_stamps(args.stamps))
    n = min(len(files), len(t_img))
    files, t_img = files[:n], t_img[:n]
    if args.stride > 1:
        files = files[:: args.stride]
        t_img = t_img[:: args.stride]
    if args.max_frames:
        files = files[: args.max_frames]
        t_img = t_img[: args.max_frames]
    return t_img, files


def gray_at(item):
    if isinstance(item, np.ndarray):
        return item
    return cv2.imread(str(item), cv2.IMREAD_GRAYSCALE)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", type=Path, default=None)
    ap.add_argument("--stamps", type=Path, default=None)
    ap.add_argument("--bag", type=Path, default=None, help="VINS bag /cam0/image_raw")
    ap.add_argument("--imu", type=Path, default=None)
    ap.add_argument("--baro", type=Path, default=None)
    ap.add_argument("--gt", type=Path, default=None)
    ap.add_argument("--yaml", type=Path, default=None, help="intrinsics VINS yaml")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--title", type=str, default="VO без коррекции по GPS, курс AHRS DJI")
    ap.add_argument("--video", type=Path, default=None)
    ap.add_argument("--height", type=float, default=None,
                    help="фиксированная высота, м (если баро/IMU непригодны)")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--camera-stamps", type=Path, default=None)
    ap.add_argument("--camera-offset", type=float, default=None,
                    help="сек: DJI→camera; auto из timestamps.csv")
    args = ap.parse_args()

    t_img, frames = load_frames(args)
    t_img = np.asarray(t_img, dtype=np.float64)
    cam_off = args.camera_offset
    if cam_off is None:
        cam_off = detect_camera_offset(args.camera_stamps, args.imu, float(t_img[0]))
    if abs(cam_off) > 0.05:
        print(f"DJI→camera offset: {cam_off:.3f} с (header.stamp)")

    if args.yaml and args.yaml.is_file():
        k, dist = load_k_yaml(args.yaml)
        print(f"K из {args.yaml.name}: fx={k[0, 0]:.1f} fy={k[1, 1]:.1f}")
    else:
        k, dist = K_DEFAULT, DIST_DEFAULT
    fx, fy = k[0, 0], k[1, 1]

    alt = np.full(len(t_img), 80.0)
    if args.imu and args.imu.is_file():
        imu = load_imu(args.imu, camera_offset=cam_off)
        if len(imu["t"]) < 10:
            raise SystemExit(f"мало AHRS в {args.imu}")
        yaw_s = np.array([
            quat_yaw_enu(
                interp_col(np.array([tt]), imu["t"], imu["qx"])[0],
                interp_col(np.array([tt]), imu["t"], imu["qy"])[0],
                interp_col(np.array([tt]), imu["t"], imu["qz"])[0],
                interp_col(np.array([tt]), imu["t"], imu["qw"])[0],
            )
            for tt in t_img
        ])
        alt = estimate_cruise_alt(imu, t_img)
    else:
        yaw_s = np.zeros(len(t_img))

    if args.baro and args.baro.is_file():
        tb, zb = load_csv_series(args.baro)
        if abs(cam_off) > 0.05:
            tb = tb - cam_off
        if usable_baro(tb, zb):
            alt = interp_col(t_img, tb, zb)
            print(f"высота: баро (медиана {np.median(alt):.1f} м)")
        else:
            print(f"баро не AGL (p80={np.percentile(np.abs(zb), 80):.2f} м) — IMU/крейсер")
    if args.height is not None:
        alt[:] = args.height
        print(f"высота: фиксированная {args.height:.1f} м")
    else:
        print(f"высота: median={np.median(alt):.1f} max={alt.max():.1f} м")

    keys = ("planimetry", "homography", "essential")
    pos = {k: np.zeros(3) for k in keys}
    traj = {k: [pos[k].copy()] for k in keys}
    t_out = [float(t_img[0])]

    prev = gray_at(frames[0])
    if prev is None:
        raise SystemExit("не читается первый кадр")

    video = TrackVideo(args.video, fps=10.0) if args.video else None
    if video is not None:
        hud = args.title.split(":")[0].replace(" ", "")[:24]
        video.write(prev, None, None, hud)

    for i in range(1, len(frames)):
        gray = gray_at(frames[i])
        if gray is None:
            for kk in keys:
                traj[kk].append(pos[kk].copy())
            t_out.append(float(t_img[i]))
            continue
        pair = track(prev, gray, k, dist)
        prev = gray
        h = max(float(alt[i]), 5.0)
        yaw = float(yaw_s[i])
        dp = {kk: np.zeros(3) for kk in keys}
        pix0 = pix1 = None
        if pair is not None:
            u0, u1, pix0, pix1 = pair
            dp["planimetry"] = rotate_xy(step_planimetry(u0, u1, h, fx, fy), yaw)
            hs = step_homography(u0, u1, h, k)
            if hs is not None:
                t_imu, _inl = hs
                dp["homography"] = rotate_xy(t_imu, yaw)
            es = step_essential(u0, u1, h, k, fx, fy)
            if es is not None:
                t_imu, _inl = es
                dp["essential"] = rotate_xy(t_imu, yaw)
            lim = step_limit_m(u0, u1, h, fx)
            for kk in keys:
                nxy = float(np.linalg.norm(dp[kk][:2]))
                if nxy > lim:
                    dp[kk][:2] *= lim / nxy
        for kk in keys:
            pos[kk] = pos[kk] + dp[kk]
            pos[kk][2] = h
            traj[kk].append(pos[kk].copy())
        t_out.append(float(t_img[i]))
        if video is not None:
            video.write(gray, pix0, pix1, f"{hud} {i}/{len(frames)}")
        if i % 400 == 0:
            print(f"  кадр {i}/{len(frames)}")

    if video is not None:
        video.close()

    t_arr = np.asarray(t_out)
    arr = {kk: np.vstack(traj[kk]) for kk in keys}
    args.out.mkdir(parents=True, exist_ok=True)
    for kk in keys:
        write_tum(args.out / f"{kk}.tum", t_arr, arr[kk])

    if args.gt and args.gt.is_file():
        tg, pg = read_tum(args.gt)
        gti = np.column_stack([np.interp(t_arr, tg, pg[:, j]) for j in range(3)])
        stats_raw, stats_se2, aligned = {}, {}, {}
        for kk in keys:
            stats_raw[kk] = metrics(arr[kk], gti)
            aligned[kk] = se2_align_constant_yaw(arr[kk], gti, 55.0, t_arr)
            stats_se2[kk] = metrics(aligned[kk], gti)
            write_tum(args.out / f"{kk}_aligned.tum", t_arr, aligned[kk])
            print(f"{kk:12s} RMSE raw {stats_raw[kk]['rmse']:.1f} / SE2 {stats_se2[kk]['rmse']:.1f} м  "
                  f"путь {stats_raw[kk]['path_est']:.0f}/{stats_raw[kk]['path_gt']:.0f} "
                  f"({stats_raw[kk]['path_ratio']:.2f})  "
                  f"конец←старт {stats_raw[kk]['end_from_start']:.1f} м")
        plot_report(args.out, t_arr, gti, arr, stats_raw, args.title)
        plot_report(args.out, t_arr, gti, aligned, stats_se2,
                    args.title + " (SE2)", stem="nadir_vo_se2")
        plot_figure2(args.out / "vo_overlay_error.png", t_arr, gti, arr, stats_raw, args.title)
        plot_figure3(args.out / "vo_three_panels.png", gti, arr, stats_raw)
        best = min(("planimetry", "homography", "essential"),
                   key=lambda x: stats_raw[x]["rmse"])
        write_tum(args.out / "nadir_vo.tum", t_arr, arr[best])
        (args.out / "nadir_vo_stats.txt").write_text(
            "\n".join(
                f"{k}: RMSE_raw={stats_raw[k]['rmse']:.3f} RMSE_SE2={stats_se2[k]['rmse']:.3f} "
                f"path_ratio={stats_raw[k]['path_ratio']:.4f} path={stats_raw[k]['path_est']:.1f} "
                f"end={stats_raw[k]['end_from_start']:.2f}"
                for k in keys
            )
            + f"\nbest={best}\n",
            encoding="utf-8",
        )
        print(f"лучший канал: {best} → {args.out}/nadir_vo.tum")
        print(f"графики: {args.out}/vo_overlay_error.png  {args.out}/vo_three_panels.png")
    else:
        write_tum(args.out / "nadir_vo.tum", t_arr, arr["homography"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
