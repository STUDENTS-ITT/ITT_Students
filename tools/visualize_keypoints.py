#!/usr/bin/env python3
"""Визуализация ключевых точек VINS-Mono и DSO + короткое видео (~10 с).

VINS-Mono: Shi-Tomasi углы + KLT-треки (как feature_tracker).
DSO: отбор точек по градиенту в блоках 32×32 (упрощённая модель DSO).

Выход:
  results/combined/<dataset>/keypoints/frame_XXXX.png
  results/combined/<dataset>/video_10s.mp4
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def vins_features(gray: np.ndarray, max_corners: int = 200) -> np.ndarray:
    """Shi-Tomasi — как в VINS feature_tracker (глобальный отбор)."""
    pts = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=max_corners,
        qualityLevel=0.01,
        minDistance=10,
        blockSize=3,
    )
    return pts if pts is not None else np.empty((0, 1, 2), dtype=np.float32)


def vins_features_grid(
    gray: np.ndarray,
    block: int = 32,
    max_corners: int = 200,
    min_dist: int = 8,
) -> np.ndarray:
    """Сеточный Shi-Tomasi + fallback по градиенту — как patch feature_tracker."""
    h, w = gray.shape
    sob_gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    sob_gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    grad = cv2.magnitude(sob_gx, sob_gy)
    eig = cv2.cornerMinEigenVal(gray, 3, 3)

    grid_cols = (w + block - 1) // block
    grid_rows = (h + block - 1) // block

    candidates: list[tuple[float, float, float]] = []
    for gy_i in range(grid_rows):
        for gx_i in range(grid_cols):
            x0 = gx_i * block
            y0 = gy_i * block
            cw = min(block, w - x0)
            ch = min(block, h - y0)
            if cw < 5 or ch < 5:
                continue

            cell = gray[y0 : y0 + ch, x0 : x0 + cw]
            cell_pts = cv2.goodFeaturesToTrack(
                cell,
                maxCorners=1,
                qualityLevel=0.01,
                minDistance=min_dist,
                blockSize=3,
            )
            if cell_pts is not None:
                px, py = cell_pts.reshape(-1, 2)[0]
                ix = min(max(int(round(px)) + x0, 0), w - 1)
                iy = min(max(int(round(py)) + y0, 0), h - 1)
                candidates.append((float(eig[iy, ix]), x0 + px, y0 + py))
            else:
                patch = grad[y0 : y0 + ch, x0 : x0 + cw]
                _, max_val, _, max_loc = cv2.minMaxLoc(patch)
                if max_val > 2.0:
                    candidates.append((float(max_val), x0 + max_loc[0], y0 + max_loc[1]))

    if not candidates:
        return np.empty((0, 1, 2), dtype=np.float32)

    candidates.sort(key=lambda t: t[0], reverse=True)
    keep = candidates[:max_corners]
    pts = np.array([[x, y] for _, x, y in keep], dtype=np.float32).reshape(-1, 1, 2)
    return pts


def dso_features(gray: np.ndarray, block: int = 32, target: int = 200) -> np.ndarray:
    """Упрощённый отбор DSO: максимум градиента в каждом блоке."""
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    grad = cv2.magnitude(gx, gy)
    h, w = gray.shape
    points = []
    for y in range(0, h - block, block):
        for x in range(0, w - block, block):
            patch = grad[y : y + block, x : x + block]
            if patch.size == 0:
                continue
            _, _, _, max_loc = cv2.minMaxLoc(patch)
            points.append([x + max_loc[0], y + max_loc[1]])
    pts = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    if len(pts) > target:
        idx = np.linspace(0, len(pts) - 1, target, dtype=int)
        pts = pts[idx]
    return pts


def draw_points(img: np.ndarray, pts: np.ndarray, color: tuple, label: str) -> np.ndarray:
    out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
    for p in pts.reshape(-1, 2):
        cv2.circle(out, (int(p[0]), int(p[1])), 3, color, -1, lineType=cv2.LINE_AA)
    cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
    return out


def draw_tracks(prev: np.ndarray, curr: np.ndarray, prev_pts, curr_pts, color) -> np.ndarray:
    out = cv2.cvtColor(curr, cv2.COLOR_GRAY2BGR)
    for p0, p1 in zip(prev_pts.reshape(-1, 2), curr_pts.reshape(-1, 2)):
        p0i = (int(p0[0]), int(p0[1]))
        p1i = (int(p1[0]), int(p1[1]))
        cv2.line(out, p0i, p1i, color, 1, cv2.LINE_AA)
        cv2.circle(out, p1i, 3, color, -1, lineType=cv2.LINE_AA)
    cv2.putText(out, "VINS KLT tracks", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return out


def process_dataset(
    images_dir: Path,
    out_dir: Path,
    fps: float = 10.0,
    duration_sec: float = 10.0,
    width: int = 640,
    start_frame: int = 0,
    vins_grid: bool = True,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    kp_dir = out_dir / "keypoints"
    kp_dir.mkdir(exist_ok=True)

    files = sorted(images_dir.glob("*"))
    files = [f for f in files if f.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}]
    if not files:
        raise SystemExit(f"Нет кадров в {images_dir}")

    n_frames = min(int(duration_sec * fps), len(files) - start_frame)
    sel = files[start_frame : start_frame + n_frames]
    if len(sel) < 2:
        raise SystemExit(f"Мало кадров: {len(sel)}")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_path = out_dir / "video_10s.mp4"
    writer = None
    prev_gray = None
    prev_pts = None
    sample_indices = {0, n_frames // 2, n_frames - 1}
    stats = {"frames": n_frames, "vins_pts": [], "dso_pts": []}

    for i, fp in enumerate(sel):
        img = cv2.imread(str(fp), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        h, w = img.shape
        scale = width / w
        nh = int(h * scale)
        nh = max(nh - nh % 2, 2)
        gray = cv2.resize(img, (width, nh), interpolation=cv2.INTER_AREA)

        v_pts = vins_features_grid(gray) if vins_grid else vins_features(gray)
        d_pts = dso_features(gray)
        stats["vins_pts"].append(len(v_pts))
        stats["dso_pts"].append(len(d_pts))

        vins_label = f"VINS grid {len(v_pts)} pts" if vins_grid else f"VINS {len(v_pts)} pts"
        panel = np.hstack([
            draw_points(gray, v_pts, (0, 220, 255), vins_label),
            draw_points(gray, d_pts, (0, 180, 0), f"DSO {len(d_pts)} pts"),
        ])

        if prev_gray is not None and prev_pts is not None and len(prev_pts) > 0:
            next_pts, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None)
            if next_pts is not None and st is not None:
                good = st.reshape(-1) == 1
                if good.sum() > 5:
                    track_panel = draw_tracks(
                        prev_gray, gray, prev_pts[good], next_pts[good], (0, 220, 255)
                    )
                    panel = np.hstack([panel, track_panel])

        # Фиксированный размер кадра для VideoWriter
        target_h, target_w = nh, width * 3
        if panel.shape[1] != target_w or panel.shape[0] != target_h:
            panel = cv2.resize(panel, (target_w, target_h), interpolation=cv2.INTER_AREA)

        if writer is None:
            # AVI надёжнее mp4v на Linux без ffmpeg codecs
            suffix = f"_{int(duration_sec)}s" if duration_sec != 10.0 else "_10s"
            video_path = out_dir / f"video{suffix}.avi"
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            writer = cv2.VideoWriter(str(video_path), fourcc, fps, (target_w, target_h))
        writer.write(panel)

        if i in sample_indices:
            cv2.imwrite(str(kp_dir / f"frame_{i:04d}_combined.png"), panel)
            cv2.imwrite(str(kp_dir / f"frame_{i:04d}_vins.png"), draw_points(gray, v_pts, (0, 220, 255), "VINS"))
            cv2.imwrite(str(kp_dir / f"frame_{i:04d}_dso.png"), draw_points(gray, d_pts, (0, 180, 0), "DSO"))

        prev_gray = gray
        prev_pts = v_pts

    if writer is not None:
        writer.release()

    stats["video"] = str(video_path)
    stats["avg_vins_pts"] = float(np.mean(stats["vins_pts"])) if stats["vins_pts"] else 0
    stats["avg_dso_pts"] = float(np.mean(stats["dso_pts"])) if stats["dso_pts"] else 0
    return stats


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--legacy-vins", action="store_true", help="глобальный Shi-Tomasi вместо сетки 32×32")
    args = ap.parse_args()

    stats = process_dataset(
        args.images, args.out, args.fps, args.duration, args.width, args.start,
        vins_grid=not args.legacy_vins,
    )
    print(f"OK: {args.out}")
    print(f"  frames={stats['frames']} avg_vins={stats['avg_vins_pts']:.0f} avg_dso={stats['avg_dso_pts']:.0f}")
    print(f"  video={stats['video']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
