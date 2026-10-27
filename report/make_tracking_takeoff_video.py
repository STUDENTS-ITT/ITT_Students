#!/usr/bin/env python3
"""Short FAST keypoint + match demo for HKairport03 takeoff (nadir VO preview)."""

from __future__ import annotations

import csv
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IMAGES = ROOT / "camera" / "images"
ALT_CSV = ROOT / "altitude_per_frame.csv"
OUT_VIDEO = ROOT / "report" / "tracking_takeoff.mp4"
OUT_TIMING = ROOT / "report" / "tracking_timing.txt"

TARGET_WIDTH = 960
VIDEO_FPS = 10.0
VIDEO_SECONDS = 80.0
MATCH_EVERY = 1  # линия соответствий на каждом кадре
KEEP_EVERY = 1  # предыдущий кадр — база для соответствий
MAX_MATCH_LINES = 60
MAX_KP_DRAW = 800


def load_altitudes(path: Path) -> dict[int, float]:
    alt: dict[int, float] = {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            alt[int(row["frame"])] = float(row["altitude"])
    return alt


def end_frame_index(alt: dict[int, float]) -> int:
    # 50 с ролика при 10 кадр/с — взлёт и начало маршрута, не весь полёт.
    return int(VIDEO_SECONDS * VIDEO_FPS) - 1


def resize_to_width(bgr: np.ndarray, width: int) -> np.ndarray:
    h, w = bgr.shape[:2]
    if w == width:
        return bgr
    scale = width / float(w)
    new_h = int(round(h * scale))
    return cv2.resize(bgr, (width, new_h), interpolation=cv2.INTER_AREA)


def detect_fast(gray: np.ndarray, fast_min_contrast: float = 0.1, fast_min_quality: float = 0.05):
    thr = max(1, int(round(fast_min_contrast * 255)))
    fast = cv2.FastFeatureDetector_create(thr, True)
    kps = fast.detect(gray, None)
    if not kps:
        return []
    mx = max(k.response for k in kps)
    q_thr = fast_min_quality * mx
    return [k for k in kps if k.response >= q_thr]


def open_video_writer(path: Path, fps: float, size: tuple[int, int]) -> cv2.VideoWriter:
    for fourcc in ("avc1", "H264", "mp4v"):
        writer = cv2.VideoWriter(
            str(path), cv2.VideoWriter_fourcc(*fourcc), fps, size, True
        )
        if writer.isOpened():
            return writer
        writer.release()
    raise RuntimeError("Could not open VideoWriter with avc1/H264/mp4v")


def main() -> int:
    OUT_VIDEO.parent.mkdir(parents=True, exist_ok=True)
    alt = load_altitudes(ALT_CSV)
    end_idx = end_frame_index(alt)

    orb = cv2.ORB_create(nfeatures=2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    prev_kept: dict | None = None
    kept_counter = 0

    frame_path = IMAGES / "000000.jpg"
    probe = cv2.imread(str(frame_path))
    if probe is None:
        print(f"Failed to read {frame_path}", file=sys.stderr)
        return 1
    out_size = resize_to_width(probe, TARGET_WIDTH).shape[1::-1]  # (w, h)

    writer = open_video_writer(OUT_VIDEO, VIDEO_FPS, out_size)

    times_ms: list[float] = []
    n_written = 0
    t0_all = time.perf_counter()

    for frame_idx in range(0, end_idx + 1):
        t0 = time.perf_counter()

        img_path = IMAGES / f"{frame_idx:06d}.jpg"
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            print(f"Warning: missing {img_path}, stopping early", file=sys.stderr)
            break

        bgr = resize_to_width(bgr, TARGET_WIDTH)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        vis = bgr.copy()

        kps = detect_fast(gray)
        if len(kps) > MAX_KP_DRAW:
            kps = sorted(kps, key=lambda k: k.response, reverse=True)[:MAX_KP_DRAW]

        n_matches = 0
        if (
            prev_kept is not None
            and frame_idx % MATCH_EVERY == 0
            and kps
            and prev_kept["kps"]
        ):
            _, des_cur = orb.compute(gray, kps)
            if des_cur is not None and len(des_cur) and prev_kept["des"] is not None:
                pairs = bf.knnMatch(prev_kept["des"], des_cur, k=2)
                pk = prev_kept["kps"]
                kept = []
                for pair in pairs:
                    if len(pair) < 2:
                        continue
                    a, b = pair
                    if a.distance >= 0.75 * b.distance:
                        continue
                    p0 = pk[a.queryIdx].pt
                    p1 = kps[a.trainIdx].pt
                    if (p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2 > 70.0 ** 2:
                        continue
                    kept.append((a, p0, p1))
                kept = sorted(kept, key=lambda t: t[0].distance)[:MAX_MATCH_LINES]
                n_matches = len(kept)
                for _, p0, p1 in kept:
                    q = (int(round(p0[0])), int(round(p0[1])))
                    r = (int(round(p1[0])), int(round(p1[1])))
                    cv2.line(vis, q, r, (0, 180, 255), 1, cv2.LINE_AA)

        cv2.drawKeypoints(
            vis,
            kps,
            vis,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        altitude = alt.get(frame_idx)
        alt_str = f"{altitude:.2f} m" if altitude is not None else "n/a"
        elapsed = time.perf_counter() - t0
        times_ms.append(elapsed * 1000.0)
        proc_fps = 1000.0 / times_ms[-1] if times_ms[-1] > 0 else 0.0

        lines = [
            f"frame {frame_idx} / {end_idx}",
            f"alt {alt_str}",
            f"FAST kp {len(kps)}",
            f"matches {n_matches}",
            f"proc {times_ms[-1]:.1f} ms ({proc_fps:.1f} fps)",
        ]
        y = 28
        for line in lines:
            cv2.putText(
                vis,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                vis,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y += 26

        writer.write(vis)
        n_written += 1

        kept_counter += 1
        if kept_counter >= KEEP_EVERY:
            kept_counter = 0
            _, des = orb.compute(gray, kps)
            if des is not None and len(kps):
                prev_kept = {"kps": kps, "des": des, "idx": frame_idx}
            else:
                prev_kept = None

    writer.release()
    total_s = time.perf_counter() - t0_all

    if not times_ms:
        print("No frames processed", file=sys.stderr)
        return 1

    ms_per_frame = float(np.mean(times_ms))
    ms_median = float(np.median(times_ms))
    duration_s = n_written / VIDEO_FPS

    summary = (
        f"frames_source={0}..{min(end_idx, 0 + n_written - 1)} (count={n_written})\n"
        f"end_frame_rule=first_{VIDEO_SECONDS:.0f}s_at_{VIDEO_FPS:.0f}fps -> end_idx={end_idx}\n"
        f"output={OUT_VIDEO}\n"
        f"resolution={out_size[0]}x{out_size[1]}\n"
        f"video_fps={VIDEO_FPS}\n"
        f"video_duration_s={duration_s:.2f}\n"
        f"ms_per_frame_mean={ms_per_frame:.2f}\n"
        f"ms_per_frame_median={ms_median:.2f}\n"
        f"total_wall_s={total_s:.2f}\n"
        f"opencv={cv2.__version__}\n"
        f"note=ORB descriptors at FAST keypoints (FREAK/xfeatures2d unavailable in this OpenCV build)\n"
    )
    OUT_TIMING.write_text(summary, encoding="utf-8")

    print(summary)
    print(f"Wrote {OUT_VIDEO} ({duration_s:.1f}s, {ms_per_frame:.1f} ms/frame mean)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
