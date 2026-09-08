#!/usr/bin/env python3
"""Извлечение сжатых кадров камеры из rosbag MARS-LVIG.

Создаёт:
  <out>/images/000000.jpg ...
  <out>/timestamps.csv  (stamp_ns, index)

Пример:
  source /opt/ros/noetic/setup.bash
  python3 tools/bag_extract_camera.py \\
      --bag data/mars/mars_hkairport03.bag \\
      --out data/mars/camera
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    import rosbag
except ImportError:
    sys.exit("Нужен ROS: source /opt/ros/noetic/setup.bash")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--topic", default="/left_camera/image/compressed")
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args()

    if not args.bag.is_file():
        raise SystemExit(f"Нет bag: {args.bag}")

    img_dir = args.out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)
    stamps_path = args.out / "timestamps.csv"

    count = 0
    with rosbag.Bag(str(args.bag)) as bag, stamps_path.open("w", encoding="utf-8") as sf:
        sf.write("index,stamp_ns\n")
        for _topic, msg, _t in bag.read_messages(topics=[args.topic]):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print(f"  пропуск #{count}: не декодируется JPEG", file=sys.stderr)
                continue

            stamp_ns = msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs
            name = f"{count:06d}.jpg"
            cv2.imwrite(str(img_dir / name), img)
            sf.write(f"{count},{stamp_ns}\n")
            count += 1

            if count % 200 == 0:
                print(f"  {count} кадров")

            if args.limit and count >= args.limit:
                break

    if count == 0:
        raise SystemExit(f"В {args.bag} нет сообщений по {args.topic}")

    probe = cv2.imread(str(img_dir / "000000.jpg"), cv2.IMREAD_GRAYSCALE)
    h, w = probe.shape[:2]
    print(f"Готово: {count} кадров, {w}x{h}")
    print(f"  {img_dir}")
    print(f"  {stamps_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
