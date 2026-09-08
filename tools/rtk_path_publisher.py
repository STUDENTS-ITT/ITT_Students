#!/usr/bin/env python3
"""RTK-эталон для RViz: nav_msgs/Path по времени VINS.

Синхронизация как у benchmark_publisher в VINS-Mono: подписка на
/vins_estimator/odometry, накопление точек RTK TUM до текущего stamp.

    rosrun не нужен — запуск из run_vins_mars_live.sh или вручную:

    python3 tools/rtk_path_publisher.py \\
        --gt data/mars/mars_hkairport03_gt.tum \\
        --path-topic /mars/rtk_path
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


def load_tum(path: Path) -> list[tuple[float, float, float, float]]:
    rows: list[tuple[float, float, float, float]] = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                rows.append((float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                continue
    if not rows:
        raise SystemExit(f"Пустой TUM: {path}")
    return rows


class RtkPathPublisher:
    def __init__(
        self,
        gt_rows: list[tuple[float, float, float, float]],
        path_topic: str,
        odom_topic: str,
        frame_id: str,
    ) -> None:
        self._gt = gt_rows
        self._idx = 0
        self._path = Path()
        self._path.header.frame_id = frame_id
        self._pub_path = rospy.Publisher(path_topic, Path, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=200)
        rospy.loginfo(
            "RTK path publisher: %d точек, topic=%s, sync=%s",
            len(gt_rows),
            path_topic,
            odom_topic,
        )

    def _on_odom(self, msg: Odometry) -> None:
        t = msg.header.stamp.to_sec()
        updated = False
        while self._idx < len(self._gt) and self._gt[self._idx][0] <= t:
            ts, x, y, z = self._gt[self._idx]
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.from_sec(ts)
            ps.header.frame_id = self._path.header.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            self._path.poses.append(ps)
            self._idx += 1
            updated = True
        if updated and self._path.poses:
            self._path.header.stamp = msg.header.stamp
            self._pub_path.publish(self._path)


def main() -> None:
    ap = argparse.ArgumentParser(description="RTK ground truth → nav_msgs/Path для RViz")
    ap.add_argument(
        "--gt",
        type=Path,
        default=Path("data/mars/mars_hkairport03_gt.tum"),
        help="TUM с RTK (timestamp tx ty tz ...)",
    )
    ap.add_argument("--path-topic", default="/mars/rtk_path")
    ap.add_argument("--odom-topic", default="/vins_estimator/odometry")
    ap.add_argument("--frame-id", default="world")
    args = ap.parse_args()

    if not args.gt.is_file():
        raise SystemExit(f"Нет файла: {args.gt}")

    rospy.init_node("rtk_path_publisher", anonymous=False)
    RtkPathPublisher(load_tum(args.gt), args.path_topic, args.odom_topic, args.frame_id)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
