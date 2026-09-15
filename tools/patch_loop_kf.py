#!/usr/bin/env python3
"""Патч VINS-Mono: pose_graph на надирном поле (курс, не масштаб).

Почему loop.csv пустой при keyframe_parallax 2–3:

  pose_graph_node при скачке штампа (rosbag play -s, разрыв >1 с, штамп
  назад) вызывает new_sequence(). После 5 раз — ROS_BREAK(), нода мертва.
  Это не parallax и не skip_dis.

Дополнительно: findConnection требует |Δyaw|<30° и |Δt|<20 м — при уже
ушедшем курсе петля овальной трассы отвергается, хотя BRIEF нашёл кадр.

IMU sum_dt>10 с отбрасывает преинтеграцию, если кейфреймы редкие.

Не трогает: ground prior, acc-bias prior, td, INIT_DEPTH, step-clamp.

    python3 tools/patch_loop_kf.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_loop_kf.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-LOOP-KF]"


def once(text: str, old: str, new: str, path: Path) -> str:
    if MARKER in new and new in text:
        return text
    if old not in text:
        raise SystemExit(f"Не найден якорь в {path}:\n{old[:180]}")
    return text.replace(old, new, 1)


def backup(path: Path) -> None:
    bak = path.with_suffix(path.suffix + ".loopkf_orig")
    if not bak.is_file():
        shutil.copy2(path, bak)


def patch_pose_graph_node(vins: Path) -> None:
    path = vins / "pose_graph/src/pose_graph_node.cpp"
    text = path.read_text(encoding="utf-8", errors="replace")
    old = """    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();"""
    new = """    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        // """ + MARKER + """ rosbag play -s даёт скачок штампа. new_sequence()
        // после 5 раз делает ROS_BREAK() — pose_graph умирает, loop.csv пуст.
        ROS_WARN("%s image discontinue (dt=%.3f s) — drain buffers, keep sequence",
                 \"""" + MARKER + """\", image_msg->header.stamp.toSec() - last_image_time);
        m_buf.lock();
        while (!image_buf.empty()) image_buf.pop();
        while (!point_buf.empty()) point_buf.pop();
        while (!pose_buf.empty()) pose_buf.pop();
        image_buf.push(image_msg);
        m_buf.unlock();
    }
    last_image_time = image_msg->header.stamp.toSec();"""
    text = once(text, old, new, path)

    old_add = """                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;"""
    new_add = """                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
                ROS_INFO_THROTTLE(5.0, "%s pose_graph keyframes %d",
                                 \"""" + MARKER + """\", frame_index);"""
    text = once(text, old_add, new_add, path)

    old_drain = """                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();"""
    new_drain = """                pose_msg = pose_buf.front();
                pose_buf.pop();
                // """ + MARKER + """ do not drain pose_buf: bag replay must keep skip_dis KFs"""
    text = once(text, old_drain, new_drain, path)
    backup(path)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")


def patch_keyframe(vins: Path) -> None:
    kh = vins / "pose_graph/src/keyframe.h"
    kc = vins / "pose_graph/src/keyframe.cpp"
    ht = kh.read_text(encoding="utf-8", errors="replace")
    ct = kc.read_text(encoding="utf-8", errors="replace")
    ht = once(ht, "#define MIN_LOOP_NUM 25",
              "#define MIN_LOOP_NUM 15  // " + MARKER, kh)
    ct = once(
        ct,
        "\tconst int fast_th = 20; // corner detector response threshold",
        "\tconst int fast_th = 10; // " + MARKER + " grass/concrete at 80 m",
        kc,
    )
    ct = once(
        ct,
        "    if (bestIndex != -1 && bestDist < 80)",
        "    if (bestIndex != -1 && bestDist < 100)  // " + MARKER,
        kc,
    )
    ct = once(
        ct,
        "	    if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0)",
        "	    // " + MARKER + " heading already drifted on the oval: 30°/20 m\n"
        "	    // rejects the lap. Allow the 4-DoF graph to pull yaw back.\n"
        "	    if (abs(relative_yaw) < 90.0 && relative_t.norm() < 150.0)",
        kc,
    )
    backup(kh)
    backup(kc)
    kh.write_text(ht, encoding="utf-8")
    kc.write_text(ct, encoding="utf-8")
    print(f"patched {kh.name} + {kc.name}")


def patch_detect_loop(vins: Path) -> None:
    path = vins / "pose_graph/src/pose_graph.cpp"
    text = path.read_text(encoding="utf-8", errors="replace")
    text = once(
        text,
        "    if (ret.size() >= 1 &&ret[0].Score > 0.05)",
        "    if (ret.size() >= 1 &&ret[0].Score > 0.03)  // " + MARKER,
        path,
    )
    text = once(
        text,
        "            if (ret[i].Score > 0.015)",
        "            if (ret[i].Score > 0.010)  // " + MARKER,
        path,
    )
    text = once(
        text,
        "            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))",
        "            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.010))  // "
        + MARKER,
        path,
    )
    text = once(
        text,
        "    if (find_loop && frame_index > 50)",
        "    if (find_loop && frame_index > 30)  // " + MARKER,
        path,
    )
    backup(path)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")


def patch_visualization(vins: Path) -> None:
    path = vins / "vins_estimator/src/utility/visualization.cpp"
    text = path.read_text(encoding="utf-8", errors="replace")
    old = "    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)"
    new = (
        "    // "
        + MARKER
        + " pose_graph must see poses even when VINS MARGIN_SECOND_NEW\n"
        "    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)"
    )
    text = once(text, old, new, path)
    backup(path)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")


def patch_estimator(vins: Path) -> None:
    path = vins / "vins_estimator/src/estimator.cpp"
    text = path.read_text(encoding="utf-8", errors="replace")
    old_kf = (
        "    if (f_manager.addFeatureCheckParallax(frame_count, image, td))\n"
        "        marginalization_flag = MARGIN_OLD;\n"
        "    else\n"
        "        marginalization_flag = MARGIN_SECOND_NEW;"
    )
    new_kf = old_kf + """
    // """ + MARKER + """
    {
        static int kf_n = 0, nkf_n = 0;
        if (marginalization_flag == MARGIN_OLD) kf_n++; else nkf_n++;
        ROS_INFO_THROTTLE(10.0, "%s vins keyframes %d / %d frames (%.0f%%)",
                         \"""" + MARKER + """\", kf_n, kf_n + nkf_n,
                         100.0 * kf_n / std::max(1, kf_n + nkf_n));
    }"""
    text = once(text, old_kf, new_kf, path)
    backup(path)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")


def revert(vins: Path) -> None:
    rels = [
        "pose_graph/src/pose_graph_node.cpp",
        "pose_graph/src/keyframe.h",
        "pose_graph/src/keyframe.cpp",
        "pose_graph/src/pose_graph.cpp",
        "vins_estimator/src/estimator.cpp",
        "vins_estimator/src/utility/visualization.cpp",
    ]
    for rel in rels:
        path = vins / rel
        bak = path.with_suffix(path.suffix + ".loopkf_orig")
        if bak.is_file():
            shutil.copy2(bak, path)
            print(f"restored {path}")
        else:
            print(f"нет бэкапа {bak}", file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()
    if args.revert:
        revert(args.vins)
        return 0
    patch_pose_graph_node(args.vins)
    patch_keyframe(args.vins)
    patch_detect_loop(args.vins)
    patch_estimator(args.vins)
    patch_visualization(args.vins)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
