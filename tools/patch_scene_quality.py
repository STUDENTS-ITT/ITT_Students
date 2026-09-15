#!/usr/bin/env python3
"""Инкремент к уже патченному VINS: качество кадра, не имя сцены.

  1. Короткий трек — нулевой шаг XY (не копить VO над водой).
  2. INIT_DEPTH с баро только если баро похоже на AGL (пик ≥ 15 м).
  3. Приор земли слабее на склоне и если мало фич.
  4. Δz=0 только на плоском крейсере, не на рельефе.

    python3 tools/patch_scene_quality.py --vins ~/catkin_ws/src/VINS-Mono
"""
from __future__ import annotations

import argparse
from pathlib import Path

MARKER = "[VNAV-SCENE-Q]"

FAIL_OLD = """        Vector3d dP = tmp_P - last_P;
        const double vis_step = dP.norm();
        double imu_n = 0.2;
        if (pre_integrations[WINDOW_SIZE] != nullptr)
            imu_n = pre_integrations[WINDOW_SIZE]->delta_p.norm();
        // 1.5 м/кадр ≈ 15 м/с при 10 Гц — выше крейсера MARS, ниже выбросов VO.
        const double max_step = std::max(1.5, 3.0 * std::max(imu_n, 0.05));
        if (vis_step > max_step && vis_step > 1e-6)
        {
            Ps[WINDOW_SIZE] = last_P + dP * (max_step / vis_step);
            ROS_WARN("step clamped %.2f m -> %.2f m (imu Δp=%.2f, no reboot)",
                     vis_step, max_step, imu_n);
            tmp_P = Ps[WINDOW_SIZE];
        }"""

FAIL_NEW = """        Vector3d dP = tmp_P - last_P;
        const double vis_step = dP.norm();
        double imu_n = 0.2;
        if (pre_integrations[WINDOW_SIZE] != nullptr)
            imu_n = pre_integrations[WINDOW_SIZE]->delta_p.norm();
        // """ + MARKER + """ не копить шаг, если трек короче порога (вода / мало углов)
        if (f_manager.last_track_num < 20)
        {
            Ps[WINDOW_SIZE].x() = last_P.x();
            Ps[WINDOW_SIZE].y() = last_P.y();
            ROS_WARN_THROTTLE(2.0, "%s zero XY (track=%d)",
                              "[VNAV-SCENE-Q]", f_manager.last_track_num);
            tmp_P = Ps[WINDOW_SIZE];
        }
        else
        {
            const double max_step = std::max(1.5, 3.0 * std::max(imu_n, 0.05));
            if (vis_step > max_step && vis_step > 1e-6)
            {
                const double k = max_step / vis_step;
                Ps[WINDOW_SIZE] = last_P + dP * k;
                Vs[WINDOW_SIZE] *= k;
                if (vis_step > 3.0 * max_step)
                {
                    for (auto &it : f_manager.feature)
                        if (it.estimated_depth > 0.0)
                            it.estimated_depth = INIT_DEPTH;
                }
                ROS_WARN("step clamped %.2f m -> %.2f m (imu Δp=%.2f, track=%d)",
                         vis_step, max_step, imu_n, f_manager.last_track_num);
                tmp_P = Ps[WINDOW_SIZE];
            }
        }"""

IMG_OLD = """    if (USE_BARO_GROUND && cur_baro_h > 5.0)
    {
        INIT_DEPTH = cur_baro_h;
        ROS_INFO_THROTTLE(10.0, "%s h=%.1f m -> INIT_DEPTH", "[VNAV-BARO]", cur_baro_h);
    }"""

IMG_NEW = """    // """ + MARKER + """ живой AGL только если баро не «мёртвый» (~0 на MARS)
    if (USE_BARO_GROUND && cur_baro_h > 15.0)
    {
        INIT_DEPTH = cur_baro_h;
        ROS_INFO_THROTTLE(10.0, "%s h=%.1f m -> INIT_DEPTH", "[VNAV-BARO]", cur_baro_h);
    }"""

DZ_OLD = """    if (USE_BARO_GROUND)
    {
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            int j = i + 1;
            const bool bi = baro_h[i] > 5.0;
            const bool bj = baro_h[j] > 5.0;
            // HKairport03: height_above_takeoff ≈ 0 при реальных 80 м.
            // Тогда «баро Z всегда» = крейсер, Δz = 0 (не RTK).
            if (!(bi && bj) && INIT_DEPTH <= 10.0)
                continue;
            const double dz_meas = (bi && bj) ? (baro_h[j] - baro_h[i]) : 0.0;
            ceres::CostFunction *dz = new ceres::AutoDiffCostFunction<VnavBaroDeltaZFactor, 1, 7, 7>(
                new VnavBaroDeltaZFactor(dz_meas, 1.0 / BARO_Z_SIGMA));
            problem.AddResidualBlock(dz, NULL, para_Pose[i], para_Pose[j]);
            baro_dz_cnt++;
        }
    }"""

DZ_NEW = """    double z_spread = 0.0;
    for (int k = 0; k < WINDOW_SIZE; k++)
        z_spread += std::fabs(Ps[k + 1].z() - Ps[k].z());
    const double tex = std::min(1.0, f_manager.last_track_num / 80.0);
    const double slope_scale = 1.0 + std::min(3.0, z_spread / 8.0);
    const double sigma_eff = BARO_GROUND_SIGMA * slope_scale / std::max(tex, 0.35);

    if (USE_BARO_GROUND)
    {
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            int j = i + 1;
            const bool bi = baro_h[i] > 15.0;
            const bool bj = baro_h[j] > 15.0;
            double dz_meas = 0.0;
            if (bi && bj)
                dz_meas = baro_h[j] - baro_h[i];
            else if (INIT_DEPTH > 10.0)
                dz_meas = 0.0; // мёртвое баро: держать крейсер, даже если Z уже поплыл
            else
                continue;
            ceres::CostFunction *dz = new ceres::AutoDiffCostFunction<VnavBaroDeltaZFactor, 1, 7, 7>(
                new VnavBaroDeltaZFactor(dz_meas, 1.0 / BARO_Z_SIGMA));
            problem.AddResidualBlock(dz, NULL, para_Pose[i], para_Pose[j]);
            baro_dz_cnt++;
        }
    }"""

GROUND_OLD = """        if (USE_BARO_GROUND && h_agl > 5.0)
        {
            ceres::CostFunction *gd = new ceres::AutoDiffCostFunction<VnavGroundDepthFactor, 1, 1>(
                new VnavGroundDepthFactor(1.0 / h_agl, h_agl * h_agl / BARO_GROUND_SIGMA));
            problem.AddResidualBlock(gd, loss_function, para_Feature[feature_index]);
            baro_ground_cnt++;
        }"""

GROUND_NEW = """        // """ + MARKER + """ масштаб только с текстурных кадров; слабее на склоне
        if (USE_BARO_GROUND && h_agl > 5.0 && tex >= 0.30)
        {
            ceres::CostFunction *gd = new ceres::AutoDiffCostFunction<VnavGroundDepthFactor, 1, 1>(
                new VnavGroundDepthFactor(1.0 / h_agl, h_agl * h_agl / sigma_eff));
            problem.AddResidualBlock(gd, loss_function, para_Feature[feature_index]);
            baro_ground_cnt++;
        }"""

LOG_OLD = """        ROS_INFO_THROTTLE(10.0, "[VNAV-BARO] dz factors %d, ground depth priors %d",
                          baro_dz_cnt, baro_ground_cnt);"""

LOG_NEW = """        ROS_INFO_THROTTLE(10.0,
            "[VNAV-BARO] dz=%d ground=%d tex=%.2f sigma=%.1f zspread=%.1f",
            baro_dz_cnt, baro_ground_cnt, tex, sigma_eff, z_spread);"""

NODE_INC_OLD = """#include <deque>
#include <cmath>
#include <geometry_msgs/PointStamped.h>"""

NODE_INC_NEW = """#include <deque>
#include <cmath>
#include <algorithm>
#include <vector>
#include <geometry_msgs/PointStamped.h>"""

NODE_AT_OLD = """    return best_dt < 1.0 ? best : -1.0;
}"""

NODE_AT_NEW = """    return best_dt < 1.0 ? best : -1.0;
}

// """ + MARKER + """ height_above_takeoff ≈ 0 на аэродроме — не AGL
static bool baro_looks_agl()
{
    std::lock_guard<std::mutex> lk(m_baro);
    if (baro_buf.size() < 30)
        return false;
    std::vector<double> absz;
    absz.reserve(baro_buf.size());
    for (const auto &p : baro_buf)
        absz.push_back(std::fabs(p.second));
    const double peak = *std::max_element(absz.begin(), absz.end());
    std::nth_element(absz.begin(), absz.begin() + static_cast<int>(absz.size() * 0.8), absz.end());
    const double p80 = absz[static_cast<int>(absz.size() * 0.8)];
    return peak >= 15.0 && p80 >= 10.0;
}"""

NODE_FEED_OLD = """            if (USE_BARO_GROUND)
                estimator.setBaroHeight(baro_at(img_msg->header.stamp.toSec()));"""

NODE_FEED_NEW = """            if (USE_BARO_GROUND)
            {
                double h = baro_at(img_msg->header.stamp.toSec());
                if (!baro_looks_agl())
                    h = -1.0;
                estimator.setBaroHeight(h);
            }"""

EDITS = {
    "vins_estimator/src/estimator.cpp": [
        (FAIL_OLD, FAIL_NEW),
        (IMG_OLD, IMG_NEW),
        (DZ_OLD, DZ_NEW),
        (GROUND_OLD, GROUND_NEW),
        (LOG_OLD, LOG_NEW),
    ],
    "vins_estimator/src/estimator_node.cpp": [
        (NODE_INC_OLD, NODE_INC_NEW),
        (NODE_AT_OLD, NODE_AT_NEW),
        (NODE_FEED_OLD, NODE_FEED_NEW),
    ],
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    args = ap.parse_args()
    for rel, edits in EDITS.items():
        path = args.vins / rel
        if not path.is_file():
            raise SystemExit(f"Нет {path}")
        text = path.read_text(encoding="utf-8", errors="replace")
        if MARKER in text and "zero XY" in text:
            print(f"уже патчен {path}")
            continue
        for old, new in edits:
            if old not in text:
                raise SystemExit(f"Не найден якорь в {path}:\n{old[:160]}")
            text = text.replace(old, new, 1)
        path.write_text(text, encoding="utf-8")
        print(f"patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
