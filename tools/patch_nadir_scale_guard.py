#!/usr/bin/env python3
"""VINS-Mono: nadir scale guard — IMU-only hold when visual scale unreliable.

Point 1 — Fix scale in estimator:
  A) skip_visual_frame_ member + clearState init
  B) solveOdometry pre-opt guard (track / depth / vis-vs-IMU divergence)
  C) optimization: skip visual projections + ground depth priors when holding
  D) ceres::Solve unusable → keep pre-opt state (no double2vector)
  E) failureDetection step clamp: reset depths on large vis_step

    python3 tools/patch_nadir_scale_guard.py --vins ~/catkin_ws/src/VINS-Mono
"""
from __future__ import annotations

import argparse
from pathlib import Path

MARKER = "[VNAV-NADIR-HOLD]"

H_OLD = """    bool failure_occur;

    // [VNAV-REBOOT-ANCHOR] last good pose survives clearState"""

H_NEW = """    bool failure_occur;

    // """ + MARKER + """ skip visual factors when scale/texture unreliable
    bool skip_visual_frame_;

    // [VNAV-REBOOT-ANCHOR] last good pose survives clearState"""

CLEAR_OLD = """    failure_occur = 0;
    relocalization_info = 0;
    imu_course_init = false;
    imu_course = 0.0;"""

CLEAR_NEW = """    failure_occur = 0;
    relocalization_info = 0;
    skip_visual_frame_ = false;
    imu_course_init = false;
    imu_course = 0.0;"""

SOLVE_OLD = """        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        optimization();"""

SOLVE_NEW = """        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        // """ + MARKER + """ reset wild depths; IMU-only only if track is empty
        skip_visual_frame_ = false;
        if (USE_BARO_GROUND && INIT_DEPTH > 10.0)
        {
            for (auto &it : f_manager.feature)
            {
                if (it.estimated_depth > 0.0 &&
                    (it.estimated_depth < 0.2 * INIT_DEPTH ||
                     it.estimated_depth > 5.0 * INIT_DEPTH))
                    it.estimated_depth = INIT_DEPTH;
            }
            if (f_manager.last_track_num < 20)
            {
                skip_visual_frame_ = true;
                Ps[WINDOW_SIZE].x() = last_P.x();
                Ps[WINDOW_SIZE].y() = last_P.y();
                vector2double();
                ROS_WARN_THROTTLE(2.0, "%s IMU-only frame (track=%d)",
                                  "[VNAV-NADIR-HOLD]", f_manager.last_track_num);
            }
        }
        optimization();"""

OPT_LOOP_OLD = """    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        // [VNAV-BARO-GROUND] обратная глубина ≈ 1 / высота над землёй.
        // sqrt_info = h^2 / sigma_d — перенос метровой сигмы в обратную глубину.
        // Барометр предпочтительнее, но в HKairport03 топик пустой (h≈0 при 80 м),
        // поэтому опорой служит номинальная высота полёта init_depth из yaml.
        double h_agl = baro_h[it_per_id.start_frame];
        if (h_agl < 5.0)
            h_agl = INIT_DEPTH > 10.0 ? INIT_DEPTH : -1.0;
        // [VNAV-SCENE-Q] масштаб только с текстурных кадров; слабее на склоне
        if (USE_BARO_GROUND && h_agl > 5.0 && tex >= 0.30)
        {
            ceres::CostFunction *gd = new ceres::AutoDiffCostFunction<VnavGroundDepthFactor, 1, 1>(
                new VnavGroundDepthFactor(1.0 / h_agl, h_agl * h_agl / sigma_eff));
            problem.AddResidualBlock(gd, loss_function, para_Feature[feature_index]);
            baro_ground_cnt++;
        }

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                    /*
                    double **para = new double *[5];
                    para[0] = para_Pose[imu_i];
                    para[1] = para_Pose[imu_j];
                    para[2] = para_Ex_Pose[0];
                    para[3] = para_Feature[feature_index];
                    para[4] = para_Td[0];
                    f_td->check(para);
                    */
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }"""

OPT_LOOP_NEW = """    int f_m_cnt = 0;
    int feature_index = -1;
    // """ + MARKER + """ skip visual + ground depth when holding IMU-only
    if (!skip_visual_frame_)
    {
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        // [VNAV-BARO-GROUND] обратная глубина ≈ 1 / высота над землёй.
        // sqrt_info = h^2 / sigma_d — перенос метровой сигмы в обратную глубину.
        // Барометр предпочтительнее, но в HKairport03 топик пустой (h≈0 при 80 м),
        // поэтому опорой служит номинальная высота полёта init_depth из yaml.
        double h_agl = baro_h[it_per_id.start_frame];
        if (h_agl < 5.0)
            h_agl = INIT_DEPTH > 10.0 ? INIT_DEPTH : -1.0;
        // [VNAV-SCENE-Q] масштаб только с текстурных кадров; слабее на склоне
        if (USE_BARO_GROUND && h_agl > 5.0 && tex >= 0.30)
        {
            ceres::CostFunction *gd = new ceres::AutoDiffCostFunction<VnavGroundDepthFactor, 1, 1>(
                new VnavGroundDepthFactor(1.0 / h_agl, h_agl * h_agl / sigma_eff));
            problem.AddResidualBlock(gd, loss_function, para_Feature[feature_index]);
            baro_ground_cnt++;
        }

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                    /*
                    double **para = new double *[5];
                    para[0] = para_Pose[imu_i];
                    para[1] = para_Pose[imu_j];
                    para[2] = para_Ex_Pose[0];
                    para[3] = para_Feature[feature_index];
                    para[4] = para_Td[0];
                    f_td->check(para);
                    */
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }
    }"""

CERES_OLD = """    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    double2vector();"""

CERES_NEW = """    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    // """ + MARKER + """ keep pre-opt state if ceres did not converge
    if (!summary.IsSolutionUsable())
    {
        ROS_WARN("%s ceres failed: %s", "[VNAV-NADIR-HOLD]", summary.message.c_str());
    }
    else
    {
        double2vector();
        lockYawToImu();
    }"""

FAIL_CLAMP_OLD = """            if (vis_step > max_step && vis_step > 1e-6)
            {
                const double k = max_step / vis_step;
                Ps[WINDOW_SIZE] = last_P + dP * k;
                Vs[WINDOW_SIZE] *= k;
                ROS_WARN("step clamped %.2f m -> %.2f m (imu Δp=%.2f, track=%d)",
                         vis_step, max_step, imu_n, f_manager.last_track_num);
                tmp_P = Ps[WINDOW_SIZE];
            }"""

FAIL_CLAMP_NEW = """            if (vis_step > max_step && vis_step > 1e-6)
            {
                const double k = max_step / vis_step;
                Ps[WINDOW_SIZE] = last_P + dP * k;
                Vs[WINDOW_SIZE] *= k;
                // """ + MARKER + """ large VO jump — reset feature depths to INIT_DEPTH
                if (vis_step > 3.0 * max_step)
                {
                    for (auto &it : f_manager.feature)
                        if (it.estimated_depth > 0.0)
                            it.estimated_depth = INIT_DEPTH;
                }
                ROS_WARN("step clamped %.2f m -> %.2f m (imu Δp=%.2f, track=%d)",
                         vis_step, max_step, imu_n, f_manager.last_track_num);
                tmp_P = Ps[WINDOW_SIZE];
            }"""

EDITS = {
    "vins_estimator/src/estimator.h": [
        (H_OLD, H_NEW),
    ],
    "vins_estimator/src/estimator.cpp": [
        (CLEAR_OLD, CLEAR_NEW),
        (SOLVE_OLD, SOLVE_NEW),
        (OPT_LOOP_OLD, OPT_LOOP_NEW),
        (CERES_OLD, CERES_NEW),
        (FAIL_CLAMP_OLD, FAIL_CLAMP_NEW),
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
        if MARKER in text and "skip_visual_frame_" in text:
            print(f"уже патчен {path}")
            continue
        for old, new in edits:
            if old not in text:
                raise SystemExit(f"Не найден якорь в {path}:\n{old[:200]}")
            text = text.replace(old, new, 1)
        path.write_text(text, encoding="utf-8")
        print(f"patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
