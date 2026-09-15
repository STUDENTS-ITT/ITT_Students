#!/usr/bin/env python3
"""Патч VINS-Mono: Eigen crash на пустых матрицах при marginalization.

На надире skip_visual_frame_ может убрать все визуальные факторы; тогда
marginalize() строит 0×0 Schur и падает в maxCoeff / SelfAdjointEigenSolver.

    python3 tools/patch_nadir_empty_marg.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_nadir_empty_marg.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

MARKER = "[VNAV-NADIR-MARG]"

# --- marginalization_factor.cpp ---

OLD_MARG_START = """void MarginalizationInfo::marginalize()
{
    int pos = 0;
    for (auto &it : parameter_block_idx)
    {
        it.second = pos;
        pos += localSize(parameter_block_size[it.first]);
    }

    m = pos;

    for (const auto &it : parameter_block_size)
    {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
        {
            parameter_block_idx[it.first] = pos;
            pos += localSize(it.second);
        }
    }

    n = pos - m;

    //ROS_DEBUG("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());

    TicToc t_summing;"""

NEW_MARG_START = """void MarginalizationInfo::marginalize()
{
    // """ + MARKER + """ guard empty factor graph / zero-size Schur blocks
    if (factors.empty())
    {
        m = 0;
        n = 0;
        linearized_jacobians.resize(0, 0);
        linearized_residuals.resize(0);
        ROS_WARN(""" + '"' + MARKER + """ skip: no factors");
        return;
    }

    int pos = 0;
    for (auto &it : parameter_block_idx)
    {
        it.second = pos;
        pos += localSize(parameter_block_size[it.first]);
    }

    m = pos;

    for (const auto &it : parameter_block_size)
    {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
        {
            parameter_block_idx[it.first] = pos;
            pos += localSize(it.second);
        }
    }

    n = pos - m;

    if (pos <= 0)
    {
        m = 0;
        n = 0;
        linearized_jacobians.resize(0, 0);
        linearized_residuals.resize(0);
        ROS_WARN(""" + '"' + MARKER + """ skip: pos==0");
        return;
    }
    if (m <= 0)
    {
        n = pos;
        linearized_jacobians.resize(0, 0);
        linearized_residuals.resize(0);
        ROS_WARN(""" + '"' + MARKER + """ skip Schur: m==0 (n=%d)", n);
        return;
    }
    if (n <= 0)
    {
        linearized_jacobians.resize(0, 0);
        linearized_residuals.resize(0);
        ROS_WARN(""" + '"' + MARKER + """ skip: n==0");
        return;
    }

    //ROS_DEBUG("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());

    TicToc t_summing;"""

# --- estimator.cpp: gate visual margin factors ---

OLD_VIS_MARGIN = """        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;"""

NEW_VIS_MARGIN = """        // """ + MARKER + """ skip visual margin factors on IMU-only frames
        if (!skip_visual_frame_)
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }
        else if (f_m_cnt == 0)
        {
            ROS_WARN_THROTTLE(1.0, """ + '"' + MARKER + """ IMU-only frame: no visual margin factors");
        }

        if (marginalization_info->factors.empty())
        {
            ROS_WARN(""" + '"' + MARKER + """ skip marginalization: no residual factors");
            delete marginalization_info;
        }
        else
        {
            TicToc t_pre_margin;
            marginalization_info->preMarginalize();
            ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

            TicToc t_margin;
            marginalization_info->marginalize();
            ROS_DEBUG("marginalization %f ms", t_margin.toc());

            if (marginalization_info->m > 0 && marginalization_info->n > 0)
            {
                std::unordered_map<long, double *> addr_shift;
                for (int i = 1; i <= WINDOW_SIZE; i++)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                for (int i = 0; i < NUM_OF_CAM; i++)
                    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
                if (ESTIMATE_TD)
                {
                    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
                }
                vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

                if (last_marginalization_info)
                    delete last_marginalization_info;
                last_marginalization_info = marginalization_info;
                last_marginalization_parameter_blocks = parameter_blocks;
            }
            else
            {
                ROS_WARN(""" + '"' + MARKER + """ invalid margin dims m=%d n=%d, keep previous",
                         marginalization_info->m, marginalization_info->n);
                delete marginalization_info;
            }
        }"""

OLD_MARG_SECOND = """            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;"""

NEW_MARG_SECOND = """            if (marginalization_info->factors.empty())
            {
                ROS_WARN(""" + '"' + MARKER + """ skip second marginalization: no factors");
                delete marginalization_info;
            }
            else
            {
                TicToc t_pre_margin;
                ROS_DEBUG("begin marginalization");
                marginalization_info->preMarginalize();
                ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

                TicToc t_margin;
                ROS_DEBUG("begin marginalization");
                marginalization_info->marginalize();
                ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

                if (marginalization_info->m > 0 && marginalization_info->n > 0)
                {
                    std::unordered_map<long, double *> addr_shift;
                    for (int i = 0; i <= WINDOW_SIZE; i++)
                    {
                        if (i == WINDOW_SIZE - 1)
                            continue;
                        else if (i == WINDOW_SIZE)
                        {
                            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                        }
                        else
                        {
                            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                        }
                    }
                    for (int i = 0; i < NUM_OF_CAM; i++)
                        addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
                    if (ESTIMATE_TD)
                    {
                        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
                    }

                    vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
                    if (last_marginalization_info)
                        delete last_marginalization_info;
                    last_marginalization_info = marginalization_info;
                    last_marginalization_parameter_blocks = parameter_blocks;
                }
                else
                {
                    ROS_WARN(""" + '"' + MARKER + """ invalid second margin m=%d n=%d",
                             marginalization_info->m, marginalization_info->n);
                    delete marginalization_info;
                }
            }"""

# --- imu_factor.h ---

OLD_IMU_JAC = """            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");"""

NEW_IMU_JAC = """            if (pre_integration->jacobian.size() > 0 &&
                (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8))
            {
                ROS_WARN("numerical unstable in preintegration");"""

OLD_POSE_JAC = """                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in preintegration");"""

NEW_POSE_JAC = """                if (jacobian_pose_i.size() > 0 &&
                    (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8))
                {
                    ROS_WARN("numerical unstable in preintegration");"""


def patch_file(
    path: Path,
    bak_suffix: str,
    replacements: list[tuple[str, str, str]],
    *,
    already_patched: str | None = None,
) -> None:
    if not path.is_file():
        raise SystemExit(f"Нет {path}")
    text = path.read_text(encoding="utf-8", errors="replace")
    probe = already_patched if already_patched is not None else MARKER
    if probe in text:
        print(f"уже патчен {path}")
        return
    bak = path.with_suffix(path.suffix + bak_suffix)
    for label, old, new in replacements:
        if old not in text:
            raise SystemExit(f"{path}: не найден блок {label}")
        text = text.replace(old, new, 1)
    if not bak.is_file():
        shutil.copy2(path, bak)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")


def revert_file(path: Path, bak_suffix: str) -> None:
    bak = path.with_suffix(path.suffix + bak_suffix)
    if not bak.is_file():
        raise SystemExit(f"нет бэкапа {bak}")
    shutil.copy2(bak, path)
    print(f"restored {path}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()
    root = args.vins / "vins_estimator/src"

    if args.revert:
        revert_file(root / "factor/marginalization_factor.cpp", ".marg_empty_orig")
        revert_file(root / "estimator.cpp", ".marg_empty_orig")
        revert_file(root / "factor/imu_factor.h", ".marg_empty_orig")
        return 0

    patch_file(
        root / "factor/marginalization_factor.cpp",
        ".marg_empty_orig",
        [("marginalize start", OLD_MARG_START, NEW_MARG_START)],
    )
    patch_file(
        root / "estimator.cpp",
        ".marg_empty_orig",
        [
            ("visual margin + MARGIN_OLD", OLD_VIS_MARGIN, NEW_VIS_MARGIN),
            ("MARGIN_SECOND", OLD_MARG_SECOND, NEW_MARG_SECOND),
        ],
    )
    patch_file(
        root / "factor/imu_factor.h",
        ".marg_empty_orig",
        [
            ("imu jacobian guard", OLD_IMU_JAC, NEW_IMU_JAC),
            ("pose jacobian guard", OLD_POSE_JAC, NEW_POSE_JAC),
        ],
        already_patched="pre_integration->jacobian.size() > 0",
    )

    fm = root / "feature_manager.cpp"
    fm_text = fm.read_text(encoding="utf-8", errors="replace")
    if "if (svd_idx < 4)" in fm_text:
        print(f"feature_manager triangulate svd_idx guard OK ({fm})")
    else:
        print(f"WARN: svd_idx guard missing in {fm}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
