#!/usr/bin/env python3
"""Патч VINS-Mono: курс держит гироскоп, не надирный визуал.

На поле на 80 м сотни проекционных невязок выглядят как yaw вокруг
оптической оси (= почти вертикаль). IMU-фактор с gyr_n=0.05 слишком слаб,
и окно закручивает траекторию в спираль. RTK в estimator не идёт.

Два ограничения без СНС:

  1. yaw вокруг мировой гравитации: Δyaw(Qi,Qj) ≈ yaw(IMU Δq);
  2. приор нулевого смещения гироскопа, чтобы Bgs не съедал этот yaw.

    python3 tools/patch_gyro_yaw.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_gyro_yaw.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

MARKER = "[VNAV-GYRO-YAW]"

PH_OLD = "extern double ACC_BIAS_PRIOR_SIGMA;"
PH_NEW = """extern double ACC_BIAS_PRIOR_SIGMA;
// """ + MARKER + """
extern double GYRO_YAW_SIGMA;
extern double GYR_BIAS_PRIOR_SIGMA;
extern double PROJECTION_INFO_SCALE;"""

PC_DECL_OLD = "double ACC_BIAS_PRIOR_SIGMA;"
PC_DECL_NEW = """double ACC_BIAS_PRIOR_SIGMA;
// """ + MARKER + """
double GYRO_YAW_SIGMA;
double GYR_BIAS_PRIOR_SIGMA;
double PROJECTION_INFO_SCALE;"""

PC_READ_OLD = """    ACC_BIAS_PRIOR_SIGMA = fsSettings["acc_bias_prior_sigma"].empty()
                               ? 0.0 : (double)fsSettings["acc_bias_prior_sigma"];
    if (USE_BARO_GROUND)
        ROS_WARN("baro ground prior: topic=%s ground_sigma=%.1f z_sigma=%.2f",
                 BARO_TOPIC.c_str(), BARO_GROUND_SIGMA, BARO_Z_SIGMA);"""
PC_READ_NEW = """    ACC_BIAS_PRIOR_SIGMA = fsSettings["acc_bias_prior_sigma"].empty()
                               ? 0.0 : (double)fsSettings["acc_bias_prior_sigma"];
    // """ + MARKER + """
    GYRO_YAW_SIGMA = fsSettings["gyro_yaw_sigma"].empty()
                         ? 0.0 : (double)fsSettings["gyro_yaw_sigma"];
    GYR_BIAS_PRIOR_SIGMA = fsSettings["gyr_bias_prior_sigma"].empty()
                               ? 0.0 : (double)fsSettings["gyr_bias_prior_sigma"];
    PROJECTION_INFO_SCALE = fsSettings["projection_info_scale"].empty()
                                ? 1.0 : (double)fsSettings["projection_info_scale"];
    if (USE_BARO_GROUND)
        ROS_WARN("baro ground prior: topic=%s ground_sigma=%.1f z_sigma=%.2f",
                 BARO_TOPIC.c_str(), BARO_GROUND_SIGMA, BARO_Z_SIGMA);
    if (GYRO_YAW_SIGMA > 0.0 || GYR_BIAS_PRIOR_SIGMA > 0.0)
        ROS_WARN("gyro yaw: sigma=%.3f rad, gyr_bias_prior=%.3f, vis_scale=%.2f",
                 GYRO_YAW_SIGMA, GYR_BIAS_PRIOR_SIGMA, PROJECTION_INFO_SCALE);"""

EC_SET_OLD = """    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;"""
EC_SET_NEW = """    f_manager.setRic(ric);
    {
        // """ + MARKER + """ ослабить визуал на надире, не трогая IMU
        const double vis = PROJECTION_INFO_SCALE > 1e-6 ? PROJECTION_INFO_SCALE : 1.0;
        ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * vis * Matrix2d::Identity();
        ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * vis * Matrix2d::Identity();
    }
    td = TD;"""

EC_FACTORS_OLD = """    double sqrt_info_;
};

// [VNAV-BARO-GROUND]
void Estimator::setBaroHeight(double h)"""
EC_FACTORS_NEW = """    double sqrt_info_;
};

// """ + MARKER + """ yaw вокруг мировой гравитации (ось +Z VINS) по IMU Δq.
// Надирный визуал не должен перетягивать курс: сотни проекций выглядят
// как вращение вокруг оптической оси ≈ вертикали.
struct VnavGyroYawFactor
{
    VnavGyroYawFactor(const Eigen::Quaterniond &dq,
                      const Eigen::Matrix3d &dq_dbg,
                      const Eigen::Vector3d &bg0,
                      double sqrt_info)
        : dqx_(dq.x()), dqy_(dq.y()), dqz_(dq.z()), dqw_(dq.w()),
          sqrt_info_(sqrt_info)
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                dq_dbg_[r * 3 + c] = dq_dbg(r, c);
        bg0_[0] = bg0.x();
        bg0_[1] = bg0.y();
        bg0_[2] = bg0.z();
    }

    template <typename T>
    bool operator()(const T *const pose_i, const T *const speed_bias_i,
                    const T *const pose_j, T *residual) const
    {
        const Eigen::Quaternion<T> Qi = Eigen::Quaternion<T>(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);
        const Eigen::Quaternion<T> Qj = Eigen::Quaternion<T>(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);
        const Eigen::Quaternion<T> dQ = Eigen::Quaternion<T>(T(dqw_), T(dqx_), T(dqy_), T(dqz_));

        Eigen::Matrix<T, 3, 1> dbg;
        dbg << speed_bias_i[6] - T(bg0_[0]),
               speed_bias_i[7] - T(bg0_[1]),
               speed_bias_i[8] - T(bg0_[2]);
        Eigen::Matrix<T, 3, 1> dtheta = Eigen::Matrix<T, 3, 1>::Zero();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                dtheta(r) += T(dq_dbg_[r * 3 + c]) * dbg(c);

        Eigen::Quaternion<T> dQ_corr = dQ * Utility::deltaQ(dtheta);
        if (dQ_corr.w() < T(0))
            dQ_corr.coeffs() *= T(-1);

        const Eigen::Quaternion<T> qij = Qi.conjugate() * Qj;
        Eigen::Quaternion<T> qerr = dQ_corr.conjugate() * qij;
        if (qerr.w() < T(0))
            qerr.coeffs() *= T(-1);

        const Eigen::Matrix<T, 3, 1> ez_i =
            Qi.conjugate() * Eigen::Matrix<T, 3, 1>(T(0), T(0), T(1));
        const Eigen::Matrix<T, 3, 1> v(qerr.x(), qerr.y(), qerr.z());
        residual[0] = T(2.0) * v.dot(ez_i) * T(sqrt_info_);
        return true;
    }

    double dqx_, dqy_, dqz_, dqw_;
    double dq_dbg_[9];
    double bg0_[3];
    double sqrt_info_;
};

// """ + MARKER + """ приор нулевого смещения гироскопа (рад/с).
struct VnavGyrBiasPrior
{
    explicit VnavGyrBiasPrior(double sqrt_info) : sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T *const speed_bias, T *residual) const
    {
        residual[0] = speed_bias[6] * T(sqrt_info_);
        residual[1] = speed_bias[7] * T(sqrt_info_);
        residual[2] = speed_bias[8] * T(sqrt_info_);
        return true;
    }

    double sqrt_info_;
};

// [VNAV-BARO-GROUND]
void Estimator::setBaroHeight(double h)"""

EC_IMU_OLD = """        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }

    // [VNAV-BARO-GROUND]
    int baro_dz_cnt = 0;
    int baro_ground_cnt = 0;
    if (ACC_BIAS_PRIOR_SIGMA > 0.0)
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            ceres::CostFunction *bp = new ceres::AutoDiffCostFunction<VnavAccBiasPrior, 3, 9>(
                new VnavAccBiasPrior(1.0 / ACC_BIAS_PRIOR_SIGMA));
            problem.AddResidualBlock(bp, NULL, para_SpeedBias[i]);
        }
    }"""
EC_IMU_NEW = """        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        // """ + MARKER + """
        if (GYRO_YAW_SIGMA > 0.0)
        {
            const Eigen::Matrix3d dq_dbg =
                pre_integrations[j]->jacobian.template block<3, 3>(O_R, O_BG);
            ceres::CostFunction *gy = new ceres::AutoDiffCostFunction<VnavGyroYawFactor, 1, 7, 9, 7>(
                new VnavGyroYawFactor(pre_integrations[j]->delta_q, dq_dbg,
                                      pre_integrations[j]->linearized_bg,
                                      1.0 / GYRO_YAW_SIGMA));
            problem.AddResidualBlock(gy, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j]);
        }
    }

    // [VNAV-BARO-GROUND]
    int baro_dz_cnt = 0;
    int baro_ground_cnt = 0;
    if (ACC_BIAS_PRIOR_SIGMA > 0.0)
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            ceres::CostFunction *bp = new ceres::AutoDiffCostFunction<VnavAccBiasPrior, 3, 9>(
                new VnavAccBiasPrior(1.0 / ACC_BIAS_PRIOR_SIGMA));
            problem.AddResidualBlock(bp, NULL, para_SpeedBias[i]);
        }
    }
    // """ + MARKER + """
    if (GYR_BIAS_PRIOR_SIGMA > 0.0)
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            ceres::CostFunction *gbp = new ceres::AutoDiffCostFunction<VnavGyrBiasPrior, 3, 9>(
                new VnavGyrBiasPrior(1.0 / GYR_BIAS_PRIOR_SIGMA));
            problem.AddResidualBlock(gbp, NULL, para_SpeedBias[i]);
        }
    }"""

EDITS = {
    "vins_estimator/src/parameters.h": [(PH_OLD, PH_NEW)],
    "vins_estimator/src/parameters.cpp": [(PC_DECL_OLD, PC_DECL_NEW), (PC_READ_OLD, PC_READ_NEW)],
    "vins_estimator/src/estimator.cpp": [
        (EC_SET_OLD, EC_SET_NEW),
        (EC_FACTORS_OLD, EC_FACTORS_NEW),
        (EC_IMU_OLD, EC_IMU_NEW),
    ],
}

SUFFIX = ".gyroyaw_orig"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, default=Path.home() / "catkin_ws/src/VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if args.revert:
        for rel in EDITS:
            path = args.vins / rel
            bak = path.with_suffix(path.suffix + SUFFIX)
            if bak.is_file():
                shutil.copy2(bak, path)
                print(f"restored {path}")
        return 0

    for rel, edits in EDITS.items():
        path = args.vins / rel
        if not path.is_file():
            raise SystemExit(f"Нет {path}")
        text = path.read_text(encoding="utf-8", errors="replace")
        if MARKER in text:
            print(f"уже патчен {path}")
            continue
        for old, new in edits:
            if text.count(old) < 1:
                raise SystemExit(f"Не найден якорь в {path}:\n{old[:160]}")
            text = text.replace(old, new, 1)
        bak = path.with_suffix(path.suffix + SUFFIX)
        if not bak.is_file():
            shutil.copy2(path, bak)
        path.write_text(text, encoding="utf-8")
        print(f"patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
