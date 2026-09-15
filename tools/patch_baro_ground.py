#!/usr/bin/env python3
"""Патч VINS-Mono: барометр как опора метрического масштаба на надире.

Монокуляр над плоским полем вырожден по масштабу: высота почти не меняется,
IMU возбуждён слабо, глубина фич уходит от реальной. Барометр
(height_above_takeoff, бортовой, не RTK) даёт два дешёвых ограничения:

  1. приор обратной глубины: точка лежит на земле, depth ≈ высота над землёй;
  2. разность высот между кадрами окна: Δz визуала = Δz барометра.

Плюс INIT_DEPTH становится текущей высотой, а не константой из yaml.

    python3 tools/patch_baro_ground.py --vins ~/catkin_ws/src/VINS-Mono
    python3 tools/patch_baro_ground.py --vins ~/catkin_ws/src/VINS-Mono --revert
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

MARKER = "[VNAV-BARO-GROUND]"

# --------------------------------------------------------------------------
# parameters.h
# --------------------------------------------------------------------------
PH_OLD = "extern std::string IMU_TOPIC;"
PH_NEW = """extern std::string IMU_TOPIC;
// """ + MARKER + """
extern std::string BARO_TOPIC;
extern int USE_BARO_GROUND;
extern double BARO_GROUND_SIGMA;
extern double BARO_Z_SIGMA;
extern double ACC_BIAS_PRIOR_SIGMA;"""

# --------------------------------------------------------------------------
# parameters.cpp
# --------------------------------------------------------------------------
PC_DECL_OLD = "std::string IMU_TOPIC;"
PC_DECL_NEW = """std::string IMU_TOPIC;
// """ + MARKER + """
std::string BARO_TOPIC;
int USE_BARO_GROUND;
double BARO_GROUND_SIGMA;
double BARO_Z_SIGMA;
double ACC_BIAS_PRIOR_SIGMA;"""

PC_READ_OLD = '    fsSettings["imu_topic"] >> IMU_TOPIC;'
PC_READ_NEW = '''    fsSettings["imu_topic"] >> IMU_TOPIC;

    // ''' + MARKER + '''
    BARO_TOPIC = "";
    if (!fsSettings["baro_topic"].empty())
        fsSettings["baro_topic"] >> BARO_TOPIC;
    USE_BARO_GROUND = fsSettings["use_baro_ground"].empty()
                          ? 0 : (int)fsSettings["use_baro_ground"];
    BARO_GROUND_SIGMA = fsSettings["baro_ground_sigma"].empty()
                            ? 25.0 : (double)fsSettings["baro_ground_sigma"];
    BARO_Z_SIGMA = fsSettings["baro_z_sigma"].empty()
                       ? 0.5 : (double)fsSettings["baro_z_sigma"];
    ACC_BIAS_PRIOR_SIGMA = fsSettings["acc_bias_prior_sigma"].empty()
                               ? 0.0 : (double)fsSettings["acc_bias_prior_sigma"];
    if (USE_BARO_GROUND)
        ROS_WARN("baro ground prior: topic=%s ground_sigma=%.1f z_sigma=%.2f",
                 BARO_TOPIC.c_str(), BARO_GROUND_SIGMA, BARO_Z_SIGMA);'''

# --------------------------------------------------------------------------
# estimator.h
# --------------------------------------------------------------------------
EH_OLD = "    std_msgs::Header Headers[(WINDOW_SIZE + 1)];"
EH_NEW = """    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    // """ + MARKER + """ высота над точкой взлёта, м; -1 = нет данных
    double baro_h[(WINDOW_SIZE + 1)];
    double cur_baro_h;
    void setBaroHeight(double h);"""

# --------------------------------------------------------------------------
# estimator.cpp
# --------------------------------------------------------------------------
EC_FACTORS_OLD = "void Estimator::clearState()"
EC_FACTORS_NEW = """// """ + MARKER + """ приор обратной глубины: фича лежит на земле
struct VnavGroundDepthFactor
{
    VnavGroundDepthFactor(double inv_dep_meas, double sqrt_info)
        : inv_dep_meas_(inv_dep_meas), sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T *const inv_dep, T *residual) const
    {
        residual[0] = (inv_dep[0] - T(inv_dep_meas_)) * T(sqrt_info_);
        return true;
    }

    double inv_dep_meas_;
    double sqrt_info_;
};

// """ + MARKER + """ разность высот двух кадров окна по барометру.
// Мировая ось Z выровнена по гравитации, а double2vector() правит только yaw,
// поэтому разность z инвариантна к переякориванию окна.
struct VnavBaroDeltaZFactor
{
    VnavBaroDeltaZFactor(double dz_meas, double sqrt_info)
        : dz_meas_(dz_meas), sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T *const pose_i, const T *const pose_j, T *residual) const
    {
        residual[0] = ((pose_j[2] - pose_i[2]) - T(dz_meas_)) * T(sqrt_info_);
        return true;
    }

    double dz_meas_;
    double sqrt_info_;
};

// """ + MARKER + """ слабый приор нулевого смещения акселерометра.
// На крейсере IMU почти не возбуждён, масштаб задаёт приор земли, и всё
// расхождение уходит в Bas. Тот пробивает порог 2.5 м/с² в failureDetection
// и роняет оценщик (25 перезапусков за 90 с).
struct VnavAccBiasPrior
{
    explicit VnavAccBiasPrior(double sqrt_info) : sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T *const speed_bias, T *residual) const
    {
        residual[0] = speed_bias[3] * T(sqrt_info_);
        residual[1] = speed_bias[4] * T(sqrt_info_);
        residual[2] = speed_bias[5] * T(sqrt_info_);
        return true;
    }

    double sqrt_info_;
};

// """ + MARKER + """
void Estimator::setBaroHeight(double h)
{
    cur_baro_h = h;
}

void Estimator::clearState()"""

EC_CLEAR_OLD = """    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;"""
EC_CLEAR_NEW = """    // """ + MARKER + """
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
        baro_h[i] = -1.0;
    cur_baro_h = -1.0;

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;"""

EC_IMG_OLD = "    Headers[frame_count] = header;"
EC_IMG_NEW = """    Headers[frame_count] = header;

    // """ + MARKER + """ глубина инициализации = реальная высота над землёй
    baro_h[frame_count] = cur_baro_h;
    if (USE_BARO_GROUND && cur_baro_h > 5.0)
    {
        INIT_DEPTH = cur_baro_h;
        ROS_INFO_THROTTLE(10.0, "%s h=%.1f m -> INIT_DEPTH", "[VNAV-BARO]", cur_baro_h);
    }"""

EC_SLIDE1_OLD = """                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);"""
EC_SLIDE1_NEW = """                Headers[i] = Headers[i + 1];
                baro_h[i] = baro_h[i + 1];   // """ + MARKER + """
                Ps[i].swap(Ps[i + 1]);"""

EC_SLIDE2_OLD = """            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];"""
EC_SLIDE2_NEW = """            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            baro_h[WINDOW_SIZE] = baro_h[WINDOW_SIZE - 1];   // """ + MARKER + """
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];"""

EC_SLIDE3_OLD = """            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];"""
EC_SLIDE3_NEW = """            Headers[frame_count - 1] = Headers[frame_count];
            baro_h[frame_count - 1] = baro_h[frame_count];   // """ + MARKER + """
            Ps[frame_count - 1] = Ps[frame_count];"""

EC_DZ_OLD = """        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    int f_m_cnt = 0;"""
EC_DZ_NEW = """        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }

    // """ + MARKER + """
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
    if (USE_BARO_GROUND)
    {
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            int j = i + 1;
            const bool bi = baro_h[i] > 5.0;
            const bool bj = baro_h[j] > 5.0;
            if (!(bi && bj) && INIT_DEPTH <= 10.0)
                continue;
            const double dz_meas = (bi && bj) ? (baro_h[j] - baro_h[i]) : 0.0;
            ceres::CostFunction *dz = new ceres::AutoDiffCostFunction<VnavBaroDeltaZFactor, 1, 7, 7>(
                new VnavBaroDeltaZFactor(dz_meas, 1.0 / BARO_Z_SIGMA));
            problem.AddResidualBlock(dz, NULL, para_Pose[i], para_Pose[j]);
            baro_dz_cnt++;
        }
    }
    int f_m_cnt = 0;"""

EC_GROUND_OLD = """        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;"""
EC_GROUND_NEW = """        ++feature_index;

        // """ + MARKER + """ обратная глубина ≈ 1 / высота над землёй.
        // sqrt_info = h^2 / sigma_d — перенос метровой сигмы в обратную глубину.
        // Барометр предпочтительнее, но в HKairport03 топик пустой (h≈0 при 80 м),
        // поэтому опорой служит номинальная высота полёта init_depth из yaml.
        double h_agl = baro_h[it_per_id.start_frame];
        if (h_agl < 5.0)
            h_agl = INIT_DEPTH > 10.0 ? INIT_DEPTH : -1.0;
        if (USE_BARO_GROUND && h_agl > 5.0)
        {
            ceres::CostFunction *gd = new ceres::AutoDiffCostFunction<VnavGroundDepthFactor, 1, 1>(
                new VnavGroundDepthFactor(1.0 / h_agl, h_agl * h_agl / BARO_GROUND_SIGMA));
            problem.AddResidualBlock(gd, loss_function, para_Feature[feature_index]);
            baro_ground_cnt++;
        }

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;"""

EC_LOG_OLD = """    ROS_DEBUG("visual measurement count: %d", f_m_cnt);"""
EC_LOG_NEW = """    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    if (USE_BARO_GROUND)   // """ + MARKER + """
        ROS_INFO_THROTTLE(10.0, "[VNAV-BARO] dz factors %d, ground depth priors %d",
                          baro_dz_cnt, baro_ground_cnt);"""

# --------------------------------------------------------------------------
# estimator_node.cpp
# --------------------------------------------------------------------------
EN_INC_OLD = '#include "utility/visualization.h"'
EN_INC_NEW = '''#include "utility/visualization.h"

// ''' + MARKER + '''
#include <deque>
#include <cmath>
#include <geometry_msgs/PointStamped.h>'''

EN_BUF_OLD = "double last_imu_t = 0;"
EN_BUF_NEW = """double last_imu_t = 0;

// """ + MARKER + """ высота над точкой взлёта с бортового барометра
std::mutex m_baro;
std::deque<std::pair<double, double>> baro_buf;

void baro_callback(const geometry_msgs::PointStampedConstPtr &msg)
{
    std::lock_guard<std::mutex> lk(m_baro);
    baro_buf.emplace_back(msg->header.stamp.toSec(), msg->point.z);
    while (baro_buf.size() > 8000)
        baro_buf.pop_front();
}

// Ближайший отсчёт к метке кадра; дальше 1 с считаем, что данных нет.
static double baro_at(double t)
{
    std::lock_guard<std::mutex> lk(m_baro);
    double best = -1.0, best_dt = 1e9;
    for (auto it = baro_buf.rbegin(); it != baro_buf.rend(); ++it)
    {
        const double dt = std::fabs(it->first - t);
        if (dt < best_dt)
        {
            best_dt = dt;
            best = it->second;
        }
        else if (it->first < t - 1.0)
            break;
    }
    return best_dt < 1.0 ? best : -1.0;
}"""

EN_FEED_OLD = "            estimator.processImage(image, img_msg->header);"
EN_FEED_NEW = """            // """ + MARKER + """
            if (USE_BARO_GROUND)
                estimator.setBaroHeight(baro_at(img_msg->header.stamp.toSec()));
            estimator.processImage(image, img_msg->header);"""

EN_SUB_OLD = '    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);'
EN_SUB_NEW = '''    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    // ''' + MARKER + '''
    ros::Subscriber sub_baro;
    if (USE_BARO_GROUND && !BARO_TOPIC.empty())
    {
        sub_baro = n.subscribe(BARO_TOPIC, 2000, baro_callback);
        ROS_WARN("baro ground prior enabled on %s", BARO_TOPIC.c_str());
    }'''

EDITS = {
    "vins_estimator/src/parameters.h": [(PH_OLD, PH_NEW)],
    "vins_estimator/src/parameters.cpp": [(PC_DECL_OLD, PC_DECL_NEW), (PC_READ_OLD, PC_READ_NEW)],
    "vins_estimator/src/estimator.h": [(EH_OLD, EH_NEW)],
    "vins_estimator/src/estimator.cpp": [
        (EC_FACTORS_OLD, EC_FACTORS_NEW),
        (EC_CLEAR_OLD, EC_CLEAR_NEW),
        (EC_IMG_OLD, EC_IMG_NEW),
        (EC_SLIDE1_OLD, EC_SLIDE1_NEW),
        (EC_SLIDE2_OLD, EC_SLIDE2_NEW),
        (EC_SLIDE3_OLD, EC_SLIDE3_NEW),
        (EC_DZ_OLD, EC_DZ_NEW),
        (EC_GROUND_OLD, EC_GROUND_NEW),
        (EC_LOG_OLD, EC_LOG_NEW),
    ],
    "vins_estimator/src/estimator_node.cpp": [
        (EN_INC_OLD, EN_INC_NEW),
        (EN_BUF_OLD, EN_BUF_NEW),
        (EN_FEED_OLD, EN_FEED_NEW),
        (EN_SUB_OLD, EN_SUB_NEW),
    ],
}

SUFFIX = ".baroground_orig"


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
                raise SystemExit(f"Не найден якорь в {path}:\n{old[:120]}")
            text = text.replace(old, new, 1)
        bak = path.with_suffix(path.suffix + SUFFIX)
        if not bak.is_file():
            shutil.copy2(path, bak)
        path.write_text(text, encoding="utf-8")
        print(f"patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
