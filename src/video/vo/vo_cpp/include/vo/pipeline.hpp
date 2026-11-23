// Главный конвейер: траектория одним методом и всеми методами сразу
// (порт run_visual_odometry / run_all_visual_odometry из vo_final2.py),
// экспорт трека в формате GPS, таблицы и графики.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vo/config.hpp"
#include "vo/sequence.hpp"
#include "vo/util.hpp"

namespace vo {

struct PlaneInfo {
    int col_x = 0, col_y = 1;          // индексы столбцов positions
    std::string label_x, label_y;      // "X", "Y"
    std::string suffix;                // "xy"
    std::string name;                  // "X-Y"
};
// Плоскость двумерного графика по cfg.vis.trajectory_plane (XY|XZ|YZ).
PlaneInfo trajectory_plane(const Config& cfg);

// СИСТЕМА КООРДИНАТ траектории:
// 
// ВНУТРЕННЕЕ ПРЕДСТАВЛЕНИЕ (positions, x/y/z в TrajectoryRow):
// - positions[0] = x = восток (East)
// - positions[1] = y = север (North)
// - positions[2] = z = вверх (Up)
// Это стандартная система ENU (East-North-Up).
//
// ГРАФИКИ (trajectory_plane):
// - снизу Z = восток (positions[0])
// - слева X = север (positions[1])
// - Y = вверх (positions[2])
// На 2D графике "XY" горизонталь — восток, вертикаль — север.
struct TrajectoryRow {
    int frame = 0;                     // номер кадра, 1-based
    std::string file;
    double x = 0, y = 0, z = 0;        // координаты (см. комментарий о СК выше)
    double altitude = 0;
    double speed_mps = 0, heading_deg = 0, turn_deg = 0, step_m = 0, shift_pct = 0;
    int frame_step = 0, matches = 0, inliers = 0;
    double inlier_ratio_pct = 0, step_time_ms = 0;
    std::string status, model;
    int rotation_mode = 0;
    double used_frame_pct = 100.0;
    int lk_tracks = 0;                 // соответствий от трекинга Лукаса-Канаде
};

struct VoRun {
    std::vector<TrajectoryRow> rows;
    std::vector<cv::Vec3d> positions;       // метры: [0]=восток, [1]=север, [2]=вверх (ENU)
    std::vector<cv::Matx33d> rotations;
    std::vector<int> frame_ids;             // 0-based
    std::string folder;                     // где лежат результаты
    double elapsed_sec = 0;
};

struct TrajectorySummary {
    std::string algorithm;
    int num_frames = 0, successful_steps = 0;
    double mean_matches = 0, mean_inliers = 0, mean_inlier_ratio_pct = 0;
    double path_length = 0, final_x = 0, final_y = 0, final_z = 0, elapsed_sec = 0;
};

// Траектория одной связкой. sequence == nullptr - открыть cfg.data_folder.
// algorithm пусто - первая из cfg.vo.algorithms. Результаты:
// <results>/visual_odometry/<связка>/{trajectory.csv, vo_track_gps.csv, *.png, matches/}.
VoRun run_visual_odometry(const Config& cfg, Sequence* sequence = nullptr,
                          const std::string& algorithm = "", bool verbose = true);

// Траектория каждой доступной (или заказанной) связкой + сравнительные графики.
std::vector<TrajectorySummary> run_all_visual_odometry(Config cfg, bool verbose = true);

// CSV с колонками TRAJECTORY_COLUMNS.
void write_trajectory_csv(const std::vector<TrajectoryRow>& rows, const std::string& path);

// timestamp,lat,lon,alt,speed,heading. Возвращает путь к файлу ("" если пусто).
std::string write_gps_track(const std::vector<cv::Vec3d>& positions,
                            const std::vector<int>& frame_ids, double frame_rate,
                            const std::string& folder, const Config& cfg,
                            const std::vector<double>* altitude = nullptr, bool one_based = false);

// Графики: траектория в плоскости и три проекции.
void plot_trajectory(const std::vector<cv::Vec3d>& positions, const std::string& folder,
                     const std::string& alg_name, const Config& cfg);
void plot_trajectory_comparison(const std::vector<std::string>& names,
                                const std::vector<std::vector<cv::Vec3d>>& positions,
                                const std::string& folder, const Config& cfg);
// Два кадра рядом, соответствия соединены линиями (не более ~300 линий).
void save_match_figure(const cv::Mat& image1, const cv::Mat& image2,
                       const std::vector<cv::Point2f>& points1,
                       const std::vector<cv::Point2f>& points2, const std::string& filename,
                       const std::string& title);

}  // namespace vo
