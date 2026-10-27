// Настраиваемые параметры (аналог build_config() из vo_final2.py).
// Значения по умолчанию совпадают с константами в шапке Python-версии.
#pragma once

#include <array>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace vo {

struct AltitudeCfg {
    std::string source = "const";       // "const" | "file"
    double value = 100.0;               // м, для "const"
    std::string file;                   // путь, для "file"
    std::string column = "auto";        // имя, номер или "auto"
    std::string frame_column = "auto";  // колонка кадра/времени
};

struct PreprocessCfg {
    bool undistort = true;
    bool use_clahe = false;
    double clahe_clip_limit = 2.0;
    int clahe_tile_grid = 8;
    std::vector<cv::Rect2d> osd_mask;   // прямоугольники [x, y, w, h]
    double osd_margin = 12.0;
    double osd_blur_sigma = 4.0;
    int osd_edge_radius = 6;
};

// Маска теней: связные тёмные пятна площади около тени аппарата.
// Не диск вокруг главной точки — только пиксели самой маски.
struct ShadowCfg {
    bool enable = true;
    double aircraft_span_m = 1.5;       // M300 с пропеллерами, м
    double max_altitude_m = 40.0;       // выше тень слишком мелкая
    double static_flow_px = 1.5;        // пары с почти нулевым смещением, px
};

struct CameraCfg {
    std::string calibration_file = "calibration/camera_calibration.json";
    std::optional<std::array<double, 2>> focal_length;      // fx, fy
    std::optional<std::array<double, 2>> principal_point;   // cx, cy (0-based)
    std::vector<double> distortion;                         // k1 k2 p1 p2 k3
    bool allow_approximate = true;
    double approximate_focal_factor = 0.9;
};

struct FeaturesCfg {
    int max_features = 1500;
    bool select_uniform = true;
    double default_keypoint_size = 7.0;
    // FAST параметры
    double fast_min_quality = 0.05;
    double fast_min_contrast = 0.1;
};

struct MatchingCfg {
    double max_ratio = 0.75;
    bool unique = true;
    double binary_threshold = 60.0;     // % от числа бит
    double float_threshold = 10.0;      // % от максимума SSD
    bool squared_float_metric = true;
    bool normalize_float_features = true;
};

struct RansacCfg {
    int min_matches = 8;
    int min_inliers = 12;
    int max_num_trials = 2000;
    double confidence = 99.9;           // проценты
    double max_distance = 1.0;          // порог проективного расстояния, px
    std::string method = "RANSAC";      // RANSAC | LMEDS | MAGSAC
    // Поддерживается только homography
    double homography_threshold = 3.0;
};

// Трекинг Лукаса-Канаде поверх соответствий FAST-FREAK. На поворотах
// дескрипторы теряют часть пар, а LK по соседним кадрам даёт больше точных
// (субпиксельных) соответствий для гомографии.
struct LkCfg {
    std::string mode = "off";           // off | rotation | always | only
    int window = 21;                    // окно, px
    int levels = 4;                     // уровни пирамиды
    int max_iter = 30;
    double epsilon = 0.01;
    double fb_threshold_px = 1.0;       // порог прямого-обратного трекинга
    double min_eig = 1e-4;
    bool use_initial_flow = true;       // предсказание по гомографии из FREAK
};

struct VoCfg {
    std::vector<std::string> algorithms = {"FAST-FREAK"};
    std::optional<double> plane_distance;
    bool use_metric_step = true;
    double translation_scale = 1.0;
    double step_length_limit = 3.0;
    double min_translate_alt_m = 5.0;   // ниже — только курс, без шага в метрах
    int max_saved_match_figures = 5;
};

struct VisualizationCfg {
    bool show_video = true;
    int video_width = 1100;
    bool video_points = true;
    bool video_tracks = true;
    bool video_dim = true;
    bool show_plots = false;
    bool live_plot = false;
    int live_plot_every = 10;
    bool save_match_examples = true;
    std::string trajectory_plane = "XY";   // XY | XZ | YZ
    int plot_width = 1040;                 // px; в Python был figsize*dpi
    int plot_height = 910;
};

struct FrameSelectCfg {
    std::string mode = "adaptive";      // adaptive | fixed
    double target_shift_pct = 10.0;
    int min_step = 1;
    int max_step = 120;
    int start_step = 5;
    double cruise_altitude_m = 50.0;    // выше этого потолок шага — cruise_max_step
    int cruise_max_step = 20;           // 2 с при 10 Гц; 120 кадров на крене вредны
    double damping = 0.5;
    double max_growth = 1.5;
    double turn_limit_dps = 3.0;
    double turn_reject_dps = 0.0;
    double turn_reject_min_deg = 4.0;
    double weak_turn_inlier_ratio = 0.50;   // большой поворот при меньшей доле инлаеров не пишется
    double turn_noise_floor_deg = 1.0;
    double turn_gate_fraction = 0.3;
};

struct GeometryCfg {
    double scale_correction = 1.0;
    cv::Point2d initial_coords{0.0, 0.0};
    double initial_heading_deg = 0.0;
};

struct RotationCropCfg {
    bool enable = true;
    double threshold_dps = 9.0;
    int delay_steps = 2;
    bool at_start = false;
    std::array<double, 4> rotation_cut{0.25, 0.25, 0.00, 0.30};  // L R T B
    std::array<double, 4> normal_cut{0.0, 0.0, 0.0, 0.0};
    std::string angle_source = "inplane";                        // inplane | total
};

struct PerfCfg {
    int num_threads = 0;
    bool profile = false;
    bool verbose_per_frame = true;
    bool fast_skip = true;
    bool save_match_figures = true;
    std::string preset = "balanced";
};

struct TelemetryCfg {
    std::string camera_timestamps;  // пусто — авто рядом с data_folder
    std::string attitude;           // pусто — <project_root>/other/dji_osdk_ros_attitude.csv
};

struct ExportCfg {
    std::string gps_file = "vo_track_gps.csv";
    std::string console_file = "console_log.txt";
    double ref_lat = 55.7558;
    double ref_lon = 37.6173;
    double ref_alt = 0.0;
    double scale = 1.0;
    double frame_rate = 25.0;
    std::string plane = "auto";
    bool north_flip = false;
};

struct InsExportCfg {
    bool enable = false;
    std::string output_dir = "ins_output";
    double frame_rate = 25.0;
};

struct Config {
    std::string project_root = ".";
    std::string data_folder = "video.mp4";
    std::string results_folder = "results";
    // < 0 - все кадры (в Python: float('inf')).
    long long max_frames = -1;
    int frame_step = 5;

    AltitudeCfg altitude;
    PreprocessCfg preprocess;
    ShadowCfg shadow;
    CameraCfg camera;
    FeaturesCfg features;
    MatchingCfg matching;
    RansacCfg ransac;
    LkCfg lk;
    VoCfg vo;
    VisualizationCfg vis;
    FrameSelectCfg frame_select;
    GeometryCfg geometry;
    RotationCropCfg rotation_crop;
    TelemetryCfg telemetry;
    PerfCfg perf;
    ExportCfg export_cfg;
    InsExportCfg ins_export;
};

// Config с предустановкой скорости (по умолчанию "balanced").
Config build_config(const std::string& project_root = ".");

// accurate | balanced | realtime | fast. Бросает std::runtime_error на неизвестное имя.
void apply_speed_preset(Config& cfg, const std::string& name);

}  // namespace vo
