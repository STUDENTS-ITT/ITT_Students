// run_all_hkairport03 - прогон всех моделей движения на наборе HKairport03 и
// сверка с RTK (C++-порт run_all_HKairport03.py).
//
//   run_all_hkairport03 [--dataset DIR] [--out DIR] [--show-video]
//
// Скрипт делает четыре вещи:
//   1) готовит входные файлы набора: калибровку камеры, высоту по кадрам и
//      эталонный трек GPS (из RTK);
//   2) прогоняет конвейер одной моделью: homography;
//   3) замеряет, сколько миллисекунд стоит каждый этап конвейера;
//   4) сверяет траектории с RTK и рисует графики.
//
// Коррекции по GPS/ИНС в самой одометрии НЕТ. RTK участвует как эталон для
// сверки и как замена высотомера: вертикальный канал выгружается в
// altitude_per_frame.csv и подаётся в конвейер как высота по кадрам.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vo/altitude.hpp"
#include "vo/calibration.hpp"
#include "vo/config.hpp"
#include "vo/features.hpp"
#include "vo/matching.hpp"
#include "vo/motion.hpp"
#include "vo/pipeline.hpp"
#include "vo/plot.hpp"
#include "vo/util.hpp"

namespace fs = std::filesystem;
using vo::fmt;

// =============================================================================
//  ПАРАМЕТРЫ НАБОРА HKairport03
// =============================================================================
namespace {

constexpr double kFps = 10.0;               // аппаратная синхронизация PPS

// Камера Hikvision CA-050-11UC: матрица Sony IMX264, пиксель 3.45 мкм, объектив
// 5 мм. Отсюда f = 5 мм / 3.45 мкм = 1449 px. Без калибровки подставилось бы
// приближение 0.9*2448 = 2203 px - ошибка масштаба в полтора раза. Дисторсия
// нулевая: кадры уже исправлены.
constexpr double kPixelUm = 3.45;
constexpr double kLensMm = 5.0;
constexpr int kWidth = 2448, kHeight = 2048;

// Связка особых точек. FAST-FREAK: детектор FAST + двоичный дескриптор FREAK.
const char* const kAlgorithm = "FAST-FREAK";

// Единственная модель движения.
const std::vector<std::string> kModels = {"homography"};

// Масштаб не подгоняется: f и высота известны из паспорта и телеметрии.
constexpr double kScaleCorrection = 1.0;

// Границы фаз полёта (с от первого кадра) и развороты между галсами.
const std::pair<double, double> kTurns[] = {{105, 120}, {140, 155}, {185, 200}, {232, 247}, {275, 292}};

struct Paths {
    std::string here;            // каталог набора
    std::string out;             // куда писать результаты
    std::string frames, calibration, altitude_file, gps_reference, rtk_csv, camera_csv, attitude_csv;
    std::string results, plots_dir, plot_models, plot_profile, plot_by_model;
};

std::string join(const std::string& a, const std::string& b) {
    return (fs::u8path(a) / fs::u8path(b)).u8string();
}

Paths make_paths(const std::string& here, const std::string& out) {
    Paths p;
    p.here = here;
    p.out = out;
    p.frames = join(here, "camera/images");      // 3657 кадров, 2448x2048
    p.calibration = join(here, "calibration/camera.json");
    p.altitude_file = join(here, "altitude_per_frame.csv");
    p.gps_reference = join(here, "gps_reference.csv");
    p.rtk_csv = join(here, "gps/dji_osdk_ros_rtk_position.csv");
    p.camera_csv = join(here, "camera/timestamps.csv");
    p.attitude_csv = join(here, "other/dji_osdk_ros_attitude.csv");
    p.results = join(out, "results_HKairport03");
    p.plots_dir = join(out, "trajectory_plots");
    p.plot_models = join(p.plots_dir, "HK_01_models.png");        // трек и ошибка
    p.plot_profile = join(p.plots_dir, "HK_02_profile.png");      // условия полёта
    p.plot_by_model = join(p.plots_dir, "HK_03_by_model.png");    // модели порознь
    return p;
}

// ---------------------------------------------------------------------------
//  Мелкий CSV-ридер (без кавычек, разделитель запятая)
// ---------------------------------------------------------------------------
struct Csv {
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    int col(const std::string& name) const {
        for (size_t i = 0; i < header.size(); ++i)
            if (header[i] == name) return static_cast<int>(i);
        throw std::runtime_error("В CSV нет колонки \"" + name + "\"");
    }
    std::vector<double> numbers(int c) const {
        std::vector<double> v;
        v.reserve(rows.size());
        for (const auto& r : rows) v.push_back(c < static_cast<int>(r.size()) ? std::strtod(r[c].c_str(), nullptr) : std::nan(""));
        return v;
    }
    std::vector<double> numbers(const std::string& name) const { return numbers(col(name)); }
};

Csv read_csv(const std::string& path) {
    std::ifstream in(fs::u8path(path), std::ios::binary);
    if (!in) throw std::runtime_error("Не удалось открыть файл: " + path);
    Csv csv;
    std::string line;
    bool first = true;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (first && line.size() >= 3 && static_cast<unsigned char>(line[0]) == 0xEF) line.erase(0, 3);
        auto cells = vo::split(line, ',');
        for (auto& c : cells) c = vo::trim(c);
        if (first) {
            csv.header = cells;
            first = false;
        } else {
            csv.rows.push_back(std::move(cells));
        }
    }
    return csv;
}

// Метка ДАТЧИКА в секундах, а не bag_time: запись в bag отстаёт от съёмки на
// 2.8 с, а при 9 м/с это 25 метров - весь бюджет ошибки.
std::vector<double> sensor_time(const std::string& path, int sec_col, int nsec_col) {
    const Csv csv = read_csv(path);
    const auto s = csv.numbers(sec_col), ns = csv.numbers(nsec_col);
    std::vector<double> t(s.size());
    for (size_t i = 0; i < s.size(); ++i) t[i] = s[i] + ns[i] * 1e-9;
    return t;
}

struct Rtk {
    std::vector<double> t, lat, lon, alt;
};

Rtk read_rtk(const Paths& P) {
    const Csv csv = read_csv(P.rtk_csv);
    Rtk r;
    r.lat = csv.numbers("latitude");
    r.lon = csv.numbers("longitude");
    r.alt = csv.numbers("altitude");
    r.t = sensor_time(P.rtk_csv, 3, 4);
    return r;
}

// WGS-84 -> местные метры (восток, север) от первой точки.
void to_local(const std::vector<double>& lat, const std::vector<double>& lon,
              std::vector<double>& east, std::vector<double>& north) {
    const double lat0 = lat[0], lon0 = lon[0];
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    const double s = std::sin(vo::deg2rad(lat0));
    const double rn = a / std::sqrt(1 - e2 * s * s);
    const double rm = rn * (1 - e2) / (1 - e2 * s * s);
    east.resize(lat.size());
    north.resize(lat.size());
    for (size_t i = 0; i < lat.size(); ++i) {
        east[i] = vo::deg2rad(lon[i] - lon0) * rn * std::cos(vo::deg2rad(lat0));
        north[i] = vo::deg2rad(lat[i] - lat0) * rm;
    }
}

double path_length(const std::vector<double>& x, const std::vector<double>& y) {
    double s = 0;
    for (size_t i = 1; i < x.size(); ++i) s += std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
    return s;
}

struct Truth {
    std::vector<double> t_rtk, east, north, height, t_cam;
};

// ---------------------------------------------------------------------------
//  1. ПОДГОТОВКА ВХОДНЫХ ФАЙЛОВ
// ---------------------------------------------------------------------------
Truth prepare(const Paths& P) {
    vo::make_dirs(vo::parent_path(P.calibration));
    const double focal = kLensMm * 1e-3 / (kPixelUm * 1e-6);
    {
        std::ofstream js(fs::u8path(P.calibration), std::ios::binary);
        js << "{\n"
           << fmt("  \"fx\": %.15g,\n  \"fy\": %.15g,\n", focal, focal)
           << fmt("  \"cx\": %.15g,\n  \"cy\": %.15g,\n", (kWidth - 1) / 2.0, (kHeight - 1) / 2.0)
           << fmt("  \"imageSize\": [\n    %d,\n    %d\n  ],\n", kHeight, kWidth)
           << "  \"distortion\": [\n    0.0,\n    0.0,\n    0.0,\n    0.0,\n    0.0\n  ],\n"
           << "  \"note\": \"Hikvision CA-050-11UC, 2448x2048, pixel 3.45 um, lens 5 mm\"\n}\n";
    }

    const Rtk rtk = read_rtk(P);
    const std::vector<double> t_cam = sensor_time(P.camera_csv, 6, 7);
    std::vector<double> head(rtk.alt.begin(), rtk.alt.begin() + std::min<size_t>(20, rtk.alt.size()));
    const double ground = vo::median(head);

    // Высота над точкой взлёта на КАЖДЫЙ кадр камеры. Топик
    // height_above_takeoff в этом наборе не заполнен (везде около нуля),
    // поэтому высота берётся из вертикального канала RTK.
    std::vector<double> rtk_h(rtk.alt.size());
    for (size_t i = 0; i < rtk.alt.size(); ++i) rtk_h[i] = rtk.alt[i];
    std::vector<double> height = vo::interp(t_cam, rtk.t, rtk_h);
    for (double& h : height) h = std::max(h - ground, 0.05);
    {
        std::ofstream out(fs::u8path(P.altitude_file), std::ios::binary);
        out << "frame,altitude\n";
        for (size_t i = 0; i < height.size(); ++i) out << i << "," << fmt("%.3f", height[i]) << "\n";
    }

    std::vector<double> east, north;
    to_local(rtk.lat, rtk.lon, east, north);
    {
        std::ofstream out(fs::u8path(P.gps_reference), std::ios::binary);
        out << "time_s,latitude,longitude,east_m,north_m,height_m\n";
        for (size_t i = 0; i < rtk.t.size(); ++i)
            out << fmt("%.3f,%.9f,%.9f,%.3f,%.3f,%.3f\n", rtk.t[i] - t_cam[0], rtk.lat[i], rtk.lon[i],
                       east[i], north[i], rtk.alt[i] - ground);
    }

    std::cout << "Входные файлы готовы:\n"
              << fmt("  калибровка       camera.json  (f = %.1f px)\n", focal)
              << fmt("  высота по кадрам altitude_per_frame.csv  (%.1f..%.1f м)\n",
                     *std::min_element(height.begin(), height.end()),
                     *std::max_element(height.begin(), height.end()))
              << fmt("  эталон GPS       gps_reference.csv  (%zu точек, путь %.0f м)\n", rtk.t.size(),
                     path_length(east, north));

    Truth tr;
    tr.t_rtk.resize(rtk.t.size());
    tr.height.resize(rtk.t.size());
    for (size_t i = 0; i < rtk.t.size(); ++i) {
        tr.t_rtk[i] = rtk.t[i] - t_cam[0];
        tr.height[i] = rtk.alt[i] - ground;
    }
    tr.east = east;
    tr.north = north;
    tr.t_cam.resize(t_cam.size());
    for (size_t i = 0; i < t_cam.size(); ++i) tr.t_cam[i] = t_cam[i] - t_cam[0];
    return tr;
}

// ---------------------------------------------------------------------------
//  2. ПРОГОН МОДЕЛЕЙ
// ---------------------------------------------------------------------------
double g_scale_correction = kScaleCorrection;

vo::Config make_config(const Paths& P, bool show_video) {
    vo::Config cfg = vo::build_config(P.here);
    cfg.data_folder = P.frames;
    cfg.camera.calibration_file = P.calibration;
    cfg.telemetry.camera_timestamps = P.camera_csv;
    cfg.telemetry.attitude = P.attitude_csv;
    cfg.altitude.source = "file";
    cfg.altitude.file = P.altitude_file;
    cfg.geometry.scale_correction = g_scale_correction;
    cfg.export_cfg.frame_rate = kFps;
    cfg.vo.algorithms = {kAlgorithm};
    cfg.perf.verbose_per_frame = false;                 // --quiet
    cfg.vis.show_video = show_video;
    return cfg;
}

std::map<std::string, double> run_models(const Paths& P, bool show_video, const std::string& lk_mode) {
    std::map<std::string, double> elapsed;
    for (const std::string& model : kModels) {
        std::cout << "\n--- модель " << model << " ---\n";
        const double started = vo::now_sec();
        // Каждая модель пишет в СВОЮ папку: имя подпапки внутри results
        // складывается из связки точек, а не из модели движения.
        vo::Config cfg = make_config(P, show_video);
        cfg.lk.mode = lk_mode;
        cfg.results_folder = join(P.results, model);
        try {
            vo::run_visual_odometry(cfg);
        } catch (const std::exception& e) {
            std::cout << "[ОШИБКА] модель " << model << " завершилась с ошибкой: " << e.what() << "\n";
        }
        elapsed[model] = vo::now_sec() - started;
    }
    return elapsed;
}

// ---------------------------------------------------------------------------
//  3. ЗАМЕР ВРЕМЕНИ ПО ЭТАПАМ
// ---------------------------------------------------------------------------
struct Timing {
    int points = 0, matches = 0;
    double detect = 0, match = 0;
    std::map<std::string, double> models;
};

// Модели сравниваются на ОДНИХ И ТЕХ ЖЕ соответствиях: иначе сравнение
// нечестное - разные модели уводят адаптивный выбор кадров на разный шаг.
Timing benchmark(const Paths& P, const std::vector<int>& frames = {1200, 1208, 1600, 1608, 2000, 2008, 2400, 2408},
                 int repeats = 5) {
    vo::Config cfg = make_config(P, false);
    cfg.vo.algorithms = {kAlgorithm};
    vo::Intrinsics intr;
    intr.fx = intr.fy = kLensMm * 1e-3 / (kPixelUm * 1e-6);
    intr.cx = (kWidth - 1) / 2.0;
    intr.cy = (kHeight - 1) / 2.0;
    intr.height = kHeight;
    intr.width = kWidth;

    vo::AlgorithmRunner runner(vo::select_algorithms(cfg).at(0), cfg);

    std::vector<cv::Mat> images;
    for (int f : frames) {
        cv::Mat im = cv::imread(join(P.frames, fmt("%06d.jpg", f)), cv::IMREAD_GRAYSCALE);
        if (im.empty()) throw std::runtime_error(fmt("Нет кадра %06d.jpg в %s", f, P.frames.c_str()));
        images.push_back(im);
    }

    std::vector<vo::FeatureData> data(images.size());
    double started = vo::now_sec();
    for (int r = 0; r < repeats; ++r)
        for (size_t i = 0; i < images.size(); ++i) data[i] = vo::detect_and_describe(images[i], runner, cfg, nullptr);
    Timing t;
    t.detect = 1000 * (vo::now_sec() - started) / (repeats * static_cast<double>(images.size()));

    started = vo::now_sec();
    for (int r = 0; r < repeats; ++r)
        for (size_t i = 0; i + 1 < images.size(); i += 2) vo::match_feature_sets(data[i], data[i + 1], cfg);
    t.match = 1000 * (vo::now_sec() - started) / (repeats * static_cast<double>(images.size() / 2));

    std::vector<std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>>> pairs;
    for (size_t i = 0; i + 1 < images.size(); i += 2) {
        const auto m = vo::match_feature_sets(data[i], data[i + 1], cfg);
        std::vector<cv::Point2d> a, b;
        for (const auto& pr : m.pairs) {
            a.push_back(data[i].points[static_cast<size_t>(pr.first)]);
            b.push_back(data[i + 1].points[static_cast<size_t>(pr.second)]);
        }
        pairs.emplace_back(std::move(a), std::move(b));
    }

    // Только homography
    for (const auto& p : pairs) vo::estimate_motion(p.first, p.second, intr, cfg, 80.0);   // прогрев
    started = vo::now_sec();
    for (int r = 0; r < repeats; ++r)
        for (const auto& p : pairs) vo::estimate_motion(p.first, p.second, intr, cfg, 80.0);
    t.models["homography"] = 1000 * (vo::now_sec() - started) / (repeats * static_cast<double>(pairs.size()));
    t.points = data[0].num_valid;
    t.matches = static_cast<int>(pairs[0].first.size());
    return t;
}

// ---------------------------------------------------------------------------
//  4. СВЕРКА С RTK
// ---------------------------------------------------------------------------
struct ModelTrack {
    std::vector<double> t, x, y;
};

// Траектория модели: время кадров и координаты X, Y. Неудачные шаги
// ('invalid_pose', 'too_few_inliers') положение не меняют - их в треке нет.
std::optional<ModelTrack> load_track(const Paths& P, const std::string& model, const std::vector<double>& t_cam) {
    const std::string path = join(join(join(join(P.results, model), "visual_odometry"), kAlgorithm), "trajectory.csv");
    if (!fs::exists(fs::u8path(path))) return std::nullopt;
    const Csv csv = read_csv(path);
    const int cf = csv.col("Frame"), cx = csv.col("X"), cy = csv.col("Y"), cs = csv.col("Status");
    ModelTrack tr;
    for (const auto& r : csv.rows) {
        if (vo::starts_with(r[cs], "origin") || vo::starts_with(r[cs], "ok")) {
            const size_t frame = static_cast<size_t>(std::atoi(r[cf].c_str())) - 1;
            if (frame >= t_cam.size()) continue;
            tr.t.push_back(t_cam[frame]);
            tr.x.push_back(std::strtod(r[cx].c_str(), nullptr));
            tr.y.push_back(std::strtod(r[cy].c_str(), nullptr));
        }
    }
    return tr;
}

// Совмещение поворотом и сдвигом на окне [i0, i1). Масштаб НЕ подгоняется:
// он и есть проверяемая величина. Поворот нужен, потому что одометрия не
// знает, где север - её оси привязаны к первому кадру.
//
// Глобальный Kabsch по ВСЕМ точкам (старый вариант) прячет накопление
// ошибки: противоположные сносы на разных галсах компенсируются, и кривая
// «ошибки» скачет вверх-вниз. Честный протокол - подогнать SE(2) только
// на первом крейсерском участке и применить то же преобразование ко всему
// треку: тогда дрейф виден как рост ATE со временем.
std::vector<cv::Point2d> align(const std::vector<cv::Point2d>& track, const std::vector<cv::Point2d>& truth,
                               size_t i0, size_t i1) {
    if (track.size() != truth.size() || track.size() < 2)
        return track;
    if (i1 > track.size()) i1 = track.size();
    if (i1 <= i0 + 2) {
        i0 = 0;
        i1 = track.size();
    }
    cv::Point2d ct(0, 0), cg(0, 0);
    const double n = static_cast<double>(i1 - i0);
    for (size_t i = i0; i < i1; ++i) {
        ct += track[i];
        cg += truth[i];
    }
    ct *= 1.0 / n;
    cg *= 1.0 / n;

    cv::Matx22d M = cv::Matx22d::zeros();     // (track - ct)^T (truth - cg)
    for (size_t i = i0; i < i1; ++i) {
        const cv::Point2d a = track[i] - ct, b = truth[i] - cg;
        M(0, 0) += a.x * b.x;
        M(0, 1) += a.x * b.y;
        M(1, 0) += a.y * b.x;
        M(1, 1) += a.y * b.y;
    }
    cv::Mat w, u, vt;
    cv::SVD::compute(cv::Mat(M), w, u, vt);
    cv::Mat d = cv::Mat::eye(2, 2, CV_64F);
    if (cv::determinant(u * vt) < 0) d.at<double>(1, 1) = -1;
    cv::Mat rot = (u * d * vt).t();
    const cv::Matx22d R(rot);
    std::vector<cv::Point2d> out;
    out.reserve(track.size());
    const cv::Vec2d rct = R * cv::Vec2d(ct.x, ct.y);
    for (const auto& p : track) {
        const cv::Vec2d r = R * cv::Vec2d(p.x, p.y);
        out.emplace_back(r[0] + cg.x - rct[0], r[1] + cg.y - rct[1]);
    }
    return out;
}

// Якорь SE(2): первые 10% пути эталона. Первые 50 м здесь — взлёт/зависание,
// курс по ним не определяется. Первый разворот (~105 с) в якорь не входит:
// иначе Kabsch усредняет два курса.
std::pair<size_t, size_t> prefix_window(const std::vector<double>& t,
                                        const std::vector<cv::Point2d>& truth) {
    const size_t n = truth.size();
    if (n < 3) return {0, n};
    std::vector<double> s(n, 0.0);
    for (size_t i = 1; i < n; ++i)
        s[i] = s[i - 1] + std::hypot(truth[i].x - truth[i - 1].x, truth[i].y - truth[i - 1].y);
    const double cap = 0.10 * s.back();
    size_t i1 = n;
    for (size_t i = 0; i < n; ++i)
        if (s[i] > cap) {
            i1 = i;
            break;
        }
    size_t cut = i1;
    for (size_t i = 0; i < i1 && i < t.size(); ++i)
        if (t[i] >= 105.0) {
            cut = i;
            break;
        }
    if (cut >= 20) i1 = cut;
    if (i1 < 3) i1 = std::min<size_t>(n, 20);
    return {0, i1};
}

double rms_of(const std::vector<double>& err) {
    if (err.empty()) return 0.0;
    double sq = 0.0;
    for (double e : err) sq += e * e;
    return std::sqrt(sq / static_cast<double>(err.size()));
}

// RPE: разница векторов сдвига на отрезке L метров эталона, после SE(2).
// Считаем среднее (не RMS): так цифра сравнима с KITTI-style mean RPE.
double rpe_at(const std::vector<cv::Point2d>& fitted, const std::vector<cv::Point2d>& truth,
              double length_m) {
    if (fitted.size() != truth.size() || fitted.size() < 3 || length_m <= 0.0) return 0.0;
    const size_t n = truth.size();
    std::vector<double> s(n, 0.0);
    for (size_t i = 1; i < n; ++i)
        s[i] = s[i - 1] + std::hypot(truth[i].x - truth[i - 1].x, truth[i].y - truth[i - 1].y);
    double sum = 0.0;
    int cnt = 0;
    size_t j = 0;
    for (size_t i = 0; i < n; ++i) {
        while (j < n && s[j] - s[i] < length_m) ++j;
        if (j >= n) break;
        const double dx = (fitted[j].x - fitted[i].x) - (truth[j].x - truth[i].x);
        const double dy = (fitted[j].y - fitted[i].y) - (truth[j].y - truth[i].y);
        sum += std::hypot(dx, dy);
        ++cnt;
    }
    return cnt ? sum / static_cast<double>(cnt) : 0.0;
}

struct Attitude {
    std::vector<double> t, roll, pitch;
    bool ok = false;
};

// Крен и тангаж корпуса, градусы, из кватерниона DJI.
Attitude read_attitude(const Paths& P) {
    Attitude a;
    if (!fs::exists(fs::u8path(P.attitude_csv))) return a;
    try {
        const Csv csv = read_csv(P.attitude_csv);
        const auto tb = csv.numbers("bag_time_ns");
        const auto qx = csv.numbers("quaternion.x"), qy = csv.numbers("quaternion.y"),
                   qz = csv.numbers("quaternion.z"), qw = csv.numbers("quaternion.w");
        for (size_t i = 0; i < tb.size(); ++i) {
            a.t.push_back(tb[i] * 1e-9 - tb[0] * 1e-9);
            const double x = qx[i], y = qy[i], z = qz[i], w = qw[i];
            a.roll.push_back(vo::rad2deg(std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))));
            a.pitch.push_back(vo::rad2deg(std::asin(std::min(1.0, std::max(-1.0, 2 * (w * y - z * x))))));
        }
        a.ok = true;
    } catch (const std::exception&) {
    }
    return a;
}

const std::map<std::string, cv::Scalar>& model_colors() {
    static const std::map<std::string, cv::Scalar> c = {
        {"homography", vo::tab_color(2)}, {"similarity", vo::tab_color(1)}, {"essential", vo::tab_color(3)}};
    return c;
}

void plot_profile(const Paths& P, const Truth& T) {
    std::vector<vo::Axes> ax(4);
    vo::Series h;
    h.x = T.t_rtk;
    h.y = T.height;
    h.color = vo::kBlack;
    ax[0].series.push_back(h);
    ax[0].ylabel = "height, m";
    ax[0].title = "HKairport03: flight conditions and frame selection";
    ax[0].legend = false;

    vo::Series speed;
    for (size_t i = 1; i < T.t_rtk.size(); ++i) {
        speed.x.push_back(T.t_rtk[i - 1]);
        const double dt = T.t_rtk[i] - T.t_rtk[i - 1];
        speed.y.push_back(dt > 0 ? std::hypot(T.east[i] - T.east[i - 1], T.north[i] - T.north[i - 1]) / dt : 0.0);
    }
    speed.color = vo::kBlack;
    ax[1].series.push_back(speed);
    ax[1].ylabel = "speed, m/s";
    ax[1].legend = false;

    const Attitude att = read_attitude(P);
    if (att.ok) {
        vo::Series roll, pitch;
        roll.x = pitch.x = att.t;
        roll.y = att.roll;
        pitch.y = att.pitch;
        roll.label = "roll";
        pitch.label = "pitch";
        roll.color = vo::tab_color(0);
        pitch.color = vo::tab_color(1);
        roll.width = pitch.width = 1;
        ax[2].series = {roll, pitch};
        ax[2].legend_upper_right = true;
        ax[2].title = "";
    }
    ax[2].ylabel = "body tilt, deg";

    const std::map<std::string, std::string> names = {{"homography", "homography"}};
    for (const char* model : {"homography"}) {
        const std::string path = join(join(join(join(P.results, model), "visual_odometry"), kAlgorithm), "trajectory.csv");
        if (!fs::exists(fs::u8path(path))) continue;
        const Csv csv = read_csv(path);
        const int cf = csv.col("Frame"), cs = csv.col("Status"), cst = csv.col("FrameStep");
        vo::Series s;
        s.style = vo::Series::Style::Dots;
        s.width = 2;
        s.color = model_colors().at(model);
        s.label = names.at(model);
        for (const auto& r : csv.rows)
            if (vo::starts_with(r[cs], "ok")) {
                const size_t fr = static_cast<size_t>(std::atoi(r[cf].c_str())) - 1;
                if (fr >= T.t_cam.size()) continue;
                s.x.push_back(T.t_cam[fr]);
                s.y.push_back(std::strtod(r[cst].c_str(), nullptr));
            }
        ax[3].series.push_back(s);
    }
    ax[3].legend_upper_right = true;
    ax[3].ylabel = "selected step, frames";
    ax[3].xlabel = "time, s";

    for (auto& a : ax)
        for (double moment : {60.0, 95.0, 310.0}) a.vlines.push_back({moment, "", vo::kGray});
    vo::save_figure(P.plot_profile, vo::render_figure(ax, 4, 1, {1437, 1265}));
}

struct ModelFit {
    std::vector<cv::Point2d> fitted;
    double rms = 0, ratio = 0;
};

void plot_by_model(const Paths& P, const std::map<std::string, ModelFit>& tracks, const Truth& T) {
    std::vector<std::string> present;
    for (const auto& m : kModels)
        if (tracks.count(m)) present.push_back(m);
    if (present.empty()) return;

    // Одинаковые пределы по всем панелям (равный масштаб обязателен).
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
    auto upd = [&](double x, double y) {
        xmin = std::min(xmin, x); xmax = std::max(xmax, x);
        ymin = std::min(ymin, y); ymax = std::max(ymax, y);
    };
    for (size_t i = 0; i < T.east.size(); ++i) upd(T.east[i], T.north[i]);
    for (const auto& m : present)
        for (const auto& p : tracks.at(m).fitted) upd(p.x, p.y);
    const double px = (xmax - xmin) * 0.05, py = (ymax - ymin) * 0.05;

    const std::map<std::string, std::string> titles = {
        {"similarity", "planar"}, {"homography", "homography"}, {"essential", "essential matrix"}};
    std::vector<vo::Axes> ax;
    for (const auto& m : present) {
        const ModelFit& f = tracks.at(m);
        vo::Axes a;
        vo::Series truth, est, start;
        truth.x = T.east;
        truth.y = T.north;
        truth.color = vo::kBlack;
        truth.width = 2.5;
        truth.label = "RTK";
        est.color = model_colors().at(m);
        est.label = "odometry";
        for (const auto& p : f.fitted) {
            est.x.push_back(p.x);
            est.y.push_back(p.y);
        }
        start.x = {T.east[0]};
        start.y = {T.north[0]};
        start.style = vo::Series::Style::Marker;
        start.color = vo::kBlack;
        a.series = {truth, est, start};
        a.title = fmt("%s | ATE %.0f m, path %.2f of reference", titles.at(m).c_str(), f.rms, f.ratio);
        a.xlabel = "east, m";
        a.ylabel = ax.empty() ? "north, m" : "";
        a.equal = true;
        a.limits = std::array<double, 4>{xmin - px, xmax + px, ymin - py, ymax + py};
        ax.push_back(a);
    }
    const int n = static_cast<int>(present.size());
    vo::save_figure(P.plot_by_model, vo::render_figure(ax, 1, n, {540 * n, 600}));
}

struct Summary {
    double length = 0, ratio = 0, rms = 0, rms_global = 0, max_err = 0, closure = 0;
    double ate_end = 0, rpe10 = 0, rpe50 = 0, rpe100 = 0;
};

std::map<std::string, Summary> compare(const Paths& P, const Truth& T, const Timing& timing,
                                       const std::map<std::string, double>& elapsed) {
    const double truth_path = path_length(T.east, T.north);

    vo::Axes ax_track, ax_err;
    {
        vo::Series truth, start;
        truth.x = T.east;
        truth.y = T.north;
        truth.color = vo::kBlack;
        truth.width = 3;
        truth.label = fmt("RTK (reference), %.0f m", truth_path);
        start.x = {T.east[0]};
        start.y = {T.north[0]};
        start.style = vo::Series::Style::Marker;
        start.color = vo::kBlack;
        ax_track.series = {truth, start};
    }

    const std::map<std::string, std::string> titles = {
        {"homography", "homography"}, {"similarity", "planar"}, {"essential", "essential"}};
    std::map<std::string, ModelFit> tracks;
    std::map<std::string, Summary> summary;
    std::cout << "Выравнивание ATE: SE(2) по первым 10% пути эталона (до первого разворота).\n"
                 "Глобальный Kabsch по всему треку (globRMS) занижает ошибку — это не точность.\n";
    std::cout << "\n"
              << fmt("%-14s %8s %6s %8s %8s %8s %8s %8s %8s %9s %9s", "model", "path,m", "rel.", "ATE,m",
                     "globRMS", "max,m", "end,m", "RPE10", "RPE50", "RPE100", "run,s")
              << "\n"
              << std::string(110, '-') << "\n";
    for (const std::string& model : kModels) {
        if (!fs::is_directory(fs::u8path(join(P.results, model)))) {
            std::cout << fmt("%-14s no run", titles.at(model).c_str()) << "\n";
            continue;
        }
        const auto trk = load_track(P, model, T.t_cam);
        if (!trk || trk->t.size() < 2) {
            std::cout << fmt("%-14s no trajectory", titles.at(model).c_str()) << "\n";
            continue;
        }
        std::vector<cv::Point2d> track, truth;
        const auto te = vo::interp(trk->t, T.t_rtk, T.east), tn = vo::interp(trk->t, T.t_rtk, T.north);
        for (size_t i = 0; i < trk->t.size(); ++i) {
            track.emplace_back(trk->x[i], trk->y[i]);
            truth.emplace_back(te[i], tn[i]);
        }
        const auto win = prefix_window(trk->t, truth);
        const auto fitted = align(track, truth, win.first, win.second);
        const auto fitted_global = align(track, truth, 0, track.size());

        std::vector<double> err(fitted.size()), err_g(fitted_global.size());
        double mx = 0;
        for (size_t i = 0; i < fitted.size(); ++i) {
            err[i] = std::hypot(fitted[i].x - truth[i].x, fitted[i].y - truth[i].y);
            err_g[i] = std::hypot(fitted_global[i].x - truth[i].x, fitted_global[i].y - truth[i].y);
            mx = std::max(mx, err[i]);
        }
        const double rms = rms_of(err);
        const double rms_g = rms_of(err_g);
        const double length = path_length(trk->x, trk->y);
        Summary s;
        s.length = length;
        s.ratio = length / truth_path;
        s.rms = rms;
        s.rms_global = rms_g;
        s.max_err = mx;
        s.closure = std::hypot(trk->x.back() - trk->x.front(), trk->y.back() - trk->y.front());
        s.ate_end = err.empty() ? 0.0 : err.back();
        s.rpe10 = rpe_at(fitted, truth, 10.0);
        s.rpe50 = rpe_at(fitted, truth, 50.0);
        s.rpe100 = rpe_at(fitted, truth, 100.0);
        summary[model] = s;
        tracks[model] = {fitted, rms, s.ratio};
        const auto it = elapsed.find(model);
        std::cout << fmt("%-14s %8.0f %6.2f %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %9.1f %9.0f",
                         titles.at(model).c_str(), length, s.ratio, rms, rms_g, mx, s.ate_end, s.rpe10, s.rpe50,
                         s.rpe100, it == elapsed.end() ? 0.0 : it->second)
                  << "\n";
        std::cout << fmt("               ATE-окно кадров %zu..%zu (t=%.0f..%.0f с), замыкание VO %.1f м\n",
                         win.first, win.second > 0 ? win.second - 1 : 0,
                         trk->t[win.first], trk->t[std::min(win.second, trk->t.size()) - 1], s.closure);

        vo::Series st, se;
        st.color = model_colors().at(model);
        st.width = 1.5;
        st.label = fmt("%s, ATE %.0f m", titles.at(model).c_str(), rms);
        for (const auto& p : fitted) {
            st.x.push_back(p.x);
            st.y.push_back(p.y);
        }
        ax_track.series.push_back(st);
        se.x = trk->t;
        se.y = err;
        se.color = st.color;
        se.label = titles.at(model);
        ax_err.series.push_back(se);
    }

    std::cout << fmt("\nRTK reference: path %.0f m, closure error %.1f m", truth_path,
                     std::hypot(T.east.back() - T.east.front(), T.north.back() - T.north.front()))
              << "\n";

    std::cout << fmt("\nTime per stage (%s, frame %dx%d):", kAlgorithm, kWidth, kHeight) << "\n"
              << fmt("  detection + descriptors   %6.1f ms per frame (%d points)", timing.detect, timing.points) << "\n"
              << fmt("  descriptor matching       %6.1f ms per pair (%d matches)", timing.match, timing.matches) << "\n";
    for (const auto& m : kModels) {
        const auto it = timing.models.find(m);
        if (it != timing.models.end()) std::cout << fmt("  model %-18s %6.1f ms per pair", titles.at(m).c_str(), it->second) << "\n";
    }

    ax_track.title = std::string("HKairport03: VO without GPS/INS correction, ") + kAlgorithm;
    ax_track.xlabel = "east, m";
    ax_track.ylabel = "north, m";
    ax_track.equal = true;
    ax_err.title = "position error vs time (blue: turns)";
    ax_err.xlabel = "time, s";
    ax_err.ylabel = "error, m";
    ax_err.legend_upper_right = true;
    ax_err.vlines = {{60, "end of climb", vo::kGray}, {95, "route start", vo::kGray}, {310, "descent start", vo::kGray}};
    for (const auto& t : kTurns) ax_err.spans.push_back({t.first, t.second, cv::Scalar(230, 160, 90), 0.15});

    vo::save_figure(P.plot_models, vo::render_figure({ax_track, ax_err}, 1, 2, {1550, 700}));
    plot_by_model(P, tracks, T);
    plot_profile(P, T);
    std::cout << "\nГрафики:\n  " << P.plot_models << "\n  " << P.plot_by_model << "\n  " << P.plot_profile << "\n";
    return summary;
}

void print_help() {
    std::cout << "Использование: run_all_hkairport03 [--dataset DIR] [--out DIR] [--lk MODE] [--scale-correction S] [--show-video] [--skip-benchmark] [--skip-models]\n"
                 "  --scale-correction S  geometry.scale_correction (по умолчанию 1.0)\n"
                 "  --lk MODE         off (по умолчанию, только FAST-FREAK) | rotation | only | always\n"
                 "  --dataset DIR     каталог набора (camera/images, camera/timestamps.csv, gps/, other/);\n"
                 "                    по умолчанию текущий каталог\n"
                 "  --out DIR         куда писать results_HKairport03/ и trajectory_plots/ (по умолчанию .)\n"
                 "  --show-video      показывать окно с видеопотоком (включено по умолчанию)\n"
                 "  --skip-benchmark  пропустить замер времени по этапам\n"
                 "  --skip-models     не гонять VO, только сверка уже посчитанной траектории\n";
}

}  // namespace

int main(int argc, char** argv) {
    vo::init_console();
    std::string dataset = ".", out = ".";
    bool show_video = true, skip_benchmark = false, skip_models = false;
    std::string lk_mode = "off";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--dataset" && i + 1 < argc) dataset = argv[++i];
        else if (a == "--out" && i + 1 < argc) out = argv[++i];
        else if (a == "--lk" && i + 1 < argc) lk_mode = argv[++i];
        else if (a == "--scale-correction" && i + 1 < argc)
            g_scale_correction = std::strtod(argv[++i], nullptr);
        else if (a == "--show-video") show_video = true;
        else if (a == "--skip-benchmark") skip_benchmark = true;
        else if (a == "--skip-models") skip_models = true;
        else if (a == "-h" || a == "--help") { print_help(); return 0; }
        else { std::cerr << "Неизвестный аргумент: " << a << "\n"; print_help(); return 2; }
    }

    try {
        const Paths P = make_paths(dataset, out);
        std::cout << "=== HKairport03: homography, связка " << kAlgorithm << " ===\n\n";
        const Truth T = prepare(P);
        if (lk_mode != "rotation" && lk_mode != "only" && lk_mode != "off" && lk_mode != "always")
            throw std::runtime_error("--lk: допустимы rotation, always, only, off");
        const auto elapsed = skip_models ? std::map<std::string, double>{} : run_models(P, show_video, lk_mode);
        if (skip_models) skip_benchmark = true;

        Timing timing;
        for (const auto& m : kModels) timing.models[m] = 0.0;
        if (!skip_benchmark) {
            std::cout << "\n--- замер времени по этапам ---\n";
            timing = benchmark(P);
        }
        compare(P, T, timing, elapsed);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ОШИБКА] " << e.what() << "\n";
        return 1;
    }
}
