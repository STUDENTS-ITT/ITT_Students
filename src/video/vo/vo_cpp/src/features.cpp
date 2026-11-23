#include "vo/features.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_set>

#include <opencv2/imgproc.hpp>

#if __has_include(<opencv2/features.hpp>)
#  include <opencv2/features.hpp>     // OpenCV 5.x
#else
#  include <opencv2/features2d.hpp>   // OpenCV 4.x
#endif

#if VO_HAVE_XFEATURES2D
#  include <opencv2/xfeatures2d.hpp>
#  if __has_include(<opencv2/xfeatures2d/nonfree.hpp>)
#    include <opencv2/xfeatures2d/nonfree.hpp>
#    define VO_HAVE_SURF 1
#  endif
#endif
#ifndef VO_HAVE_SURF
#  define VO_HAVE_SURF 0
#endif

#include "vo/selection.hpp"
#include "vo/util.hpp"

namespace vo {

// ---------------------------------------------------------------------------
// Список связок
// ---------------------------------------------------------------------------

const std::vector<Algorithm>& algorithm_candidates() {
    static const std::vector<Algorithm> list = {
        {"FAST-FREAK", "FAST", "FREAK"},
    };
    return list;
}

bool is_binary_descriptor(const std::string& d) {
    const std::string u = upper(d);
    return u == "FREAK" || u == "BRISK" || u == "ORB";
}

std::vector<std::string> parse_algorithms(const std::vector<std::string>& values) {
    std::vector<std::string> out;
    for (const std::string& raw : values) {
        const std::string key = lower(trim(raw));
        if (key.empty() || key == "none" || key == "all") continue;
        const Algorithm* found = nullptr;
        for (const Algorithm& a : algorithm_candidates())
            if (lower(a.name) == key) found = &a;
        if (!found) {
            std::string known;
            for (const Algorithm& a : algorithm_candidates())
                known += (known.empty() ? "" : ", ") + a.name;
            throw std::runtime_error("Неизвестная связка \"" + trim(raw) +
                                     "\" в algorithms. Имя пишется целиком, допустимы: " +
                                     known + ".");
        }
        if (std::find(out.begin(), out.end(), found->name) == out.end()) out.push_back(found->name);
    }
    return out;
}

std::vector<std::string> parse_algorithms(const std::string& value) {
    std::string s = value;
    std::replace(s.begin(), s.end(), ';', ',');
    return parse_algorithms(split(s, ','));
}

// ---------------------------------------------------------------------------
// Создание детекторов и дескрипторов
// ---------------------------------------------------------------------------

namespace {

cv::Ptr<cv::Feature2D> make_feature(const std::string& name_in, const FeaturesCfg& f) {
    const std::string name = upper(name_in);
    // FREAK — единственный поддерживаемый дескриптор, требует xfeatures2d.
    if (name == "FREAK") {
#if CV_VERSION_MAJOR >= 5
#  if VO_HAVE_XFEATURES2D
        return cv::xfeatures2d::FREAK::create();
#  endif
#else
#  if VO_HAVE_XFEATURES2D
        return cv::xfeatures2d::FREAK::create();
#  endif
#endif
    }
    return {};
}

}  // namespace

struct AlgorithmRunner::Impl {
    FeaturesCfg f;
    cv::Ptr<cv::Feature2D> desc_impl;  // только FREAK
    cv::Ptr<cv::FastFeatureDetector> fast;

    // FAST детектор с двумя порогами (как в MATLAB): MinContrast (абсолютный)
    // и MinQuality (относительный от максимума отклика).
    std::vector<cv::KeyPoint> detect_fast(const cv::Mat& gray) {
        if (!fast) {
            // FeaturesCfg имеет fast_min_contrast [0..1], переводим в [0..255]
            const int thr = std::max(1, static_cast<int>(std::lround(f.fast_min_contrast * 255)));
            fast = cv::FastFeatureDetector::create(thr, true);
        }
        std::vector<cv::KeyPoint> kps, out;
        fast->detect(gray, kps);
        if (kps.empty()) return out;
        float mx = 0;
        for (const auto& k : kps) mx = std::max(mx, k.response);
        const double threshold = f.fast_min_quality * mx;
        for (const auto& k : kps)
            if (k.response >= threshold) out.push_back(k);
        return out;
    }
};

AlgorithmRunner::AlgorithmRunner(const Algorithm& alg, const Config& cfg)
    : alg_(alg), impl_(new Impl) {
    impl_->f = cfg.features;
    binary_ = is_binary_descriptor(alg.descriptor);

    // Детектор: только FAST
    if (upper(alg.detector) != "FAST") {
        throw std::runtime_error("Детектор " + alg.detector + " не поддерживается (только FAST)");
    }
    // Дескриптор: только FREAK, требуется xfeatures2d
    impl_->desc_impl = make_feature("FREAK", cfg.features);
    if (!impl_->desc_impl)
        throw std::runtime_error("Дескриптор FREAK отсутствует (требуется OpenCV с xfeatures2d)");
}

AlgorithmRunner::~AlgorithmRunner() = default;
AlgorithmRunner::AlgorithmRunner(AlgorithmRunner&&) noexcept = default;
AlgorithmRunner& AlgorithmRunner::operator=(AlgorithmRunner&&) noexcept = default;

std::vector<cv::KeyPoint> AlgorithmRunner::detect(const cv::Mat& gray) {
    // Только FAST
    return impl_->detect_fast(gray);
}

void AlgorithmRunner::compute(const cv::Mat& gray, std::vector<cv::KeyPoint>& kps,
                              cv::Mat& descriptors) {
    // Только FREAK
    impl_->desc_impl->compute(gray, kps, descriptors);
}

// ---------------------------------------------------------------------------
// Проверка доступности связок
// ---------------------------------------------------------------------------

namespace {

cv::Mat probe_image() {
    cv::Mat img = cv::Mat::zeros(128, 128, CV_8U);
    for (int y = 0; y < 128; y += 16) img.row(y).setTo(255);
    for (int x = 0; x < 128; x += 16) img.col(x).setTo(255);
    for (int i = 0; i < 128; i += 32)
        for (int j = 0; j < 128; j += 32) {
            cv::rectangle(img, {i + 4, j + 4}, {i + 20, j + 20}, 200, cv::FILLED);
            cv::circle(img, {i + 12, j + 12}, 5, 40, cv::FILLED);
        }
    cv::GaussianBlur(img, img, {3, 3}, 0);
    return img;
}

std::string short_reason(const std::string& text) {
    if (text.find("patented") != std::string::npos || text.find("NONFREE") != std::string::npos)
        return "запатентован и отключён в этой сборке OpenCV";
    if (lower(text).find("not implemented") != std::string::npos)
        return "не реализован в этой сборке OpenCV";
    std::string tail = text.substr(text.find_last_of('\n') == std::string::npos ? 0 : text.find_last_of('\n') + 1);
    if (tail.size() > 110) tail.resize(110);
    return tail;
}

}  // namespace

std::pair<bool, std::string> probe_algorithm(const Algorithm& alg) {
    static std::vector<std::pair<std::string, std::pair<bool, std::string>>> cache;
    for (const auto& c : cache)
        if (c.first == alg.name) return c.second;

    std::pair<bool, std::string> result{true, ""};
    Config cfg = build_config();
    cfg.preprocess.osd_mask.clear();
    cfg.features.max_features = 200;
    try {
        AlgorithmRunner runner(alg, cfg);
        cv::Mat img = probe_image();
        std::vector<cv::KeyPoint> kps = runner.detect(img);
        if (kps.empty()) {
            result = {false, "детектор " + alg.detector + " не нашёл точек на тестовом кадре"};
        } else {
            // Берём сильнейшие, а не первые по строкам: те лежат у границы кадра,
            // где FREAK/BRISK отбрасывают точки.
            std::sort(kps.begin(), kps.end(),
                      [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
            if (kps.size() > 100) kps.resize(100);
            cv::Mat desc;
            runner.compute(img, kps, desc);
            if (desc.empty() || kps.empty())
                result = {false, "дескриптор " + alg.descriptor + " не вычисляется"};
        }
    } catch (const cv::Exception& e) {
        result = {false, short_reason(e.what())};
    } catch (const std::exception& e) {
        result = {false, short_reason(e.what())};
    }
    cache.emplace_back(alg.name, result);
    return result;
}

std::vector<Algorithm> available_algorithms() {
    std::vector<Algorithm> out;
    for (const Algorithm& a : algorithm_candidates())
        if (probe_algorithm(a).first) out.push_back(a);
    return out;
}

std::vector<std::pair<std::string, std::string>> missing_algorithms() {
    std::vector<std::pair<std::string, std::string>> out;
    for (const Algorithm& a : algorithm_candidates()) {
        auto r = probe_algorithm(a);
        if (!r.first) out.emplace_back(a.name, r.second);
    }
    return out;
}

std::vector<Algorithm> select_algorithms(const Config& cfg) {
    const std::vector<std::string> wanted = parse_algorithms(cfg.vo.algorithms);
    if (wanted.empty()) return available_algorithms();

    std::vector<Algorithm> chosen;
    std::vector<std::string> unknown;
    for (const std::string& name : wanted) {
        const Algorithm* item = nullptr;
        for (const Algorithm& a : algorithm_candidates())
            if (a.name == name) item = &a;
        if (item && probe_algorithm(*item).first) chosen.push_back(*item);
        else unknown.push_back(name);
    }
    if (!unknown.empty()) {
        std::string have, bad, known;
        for (const Algorithm& a : available_algorithms()) have += (have.empty() ? "" : ", ") + a.name;
        for (const auto& u : unknown) bad += (bad.empty() ? "" : ", ") + u;
        for (const Algorithm& a : algorithm_candidates()) known += (known.empty() ? "" : ", ") + a.name;
        throw std::runtime_error("Недоступны в этой сборке OpenCV: " + bad + ". Доступны: " +
                                 (have.empty() ? "(нет ни одного)" : have) +
                                 ". Всего в списке проекта: " + known +
                                 ". Часть требует OpenCV, собранный с opencv_contrib.");
    }
    return chosen;
}

Algorithm pick_algorithm(const Config& cfg, const std::string& algorithm) {
    if (!algorithm.empty()) {
        const auto names = parse_algorithms(algorithm);
        if (names.empty()) throw std::runtime_error("Не указана связка детектор-дескриптор.");
        for (const Algorithm& a : available_algorithms())
            if (lower(a.name) == lower(names[0])) return a;
        std::string have;
        for (const Algorithm& a : available_algorithms()) have += (have.empty() ? "" : ", ") + a.name;
        throw std::runtime_error("Связка \"" + names[0] + "\" недоступна в этой сборке. Доступны: " + have);
    }
    const auto chosen = select_algorithms(cfg);
    if (chosen.empty())
        throw std::runtime_error(
            "Не выбрано ни одной связки детектор-дескриптор. Проверьте algorithms и то, "
            "что OpenCV собран с нужными модулями (opencv_contrib).");
    return chosen.front();
}

void check_environment(const Config* cfg, bool verbose) {
    if (!verbose) return;
    std::cout << "OpenCV " << CV_VERSION << "\n";
    std::cout << "Доступные связки детектор-дескриптор:\n";
    for (const Algorithm& a : available_algorithms()) {
        std::cout << "  " << a.name << "  (" << (is_binary_descriptor(a.descriptor) ? "бинарный, Hamming" : "float, SSD")
                  << ")\n";
    }
    const auto missing = missing_algorithms();
    if (!missing.empty()) {
        std::cout << "Недоступны:\n";
        for (const auto& m : missing) std::cout << "  " << m.first << " - " << m.second << "\n";
    }
    if (cfg) {
        try {
            const auto chosen = select_algorithms(*cfg);
            (void)chosen;
        } catch (const std::exception& e) {
            std::cout << "[ПРЕДУПРЕЖДЕНИЕ] " << e.what() << "\n";
        }
    }
}

// ---------------------------------------------------------------------------
// Отбор точек
// ---------------------------------------------------------------------------

std::vector<cv::KeyPoint> select_strongest(const std::vector<cv::KeyPoint>& kps, size_t n) {
    if (kps.size() <= n) return kps;
    std::vector<cv::KeyPoint> out = kps;
    std::stable_sort(out.begin(), out.end(),
                     [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
    out.resize(n);
    return out;
}

std::vector<cv::KeyPoint> select_uniform(const std::vector<cv::KeyPoint>& kps, size_t n,
                                         const cv::Size& image_size) {
    if (kps.size() <= n) return kps;

    // MATLAB: imageSize = [ширина, высота]; координаты начинаются с 1.
    const double W = image_size.width, H = image_size.height;

    std::vector<size_t> order(kps.size());
    std::iota(order.begin(), order.end(), size_t{0});
    std::stable_sort(order.begin(), order.end(),
                     [&](size_t a, size_t b) { return kps[a].response > kps[b].response; });

    std::vector<size_t> alive = order;   // индексы в kps, по убыванию метрики
    std::vector<size_t> selected;
    std::vector<size_t> next_alive;
    while (selected.size() < n && !alive.empty()) {
        const size_t need = n - selected.size();
        // _grid_cells: ядро selectPoints из FeaturePointsImpl.m
        const double aspect = W / H;
        const long hh = std::max<long>(static_cast<long>(std::floor(std::sqrt(need / aspect))), 1);
        const long ww = std::max<long>(static_cast<long>(std::floor(hh * aspect)), 1);
        const double step_x = W / (ww + 1), step_y = H / (hh + 1);

        std::unordered_set<long long> seen;
        next_alive.clear();
        size_t taken = 0;
        for (size_t idx : alive) {
            const double x = kps[idx].pt.x + 1.0, y = kps[idx].pt.y + 1.0;
            const long wx = std::min<long>(static_cast<long>(std::floor(x / step_x)) + 1, ww);
            const long wy = std::min<long>(static_cast<long>(std::floor(y / step_y)) + 1, hh);
            const long long flat = static_cast<long long>(wx - 1) * hh + (wy - 1);
            if (seen.insert(flat).second) {
                selected.push_back(idx);
                ++taken;
            } else {
                next_alive.push_back(idx);
            }
        }
        if (taken == 0) break;
        alive.swap(next_alive);
    }

    std::vector<cv::KeyPoint> out;
    const size_t m = std::min(selected.size(), n);
    out.reserve(m);
    for (size_t i = 0; i < m; ++i) out.push_back(kps[selected[i]]);
    return out;
}

namespace {

// Удаление точек в областях OSD (порт rejectOsdPoints.m): неподвижная
// относительно камеры графика даёт «идеальные» соответствия с нулевым
// смещением, и оценка движения смещается к нулю.
int reject_osd_points(std::vector<cv::KeyPoint>& kps, const Config& cfg) {
    const auto& rects = cfg.preprocess.osd_mask;
    if (rects.empty() || kps.empty()) return 0;
    const double margin = cfg.preprocess.osd_margin;
    std::vector<cv::KeyPoint> kept;
    kept.reserve(kps.size());
    for (const auto& k : kps) {
        bool inside = false;
        for (const auto& r : rects)
            inside = inside || (k.pt.x >= r.x - margin && k.pt.x <= r.x + r.width + margin &&
                                k.pt.y >= r.y - margin && k.pt.y <= r.y + r.height + margin);
        if (!inside) kept.push_back(k);
    }
    const int rejected = static_cast<int>(kps.size() - kept.size());
    kps.swap(kept);
    return rejected;
}

// Убрать точки за границами используемой части кадра. Отбор идёт ПОСЛЕ
// детектирования, а не маской внутри детектора: так приём работает одинаково
// для всех связок, включая собственные реализации Harris и FAST.
int reject_shadow_points(std::vector<cv::KeyPoint>& kps, const cv::Mat& mask) {
    if (mask.empty() || kps.empty()) return 0;
    std::vector<cv::KeyPoint> kept;
    kept.reserve(kps.size());
    for (const auto& k : kps) {
        const int x = cvRound(k.pt.x), y = cvRound(k.pt.y);
        if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows || mask.at<uchar>(y, x) == 0)
            kept.push_back(k);
    }
    const int rejected = static_cast<int>(kps.size() - kept.size());
    kps.swap(kept);
    return rejected;
}

int reject_cropped_points(std::vector<cv::KeyPoint>& kps, const RotationCrop* crop,
                          const cv::Size& size) {
    if (!crop || !crop->enabled() || kps.empty()) return 0;
    const auto c = crop->current_cut();
    if (c[0] <= 0 && c[1] <= 0 && c[2] <= 0 && c[3] <= 0) return 0;
    const auto b = crop->bounds(size);
    std::vector<cv::KeyPoint> kept;
    kept.reserve(kps.size());
    for (const auto& k : kps)
        if (k.pt.x >= b[0] && k.pt.x <= b[1] && k.pt.y >= b[2] && k.pt.y <= b[3]) kept.push_back(k);
    const int rejected = static_cast<int>(kps.size() - kept.size());
    kps.swap(kept);
    return rejected;
}

}  // namespace

cv::Mat make_shadow_mask(const cv::Mat& gray, double altitude_m, double fx, const ShadowCfg& cfg) {
    if (!cfg.enable || gray.empty() || gray.type() != CV_8U || fx <= 1e-9) return {};
    if (altitude_m > cfg.max_altitude_m) return {};

    const double r_full = std::max(12.0, 0.5 * fx * cfg.aircraft_span_m / std::max(altitude_m, 0.5));
    // На малой высоте тень больше кадра: держим радиус на рабочей картинке
    // около 40 px, иначе размытие с ядром в сотни пикселей не заканчивается.
    const double scale = std::min(1.0, std::min(640.0 / static_cast<double>(std::max(gray.cols, gray.rows)),
                                                40.0 / r_full));
    cv::Mat small;
    if (scale < 1.0) cv::resize(gray, small, cv::Size(), scale, scale, cv::INTER_AREA);
    else small = gray;

    const double r = r_full * scale;
    cv::Mat blur;
    cv::GaussianBlur(small, blur, cv::Size(), std::max(4.0, r));

    cv::Mat g32, b32;
    small.convertTo(g32, CV_32F);
    blur.convertTo(b32, CV_32F);
    cv::Mat mask = (g32 / (b32 + 1.0f) < 0.72f) & (g32 + 18.0f < b32);
    cv::Mat dark;
    mask.convertTo(dark, CV_8U, 255);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    cv::morphologyEx(dark, dark, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, kernel);

    cv::Mat labels, stats, cent;
    const int n = cv::connectedComponentsWithStats(dark, labels, stats, cent, 8);
    const double expect = 3.141592653589793 * r * r;
    cv::Mat selected = cv::Mat::zeros(small.size(), CV_8U);
    for (int i = 1; i < n; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < expect * 0.15 || area > expect * 8.0) continue;
        selected.setTo(255, labels == i);
    }
    if (cv::countNonZero(selected) == 0) return {};

    if (selected.size() == gray.size()) return selected;
    cv::Mat full;
    cv::resize(selected, full, gray.size(), 0, 0, cv::INTER_NEAREST);
    return full;
}

FeatureData detect_and_describe(const cv::Mat& image, AlgorithmRunner& runner, const Config& cfg,
                                const RotationCrop* crop, const cv::Mat& shadow_mask) {
    FeatureData data;
    data.is_binary = runner.is_binary();
    const double t0 = now_sec();
    auto finish = [&](const char* msg) {
        data.message = msg;
        data.time_sec = now_sec() - t0;
        return data;
    };

    try {
        std::vector<cv::KeyPoint> kps = runner.detect(image);
        data.num_detected = static_cast<int>(kps.size());
        if (kps.empty()) return finish("no_points");

        data.num_rejected_osd = reject_osd_points(kps, cfg);
        if (kps.empty()) return finish("no_points_outside_osd");

        data.num_rejected_shadow = reject_shadow_points(kps, shadow_mask);
        if (kps.empty()) return finish("no_points_outside_shadow");

        data.num_rejected_crop = reject_cropped_points(kps, crop, image.size());
        if (kps.empty()) return finish("no_points_inside_crop");

        const size_t max_features =
            std::min(static_cast<size_t>(std::max(cfg.features.max_features, 0)), kps.size());
        kps = cfg.features.select_uniform ? select_uniform(kps, max_features, image.size())
                                          : select_strongest(kps, max_features);

        cv::Mat desc;
        runner.compute(image, kps, desc);
        if (desc.empty() || kps.empty()) return finish("no_descriptors");

        data.points.reserve(kps.size());
        for (const auto& k : kps) data.points.push_back(k.pt);
        data.features = desc;
        data.num_valid = static_cast<int>(kps.size());
    } catch (const cv::Exception& e) {
        data.message = std::string("error: ") + e.what();
    } catch (const std::exception& e) {
        data.message = std::string("error: ") + e.what();
    }
    data.time_sec = now_sec() - t0;
    return data;
}

}  // namespace vo
