#include "vo/matching.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#if __has_include(<opencv2/features.hpp>)
#  include <opencv2/features.hpp>
#else
#  include <opencv2/features2d.hpp>
#endif

#include "vo/util.hpp"

// Порог MatchThreshold в MATLAB задаётся В ПРОЦЕНТАХ от максимально возможного
// расстояния, а не в абсолютных единицах:
//   * бинарные (Hamming): максимум = число бит, порог = pct/100 * numBits;
//   * вещественные (SSD, векторы нормируются): максимум ||a-b||^2 = 4.
// Ratio test применяется к той же метрике, что и порог; для SSD это отношение
// КВАДРАТОВ расстояний (MaxRatio 0.75 по квадратам ~ 0.87 по обычным).

namespace vo {

namespace {

cv::Mat normalize_rows(const cv::Mat& f) {
    cv::Mat out;
    f.convertTo(out, CV_32F);
    for (int i = 0; i < out.rows; ++i) {
        cv::Mat row = out.row(i);
        const double n = std::max(cv::norm(row), 1e-12);
        row *= 1.0 / n;
    }
    return out;
}

// Дескрипторы в виде, пригодном для BFMatcher, и максимум расстояния.
cv::Mat prepare(const cv::Mat& features, bool is_binary, const Config& cfg, double& max_dist) {
    if (is_binary) {
        max_dist = 8.0 * features.cols;
        return features;
    }
    cv::Mat f;
    if (cfg.matching.normalize_float_features) {
        f = normalize_rows(features);
        max_dist = cfg.matching.squared_float_metric ? 4.0 : 2.0;
    } else {
        features.convertTo(f, CV_32F);
        max_dist = std::numeric_limits<double>::infinity();
    }
    return f;
}

double to_metric(double d, bool is_binary, const Config& cfg) {
    if (is_binary) return d;
    return cfg.matching.squared_float_metric ? d * d : d;
}

}  // namespace

MatchResult match_feature_sets(const FeatureData& d1, const FeatureData& d2, const Config& cfg) {
    MatchResult result;
    if (d1.num_valid == 0 || d2.num_valid == 0) {
        result.message = "empty_features";
        return result;
    }
    const double t0 = now_sec();
    try {
        const bool is_binary = d1.is_binary;
        double max_dist = 0, unused = 0;
        cv::Mat f1 = prepare(d1.features, is_binary, cfg, max_dist);
        cv::Mat f2 = prepare(d2.features, is_binary, cfg, unused);

        cv::BFMatcher matcher(is_binary ? cv::NORM_HAMMING : cv::NORM_L2, false);
        const int n1 = f1.rows, n2 = f2.rows;

        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(f1, f2, knn, std::min(2, n2));

        const double inf = std::numeric_limits<double>::infinity();
        std::vector<int> best(n1, 0);
        std::vector<double> best_m(n1, inf), second_m(n1, inf);
        for (int i = 0; i < n1; ++i) {
            const auto& pair = knn[i];
            if (pair.empty()) continue;
            best[i] = pair[0].trainIdx;
            best_m[i] = to_metric(pair[0].distance, is_binary, cfg);
            if (pair.size() > 1) second_m[i] = to_metric(pair[1].distance, is_binary, cfg);
        }

        // --- абсолютный порог (MatchThreshold, % от максимума) ---
        const double pct = is_binary ? cfg.matching.binary_threshold : cfg.matching.float_threshold;
        double abs_threshold = pct / 100.0 * max_dist;
        if (is_binary) abs_threshold = std::nearbyint(abs_threshold);   // percentToLevel.m

        std::vector<char> keep(n1, 0);
        for (int i = 0; i < n1; ++i) keep[i] = std::isfinite(best_m[i]) && best_m[i] <= abs_threshold;

        // --- ratio test (MaxRatio) ---
        const double max_ratio = cfg.matching.max_ratio;
        for (int i = 0; i < n1; ++i) {
            double ratio;
            if (!std::isfinite(second_m[i])) ratio = 0.0;          // единственный кандидат
            else if (second_m[i] < 1e-6) ratio = 1.0;              // findUnambiguousMatches.m
            else ratio = best_m[i] / second_m[i];
            keep[i] = keep[i] && ratio <= max_ratio;
        }

        // --- взаимная уникальность (Unique) ---
        if (cfg.matching.unique) {
            std::vector<cv::DMatch> reverse;
            matcher.match(f2, f1, reverse);
            std::vector<int> reverse_best(n2, -1);
            for (const auto& m : reverse) reverse_best[m.queryIdx] = m.trainIdx;
            for (int i = 0; i < n1; ++i) keep[i] = keep[i] && reverse_best[best[i]] == i;
        }

        for (int i = 0; i < n1; ++i)
            if (keep[i]) {
                result.pairs.emplace_back(i, best[i]);
                result.metric.push_back(best_m[i]);
            }
        if (!result.metric.empty()) result.median_metric = median(result.metric);
    } catch (const cv::Exception& e) {
        result.message = std::string("error: ") + e.what();
    } catch (const std::exception& e) {
        result.message = std::string("error: ") + e.what();
    }
    result.time_sec = now_sec() - t0;
    return result;
}

}  // namespace vo
