// Сопоставление дескрипторов (порт matchFeatureSets.m): ближайший сосед,
// абсолютный порог в % от максимума, ratio test, взаимная уникальность.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "vo/config.hpp"
#include "vo/features.hpp"

namespace vo {

struct MatchResult {
    std::vector<std::pair<int, int>> pairs;   // (индекс в data1, индекс в data2)
    std::vector<double> metric;               // расстояние каждой пары
    double median_metric = std::nan("");
    double time_sec = 0.0;
    std::string message = "ok";
};

MatchResult match_feature_sets(const FeatureData& data1, const FeatureData& data2,
                               const Config& cfg);

}  // namespace vo
