// Особые точки: связки «детектор-дескриптор», отбор и описание точек
// (порт разделов 6-7 vo_final2.py).
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "vo/config.hpp"

namespace vo {

class RotationCrop;

struct Algorithm {
    std::string name;        // "FAST-FREAK"
    std::string detector;    // "FAST"
    std::string descriptor;  // "FREAK"
};

// FAST-FREAK (единственная поддерживаемая связка).
const std::vector<Algorithm>& algorithm_candidates();
bool is_binary_descriptor(const std::string& descriptor);

// Имена связок -> канонические (строго полные имена). Пустой список = «все».
// Бросает std::runtime_error на неизвестное имя.
std::vector<std::string> parse_algorithms(const std::vector<std::string>& values);
// То же для строки через запятую/точку с запятой.
std::vector<std::string> parse_algorithms(const std::string& value);

// Реально создаёт детектор и дескриптор и прогоняет их на тестовом кадре.
// Возвращает (доступна, причина_отказа).
std::pair<bool, std::string> probe_algorithm(const Algorithm& alg);
std::vector<Algorithm> available_algorithms();
std::vector<std::pair<std::string, std::string>> missing_algorithms();
// Связки, заказанные в cfg.vo.algorithms (пусто = все доступные).
std::vector<Algorithm> select_algorithms(const Config& cfg);
// Первая связка из списка либо указанная явно.
Algorithm pick_algorithm(const Config& cfg, const std::string& algorithm = "");
// Печать сводки о доступных связках (ключ --list).
void check_environment(const Config* cfg, bool verbose = true);

// Детектор и дескриптор одной связки, созданные один раз.
class AlgorithmRunner {
public:
    AlgorithmRunner(const Algorithm& alg, const Config& cfg);
    ~AlgorithmRunner();
    AlgorithmRunner(AlgorithmRunner&&) noexcept;
    AlgorithmRunner& operator=(AlgorithmRunner&&) noexcept;

    const Algorithm& algorithm() const { return alg_; }
    bool is_binary() const { return binary_; }

    std::vector<cv::KeyPoint> detect(const cv::Mat& gray);
    // Может удалить точки у границы кадра.
    void compute(const cv::Mat& gray, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

private:
    struct Impl;
    Algorithm alg_;
    bool binary_ = false;
    std::unique_ptr<Impl> impl_;
};

struct FeatureData {
    std::vector<cv::Point2f> points;   // координаты OpenCV, 0-based
    cv::Mat features;                  // по строке на точку
    bool is_binary = false;
    int num_detected = 0;
    int num_valid = 0;
    int num_rejected_osd = 0;
    int num_rejected_crop = 0;
    int num_rejected_shadow = 0;
    double time_sec = 0.0;
    std::string message = "ok";
};

// Маска теней, CV_8U того же размера, что кадр (255 — тень). Пустая, если
// пятен нужной площади нет или высота выше порога.
cv::Mat make_shadow_mask(const cv::Mat& gray, double altitude_m, double fx, const ShadowCfg& cfg);

// Порт detectAndDescribe.m: детектирование -> отбраковка OSD, тени и зоны
// обрезки -> равномерный отбор -> дескрипторы. crop может быть nullptr.
// shadow_mask пустая — точки по тени не режутся.
FeatureData detect_and_describe(const cv::Mat& image, AlgorithmRunner& runner,
                                const Config& cfg, const RotationCrop* crop,
                                const cv::Mat& shadow_mask = cv::Mat());

// N точек с наибольшим откликом.
std::vector<cv::KeyPoint> select_strongest(const std::vector<cv::KeyPoint>& kps, size_t n);
// selectUniform из FeaturePointsImpl.m: сетка ~N ячеек, из каждой сильнейшая
// точка; при нехватке проход повторяется без уже отобранных.
std::vector<cv::KeyPoint> select_uniform(const std::vector<cv::KeyPoint>& kps, size_t n,
                                         const cv::Size& image_size);

}  // namespace vo
