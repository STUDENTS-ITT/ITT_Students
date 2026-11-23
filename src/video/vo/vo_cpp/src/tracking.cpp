#include "vo/tracking.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace vo {

LkTracks track_points_lk(const cv::Mat& prev, const cv::Mat& curr,
                         const std::vector<cv::Point2f>& seeds, const LkCfg& cfg,
                         const cv::Mat& H_guess) {
    LkTracks out;
    if (seeds.empty() || prev.empty() || curr.empty()) return out;

    const cv::Size win(cfg.window, cfg.window);
    const cv::TermCriteria term(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, cfg.max_iter, cfg.epsilon);

    std::vector<cv::Point2f> fwd;
    int flags = 0;
    if (cfg.use_initial_flow && !H_guess.empty()) {
        cv::perspectiveTransform(seeds, fwd, H_guess);
        flags = cv::OPTFLOW_USE_INITIAL_FLOW;
    }
    std::vector<uchar> st_f, st_b;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev, curr, seeds, fwd, st_f, err, win, cfg.levels, term, flags, cfg.min_eig);

    // Обратный проход стартует с исходных точек: так он не повторяет ошибку прямого.
    std::vector<cv::Point2f> back = seeds;
    cv::calcOpticalFlowPyrLK(curr, prev, fwd, back, st_b, err, win, cfg.levels, term,
                             cv::OPTFLOW_USE_INITIAL_FLOW, cfg.min_eig);

    const cv::Rect2f bounds(0.f, 0.f, static_cast<float>(curr.cols - 1), static_cast<float>(curr.rows - 1));
    for (size_t i = 0; i < seeds.size(); ++i) {
        if (!st_f[i] || !st_b[i] || !bounds.contains(fwd[i])) continue;
        const float dx = back[i].x - seeds[i].x, dy = back[i].y - seeds[i].y;
        if (std::hypot(dx, dy) > cfg.fb_threshold_px) continue;
        out.index.push_back(static_cast<int>(i));
        out.p1.push_back(seeds[i]);
        out.p2.push_back(fwd[i]);
    }
    return out;
}

}  // namespace vo
