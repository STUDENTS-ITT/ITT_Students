#include "vo/preprocess.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "vo/util.hpp"

namespace vo {

cv::Mat to_gray_uint8(const cv::Mat& raw) {
    cv::Mat img = raw;
    if (img.channels() == 4) {
        cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
    } else if (img.channels() == 3) {
        // Те же веса Rec.601, что у MATLAB rgb2gray.
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    if (img.depth() == CV_8U) return img;

    cv::Mat out;
    if (img.depth() == CV_32F || img.depth() == CV_64F) {
        double lo, hi;
        cv::minMaxLoc(img, &lo, &hi);
        if (lo >= 0.0 && hi <= 1.0) {
            img.convertTo(out, CV_8U, 255.0);                      // im2uint8
        } else if (hi > lo) {
            img.convertTo(out, CV_8U, 255.0 / (hi - lo), -lo * 255.0 / (hi - lo));  // mat2gray
        } else {
            out = cv::Mat::zeros(img.size(), CV_8U);
        }
        return out;
    }
    if (img.depth() == CV_16U) {
        img.convertTo(out, CV_8U, 1.0 / 257.0);
        return out;
    }
    cv::normalize(img, out, 0, 255, cv::NORM_MINMAX, CV_8U);
    return out;
}

namespace {

struct OsdGeometry {
    int rows = -1, cols = -1;
    int radius = -1;
    std::vector<cv::Rect2d> rects;
    cv::Mat mask;       // CV_8U, 255 внутри OSD (пусто, если OSD нет)
    cv::Mat edge_band;  // CV_8U, 255 в полосе размытия
    bool valid = false;
};

// Маска OSD и полоса размытия зависят только от размера кадра и настроек,
// поэтому считаются один раз на прогон.
const OsdGeometry& osd_geometry(const cv::Size& size, const PreprocessCfg& pre) {
    static OsdGeometry cache;
    bool same = cache.valid && cache.rows == size.height && cache.cols == size.width &&
                cache.radius == pre.osd_edge_radius && cache.rects.size() == pre.osd_mask.size();
    if (same)
        for (size_t i = 0; i < pre.osd_mask.size(); ++i)
            if (cache.rects[i] != pre.osd_mask[i]) { same = false; break; }
    if (same) return cache;

    cache = OsdGeometry();
    cache.rows = size.height;
    cache.cols = size.width;
    cache.radius = pre.osd_edge_radius;
    cache.rects = pre.osd_mask;
    cache.valid = true;
    if (pre.osd_mask.empty()) return cache;

    cv::Mat mask = cv::Mat::zeros(size, CV_8U);
    bool any = false;
    for (const cv::Rect2d& r : pre.osd_mask) {
        const int x0 = std::max(0, static_cast<int>(std::floor(r.x)));
        const int y0 = std::max(0, static_cast<int>(std::floor(r.y)));
        const int x1 = std::min(size.width, x0 + static_cast<int>(std::ceil(r.width)) + 1);
        const int y1 = std::min(size.height, y0 + static_cast<int>(std::ceil(r.height)) + 1);
        if (x1 > x0 && y1 > y0) {
            mask(cv::Range(y0, y1), cv::Range(x0, x1)).setTo(255);
            any = true;
        }
    }
    if (!any) return cache;
    cache.mask = mask;

    if (pre.osd_blur_sigma > 0 && pre.osd_edge_radius > 0) {
        const int k = 2 * pre.osd_edge_radius + 1;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        cv::Mat dil, ero;
        cv::dilate(mask, dil, kernel);
        cv::erode(mask, ero, kernel);
        cache.edge_band = (dil > 0) & (ero == 0);
    }
    return cache;
}

}  // namespace

cv::Mat blank_osd(const cv::Mat& image, const Config& cfg) {
    const OsdGeometry& g = osd_geometry(image.size(), cfg.preprocess);
    if (g.mask.empty()) return image;

    cv::Mat out = image.clone();
    const uchar fill = static_cast<uchar>(std::lround(cv::mean(image)[0]));
    out.setTo(fill, g.mask);

    if (!g.edge_band.empty()) {
        cv::Mat blurred;
        cv::GaussianBlur(out, blurred, cv::Size(0, 0), cfg.preprocess.osd_blur_sigma);
        blurred.copyTo(out, g.edge_band);
    }
    return out;
}

cv::Mat preprocess_frame(const cv::Mat& raw, const Calibration& calib, const Config& cfg) {
    cv::Mat img = to_gray_uint8(raw);
    const PreprocessCfg& pre = cfg.preprocess;

    if (pre.undistort && calib.has_distortion()) {
        try {
            const cv::Matx33d K = calib.intrinsics.K();
            cv::Mat out;
            cv::undistort(img, out, K, calib.intrinsics.distortion, K);
            img = out;
        } catch (const cv::Exception& e) {
            std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Не удалось исправить дисторсию: " << e.what() << "\n";
        }
    }

    if (pre.use_clahe) {
        auto clahe = cv::createCLAHE(pre.clahe_clip_limit,
                                     cv::Size(pre.clahe_tile_grid, pre.clahe_tile_grid));
        cv::Mat out;
        clahe->apply(img, out);
        img = out;
    }
    return blank_osd(img, cfg);
}

}  // namespace vo
