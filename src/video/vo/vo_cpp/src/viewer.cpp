#include "vo/viewer.hpp"

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "vo/pipeline.hpp"
#include "vo/plot.hpp"
#include "vo/util.hpp"

namespace vo {

namespace {
constexpr double kDim = 0.35;   // во сколько раз гасится область за пределами обрезки
}

FrameViewer::FrameViewer(const Config& cfg) {
    enabled_ = cfg.vis.show_video;
    width_ = cfg.vis.video_width;
    draw_points_ = cfg.vis.video_points;
    draw_tracks_ = cfg.vis.video_tracks;
    dim_cropped_ = cfg.vis.video_dim;
    if (!enabled_) return;
    try {
        cv::namedWindow(window_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_, width_, width_ * 9 / 16);
    } catch (const cv::Exception& e) {
        std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Окно OpenCV недоступно (" << e.what()
                  << "); просмотр видеопотока выключен.\n";
        enabled_ = false;
    }
}

FrameViewer::~FrameViewer() { close(); }

bool FrameViewer::show(const cv::Mat& image, const std::vector<cv::Point2f>& points,
                       const std::vector<cv::Point2f>* m1, const std::vector<cv::Point2f>* m2,
                       const std::vector<char>* inliers, const CropUsed& crop,
                       const ViewInfo& info) {
    if (!enabled_) return true;
    try {
        cv::Mat vis;
        cv::cvtColor(image, vis, cv::COLOR_GRAY2BGR);

        // Область, отброшенная обрезкой, гасится - сразу видно, что алгоритм
        // на этом шаге не использует.
        if (dim_cropped_ && crop.active) {
            const int x0 = static_cast<int>(crop.bounds[0]), x1 = static_cast<int>(crop.bounds[1]);
            const int y0 = static_cast<int>(crop.bounds[2]), y1 = static_cast<int>(crop.bounds[3]);
            cv::Mat dimmed;
            vis.convertTo(dimmed, -1, kDim);
            cv::Rect keep = cv::Rect(x0, y0, x1 - x0, y1 - y0) & cv::Rect(0, 0, vis.cols, vis.rows);
            if (!keep.empty()) vis(keep).copyTo(dimmed(keep));
            vis = dimmed;
            cv::rectangle(vis, {x0, y0}, {x1 - 1, y1 - 1}, cv::Scalar(60, 200, 255), 2);
        }

        // Все отобранные точки - синие; те, что остались синими, сопоставить
        // не удалось.
        if (draw_points_)
            for (const auto& p : points)
                cv::circle(vis, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 2,
                           cv::Scalar(235, 160, 50), cv::FILLED);

        // Сопоставленные: инлаеры зелёные, отбракованные красные.
        if (m2 && !m2->empty()) {
            for (size_t i = 0; i < m2->size(); ++i) {
                const bool ok = !inliers || (i < inliers->size() && (*inliers)[i]);
                cv::circle(vis, cv::Point(static_cast<int>((*m2)[i].x), static_cast<int>((*m2)[i].y)),
                           3, ok ? cv::Scalar(80, 220, 80) : cv::Scalar(60, 60, 230), cv::FILLED);
                if (draw_tracks_ && ok && m1 && i < m1->size())
                    cv::line(vis,
                             cv::Point(static_cast<int>((*m1)[i].x), static_cast<int>((*m1)[i].y)),
                             cv::Point(static_cast<int>((*m2)[i].x), static_cast<int>((*m2)[i].y)),
                             cv::Scalar(80, 220, 80), 1);
            }
        }

        const double scale = static_cast<double>(width_) / vis.cols;
        if (scale < 1.0)
            cv::resize(vis, vis, cv::Size(width_, static_cast<int>(vis.rows * scale)), 0, 0,
                       cv::INTER_AREA);
        draw_hud(vis, info, crop.active, crop.fraction);

        cv::imshow(window_, vis);
        const int key = cv::waitKey(paused_ ? 0 : 1) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) return false;
        if (key == ' ') paused_ = !paused_;
        return true;
    } catch (const cv::Exception&) {
        enabled_ = false;   // окно закрыли - просто перестаём рисовать
        return true;
    }
}

void FrameViewer::draw_hud(cv::Mat& vis, const ViewInfo& info, bool crop_active,
                           double crop_fraction) const {
    // Текст латиницей: шрифты OpenCV содержат только ASCII.
    const std::string rows[4] = {
        fmt("frame %d/%d   step %d   shift %.1f %% of frame", info.frame, info.limit, info.step,
            info.shift),
        fmt("points %d   matches %d   inliers %d", info.points, info.matches, info.inliers),
        fmt("turn %.1f deg/s", info.rate) +
            (crop_active ? fmt("   CROP ON: %.0f %% of frame", 100 * crop_fraction)
                         : std::string("   crop off")),
        fmt("path %.0f m   %.1f steps/s", info.distance, info.rate_steps) +
            (paused_ ? "   PAUSED (space)" : "   q - quit"),
    };
    const int pad = 8, line = 20, h = pad * 2 + line * 4;
    const cv::Rect panel = cv::Rect(0, 0, std::min(520, vis.cols), std::min(h, vis.rows));
    cv::Mat roi = vis(panel);
    roi.convertTo(roi, -1, 0.35);
    for (int i = 0; i < 4; ++i)
        cv::putText(vis, rows[i], {pad, pad + line * (i + 1) - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
}

void FrameViewer::close() {
    if (!enabled_) return;
    try {
        cv::destroyWindow(window_);
    } catch (const cv::Exception&) {
    }
    enabled_ = false;
}

// ---------------------------------------------------------------------------

LiveTrajectoryPlot::LiveTrajectoryPlot(const Config& cfg) {
    enabled_ = cfg.vis.live_plot;
    every_ = std::max(1, cfg.vis.live_plot_every);
    const PlaneInfo plane = trajectory_plane(cfg);
    col_x_ = plane.col_x;
    col_y_ = plane.col_y;
    plane_ = plane.name;
    if (!enabled_) return;
    try {
        cv::namedWindow(window_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_, 700, 640);
    } catch (const cv::Exception&) {
        enabled_ = false;
    }
}

LiveTrajectoryPlot::~LiveTrajectoryPlot() { finish(); }

void LiveTrajectoryPlot::update(const std::vector<cv::Vec3d>& positions, size_t steps,
                                double distance, bool force) {
    if (!enabled_) return;
    if (!force && steps % static_cast<size_t>(every_) != 0) return;
    try {
        Axes ax;
        ax.title = fmt("Trajectory %s, step %zu, path %.0f m", plane_.c_str(), steps, distance);
        ax.equal = true;
        ax.legend = false;
        Series s;
        s.color = tab_color(0);
        for (const auto& p : positions) {
            s.x.push_back(p[col_x_]);
            s.y.push_back(p[col_y_]);
        }
        ax.series.push_back(s);
        cv::imshow(window_, render_figure({ax}, 1, 1, {700, 640}));
        cv::waitKey(1);
    } catch (const cv::Exception&) {
        enabled_ = false;
    }
}

void LiveTrajectoryPlot::finish() {
    if (!enabled_) return;
    try {
        cv::destroyWindow(window_);
    } catch (const cv::Exception&) {
    }
    enabled_ = false;
}

}  // namespace vo
