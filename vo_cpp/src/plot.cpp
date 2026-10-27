#include "vo/plot.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vo/util.hpp"

namespace vo {

const cv::Scalar kBlack(0, 0, 0);
const cv::Scalar kGray(128, 128, 128);

cv::Scalar tab_color(int i) {
    // tab10 в RGB -> BGR
    static const int rgb[10][3] = {{31, 119, 180}, {255, 127, 14}, {44, 160, 44},
                                   {214, 39, 40},  {148, 103, 189}, {140, 86, 75},
                                   {227, 119, 194}, {127, 127, 127}, {188, 189, 34},
                                   {23, 190, 207}};
    const int* c = rgb[((i % 10) + 10) % 10];
    return cv::Scalar(c[2], c[1], c[0]);
}

namespace {

const int kFont = cv::FONT_HERSHEY_SIMPLEX;

cv::Size text_size(const std::string& s, double scale) {
    int base = 0;
    return cv::getTextSize(s, kFont, scale, 1, &base);
}

void put_text(cv::Mat& img, const std::string& s, cv::Point org, double scale = 0.45,
              const cv::Scalar& color = cv::Scalar(30, 30, 30)) {
    cv::putText(img, s, org, kFont, scale, color, 1, cv::LINE_AA);
}

// Текст, повернутый на 90 градусов против часовой стрелки (подпись оси Y).
void put_text_vertical(cv::Mat& img, const std::string& s, cv::Point center, double scale) {
    const cv::Size ts = text_size(s, scale);
    cv::Mat tmp(ts.height + 8, ts.width + 8, CV_8UC3, cv::Scalar(255, 255, 255));
    put_text(tmp, s, {4, ts.height + 2}, scale);
    cv::Mat rot;
    cv::rotate(tmp, rot, cv::ROTATE_90_COUNTERCLOCKWISE);
    const int x = center.x - rot.cols / 2, y = center.y - rot.rows / 2;
    const cv::Rect r(x, y, rot.cols, rot.rows);
    const cv::Rect clip = r & cv::Rect(0, 0, img.cols, img.rows);
    if (clip.empty()) return;
    rot(cv::Rect(clip.x - r.x, clip.y - r.y, clip.width, clip.height)).copyTo(img(clip));
}

// «Красивые» отметки на оси (1, 2, 5 x 10^k).
std::vector<double> nice_ticks(double lo, double hi, int target) {
    std::vector<double> ticks;
    if (!(hi > lo)) return ticks;
    const double raw = (hi - lo) / std::max(target, 1);
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double norm = raw / mag;
    const double step = (norm < 1.5 ? 1.0 : norm < 3.5 ? 2.0 : norm < 7.5 ? 5.0 : 10.0) * mag;
    for (double v = std::ceil(lo / step) * step; v <= hi + step * 1e-9; v += step)
        ticks.push_back(std::abs(v) < step * 1e-9 ? 0.0 : v);
    return ticks;
}

std::string tick_label(double v, double step_hint) {
    if (std::abs(v) >= 1e5 || (std::abs(v) > 0 && std::abs(v) < 1e-3)) return fmt("%.1e", v);
    if (step_hint >= 1.0 || std::abs(v - std::round(v)) < 1e-9) return fmt("%.0f", v);
    if (step_hint >= 0.1) return fmt("%.1f", v);
    return fmt("%.2f", v);
}

void draw_axes(cv::Mat& canvas, const cv::Rect& cell, const Axes& ax) {
    const int ml = 72, mr = 18, mt = 34, mb = 52;
    const cv::Rect plot(cell.x + ml, cell.y + mt, std::max(cell.width - ml - mr, 10),
                        std::max(cell.height - mt - mb, 10));

    // ----- пределы данных -----
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
    for (const Series& s : ax.series)
        for (size_t i = 0; i < s.x.size() && i < s.y.size(); ++i) {
            if (!std::isfinite(s.x[i]) || !std::isfinite(s.y[i])) continue;
            xmin = std::min(xmin, s.x[i]);
            xmax = std::max(xmax, s.x[i]);
            ymin = std::min(ymin, s.y[i]);
            ymax = std::max(ymax, s.y[i]);
        }
    for (const VLine& v : ax.vlines) {
        xmin = std::min(xmin, v.x);
        xmax = std::max(xmax, v.x);
    }
    if (xmin > xmax) { xmin = 0; xmax = 1; ymin = 0; ymax = 1; }
    if (ax.limits) {
        xmin = (*ax.limits)[0]; xmax = (*ax.limits)[1];
        ymin = (*ax.limits)[2]; ymax = (*ax.limits)[3];
    } else {
        double px = (xmax - xmin) * 0.05, py = (ymax - ymin) * 0.05;
        if (px <= 0) px = 0.5;
        if (py <= 0) py = 0.5;
        xmin -= px; xmax += px; ymin -= py; ymax += py;
    }
    if (ax.equal) {
        // aspect='equal', adjustable='datalim': короткая сторона расширяется.
        const double sx = (xmax - xmin) / plot.width, sy = (ymax - ymin) / plot.height;
        const double s = std::max(sx, sy);
        const double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
        xmin = cx - s * plot.width / 2; xmax = cx + s * plot.width / 2;
        ymin = cy - s * plot.height / 2; ymax = cy + s * plot.height / 2;
    }

    auto X = [&](double x) { return plot.x + (x - xmin) / (xmax - xmin) * plot.width; };
    auto Y = [&](double y) { return plot.y + plot.height - (y - ymin) / (ymax - ymin) * plot.height; };

    // ----- фон, сетка, отметки -----
    cv::rectangle(canvas, plot, cv::Scalar(255, 255, 255), cv::FILLED);
    const auto xt = nice_ticks(xmin, xmax, std::max(plot.width / 90, 3));
    const auto yt = nice_ticks(ymin, ymax, std::max(plot.height / 60, 3));
    const double xstep = xt.size() > 1 ? xt[1] - xt[0] : 1.0, ystep = yt.size() > 1 ? yt[1] - yt[0] : 1.0;
    for (double v : xt) {
        const int px = static_cast<int>(std::lround(X(v)));
        cv::line(canvas, {px, plot.y}, {px, plot.y + plot.height}, cv::Scalar(225, 225, 225), 1);
        cv::line(canvas, {px, plot.y + plot.height}, {px, plot.y + plot.height + 4}, kBlack, 1);
        const std::string t = tick_label(v, xstep);
        put_text(canvas, t, {px - text_size(t, 0.4).width / 2, plot.y + plot.height + 18}, 0.4);
    }
    for (double v : yt) {
        const int py = static_cast<int>(std::lround(Y(v)));
        cv::line(canvas, {plot.x, py}, {plot.x + plot.width, py}, cv::Scalar(225, 225, 225), 1);
        cv::line(canvas, {plot.x - 4, py}, {plot.x, py}, kBlack, 1);
        const std::string t = tick_label(v, ystep);
        put_text(canvas, t, {plot.x - 8 - text_size(t, 0.4).width, py + 4}, 0.4);
    }

    // ----- всё, что рисуется внутри рамки, обрезается по ней -----
    cv::Mat view = canvas(plot);
    const cv::Point off(plot.x, plot.y);
    auto P = [&](double x, double y) {
        return cv::Point(static_cast<int>(std::lround(X(x))) - off.x,
                         static_cast<int>(std::lround(Y(y))) - off.y);
    };

    for (const Span& sp : ax.spans) {
        cv::Mat overlay = view.clone();
        const int a = static_cast<int>(std::lround(X(sp.x0))) - off.x;
        const int b = static_cast<int>(std::lround(X(sp.x1))) - off.x;
        cv::rectangle(overlay, {a, 0}, {b, view.rows}, sp.color, cv::FILLED);
        cv::addWeighted(overlay, sp.alpha, view, 1.0 - sp.alpha, 0, view);
    }
    for (const VLine& v : ax.vlines) {
        const int px = static_cast<int>(std::lround(X(v.x))) - off.x;
        for (int y = 0; y < view.rows; y += 10)
            cv::line(view, {px, y}, {px, std::min(y + 5, view.rows)}, v.color, 1);
        if (!v.label.empty()) {
            cv::Mat tmp(18, text_size(v.label, 0.38).width + 8, CV_8UC3, cv::Scalar(255, 255, 255));
            put_text(tmp, v.label, {4, 13}, 0.38, v.color);
            cv::Mat rot;
            cv::rotate(tmp, rot, cv::ROTATE_90_COUNTERCLOCKWISE);
            const cv::Rect r(px + 3, 4, rot.cols, rot.rows);
            const cv::Rect clip = r & cv::Rect(0, 0, view.cols, view.rows);
            if (!clip.empty())
                rot(cv::Rect(clip.x - r.x, clip.y - r.y, clip.width, clip.height)).copyTo(view(clip));
        }
    }
    for (const Series& s : ax.series) {
        const size_t n = std::min(s.x.size(), s.y.size());
        const int th = std::max(1, static_cast<int>(std::lround(s.width)));
        if (s.style == Series::Style::Line) {
            std::vector<cv::Point> pts;
            pts.reserve(n);
            for (size_t i = 0; i < n; ++i)
                if (std::isfinite(s.x[i]) && std::isfinite(s.y[i])) pts.push_back(P(s.x[i], s.y[i]));
            if (pts.size() > 1) cv::polylines(view, pts, false, s.color, th, cv::LINE_AA);
        } else {
            const int r = s.style == Series::Style::Marker ? 6 : std::max(1, th);
            for (size_t i = 0; i < n; ++i)
                if (std::isfinite(s.x[i]) && std::isfinite(s.y[i]))
                    cv::circle(view, P(s.x[i], s.y[i]), r, s.color, cv::FILLED, cv::LINE_AA);
        }
    }

    // ----- рамка, подписи -----
    cv::rectangle(canvas, plot, kBlack, 1);
    put_text(canvas, ax.title, {plot.x + (plot.width - text_size(ax.title, 0.5).width) / 2, cell.y + 22}, 0.5);
    put_text(canvas, ax.xlabel, {plot.x + (plot.width - text_size(ax.xlabel, 0.45).width) / 2, cell.y + cell.height - 10});
    if (!ax.ylabel.empty()) put_text_vertical(canvas, ax.ylabel, {cell.x + 14, plot.y + plot.height / 2}, 0.45);

    // ----- легенда -----
    std::vector<const Series*> labelled;
    for (const Series& s : ax.series)
        if (!s.label.empty()) labelled.push_back(&s);
    if (ax.legend && !labelled.empty()) {
        int w = 0;
        for (auto* s : labelled) w = std::max(w, text_size(s->label, 0.4).width);
        const int bw = w + 44, bh = static_cast<int>(labelled.size()) * 18 + 8;
        const int bx = ax.legend_upper_right ? plot.x + plot.width - bw - 8 : plot.x + 8;
        const int by = ax.legend_upper_right ? plot.y + 8 : plot.y + plot.height - bh - 8;
        cv::Mat roi = canvas(cv::Rect(bx, by, bw, bh));
        cv::Mat white(roi.size(), roi.type(), cv::Scalar(255, 255, 255));
        cv::addWeighted(white, 0.85, roi, 0.15, 0, roi);
        cv::rectangle(canvas, {bx, by}, {bx + bw, by + bh}, cv::Scalar(180, 180, 180), 1);
        int y = by + 16;
        for (auto* s : labelled) {
            cv::line(canvas, {bx + 6, y - 4}, {bx + 30, y - 4}, s->color, 2, cv::LINE_AA);
            put_text(canvas, s->label, {bx + 36, y}, 0.4);
            y += 18;
        }
    }
}

}  // namespace

cv::Mat render_figure(const std::vector<Axes>& axes, int rows, int cols, const cv::Size& size,
                      const std::string& suptitle) {
    cv::Mat canvas(size, CV_8UC3, cv::Scalar(255, 255, 255));
    const int top = suptitle.empty() ? 0 : 32;
    if (!suptitle.empty())
        put_text(canvas, suptitle, {(size.width - text_size(suptitle, 0.6).width) / 2, 22}, 0.6);
    const int cw = size.width / std::max(cols, 1), ch = (size.height - top) / std::max(rows, 1);
    for (size_t i = 0; i < axes.size(); ++i) {
        if (axes[i].hide) continue;
        const int r = static_cast<int>(i) / cols, c = static_cast<int>(i) % cols;
        if (r >= rows) break;
        draw_axes(canvas, cv::Rect(c * cw, top + r * ch, cw, ch), axes[i]);
    }
    return canvas;
}

bool save_figure(const std::string& path, const cv::Mat& image) {
    make_dirs(parent_path(path));
    try {
        return cv::imwrite(path, image);
    } catch (const cv::Exception& e) {
        std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Не удалось сохранить " << path << ": " << e.what() << "\n";
        return false;
    }
}

}  // namespace vo
