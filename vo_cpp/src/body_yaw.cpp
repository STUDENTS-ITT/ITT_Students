#include "vo/body_yaw.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

#include "vo/util.hpp"

namespace vo {

namespace fs = std::filesystem;

namespace {

int column_index(const std::vector<std::string>& header, const std::string& name) {
    for (size_t i = 0; i < header.size(); ++i)
        if (header[i] == name) return static_cast<int>(i);
    return -1;
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ',') {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r') {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

double unwrap_near(double prev, double cur) {
    double d = cur - prev;
    while (d > kPi) d -= 2.0 * kPi;
    while (d < -kPi) d += 2.0 * kPi;
    return prev + d;
}

}  // namespace

bool BodyYawTrack::load(const std::string& camera_csv, const std::string& attitude_csv) {
    ok_ = false;
    yaw_.clear();
    cam_t_.clear();
    std::ifstream cam(fs::u8path(camera_csv));
    std::ifstream att(fs::u8path(attitude_csv));
    if (!cam || !att) return false;

    std::string line;
    if (!std::getline(cam, line)) return false;
    const auto cam_header = split_csv(line);
    const int c_sec = column_index(cam_header, "stamp_sec");
    const int c_nsec = column_index(cam_header, "stamp_nsec");
    if (c_sec < 0 || c_nsec < 0) return false;
    while (std::getline(cam, line)) {
        if (line.empty()) continue;
        const auto cols = split_csv(line);
        if (static_cast<int>(cols.size()) <= std::max(c_sec, c_nsec)) continue;
        try {
            cam_t_.push_back(std::stod(cols[static_cast<size_t>(c_sec)]) +
                             std::stod(cols[static_cast<size_t>(c_nsec)]) * 1e-9);
        } catch (const std::exception&) {
        }
    }

    if (!std::getline(att, line)) return false;
    const auto att_header = split_csv(line);
    const int a_sec = column_index(att_header, "header.stamp.sec");
    const int a_nsec = column_index(att_header, "header.stamp.nanosec");
    const int vz_i = column_index(att_header, "vector.z");
    const int qx_i = column_index(att_header, "quaternion.x");
    const int qy_i = column_index(att_header, "quaternion.y");
    const int qz_i = column_index(att_header, "quaternion.z");
    const int qw_i = column_index(att_header, "quaternion.w");
    const bool gyro = a_sec >= 0 && a_nsec >= 0 && vz_i >= 0;
    if (!gyro && (a_sec < 0 || a_nsec < 0 || qx_i < 0 || qy_i < 0 || qz_i < 0 || qw_i < 0)) return false;
    const int need = gyro ? std::max({a_sec, a_nsec, vz_i})
                          : std::max({a_sec, a_nsec, qx_i, qy_i, qz_i, qw_i});

    std::vector<double> att_t, att_yaw, gyro_abs;
    while (std::getline(att, line)) {
        if (line.empty()) continue;
        const auto cols = split_csv(line);
        if (static_cast<int>(cols.size()) <= need) continue;
        try {
            const double t = std::stod(cols[static_cast<size_t>(a_sec)]) +
                             std::stod(cols[static_cast<size_t>(a_nsec)]) * 1e-9;
            att_t.push_back(t);
            if (gyro) {
                gyro_abs.push_back(std::abs(std::stod(cols[static_cast<size_t>(vz_i)])));
            } else {
                const double x = std::stod(cols[static_cast<size_t>(qx_i)]);
                const double y = std::stod(cols[static_cast<size_t>(qy_i)]);
                const double z = std::stod(cols[static_cast<size_t>(qz_i)]);
                const double w = std::stod(cols[static_cast<size_t>(qw_i)]);
                const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
                if (att_yaw.empty()) att_yaw.push_back(yaw);
                else att_yaw.push_back(unwrap_near(att_yaw.back(), yaw));
            }
        } catch (const std::exception&) {
        }
    }
    if (cam_t_.size() < 2 || att_t.size() < 2) return false;

    yaw_.assign(cam_t_.size(), 0.0);
    if (gyro) {
        size_t j = 0;
        for (size_t i = 1; i < cam_t_.size(); ++i) {
            while (j < att_t.size() && att_t[j] <= cam_t_[i - 1]) ++j;
            double peak = 0.0;
            for (size_t k = j; k < att_t.size() && att_t[k] <= cam_t_[i]; ++k)
                peak = std::max(peak, gyro_abs[k]);
            const double dt = cam_t_[i] - cam_t_[i - 1];
            yaw_[i] = yaw_[i - 1] + peak * std::max(dt, 0.0);
        }
    } else {
        size_t j = 0;
        for (size_t i = 0; i < cam_t_.size(); ++i) {
            const double t = cam_t_[i];
            while (j + 1 < att_t.size() && att_t[j + 1] < t) ++j;
            if (t <= att_t.front()) yaw_[i] = att_yaw.front();
            else if (t >= att_t.back() || j + 1 >= att_t.size()) yaw_[i] = att_yaw.back();
            else {
                const double dt = att_t[j + 1] - att_t[j];
                const double w = dt > 1e-9 ? (t - att_t[j]) / dt : 0.0;
                yaw_[i] = att_yaw[j] + w * (att_yaw[j + 1] - att_yaw[j]);
            }
        }
    }
    ok_ = true;
    return true;
}

double BodyYawTrack::yaw_rad(size_t frame) const {
    if (!ok_ || frame >= yaw_.size()) return 0.0;
    return yaw_[frame];
}

double BodyYawTrack::delta_rad(size_t frame_a, size_t frame_b) const {
    if (!ok_) return 0.0;
    const size_t a = std::min(frame_a, frame_b);
    const size_t b = std::max(frame_a, frame_b);
    if (a >= yaw_.size() || b >= yaw_.size()) return 0.0;
    return yaw_[b] - yaw_[a];
}

double BodyYawTrack::rate_dps(size_t frame_a, size_t frame_b) const {
    if (!ok_ || frame_a >= cam_t_.size() || frame_b >= cam_t_.size() || frame_a == frame_b) return 0.0;
    size_t a = frame_a, b = frame_b;
    if (a > b) std::swap(a, b);
    double peak = 0.0;
    for (size_t i = a + 1; i <= b; ++i) {
        const double dt = cam_t_[i] - cam_t_[i - 1];
        if (dt <= 1e-6) continue;
        peak = std::max(peak, std::abs(yaw_[i] - yaw_[i - 1]) / dt * (180.0 / kPi));
    }
    return peak;
}

void rotate_points_2d(std::vector<cv::Point2f>& pts, double cx, double cy, double rad) {
    if (std::abs(rad) < 1e-12) return;
    const double c = std::cos(rad), s = std::sin(rad);
    for (cv::Point2f& p : pts) {
        const double x = p.x - cx, y = p.y - cy;
        p.x = static_cast<float>(cx + c * x - s * y);
        p.y = static_cast<float>(cy + s * x + c * y);
    }
}

void integrate_arc_xy(double x0, double y0, double h0, double step_m, size_t frame_a, size_t frame_b,
                      const BodyYawTrack& body, double& x_out, double& y_out) {
    x_out = x0;
    y_out = y0;
    if (step_m <= 0.0 || !body.ok()) return;
    size_t a = std::min(frame_a, frame_b);
    size_t b = std::max(frame_a, frame_b);
    if (b <= a) return;

    const double total = body.delta_rad(a, b);
    const double abs_total = std::abs(total);
    if (abs_total < 1e-6) {
        x_out += step_m * std::cos(h0);
        y_out += step_m * std::sin(h0);
        return;
    }

    double x = x0, y = y0, h = h0;
    for (size_t i = a + 1; i <= b; ++i) {
        const double dth = body.yaw_rad(i) - body.yaw_rad(i - 1);
        const double frac = std::abs(dth) / abs_total;
        const double ds = step_m * frac;
        x += ds * std::cos(h);
        y += ds * std::sin(h);
        h -= dth;
    }
    x_out = x;
    y_out = y;
}

}  // namespace vo
