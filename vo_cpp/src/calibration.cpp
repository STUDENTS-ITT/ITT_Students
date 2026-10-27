#include "vo/calibration.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>

#include <opencv2/core.hpp>

#include "vo/util.hpp"

namespace vo {

Intrinsics Intrinsics::scaled(int new_height, int new_width) const {
    const double sy = static_cast<double>(new_height) / height;
    const double sx = static_cast<double>(new_width) / width;
    Intrinsics out;
    out.fx = fx * sx;
    out.fy = fy * sy;
    out.cx = cx * sx;
    out.cy = cy * sy;
    out.height = new_height;
    out.width = new_width;
    return out;
}

std::string Intrinsics::str() const {
    return fmt("Intrinsics(fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, size=%dx%d)", fx, fy,
               cx, cy, width, height);
}

bool Calibration::has_distortion() const {
    for (double d : intrinsics.distortion)
        if (std::abs(d) > 1e-8) return true;
    return false;
}

namespace {

// Плоский список чисел из узла любой вложенности ([a,b], [[a,b],[c,d]], число).
void flatten(const cv::FileNode& node, std::vector<double>& out) {
    if (node.isSeq()) {
        for (auto it = node.begin(); it != node.end(); ++it) flatten(*it, out);
    } else if (node.isInt() || node.isReal()) {
        out.push_back(static_cast<double>(node.real()));
    }
}

std::optional<std::vector<double>> get_numbers(const cv::FileStorage& fs,
                                               std::initializer_list<const char*> names) {
    for (const char* n : names) {
        cv::FileNode node = fs[n];
        if (node.empty()) continue;
        std::vector<double> v;
        flatten(node, v);
        if (!v.empty()) return v;
    }
    return std::nullopt;
}

std::optional<Intrinsics> from_storage(const cv::FileStorage& fs, int image_h,
                                       int image_w) {
    Intrinsics in;
    in.height = image_h;
    in.width = image_w;

    if (auto dist = get_numbers(fs, {"distortion", "dist", "distCoeffs", "RadialDistortion"})) {
        in.distortion = *dist;
        if (in.distortion.size() < 5) in.distortion.resize(5, 0.0);
    }
    if (auto size = get_numbers(fs, {"image_size", "imageSize", "ImageSize"})) {
        if (size->size() >= 2) {
            in.height = static_cast<int>((*size)[0]);
            in.width = static_cast<int>((*size)[1]);
        }
    }

    if (auto K = get_numbers(fs, {"K", "IntrinsicMatrix", "camera_matrix", "cameraMatrix"})) {
        if (K->size() == 9) {
            double m[9];
            std::copy(K->begin(), K->end(), m);
            // MATLAB хранит IntrinsicMatrix транспонированной.
            if (std::abs(m[2]) < 1e-9 && std::abs(m[6]) > 1e-9) {
                std::swap(m[1], m[3]);
                std::swap(m[2], m[6]);
                std::swap(m[5], m[7]);
            }
            in.fx = m[0];
            in.fy = m[4];
            in.cx = m[2];
            in.cy = m[5];
            return in;
        }
    }

    auto scalar = [&](std::initializer_list<const char*> names) -> std::optional<double> {
        auto v = get_numbers(fs, names);
        if (v) return (*v)[0];
        return std::nullopt;
    };
    auto fx = scalar({"fx", "FocalLengthX"});
    auto fy = scalar({"fy", "FocalLengthY"});
    if (!fx) {
        if (auto focal = get_numbers(fs, {"focal", "FocalLength"})) {
            fx = (*focal)[0];
            fy = focal->back();
        }
    }
    auto cx = scalar({"cx", "PrincipalPointX"});
    auto cy = scalar({"cy", "PrincipalPointY"});
    if (!cx) {
        if (auto pp = get_numbers(fs, {"principal", "PrincipalPoint"})) {
            if (pp->size() >= 2) {
                cx = (*pp)[0];
                cy = (*pp)[1];
            }
        }
    }
    if (!fx || !cx || !cy) return std::nullopt;
    in.fx = *fx;
    in.fy = fy ? *fy : *fx;
    in.cx = *cx;
    in.cy = *cy;
    return in;
}

std::optional<Intrinsics> load_file(const std::string& path, int h, int w) {
    cv::FileStorage fs;
    try {
        if (!fs.open(path, cv::FileStorage::READ)) {
            std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Не удалось открыть файл калибровки: " << path << "\n";
            return std::nullopt;
        }
    } catch (const cv::Exception& e) {
        std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Не удалось прочитать калибровку " << path << ": "
                  << e.what() << "\n";
        return std::nullopt;
    }
    return from_storage(fs, h, w);
}

}  // namespace

Calibration load_calibration(const Config& cfg, int image_height, int image_width) {
    const CameraCfg& camera = cfg.camera;
    std::optional<Intrinsics> intrinsics;
    std::string source;

    if (!camera.calibration_file.empty() &&
        std::filesystem::exists(std::filesystem::u8path(camera.calibration_file))) {
        intrinsics = load_file(camera.calibration_file, image_height, image_width);
        if (intrinsics) source = camera.calibration_file;
    }

    if (!intrinsics && camera.focal_length && camera.principal_point) {
        Intrinsics in;
        in.fx = (*camera.focal_length)[0];
        in.fy = (*camera.focal_length)[1];
        in.cx = (*camera.principal_point)[0];
        in.cy = (*camera.principal_point)[1];
        in.height = image_height;
        in.width = image_width;
        if (!camera.distortion.empty()) {
            in.distortion = camera.distortion;
            in.distortion.resize(5, 0.0);
        }
        intrinsics = in;
        source = "config";
    }

    bool approximate = false;
    if (!intrinsics) {
        if (!camera.allow_approximate)
            throw std::runtime_error(
                "Калибровка не задана. Укажите calibration_file или focal_length/"
                "principal_point.");
        Intrinsics in;
        const double focal =
            camera.approximate_focal_factor * std::max(image_height, image_width);
        in.fx = in.fy = focal;
        in.cx = (image_width - 1) / 2.0;
        in.cy = (image_height - 1) / 2.0;
        in.height = image_height;
        in.width = image_width;
        intrinsics = in;
        approximate = true;
        source = "approximate";
        std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Реальные параметры камеры не найдены. Создана "
                     "приближенная калибровка; количественная точность траектории "
                     "не гарантируется.\n";
    }

    if (intrinsics->height != image_height || intrinsics->width != image_width) {
        std::cout << "[ПРЕДУПРЕЖДЕНИЕ] Размер изображения отличается от размера "
                     "калибровки. Внутренние параметры масштабированы, коррекция "
                     "дисторсии отключена.\n";
        intrinsics = intrinsics->scaled(image_height, image_width);
    }

    Calibration calib;
    calib.intrinsics = *intrinsics;
    calib.is_approximate = approximate;
    calib.source = source;
    std::cout << "Калибровка: " << intrinsics->str() << " (источник: " << source << ")\n";
    return calib;
}

}  // namespace vo
