#include "vo/config.hpp"

#include <filesystem>
#include <stdexcept>

#include "vo/util.hpp"

namespace vo {

namespace fs = std::filesystem;

void apply_speed_preset(Config& cfg, const std::string& name_in) {
    // Пока поддерживается только homography с базовыми параметрами.
    const std::string name = lower(name_in);
    if (name != "accurate" && name != "balanced" && name != "realtime" && name != "fast") {
        throw std::runtime_error("Неизвестная предустановка скорости: " + name_in);
    }
    cfg.perf.preset = name;
}

Config build_config(const std::string& project_root) {
    Config cfg;
    cfg.project_root = project_root;

    auto absolutize = [&](std::string& p) {
        if (!p.empty() && !fs::path(p).is_absolute())
            p = (fs::path(project_root) / p).string();
    };
    absolutize(cfg.camera.calibration_file);
    absolutize(cfg.results_folder);
    absolutize(cfg.altitude.file);

    apply_speed_preset(cfg, cfg.perf.preset);
    return cfg;
}

}  // namespace vo
