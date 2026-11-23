#include "vo/ins_export.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "vo/config.hpp"
#include "vo/pipeline.hpp"
#include "vo/util.hpp"

namespace vo {

namespace fs = std::filesystem;

void export_for_ins(const std::vector<TrajectoryRow>& rows,
                    const std::vector<cv::Vec3d>& positions,
                    const std::vector<int>& frame_ids,
                    const Config& cfg) {
    if (!cfg.ins_export.enable) return;
    if (rows.empty() || positions.empty()) {
        std::cout << "[INS export] Нет данных для экспорта\n";
        return;
    }

    const size_t n = rows.size();
    if (positions.size() != n || frame_ids.size() != n) {
        throw std::runtime_error("[INS export] Размеры данных не совпадают: rows=" +
                                 std::to_string(n) + ", positions=" +
                                 std::to_string(positions.size()) + ", frame_ids=" +
                                 std::to_string(frame_ids.size()));
    }

    make_dirs(cfg.ins_export.output_dir);
    const std::string gps_path = (fs::u8path(cfg.ins_export.output_dir) / "gps.dat").u8string();
    const std::string angle_path = (fs::u8path(cfg.ins_export.output_dir) / "angle.dat").u8string();

    const double fps = cfg.ins_export.frame_rate > 0 ? cfg.ins_export.frame_rate : 25.0;

    // Запись gps.dat: timestamp (сек), x (м), y (м), altitude (м)
    {
        std::ofstream out(fs::u8path(gps_path), std::ios::binary);
        if (!out) throw std::runtime_error("[INS export] Не удалось открыть " + gps_path);

        out << "# timestamp x y altitude\n";
        for (size_t i = 0; i < n; ++i) {
            const double t = static_cast<double>(frame_ids[i]) / fps;
            const double x = positions[i][0];  // X - восток
            const double y = positions[i][1];  // Y - север
            const double alt = rows[i].altitude;
            
            // Валидация высоты: NaN/Inf приведут к некорректным вычислениям в БИНС
            if (!std::isfinite(alt)) {
                throw std::runtime_error(fmt("[INS export] Невалидная высота (%.3f) на кадре %d (t=%.3f с)",
                                             alt, frame_ids[i], t));
            }
            
            out << fmt("%.6f %.6f %.6f %.6f\n", t, x, y, alt);
        }
        out.close();
        std::cout << "[INS export] Сохранено " << n << " строк в " << gps_path << "\n";
    }

    // Запись angle.dat: timestamp (сек), heading (град)
    // Heading в VO: 0° = север, по часовой стрелке (90° = восток)
    {
        std::ofstream out(fs::u8path(angle_path), std::ios::binary);
        if (!out) throw std::runtime_error("[INS export] Не удалось открыть " + angle_path);

        out << "# timestamp heading\n";
        for (size_t i = 0; i < n; ++i) {
            const double t = static_cast<double>(frame_ids[i]) / fps;
            const double heading = rows[i].heading_deg;
            out << fmt("%.6f %.6f\n", t, heading);
        }
        out.close();
        std::cout << "[INS export] Сохранено " << n << " строк в " << angle_path << "\n";
    }

    std::cout << "[INS export] Экспорт завершён: " << cfg.ins_export.output_dir << "\n";
}

}  // namespace vo
