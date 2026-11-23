// Экспорт данных VO в формат add-kalman-report (gps.dat, angle.dat)
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace vo {

struct Config;
struct TrajectoryRow;

// Экспорт траектории в формат add-kalman-report.
// Создаёт gps.dat (timestamp, x, y, altitude) и angle.dat (timestamp, heading).
// rows - строки траектории, positions - позиции XYZ, frame_ids - номера кадров.
void export_for_ins(const std::vector<TrajectoryRow>& rows,
                    const std::vector<cv::Vec3d>& positions,
                    const std::vector<int>& frame_ids,
                    const Config& cfg);

}  // namespace vo
