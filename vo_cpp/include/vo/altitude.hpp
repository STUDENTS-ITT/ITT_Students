// Высота съёмки по кадрам (порт раздела 4b vo_final2.py).
#pragma once

#include <string>
#include <vector>

#include "vo/config.hpp"

namespace vo {

// "const" | "file". Синонимы приводятся здесь и только здесь.
std::string altitude_source(const Config& cfg);

// Высота в метрах для КАЖДОГО кадра источника (num_frames значений).
std::vector<double> load_altitude_series(const Config& cfg, size_t num_frames);

// Координата Z траектории: высота кадра минус высота первого кадра.
inline double altitude_to_z(const std::vector<double>& altitude, size_t index) {
    return altitude.at(index) - altitude.at(0);
}

// Линейная интерполяция как numpy.interp (вне диапазона - крайние значения).
// xs должен быть отсортирован по возрастанию.
std::vector<double> interp(const std::vector<double>& x, const std::vector<double>& xs,
                           const std::vector<double>& ys);

}  // namespace vo
