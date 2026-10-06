// telemetry_buffer.cpp — Реализация накопителя телеметрии.

#include "telemetry_buffer.h"

#include <limits>

namespace gui
{

double RunSeries::timeMin() const
{
    if (rows.empty()) return 0.0;
    double t = std::numeric_limits<double>::max();
    for (const auto &r : rows) t = r.time < t ? r.time : t;
    return t;
}

double RunSeries::timeMax() const
{
    if (rows.empty()) return 0.0;
    double t = std::numeric_limits<double>::lowest();
    for (const auto &r : rows) t = r.time > t ? r.time : t;
    return t;
}

} // namespace gui