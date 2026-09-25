#include "data_writer.h"

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <string>

#include "../utils/constants.h"
#include "io_error.h"

namespace data_io
{
namespace
{
constexpr int COL = 18;
constexpr int COL_TS = 22;

std::string formatTimestamp(double time_s)
{
    if (!std::isfinite(time_s) || time_s < 0.0)
    {
        time_s = 0.0;
    }
    int hours = static_cast<int>(time_s / 3600.0);
    double rem = time_s - static_cast<double>(hours) * 3600.0;
    int minutes = static_cast<int>(rem / 60.0);
    double sec = rem - static_cast<double>(minutes) * 60.0;
    if (sec >= 60.0)
    {
        sec = 0.0;
        minutes += 1;
    }
    if (minutes >= 60)
    {
        minutes -= 60;
        hours += 1;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%012.9f", hours, minutes, sec);
    return buf;
}
} // namespace

bool NavLogger::open(const std::string &path)
{
    file_.open(path);
    if (!file_.is_open())
    {
        reportOpenError(path);
        return false;
    }
    return true;
}

void NavLogger::writeHeader()
{
    file_ << std::left
          << std::setw(COL_TS) << "timestamp"
          << std::setw(COL) << "time"
          << std::setw(COL) << "lon"
          << std::setw(COL) << "lat"
          << std::setw(COL) << "alt"
          << std::setw(COL) << "heading"
          << std::setw(COL) << "pitch"
          << std::setw(COL) << "roll"
          << std::setw(COL) << "vn"
          << std::setw(COL) << "vh"
          << std::setw(COL) << "ve"
          << std::setw(COL) << "lon_sns"
          << std::setw(COL) << "lat_sns"
          << std::setw(COL) << "alt_sns"
          << std::setw(COL) << "heading_sns"
          << std::setw(COL) << "pitch_sns"
          << std::setw(COL) << "roll_sns"
          << std::setw(COL) << "vn_sns"
          << std::setw(COL) << "vh_sns"
          << std::setw(COL) << "ve_sns"
          << std::setw(COL) << "dlat_rad"
          << std::setw(COL) << "dlon_rad"
          << std::setw(COL) << "dh_m"
          << std::setw(COL) << "dVn"
          << std::setw(COL) << "dVh"
          << std::setw(COL) << "dVe"
          << std::setw(COL) << "dpsi_rad"
          << std::setw(COL) << "dtheta_rad"
          << std::setw(COL) << "dphi_rad"
          << std::setw(COL) << "ba_x"
          << std::setw(COL) << "ba_y"
          << std::setw(COL) << "ba_z"
          << std::setw(COL) << "bg_x"
          << std::setw(COL) << "bg_y"
          << std::setw(COL) << "bg_z"
          << std::setw(COL) << "dN_m"
          << std::setw(COL) << "dE_m"
          << std::endl;
}

void NavLogger::write(const NavRecord &r, const Vector &x, double lat)
{
    const double dN = x.size() > 0 ? x[0] * R_EARTH : 0.0;
    const double dE = x.size() > 1 ? x[1] * R_EARTH * std::cos(lat) : 0.0;

    file_ << std::left
          << std::setw(COL_TS) << formatTimestamp(r.time)
          << std::fixed << std::setprecision(8)
          << std::setw(COL) << r.time
          << std::setw(COL) << r.lon
          << std::setw(COL) << r.lat
          << std::setw(COL) << r.alt
          << std::setw(COL) << r.heading
          << std::setw(COL) << r.pitch
          << std::setw(COL) << r.roll
          << std::setw(COL) << r.vn
          << std::setw(COL) << r.vh
          << std::setw(COL) << r.ve
          << std::setw(COL) << r.lon_sns
          << std::setw(COL) << r.lat_sns
          << std::setw(COL) << r.alt_sns
          << std::setw(COL) << r.heading_sns
          << std::setw(COL) << r.pitch_sns
          << std::setw(COL) << r.roll_sns
          << std::setw(COL) << r.vn_sns
          << std::setw(COL) << r.vh_sns
          << std::setw(COL) << r.ve_sns;

    file_ << std::scientific << std::setprecision(8);
    for (std::size_t k = 0; k < x.size(); k++)
    {
        file_ << std::setw(COL) << x[k];
    }
    file_ << std::setw(COL) << dN << std::setw(COL) << dE << std::endl;
}

void NavLogger::close()
{
    file_.close();
}

} // namespace data_io
