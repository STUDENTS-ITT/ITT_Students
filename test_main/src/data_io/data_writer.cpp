#include "data_writer.h"

#include <iomanip>

#include "../utils/constants.h"
#include "io_error.h"

namespace data_io
{
namespace
{
// ширина колонки в выходных файлах
constexpr int COL = 15;
} // namespace

bool writeAlignment(const std::string &path, double heading, double pitch, double roll)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        reportOpenError(path);
        return false;
    }
    file << "Psi\tGamma\tTheta" << std::endl;
    file << std::fixed << std::setprecision(4) << heading << "\t" << roll << "\t" << pitch << std::endl;
    return true;
}

bool NavLogger::open(const std::string &nav_path, const std::string &error_path)
{
    nav_file_.open(nav_path);
    error_file_.open(error_path);
    if (!nav_file_.is_open())
    {
        reportOpenError(nav_path);
        return false;
    }
    if (!error_file_.is_open())
    {
        reportOpenError(error_path);
        return false;
    }
    return true;
}

void NavLogger::writeHeader()
{
    nav_file_ << std::left << std::setw(COL) << "time" << std::setw(COL) << "lon" << std::setw(COL) << "lat " << std::setw(COL)
              << "Heading" << std::setw(COL) << " Pitch" << std::setw(COL) << " Roll"
              << std::setw(COL) << "V0[0]" << std::setw(COL) << "V0[1]" << std::setw(COL) << "V0[2]" << std::setw(COL) << "h0" << std::setw(COL)
              << "Lon_sp" << std::setw(COL) << "Lat_sp " << std::setw(COL) << "Alt_sp "
              << std::setw(COL) << "Hdg_sp" << std::setw(COL) << " Roll_sp" << std::setw(COL) << "Pitch_sp" << std::setw(COL)
              << "Vn_sp" << std::setw(COL) << "Vh_sp" << std::setw(COL) << "Ve_sp " << std::endl;
    error_file_ << "hdg_err" << std::endl;
}

void NavLogger::write(const NavRecord &r)
{
    nav_file_ << std::left << std::setw(COL) << r.time
              << std::setw(COL) << r.lon * RAD_TO_DEG
              << std::setw(COL) << r.lat * RAD_TO_DEG
              << std::setw(COL) << r.heading * RAD_TO_DEG
              << std::setw(COL) << r.pitch * RAD_TO_DEG
              << std::setw(COL) << r.roll * RAD_TO_DEG
              << std::setw(COL) << r.vn
              << std::setw(COL) << r.vh
              << std::setw(COL) << r.ve
              << std::setw(COL) << r.alt
              << std::setw(COL) << r.lon_sns * RAD_TO_DEG
              << std::setw(COL) << r.lat_sns * RAD_TO_DEG
              << std::setw(COL) << r.alt_sns
              << std::setw(COL) << r.hdg_sns * RAD_TO_DEG
              << std::setw(COL) << r.roll_sns * RAD_TO_DEG
              << std::setw(COL) << r.pitch_sns * RAD_TO_DEG
              << std::setw(COL) << r.vn_sns
              << std::setw(COL) << r.vh_sns
              << std::setw(COL) << r.ve_sns
              << std::setw(COL) << r.hdg_true * RAD_TO_DEG
              << std::setw(COL) << r.lat_bins * RAD_TO_DEG << std::endl;
}

void NavLogger::writeErrors(double time, const Vector &x)
{
    error_file_ << std::left << std::setw(COL) << time;
    for (std::size_t k = 0; k < x.size(); k++)
    {
        error_file_ << std::setw(COL) << x[k];
    }
    error_file_ << std::endl;
}

void NavLogger::close()
{
    nav_file_.close();
    error_file_.close();
}

} // namespace data_io
