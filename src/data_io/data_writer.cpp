#include "data_writer.h"

#include <iomanip>

#include "../utils/constants.h"
#include "io_error.h"

namespace data_io
{
namespace
{
constexpr int COL = 15;
} // namespace

bool NavLogger::open(const std::string &result_path, const std::string &reference_path,
                     const std::string &error_path)
{
    result_file_.open(result_path);
    reference_file_.open(reference_path);
    error_file_.open(error_path);
    if (!result_file_.is_open())
    {
        reportOpenError(result_path);
        return false;
    }
    if (!reference_file_.is_open())
    {
        reportOpenError(reference_path);
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
    result_file_ << std::left
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
                 << std::endl;

    reference_file_ << std::left
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
                    << std::endl;

    error_file_ << std::left
                << std::setw(COL) << "time"
                << std::setw(COL) << "x0"
                << std::setw(COL) << "x1"
                << std::setw(COL) << "x2"
                << std::setw(COL) << "x3"
                << std::setw(COL) << "x4"
                << std::setw(COL) << "x5"
                << std::setw(COL) << "x6"
                << std::setw(COL) << "x7"
                << std::setw(COL) << "x8"
                << std::setw(COL) << "x9"
                << std::setw(COL) << "x10"
                << std::setw(COL) << "x11"
                << std::setw(COL) << "x12"
                << std::setw(COL) << "x13"
                << std::setw(COL) << "x14"
                << std::endl;
}

void NavLogger::writeResult(const NavResult &r)
{
    result_file_ << std::left
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
                 << std::endl;
}

void NavLogger::writeReference(const NavReference &r)
{
    reference_file_ << std::left
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
                    << std::endl;
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
    result_file_.close();
    reference_file_.close();
    error_file_.close();
}

} // namespace data_io
