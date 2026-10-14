// data_source.cpp — Реализация файлового источника входных данных.
//
// FileDataSource повторяет прежнюю логику nav::runSimulation по выбору файлов
// (imu.dat/imu.data, gps.dat/gps.data, angle.dat/angle.data), разбору потока
// ИМУ/СНС и чтению StartupNav.ini, но за общим интерфейсом IDataSource.

#include "data_source.h"

#include <filesystem>

#include "../navigation/simulation.h"  // nav::readStartupNav, nav::countImuRows

namespace data_io
{
namespace
{
// Выбор файла в каталоге: предпочтительное имя, иначе резервное; при
// отсутствии обоих возвращается предпочтительный путь (для сообщения об ошибке).
std::string pickFile(const std::string &dir, const std::string &preferred,
                     const std::string &fallback)
{
    const std::string first = dir + "/" + preferred;
    std::error_code ec;
    if (std::filesystem::exists(first, ec))
    {
        return first;
    }
    const std::string second = dir + "/" + fallback;
    if (std::filesystem::exists(second, ec))
    {
        return second;
    }
    return first;
}
} // namespace

FileDataSource::FileDataSource(const std::string &data_dir)
    : data_dir_(data_dir),
      imu_file_(pickFile(data_dir, "imu.dat", "imu.data")),
      gps_file_(pickFile(data_dir, "gps.dat", "gps.data")),
      angle_file_(pickFile(data_dir, "angle.dat", "angle.data")),
      startup_file_(data_dir + "/StartupNav.ini"),
      angle_exists_(std::filesystem::exists(angle_file_))
{
}

bool FileDataSource::open()
{
    error_.clear();
    if (!imu_.open(imu_file_))
    {
        error_ = "cannot open " + imu_file_;
        return false;
    }
    if (!sns_.open(gps_file_, angle_exists_ ? angle_file_ : ""))
    {
        error_ = "cannot open " + gps_file_;
        imu_.close();
        return false;
    }
    return true;
}

std::size_t FileDataSource::imuRowCount() const
{
    return nav::countImuRows(imu_file_);
}

bool FileDataSource::nextImu(std::vector<double> &row)
{
    return imu_.next(row);
}

bool FileDataSource::nextSns(nav::SnsSample &out)
{
    return sns_.next(out);
}

bool FileDataSource::startupNav(double &lon_deg, double &lat_deg, double &alt,
                                double &time_s)
{
    return nav::readStartupNav(startup_file_, lon_deg, lat_deg, alt, time_s);
}

void FileDataSource::close()
{
    imu_.close();
    sns_.close();
}

} // namespace data_io
