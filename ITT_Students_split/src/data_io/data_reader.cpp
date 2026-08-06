#include "data_reader.h"

#include <sstream>

#include "../utils/constants.h"
#include "io_error.h"

namespace data_io
{
namespace
{
// раскладка колонок gps.dat
constexpr std::size_t GPS_COL_TIME = 0;
constexpr std::size_t GPS_COL_LAT = 2;
constexpr std::size_t GPS_COL_LON = 3;
constexpr std::size_t GPS_COL_ALT = 4;
constexpr std::size_t GPS_COL_VN = 5;
constexpr std::size_t GPS_COL_VH = 6;
constexpr std::size_t GPS_COL_VE = 7;
constexpr std::size_t GPS_MIN_COLS = 8;

// раскладка колонок angle.dat; время берётся из gps.dat
constexpr std::size_t ANG_COL_ROLL = 2;
constexpr std::size_t ANG_COL_PITCH = 3;
constexpr std::size_t ANG_COL_YAW = 4;
constexpr std::size_t ANG_MIN_COLS = 5;

} // namespace

std::vector<double> &parseLine(const std::string &line, std::vector<double> &items)
{
    items.clear();
    std::stringstream ss(line);
    std::string item;
    while (ss >> item)
    {
        items.push_back(std::stod(item));
    }
    return items;
}

std::vector<std::string> splitLine(const std::string &line)
{
    std::vector<std::string> elements;
    std::stringstream ss(line);
    std::string element;
    while (ss >> element)
    {
        elements.push_back(element);
    }
    return elements;
}

bool ImuReader::open(const std::string &path)
{
    file_.open(path);
    if (!file_.is_open())
    {
        reportOpenError(path);
        return false;
    }
    std::getline(file_, line_); // заголовок
    return true;
}

bool ImuReader::next(std::vector<double> &row)
{
    if (!std::getline(file_, line_))
    {
        return false;
    }
    parseLine(line_, row);
    return true;
}

void ImuReader::close()
{
    file_.close();
}

bool SnsReader::open(const std::string &gps_path, const std::string &angle_path)
{
    gps_.open(gps_path);
    if (!gps_.is_open())
    {
        reportOpenError(gps_path);
        return false;
    }
    angle_.open(angle_path);
    if (!angle_.is_open())
    {
        reportOpenError(angle_path);
        return false;
    }
    std::getline(gps_, gps_line_);     // заголовок
    std::getline(angle_, angle_line_); // заголовок
    return true;
}

bool SnsReader::next(nav::SnsSample &out)
{
    // Короткие строки пропускаются сразу в обоих файлах: иначе потоки
    // разъедутся и отсчёты перестанут соответствовать друг другу.
    while (std::getline(gps_, gps_line_) && std::getline(angle_, angle_line_))
    {
        const std::vector<std::string> gps = splitLine(gps_line_);
        const std::vector<std::string> ang = splitLine(angle_line_);
        if (gps.size() < GPS_MIN_COLS || ang.size() < ANG_MIN_COLS)
        {
            continue;
        }

        out.time = std::stod(gps[GPS_COL_TIME]);
        out.lat = std::stod(gps[GPS_COL_LAT]) * DEG_TO_RAD;
        out.lon = std::stod(gps[GPS_COL_LON]) * DEG_TO_RAD;
        out.alt = std::stod(gps[GPS_COL_ALT]);
        out.vn = std::stod(gps[GPS_COL_VN]);
        out.vh = std::stod(gps[GPS_COL_VH]);
        out.ve = std::stod(gps[GPS_COL_VE]);

        // angle.dat yaw = авиационный курс (по ЧС от N).
        // учебник (3.1): ψ рыскания — против ЧС от N ⇒ ψ = −yaw
        out.heading = -std::stod(ang[ANG_COL_YAW]);
        out.roll = std::stod(ang[ANG_COL_ROLL]);
        out.pitch = std::stod(ang[ANG_COL_PITCH]);
        return true;
    }
    return false;
}

void SnsReader::close()
{
    gps_.close();
    angle_.close();
}

} // namespace data_io
