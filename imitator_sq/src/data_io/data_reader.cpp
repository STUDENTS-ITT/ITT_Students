// data_reader.cpp — Чтение imu.data / gps.data / angle.data (формат dataset_1786981733).

#include "data_reader.h"

#include <cmath>
#include <cstdio>
#include <sstream>

#include "../utils/constants.h"
#include "io_error.h"

namespace data_io
{
namespace
{
constexpr std::size_t GPS_MIN_COLS = 7;
constexpr std::size_t ANG_MIN_COLS = 14;
constexpr std::size_t IMU_FILE_COLS = 7; // time wx wy wz ax ay az

constexpr std::size_t GPS_COL_LAT = 1;
constexpr std::size_t GPS_COL_LON = 2;
constexpr std::size_t GPS_COL_ALT = 3;
constexpr std::size_t GPS_COL_VN = 4;
constexpr std::size_t GPS_COL_VH = 5;
constexpr std::size_t GPS_COL_VE = 6;

constexpr std::size_t ANG_COL_ROLL = 11;
constexpr std::size_t ANG_COL_PITCH = 12;
constexpr std::size_t ANG_COL_YAW = 13;
} // namespace

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

bool parseTimestamp(const std::string &token, double &seconds)
{
    int hours = 0;
    int minutes = 0;
    double sec = 0.0;
    if (std::sscanf(token.c_str(), "%d:%d:%lf", &hours, &minutes, &sec) == 3)
    {
        seconds = static_cast<double>(hours) * 3600.0 + static_cast<double>(minutes) * 60.0 + sec;
        return true;
    }
    try
    {
        seconds = std::stod(token);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool parseImuLine(const std::string &line, std::vector<double> &row)
{
    const std::vector<std::string> tok = splitLine(line);
    if (tok.size() < IMU_FILE_COLS)
    {
        return false;
    }
    double time_s = 0.0;
    if (!parseTimestamp(tok[0], time_s))
    {
        return false;
    }
    try
    {
        const double wx = std::stod(tok[1]);
        const double wy = std::stod(tok[2]);
        const double wz = std::stod(tok[3]);
        const double ax = std::stod(tok[4]);
        const double ay = std::stod(tok[5]);
        const double az = std::stod(tok[6]);
        row = {time_s, 0.0, wx, wy, wz, ax, ay, az};
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool ImuReader::open(const std::string &path)
{
    file_.open(path);
    if (!file_.is_open())
    {
        reportOpenError(path);
        return false;
    }
    std::getline(file_, line_);
    return true;
}

bool ImuReader::next(std::vector<double> &row)
{
    while (std::getline(file_, line_))
    {
        if (parseImuLine(line_, row))
        {
            return true;
        }
    }
    return false;
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
    std::getline(gps_, gps_line_);

    has_angle_ = false;
    have_ang_prev_ = false;
    have_ang_curr_ = false;
    if (!angle_path.empty())
    {
        angle_.open(angle_path);
        if (angle_.is_open())
        {
            std::getline(angle_, angle_line_);
            has_angle_ = true;
            readNextAngle();
        }
    }
    return true;
}

bool SnsReader::readNextAngle()
{
    while (std::getline(angle_, angle_line_))
    {
        const std::vector<std::string> ang = splitLine(angle_line_);
        if (ang.size() < ANG_MIN_COLS)
        {
            continue;
        }

        double t = 0.0;
        if (!parseTimestamp(ang[0], t))
        {
            continue;
        }

        try
        {
            if (have_ang_curr_)
            {
                ang_t_prev_ = ang_t_curr_;
                ang_roll_prev_ = ang_roll_curr_;
                ang_pitch_prev_ = ang_pitch_curr_;
                ang_yaw_prev_ = ang_yaw_curr_;
                have_ang_prev_ = true;
            }

            ang_t_curr_ = t;
            ang_roll_curr_ = std::stod(ang[ANG_COL_ROLL]);
            ang_pitch_curr_ = std::stod(ang[ANG_COL_PITCH]);
            ang_yaw_curr_ = std::stod(ang[ANG_COL_YAW]);
            have_ang_curr_ = true;
            return true;
        }
        catch (const std::exception &)
        {
            continue;
        }
    }
    return false;
}

bool SnsReader::next(nav::SnsSample &out)
{
    while (std::getline(gps_, gps_line_))
    {
        const std::vector<std::string> gps = splitLine(gps_line_);
        if (gps.size() < GPS_MIN_COLS)
        {
            continue;
        }
        if (!parseTimestamp(gps[0], out.time))
        {
            continue;
        }

        try
        {
            out.lat = std::stod(gps[GPS_COL_LAT]) * DEG_TO_RAD;
            out.lon = std::stod(gps[GPS_COL_LON]) * DEG_TO_RAD;
            out.alt = std::stod(gps[GPS_COL_ALT]);
            out.vn = std::stod(gps[GPS_COL_VN]);
            out.vh = std::stod(gps[GPS_COL_VH]);
            out.ve = std::stod(gps[GPS_COL_VE]);
        }
        catch (const std::exception &)
        {
            continue;
        }

        if (!has_angle_)
        {
            out.heading = 0.0;
            out.roll = 0.0;
            out.pitch = 0.0;
            return true;
        }

        while (have_ang_curr_ && ang_t_curr_ < out.time)
        {
            if (!readNextAngle())
            {
                break;
            }
        }

        if (have_ang_prev_ && have_ang_curr_)
        {
            if (std::fabs(out.time - ang_t_prev_) <= std::fabs(out.time - ang_t_curr_))
            {
                out.roll = ang_roll_prev_;
                out.pitch = ang_pitch_prev_;
                out.heading = ang_yaw_prev_;
            }
            else
            {
                out.roll = ang_roll_curr_;
                out.pitch = ang_pitch_curr_;
                out.heading = ang_yaw_curr_;
            }
        }
        else if (have_ang_curr_)
        {
            out.roll = ang_roll_curr_;
            out.pitch = ang_pitch_curr_;
            out.heading = ang_yaw_curr_;
        }
        else if (have_ang_prev_)
        {
            out.roll = ang_roll_prev_;
            out.pitch = ang_pitch_prev_;
            out.heading = ang_yaw_prev_;
        }
        else
        {
            out.heading = 0.0;
            out.roll = 0.0;
            out.pitch = 0.0;
        }
        return true;
    }
    return false;
}

void SnsReader::close()
{
    gps_.close();
    if (has_angle_)
    {
        angle_.close();
    }
}

} // namespace data_io
