// data_reader.cpp — Реализация чтения файлов imu.dat, gps.dat, angle.dat.
//
// Индексы столбцов соответствуют формату датасета fix_mav_square_1000m:
//   gps.dat:  time_s | timestamp_ns | latitude | longitude | altitude | vx | vy | vz
//   angle.dat: time_s | timestamp_ns | roll | pitch | yaw

#include "data_reader.h"

#include <cmath>
#include <sstream>

#include "../utils/constants.h"
#include "io_error.h"

namespace data_io
{
namespace
{
// Индексы столбцов gps.dat (tab-separated).
constexpr std::size_t GPS_COL_TIME = 0;
constexpr std::size_t GPS_COL_LAT = 2;
constexpr std::size_t GPS_COL_LON = 3;
constexpr std::size_t GPS_COL_ALT = 4;
constexpr std::size_t GPS_COL_VN = 5;
constexpr std::size_t GPS_COL_VH = 6;
constexpr std::size_t GPS_COL_VE = 7;
constexpr std::size_t GPS_MIN_COLS = 8;

// Индексы столбцов angle.dat (tab-separated).
constexpr std::size_t ANG_COL_TIME = 0;
constexpr std::size_t ANG_COL_ROLL = 2;
constexpr std::size_t ANG_COL_PITCH = 3;
constexpr std::size_t ANG_COL_YAW = 4;
constexpr std::size_t ANG_MIN_COLS = 5;

} // namespace

// Разбор строки на вектор чисел (split по пробелам/табуляциям).
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

// Разбор строки на вектор строк (для поэлементного чтения SnsReader).
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

// Открытие imu.dat, пропуск строки заголовка.
bool ImuReader::open(const std::string &path)
{
    file_.open(path);
    if (!file_.is_open())
    {
        reportOpenError(path);
        return false;
    }
    std::getline(file_, line_);  // пропуск заголовка (# time_s ...)
    return true;
}

// Чтение следующей строки imu.dat, разбор на числа.
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

// Открытие gps.dat (обязательный) и angle.dat (опциональный).
bool SnsReader::open(const std::string &gps_path, const std::string &angle_path)
{
    gps_.open(gps_path);
    if (!gps_.is_open())
    {
        reportOpenError(gps_path);
        return false;
    }
    std::getline(gps_, gps_line_);  // пропуск заголовка

    // angle.dat опциональный: если путь пустой или файл не открывается — работаем без него.
    has_angle_ = false;
    have_ang_prev_ = false;
    have_ang_curr_ = false;
    if (!angle_path.empty())
    {
        angle_.open(angle_path);
        if (angle_.is_open())
        {
            std::getline(angle_, angle_line_);  // пропуск заголовка
            has_angle_ = true;
            readNextAngle();  // первый отсчёт в окно curr
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

        if (have_ang_curr_)
        {
            ang_t_prev_ = ang_t_curr_;
            ang_roll_prev_ = ang_roll_curr_;
            ang_pitch_prev_ = ang_pitch_curr_;
            ang_yaw_prev_ = ang_yaw_curr_;
            have_ang_prev_ = true;
        }

        ang_t_curr_ = std::stod(ang[ANG_COL_TIME]);
        // Углы angle.dat заданы в градусах → переводим в радианы (внутренняя СК — рад).
        ang_roll_curr_ = std::stod(ang[ANG_COL_ROLL]) * DEG_TO_RAD;
        ang_pitch_curr_ = std::stod(ang[ANG_COL_PITCH]) * DEG_TO_RAD;
        ang_yaw_curr_ = std::stod(ang[ANG_COL_YAW]) * DEG_TO_RAD;
        have_ang_curr_ = true;
        return true;
    }
    return false;
}

// Чтение следующей строки gps.dat (+ angle.dat если есть), заполнение SnsSample.
// Координаты gps.dat — градусы→рад; углы angle.dat — градусы→рад, стыковка по времени.
bool SnsReader::next(nav::SnsSample &out)
{
    while (std::getline(gps_, gps_line_))
    {
        const std::vector<std::string> gps = splitLine(gps_line_);
        if (gps.size() < GPS_MIN_COLS)
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

        if (!has_angle_)
        {
            out.heading = 0.0;
            out.roll = 0.0;
            out.pitch = 0.0;
            return true;
        }

        // Подтягиваем angle, пока curr.time < gps.time.
        while (have_ang_curr_ && ang_t_curr_ < out.time)
        {
            if (!readNextAngle())
            {
                break;
            }
        }

        // Ближайший по времени среди prev и curr.
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
