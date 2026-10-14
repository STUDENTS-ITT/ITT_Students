// data_reader.cpp — Реализация чтения файлов imu.dat, gps.dat, angle.dat.
//
// Поддерживаются два формата:
//   Старый (imu.dat):   time_s | timestamp_ns | wx wy wz ax ay az
//     gps.dat:  time_s | timestamp_ns | latitude | longitude | altitude | vx | vy | vz
//     angle.dat: time_s | timestamp_ns | roll | pitch | yaw
//   Новый (dataset_1786981733, imu.data / gps.data / angle.data):
//     время в первом столбце ЧЧ:ММ:СС.дробь, столбец timestamp_ns отсутствует
//     imu.data:   время | wx wy wz ax ay az
//     gps.data:   время | latitude | longitude | altitude | vx | vy | vz
//     angle.data: время | x y z | qw qx qy qz | vx vy vz | roll | pitch | yaw
//   Формат определяется по первому столбцу строки: ':' — новый формат.

#include "data_reader.h"

#include <cmath>
#include <cstdio>
#include <exception>
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

// Новый формат dataset_1786981733: время ЧЧ:ММ:СС.дробь, без timestamp_ns.
constexpr std::size_t IMU_NEW_COLS = 7;  // время wx wy wz ax ay az
constexpr std::size_t GPS_NEW_COLS = 7;
constexpr std::size_t GPS_NEW_COL_LAT = 1;
constexpr std::size_t GPS_NEW_COL_LON = 2;
constexpr std::size_t GPS_NEW_COL_ALT = 3;
constexpr std::size_t GPS_NEW_COL_VN = 4;
constexpr std::size_t GPS_NEW_COL_VH = 5;
constexpr std::size_t GPS_NEW_COL_VE = 6;
constexpr std::size_t ANG_NEW_COLS = 14;
constexpr std::size_t ANG_NEW_COL_ROLL = 11;
constexpr std::size_t ANG_NEW_COL_PITCH = 12;
constexpr std::size_t ANG_NEW_COL_YAW = 13;

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

// Разбор времени: "ЧЧ:ММ:СС.дробь" (например 00:01:01.500000000) или
// обычное число с плавающей точкой.
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

namespace
{

// Разбор строки угловой ориентации. Формат определяется по времени:
// ':' в первом столбце — новый (14 столбцов, углы roll/pitch/yaw на 11-13),
// иначе старый (5 столбцов, углы на 2-4). Возвращает false, если строка
// не подходит под выбранный формат. Углы обоих форматов трактуются как радианы.
bool parseAngleLine(const std::string &line, double &t, double &roll,
                    double &pitch, double &yaw)
{
    const std::vector<std::string> tok = splitLine(line);
    if (tok.empty())
    {
        return false;
    }

    if (tok[0].find(':') != std::string::npos)
    {
        if (tok.size() < ANG_NEW_COLS)
        {
            return false;
        }
        if (!parseTimestamp(tok[0], t))
        {
            return false;
        }
        try
        {
            roll = std::stod(tok[ANG_NEW_COL_ROLL]);
            pitch = std::stod(tok[ANG_NEW_COL_PITCH]);
            yaw = std::stod(tok[ANG_NEW_COL_YAW]);
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    }

    if (tok.size() < ANG_MIN_COLS)
    {
        return false;
    }
    // Старый формат: повреждённая строка пропускается, а не роняет разбор.
    try
    {
        t = std::stod(tok[ANG_COL_TIME]);
        roll = std::stod(tok[ANG_COL_ROLL]);
        pitch = std::stod(tok[ANG_COL_PITCH]);
        yaw = std::stod(tok[ANG_COL_YAW]);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

// Разбор строки СНС. Формат определяется по времени: ':' — новый (7 столбцов:
// время | lat | lon | alt | vn | vh | ve), иначе старый (8 столбцов с timestamp_ns).
// Координаты обоих форматов переводятся из градусов в радианы.
bool parseGpsLine(const std::string &line, nav::SnsSample &out)
{
    const std::vector<std::string> tok = splitLine(line);
    if (tok.empty())
    {
        return false;
    }

    if (tok[0].find(':') != std::string::npos)
    {
        if (tok.size() < GPS_NEW_COLS)
        {
            return false;
        }
        if (!parseTimestamp(tok[0], out.time))
        {
            return false;
        }
        try
        {
            out.lat = std::stod(tok[GPS_NEW_COL_LAT]) * DEG_TO_RAD;
            out.lon = std::stod(tok[GPS_NEW_COL_LON]) * DEG_TO_RAD;
            out.alt = std::stod(tok[GPS_NEW_COL_ALT]);
            out.vn = std::stod(tok[GPS_NEW_COL_VN]);
            out.vh = std::stod(tok[GPS_NEW_COL_VH]);
            out.ve = std::stod(tok[GPS_NEW_COL_VE]);
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    }

    if (tok.size() < GPS_MIN_COLS)
    {
        return false;
    }
    // Старый формат: повреждённая строка пропускается, а не роняет разбор.
    try
    {
        out.time = std::stod(tok[GPS_COL_TIME]);
        out.lat = std::stod(tok[GPS_COL_LAT]) * DEG_TO_RAD;
        out.lon = std::stod(tok[GPS_COL_LON]) * DEG_TO_RAD;
        out.alt = std::stod(tok[GPS_COL_ALT]);
        out.vn = std::stod(tok[GPS_COL_VN]);
        out.vh = std::stod(tok[GPS_COL_VH]);
        out.ve = std::stod(tok[GPS_COL_VE]);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

} // namespace

// Открытие imu.dat, пропуск строки заголовка.
bool ImuReader::open(const std::string &path)
{
    // Повторное открытие того же объекта (перемотка источника): закрываем
    // прежний файл и сбрасываем состояние потока, иначе failbit остаётся
    // установленным и чтение сразу даёт EOF.
    file_.close();
    file_.clear();
    // Бинарный режим: наборы содержат бинарный мусор с байтом 0x1A, который
    // в текстовом режиме CRT считается концом файла и молча обрезает чтение.
    file_.open(path, std::ios::in | std::ios::binary);
    if (!file_.is_open())
    {
        reportOpenError(path);
        return false;
    }
    std::getline(file_, line_);  // пропуск заголовка (# time_s ...)
    return true;
}

// Чтение следующей строки imu.dat/imu.data, разбор на числа.
// Старый формат возвращается как есть; новый (время ЧЧ:ММ:СС.дробь)
// приводится к прежнему виду: time_s timestamp_ns wx wy wz ax ay az.
bool ImuReader::next(std::vector<double> &row)
{
    while (std::getline(file_, line_))
    {
        const std::vector<std::string> tok = splitLine(line_);
        if (tok.empty())
        {
            // Пустая строка — как в старом формате: возвращаем пустой ряд.
            row.clear();
            return true;
        }

        if (tok[0].find(':') != std::string::npos)
        {
            // Новый формат: ЧЧ:ММ:СС.дробь wx wy wz ax ay az.
            double time_s = 0.0;
            if (tok.size() < IMU_NEW_COLS || !parseTimestamp(tok[0], time_s))
            {
                continue;
            }
            try
            {
                row = {time_s, 0.0, std::stod(tok[1]), std::stod(tok[2]),
                       std::stod(tok[3]), std::stod(tok[4]), std::stod(tok[5]),
                       std::stod(tok[6])};
                return true;
            }
            catch (const std::exception &)
            {
                continue;
            }
        }

        // Старый формат: построчная разборка, битые строки пропускаются.
        try
        {
            parseLine(line_, row);
        }
        catch (const std::exception &)
        {
            continue;
        }
        return true;
    }
    return false;
}

void ImuReader::close()
{
    file_.close();
}

// Открытие gps.dat (обязательный) и angle.dat (опциональный).
bool SnsReader::open(const std::string &gps_path, const std::string &angle_path)
{
    // Повторное открытие того же объекта (перемотка источника): сбрасываем
    // прежние потоки и их состояние.
    gps_.close();
    gps_.clear();
    angle_.close();
    angle_.clear();

    gps_.open(gps_path, std::ios::in | std::ios::binary);
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
        angle_.open(angle_path, std::ios::in | std::ios::binary);
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
        double t = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        if (!parseAngleLine(angle_line_, t, roll, pitch, yaw))
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

        ang_t_curr_ = t;
        ang_roll_curr_ = roll;
        ang_pitch_curr_ = pitch;
        ang_yaw_curr_ = yaw;
        have_ang_curr_ = true;
        return true;
    }
    return false;
}

// Чтение следующей строки gps.dat/gps.data (+ angle.dat/angle.data если есть),
// заполнение SnsSample. Координаты gps — градусы→рад; углы — рад, стыковка по времени.
bool SnsReader::next(nav::SnsSample &out)
{
    while (std::getline(gps_, gps_line_))
    {
        if (!parseGpsLine(gps_line_, out))
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
