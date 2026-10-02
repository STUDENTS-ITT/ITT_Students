// data_reader.h — Чтение imu.data, gps.data, angle.data формата dataset_1786981733.
//
// Время в файлах: ЧЧ:ММ:СС.дробь (например 00:00:00.006000128).
// Внутри программы строка ИМУ приводится к прежнему виду:
//   time_s  timestamp_ns  wx  wy  wz  ax  ay  az
//
// gps.data:  time  lat[deg]  lon[deg]  alt[m]  vx  vy  vz
// angle.data: time  x y z  qw qx qy qz  vx vy vz  roll pitch yaw
//
// Интерфейс ImuReader / SnsReader тот же, что в исходной программе.

#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "../navigation/gps_processor.h"

namespace data_io
{

std::vector<std::string> splitLine(const std::string &line);
bool parseTimestamp(const std::string &token, double &seconds);
bool parseImuLine(const std::string &line, std::vector<double> &row);

class ImuReader
{
public:
    bool open(const std::string &path);
    bool next(std::vector<double> &row);
    void close();

private:
    std::ifstream file_;
    std::string line_;
};

class SnsReader
{
public:
    bool open(const std::string &gps_path, const std::string &angle_path = "");
    bool next(nav::SnsSample &out);
    void close();

private:
    bool readNextAngle();

    std::ifstream gps_;
    std::ifstream angle_;
    std::string gps_line_;
    std::string angle_line_;
    bool has_angle_ = false;

    bool have_ang_prev_ = false;
    bool have_ang_curr_ = false;
    double ang_t_prev_ = 0.0;
    double ang_t_curr_ = 0.0;
    double ang_roll_prev_ = 0.0;
    double ang_pitch_prev_ = 0.0;
    double ang_yaw_prev_ = 0.0;
    double ang_roll_curr_ = 0.0;
    double ang_pitch_curr_ = 0.0;
    double ang_yaw_curr_ = 0.0;
};

} // namespace data_io
