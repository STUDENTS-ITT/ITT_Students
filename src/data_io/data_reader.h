// data_reader.h — Последовательное чтение imu.dat, gps.dat, angle.dat.
//
// ImuReader:
//   Читает imu.dat построчно, пропускает заголовок.
//   Формат: time_s  timestamp_ns  wx  wy  wz  ax  ay  az
//
// SnsReader:
//   Читает gps.dat (обязательный) и angle.dat (опциональный).
//   Формат gps.dat:  time_s  timestamp_ns  latitude  longitude  altitude  vx  vy  vz
//   Формат angle.dat: time_s  timestamp_ns  roll  pitch  yaw
//   Координаты gps.dat переводятся из градусов в радианы.
//   Углы angle.dat уже в радианах, читаются как есть, стыкуются с gps по времени.
//   Если angle.dat отсутствует — углы ориентации заполняются нулями.

#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "../navigation/gps_processor.h"

namespace data_io
{

// Разбор строки на числа (разделитель — пробел/табуляция).
std::vector<double> &parseLine(const std::string &line, std::vector<double> &items);

// Разбор строки на строки (для SnsReader с ручным преобразованием типов).
std::vector<std::string> splitLine(const std::string &line);

// Чтение imu.dat: построчно, заголовок пропускается при open().
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

// Чтение gps.dat + angle.dat (опциональный): формирует SnsSample.
// Координаты переводятся в радианы.
// angle.dat стыкуется с gps по ближайшему времени (не по номеру строки).
// Если angle.dat не задан — углы orientation = 0.
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
