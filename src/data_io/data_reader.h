// data_reader.h — Последовательное чтение imu.dat, gps.dat, angle.dat.
//
// ImuReader:
//   Читает imu.dat построчно, пропускает заголовок.
//   Формат: time_s  timestamp_ns  wx  wy  wz  ax  ay  az
//
// SnsReader:
//   Параллельно читает gps.dat и angle.dat (синхронно построчно).
//   Формат gps.dat:  time_s  timestamp_ns  latitude  longitude  altitude  vx  vy  vz
//   Формат angle.dat: time_s  timestamp_ns  roll  pitch  yaw
//   Координаты переводятся из градусов в радианы.

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

// Чтение gps.dat + angle.dat: синхронно построчно, формирует SnsSample.
// Координаты переводятся в радианы.
class SnsReader
{
public:
    bool open(const std::string &gps_path, const std::string &angle_path);
    bool next(nav::SnsSample &out);
    void close();

private:
    std::ifstream gps_;
    std::ifstream angle_;
    std::string gps_line_;
    std::string angle_line_;
};

} // namespace data_io
