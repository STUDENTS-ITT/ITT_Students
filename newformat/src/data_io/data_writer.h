// data_writer.h — Запись всех результатов в один файл.
//
// Одна строка: решение БИНС + эталон СНС + вектор ошибок фильтра Калмана.

#pragma once

#include <fstream>
#include <string>

#include "../utils/types.h"

namespace data_io
{

struct NavRecord
{
    double time = 0;

    double lon = 0;
    double lat = 0;
    double alt = 0;
    double heading = 0;
    double pitch = 0;
    double roll = 0;
    double vn = 0;
    double vh = 0;
    double ve = 0;

    double lon_sns = 0;
    double lat_sns = 0;
    double alt_sns = 0;
    double heading_sns = 0;
    double pitch_sns = 0;
    double roll_sns = 0;
    double vn_sns = 0;
    double vh_sns = 0;
    double ve_sns = 0;
};

class NavLogger
{
public:
    bool open(const std::string &path);
    void writeHeader();
    void write(const NavRecord &r, const Vector &x, double lat);
    void close();

private:
    std::ofstream file_;
};

} // namespace data_io
