// data_writer.h — Запись результатов счисления в файлы.
//
// NavRecord — структура одной строки выходного файла (траектория БИНС + эталон).
// NavLogger — запись в два файла: траектория (kalman15_line2.txt) и
//            вектор ошибок фильтра Калмана (d_1.txt).
// writeAlignment — запись углов выставки (одноразовый файл Angles.dat).

#pragma once

#include <fstream>
#include <string>

#include "../utils/types.h"

namespace data_io
{

// Одна строка выходного файла:
//   - Решение БИНС (координаты, скорости, углы)
//   - Эталон СНС (координаты, скорости, углы)
//   - Доп. величины: hdg_true (курс из вращения Земли), lat_bins
struct NavRecord
{
    double time = 0;

    double lon = 0;
    double lat = 0;
    double heading = 0;
    double pitch = 0;
    double roll = 0;
    double vn = 0;
    double vh = 0;
    double ve = 0;
    double alt = 0;

    double lon_sns = 0;
    double lat_sns = 0;
    double alt_sns = 0;
    double hdg_sns = 0;
    double roll_sns = 0;
    double pitch_sns = 0;
    double vn_sns = 0;
    double vh_sns = 0;
    double ve_sns = 0;

    double hdg_true = 0;
    double lat_bins = 0;
};

// Запись углов выставки в файл Angles.dat.
bool writeAlignment(const std::string &path, double heading, double pitch, double roll);

// Набор файлов для записи результатов.
// nav_file_ — траектория (21 столбец), error_file_ — 15 ошибок Калмана.
class NavLogger
{
public:
    bool open(const std::string &nav_path, const std::string &error_path);
    void writeHeader();
    void write(const NavRecord &record);
    void writeErrors(double time, const Vector &x);
    void close();

private:
    std::ofstream nav_file_;
    std::ofstream error_file_;
};

} // namespace data_io
