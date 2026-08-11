#pragma once

// Чтение входных данных: imu.dat, gps.dat, angle.dat.

#include <fstream>
#include <string>
#include <vector>

#include "../navigation/gps_processor.h"

namespace data_io
{

// разбор строки в числа
std::vector<double> &parseLine(const std::string &line, std::vector<double> &items);

// разделение строки на элементы
std::vector<std::string> splitLine(const std::string &line);

// Последовательное чтение imu.dat: файл открывается, первая строка
// (заголовок) пропускается.
class ImuReader
{
public:
    bool open(const std::string &path);

    // читает очередную строку; возвращает false в конце файла
    bool next(std::vector<double> &row);

    void close();

private:
    std::ifstream file_;
    std::string line_;
};

// Параллельное чтение эталона: gps.dat (время, широта, долгота в градусах,
// высота, скорости N/H/E) и angle.dat (время, крен, тангаж, курс в радианах).
// За один вызов next читается по одной строке из каждого файла, поэтому
// отсчёты остаются синхронными с imu.dat.
class SnsReader
{
public:
    bool open(const std::string &gps_path, const std::string &angle_path);

    // читает очередной отсчёт; false — когда закончился любой из файлов
    bool next(nav::SnsSample &out);

    void close();

private:
    std::ifstream gps_;
    std::ifstream angle_;
    std::string gps_line_;
    std::string angle_line_;
};

} // namespace data_io
