#pragma once

// Запись результатов работы алгоритма в текстовые файлы.

#include <fstream>
#include <string>

#include "../utils/types.h"

namespace data_io
{

// Одна строка файла результатов. Углы и координаты хранятся в радианах,
// перевод в градусы выполняется при записи.
struct NavRecord
{
    double time = 0;

    // решение БИНС после коррекции
    double lon = 0;
    double lat = 0;
    double heading = 0;
    double pitch = 0;
    double roll = 0;
    double vn = 0;
    double vh = 0;
    double ve = 0;
    double alt = 0;

    // эталон (СНС + angle.dat)
    double lon_sns = 0;
    double lat_sns = 0;
    double alt_sns = 0;
    double hdg_sns = 0;
    double roll_sns = 0;
    double pitch_sns = 0;
    double vn_sns = 0;
    double vh_sns = 0;
    double ve_sns = 0;

    // курс, оценённый по проекции угловой скорости Земли
    double hdg_true = 0;

    // широта БИНС до коррекции
    double lat_bins = 0;
};

// Результат начальной выставки отдельным файлом, углы в радианах.
bool writeAlignment(const std::string &path, double heading, double pitch, double roll);

// Пишет два файла: траекторию и вектор оценённых ошибок фильтра.
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
