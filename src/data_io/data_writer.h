// data_writer.h — Запись результатов счисления в файлы.
//
// NavResult — строка результата БИНС (исправленный Калманом).
// NavReference — строка эталона СНС.
// NavLogger — запись в три файла: result.txt, reference.txt, errors.txt.

#pragma once

#include <fstream>
#include <string>

#include "../utils/types.h"

namespace data_io
{

// Результат БИНС (исправленный фильтром Калмана): 10 колонок.
struct NavResult
{
    double time = 0;
    double lon = 0;       // град
    double lat = 0;       // град
    double alt = 0;       // м
    double heading = 0;   // град
    double pitch = 0;     // град
    double roll = 0;      // град
    double vn = 0;        // м/с
    double vh = 0;        // м/с
    double ve = 0;        // м/с
};

// Эталон СНС: 10 колонок.
struct NavReference
{
    double time = 0;
    double lon = 0;       // град
    double lat = 0;       // град
    double alt = 0;       // м
    double heading = 0;   // град
    double pitch = 0;     // град
    double roll = 0;      // град
    double vn = 0;        // м/с
    double vh = 0;        // м/с
    double ve = 0;        // м/с
};

// Набор файлов для записи результатов.
class NavLogger
{
public:
    bool open(const std::string &result_path, const std::string &reference_path,
              const std::string &error_path);
    void writeHeader();
    void writeResult(const NavResult &r);
    void writeReference(const NavReference &r);
    void writeErrors(double time, const Vector &x);
    void close();

private:
    std::ofstream result_file_;
    std::ofstream reference_file_;
    std::ofstream error_file_;
};

} // namespace data_io
