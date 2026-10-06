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

// Абстрактный приёмник результатов счисления.
//
// nav::step() принимает NavSink&, поэтому один и тот же расчёт может писать
// либо в файлы (NavLogger, консольный режим), либо в память (GUI), либо
// одновременно в оба (через TeeSink).
class NavSink
{
public:
    virtual ~NavSink() = default;

    virtual void writeHeader() {}
    virtual void writeResult(const NavResult &r) = 0;
    virtual void writeReference(const NavReference &r) = 0;
    virtual void writeErrors(double time, const Vector &x) = 0;
    virtual void close() {}
};

// Набор файлов для записи результатов.
class NavLogger : public NavSink
{
public:
    bool open(const std::string &result_path, const std::string &reference_path,
              const std::string &error_path);

    void writeHeader() override;
    void writeResult(const NavResult &r) override;
    void writeReference(const NavReference &r) override;
    void writeErrors(double time, const Vector &x) override;
    void close() override;

private:
    std::ofstream result_file_;
    std::ofstream reference_file_;
    std::ofstream error_file_;
};

// Дублирование потока в два приёмника (файлы + память GUI).
class TeeSink : public NavSink
{
public:
    TeeSink(NavSink &a, NavSink &b) : a_(a), b_(b) {}

    void writeHeader() override
    {
        a_.writeHeader();
        b_.writeHeader();
    }
    void writeResult(const NavResult &r) override
    {
        a_.writeResult(r);
        b_.writeResult(r);
    }
    void writeReference(const NavReference &r) override
    {
        a_.writeReference(r);
        b_.writeReference(r);
    }
    void writeErrors(double time, const Vector &x) override
    {
        a_.writeErrors(time, x);
        b_.writeErrors(time, x);
    }
    void close() override
    {
        a_.close();
        b_.close();
    }

private:
    NavSink &a_;
    NavSink &b_;
};

} // namespace data_io
