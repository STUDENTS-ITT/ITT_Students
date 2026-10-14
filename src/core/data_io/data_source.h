// data_source.h — Абстрактный источник входных данных прогона (этап 8).
//
// Развязывает ядро счисления от файлов: nav::runSimulation читает строки ИМУ и
// отсчёты СНС через интерфейс IDataSource. Файловая реализация
// (FileDataSource) инкапсулирует прежний разбор imu.dat/gps.dat/angle.dat и
// StartupNav.ini; MockDataSource выдаёт данные из памяти (тесты, отладка).
// Позже тем же интерфейсом воспользуется потоковый источник (SerialSource в
// GUI-слое), не меняя ядро.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "data_reader.h"
#include "../navigation/gps_processor.h"

namespace data_io
{

// Общий интерфейс источника входных данных прогона.
class IDataSource
{
public:
    virtual ~IDataSource() = default;

    // Открыть или перемотать источник на начало. false — ошибка (см. error()).
    virtual bool open() = 0;

    // Есть ли в потоке СНС углы ориентации (angle.dat/angle.data).
    virtual bool hasAngle() const = 0;

    // Оценка числа строк ИМУ (0 — неизвестно; нужна для индикатора прогресса).
    virtual std::size_t imuRowCount() const = 0;

    // Следующая строка ИМУ в каноническом виде:
    // time_s, timestamp_ns, wx, wy, wz, ax, ay, az. false — конец потока.
    virtual bool nextImu(std::vector<double> &row) = 0;

    // Следующий отсчёт СНС. false — конец потока.
    virtual bool nextSns(nav::SnsSample &out) = 0;

    // Начальные координаты и время выставки из StartupNav.ini.
    // false — файла нет (вызывающий берёт первый отсчёт СНС).
    virtual bool startupNav(double &lon_deg, double &lat_deg, double &alt,
                            double &time_s) = 0;

    virtual void close() = 0;

    // Описание ошибки последнего open().
    virtual std::string error() const { return {}; }
};

// Файловый источник: imu.dat/imu.data, gps.dat/gps.data, angle.dat/angle.data,
// StartupNav.ini в каталоге данных.
class FileDataSource : public IDataSource
{
public:
    explicit FileDataSource(const std::string &data_dir);

    bool open() override;
    bool hasAngle() const override { return angle_exists_; }
    std::size_t imuRowCount() const override;
    bool nextImu(std::vector<double> &row) override;
    bool nextSns(nav::SnsSample &out) override;
    bool startupNav(double &lon_deg, double &lat_deg, double &alt,
                    double &time_s) override;
    void close() override;
    std::string error() const override { return error_; }

private:
    std::string data_dir_;
    std::string imu_file_;
    std::string gps_file_;
    std::string angle_file_;
    std::string startup_file_;
    bool angle_exists_ = false;
    std::string error_;

    ImuReader imu_;
    SnsReader sns_;
};

// Источник в памяти: строки ИМУ и отсчёты СНС задаются заранее.
class MockDataSource : public IDataSource
{
public:
    std::vector<std::vector<double>> imu_rows;
    std::vector<nav::SnsSample> sns_samples;
    bool has_angle = false;
    bool has_startup = false;
    double startup_lon_deg = 0.0;
    double startup_lat_deg = 0.0;
    double startup_alt = 0.0;
    double startup_time_s = 0.0;

    bool open() override
    {
        imu_index_ = 0;
        sns_index_ = 0;
        return true;
    }
    bool hasAngle() const override { return has_angle; }
    std::size_t imuRowCount() const override { return imu_rows.size(); }
    bool nextImu(std::vector<double> &row) override
    {
        if (imu_index_ >= imu_rows.size())
        {
            return false;
        }
        row = imu_rows[imu_index_++];
        return true;
    }
    bool nextSns(nav::SnsSample &out) override
    {
        if (sns_index_ >= sns_samples.size())
        {
            return false;
        }
        out = sns_samples[sns_index_++];
        return true;
    }
    bool startupNav(double &lon_deg, double &lat_deg, double &alt,
                    double &time_s) override
    {
        if (!has_startup)
        {
            return false;
        }
        lon_deg = startup_lon_deg;
        lat_deg = startup_lat_deg;
        alt = startup_alt;
        time_s = startup_time_s;
        return true;
    }
    void close() override {}

private:
    std::size_t imu_index_ = 0;
    std::size_t sns_index_ = 0;
};

} // namespace data_io
