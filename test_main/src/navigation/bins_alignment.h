#pragma once

// Автономная (аналитическая) начальная выставка БИНС на неподвижном участке:
// крен и тангаж — по проекциям силы тяжести на оси акселерометров, курс —
// гирокомпасированием по горизонтальной проекции угловой скорости Земли.
// Эталонные углы из angle.dat при этом не используются.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "../data_io/data_reader.h"
#include "../ins/attitude_calc.h"
#include "../ins/imu_processor.h"
#include "../math_lib/statistics.h"
#include "../utils/constants.h"
#include "../utils/types.h"
#include "gps_processor.h"

namespace nav
{

// Окно сглаживания показаний ИМУ, отсчётов. Полезный сигнал (проекция ω Земли,
// ~5e-5 рад/с) на порядок меньше шума гироскопов, без усреднения курс не выделить.
constexpr int ALIGN_SMOOTH_WINDOW = 1000;

// Скорость по СНС, ниже которой объект считается неподвижным, м/с.
constexpr double ALIGN_V_THRESHOLD = 0.01;

// Ниже этой длительности неподвижного участка гирокомпасирование бессмысленно, с.
constexpr double ALIGN_MIN_TIME = 30.0;

struct AlignmentResult
{
    bool ok = false;
    ins::Attitude att;
    double g = 0;            // ускорение силы тяжести, принятое при выставке
    double duration = 0;     // длительность использованного участка, с
    std::size_t samples = 0; // число отсчётов ИМУ на участке
};

// Результат разведочного прохода по эталону: первый отсчёт (нужен для
// стартовых координат и силы тяжести) и время начала движения.
struct SnsScan
{
    bool ok = false;
    SnsSample first;
    double motion_start = 0;
};

// Отдельный проход по эталону до начала движения: границу выставки нельзя
// узнать, не заглянув вперёд, а основной цикл читает файлы построчно.
// Если движение так и не началось, границей становится конец записи.
inline SnsScan scanSns(const std::string &gps_path, const std::string &angle_path)
{
    SnsScan res;

    data_io::SnsReader reader;
    if (!reader.open(gps_path, angle_path))
    {
        return res;
    }

    SnsSample s;
    double t_last = 0;
    while (reader.next(s))
    {
        if (!res.ok)
        {
            res.first = s;
            res.ok = true;
        }
        t_last = s.time;
        if (s.maxSpeed() > ALIGN_V_THRESHOLD)
        {
            res.motion_start = s.time;
            reader.close();
            return res;
        }
    }
    reader.close();

    res.motion_start = t_last;
    return res;
}

// Выставка по отсчётам imu.dat от начала записи до начала движения.
// При неудаче возвращается результат с ok == false.
inline AlignmentResult alignBins(const std::string &imu_path, const SnsScan &scan)
{
    AlignmentResult res;

    const double t_stop = scan.motion_start;
    if (!scan.ok || t_stop < ALIGN_MIN_TIME)
    {
        return res;
    }

    data_io::ImuReader imu;
    if (!imu.open(imu_path))
    {
        return res;
    }

    Vector ax, ay, az, wx, wz;
    std::vector<double> row;
    double t_first = 0;
    double t_last = 0;

    while (imu.next(row))
    {
        if (!ins::isValidRow(row))
        {
            continue;
        }
        const double t = ins::sampleTime(row);
        if (t >= t_stop)
        {
            break;
        }
        if (ax.empty())
        {
            t_first = t;
        }
        t_last = t;

        // выставка идёт до первой оценки смещений, показания берутся сырыми
        ax.push_back(row[ins::IMU_COL_ACCEL + 0]);
        ay.push_back(row[ins::IMU_COL_ACCEL + 1]);
        az.push_back(row[ins::IMU_COL_ACCEL + 2]);
        wx.push_back(row[ins::IMU_COL_GYRO + 0]);
        wz.push_back(row[ins::IMU_COL_GYRO + 2]);
    }
    imu.close();

    if (ax.empty())
    {
        return res;
    }

    res.g = normalGravity(scan.first.lat, scan.first.alt);

    ax = movingAverage(ax, ALIGN_SMOOTH_WINDOW);
    ay = movingAverage(ay, ALIGN_SMOOTH_WINDOW);
    az = movingAverage(az, ALIGN_SMOOTH_WINDOW);
    wx = movingAverage(wx, ALIGN_SMOOTH_WINDOW);
    wz = movingAverage(wz, ALIGN_SMOOTH_WINDOW);

    Vector psi(ax.size()), gamma(ax.size()), theta(ax.size());
    for (std::size_t k = 0; k < ax.size(); k++)
    {
        psi[k] = atan2(-wz[k], wx[k]);
        gamma[k] = atan2(az[k], ay[k]);
        theta[k] = asin(std::max(-1.0, std::min(1.0, ax[k] / res.g)));
    }

    res.att.heading = circularMean(psi);
    res.att.roll = Mean(gamma);
    res.att.pitch = Mean(theta);
    res.duration = t_last - t_first;
    res.samples = ax.size();
    res.ok = true;
    return res;
}

} // namespace nav
