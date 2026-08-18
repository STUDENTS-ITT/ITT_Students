// bins_alignment.h — Автономная выставка БИНС по данным ИМУ и СНС.
//
// Алгоритм:
//   1. scanSns — разведочный проход по gps.dat: находит первый отсчёт
//      и момент начала движения ( скорость > 0.01 м/с).
//   2. alignBins — выставка на стационаре (до момента движения):
//      - Сглаживание ИМУ фильтром скользящего среднего (окно 1000)
//      - Расчёт углов: ψ = atan2(-wz, wx), φ = atan2(az, ay), θ = arcsin(ax/g)
//      - Средний угол (circularMean для курса, Mean для крена/тангажа)
//      - Оценка смещений гироскопа по средним показаниям и эталону СНС

#pragma once

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

// Параметры выставки.
constexpr int ALIGN_SMOOTH_WINDOW = 1000;  // окно сглаживания
constexpr double ALIGN_V_THRESHOLD = 0.01; // порог скорости для определения движения (м/с)
constexpr double ALIGN_MIN_TIME = 30.0;    // минимальная длительность стоянки (с)

// Результат автономной выставки.
struct AlignmentResult
{
    bool ok = false;
    ins::Attitude att;     // начальные углы ориентации (рад)
    double g = 0;          // нормальная сила тяжести на широте старта
    double duration = 0;   // длительность стоянки (с)
    std::size_t samples = 0; // количество отсчётов ИМУ на стоянке
    Vector bg = {0.0, 0.0, 0.0}; // оценка смещения гироскопа
};

// Результат разведочного прохода по СНС.
struct SnsScan
{
    bool ok = false;
    SnsSample first;       // первый отсчёт СНС (координаты, скорость)
    double motion_start = 0; // время начала движения (с)
};

// Разведочный проход по gps.dat + angle.dat.
// Находит первый отсчёт и момент, когда скорость > ALIGN_V_THRESHOLD.
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

    // Если движение не обнаружено — считаем конец файла концом стоянки.
    res.motion_start = t_last;
    return res;
}

// Автономная выставка на стационаре (от начала файла до motion_start).
// Возвращает начальные углы и оценку смещения гироскопа.
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

    // Чтение данных ИМУ на интервале стоянки.
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

    // Нормальная сила тяжести на широте старта.
    res.g = normalGravity(scan.first.lat, scan.first.alt);

    // Сглаживание фильтром скользящего среднего.
    ax = movingAverage(ax, ALIGN_SMOOTH_WINDOW);
    ay = movingAverage(ay, ALIGN_SMOOTH_WINDOW);
    az = movingAverage(az, ALIGN_SMOOTH_WINDOW);
    wx = movingAverage(wx, ALIGN_SMOOTH_WINDOW);
    wz = movingAverage(wz, ALIGN_SMOOTH_WINDOW);

    // Расчёт углов по каждой выборке:
    //   ψ = atan2(-wz, wx)      — курс по гироскопу (метод вектора)
    //   φ = atan2(-az, ay)       — крен по акселерометру
    //   θ = arcsin(ax / g)       — тангаж по акселерометру
    Vector psi(ax.size()), gamma(ax.size()), theta(ax.size());
    for (std::size_t k = 0; k < ax.size(); k++)
    {
        psi[k] = atan2(-wz[k], wx[k]);
        gamma[k] = atan2(-az[k], ay[k]);
        theta[k] = asin(std::max(-1.0, std::min(1.0, ax[k] / res.g)));
    }

    // Средние углы: circularMean для курса (корректно на стыке 0/2π),
    // Mean для крена и тангажа.
    res.att.heading = circularMean(psi);
    res.att.roll = Mean(gamma);
    res.att.pitch = Mean(theta);
    res.duration = t_last - t_first;
    res.samples = ax.size();

    // Оценка смещения гироскопа:
    //   bg_x = mean(wx) − U_earth·cos(φ)·cos(ψ)
    //   bg_z = mean(wz) + U_earth·cos(φ)·sin(ψ)
    const double U_h = U_EARTH * cos(scan.first.lat);
    const double psi_ref = scan.first.heading;
    res.bg[0] = Mean(wx) - U_h * cos(psi_ref);
    res.bg[2] = Mean(wz) + U_h * sin(psi_ref);

    res.ok = true;
    return res;
}

} // namespace nav
