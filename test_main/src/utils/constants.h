#pragma once

// Общие физические и модельные константы проекта и модель поля силы тяжести.

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// число пи
constexpr double PI = 3.14159265358979323846;

// угловая скорость вращения Земли, рад/с
constexpr double U_EARTH = 7.292115e-5;

// радиус Земли, м
constexpr double R_EARTH = 6371e3;

// Нормальная сила тяжести на широте lat (рад) и высоте h (м), м/с^2.
inline double normalGravity(double lat, double h)
{
    const double g0 = 9.780327 * (1.0 + 0.0053024 * sin(lat) * sin(lat));
    const double scale = R_EARTH / (R_EARTH + h);
    return g0 * scale * scale;
}

// частота выдачи данных ИМУ, Гц
constexpr double IMU_RATE_HZ = 200.0;

// шаг дискретизации ИМУ, с
constexpr double IMU_PERIOD = 1.0 / IMU_RATE_HZ;

// коррекция от СНС выполняется раз в IMU_RATE отсчётов (1 Гц)
constexpr int SNS_DECIMATION = 200;

constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
