// constants.h — Общие физические и модельные константы проекта.
//
// Содержит:
//   - Константы Земли (угловая скорость, радиус, модель силы тяжести)
//   - Константы дискретизации ИМУ (частота, период, декадирование СНС)
//   - Перевод единиц (градусы <-> радианы)
//   - Модель нормальной силы тяжести (формула Клеро)

#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Число пи.
constexpr double PI = 3.14159265358979323846;

// Угловая скорость вращения Земли, рад/с.
constexpr double U_EARTH = 7.292115e-5;

// Средний радиус Земли, м.
constexpr double R_EARTH = 6371e3;

// Ускорение свободного падения на экваторе, м/с².
constexpr double G_EQ = 9.780327;

// Коэффициент формулы Клеро (зависимость g от широты).
constexpr double GRAVITY_CONSTANT = 0.0053024;

// Алиас для обратной совместимости (aligner.hpp использует NAV_R).
constexpr double NAV_R = R_EARTH;

// Частота выдачи данных ИМУ, Гц.
constexpr double IMU_RATE_HZ = 200.0;

// Шаг дискретизации ИМУ, с.
constexpr double IMU_PERIOD = 1.0 / IMU_RATE_HZ;

// Коэффициенты перевода единиц.
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// Нормальная сила тяжести на широте lat (рад) и высоте h (м).
// Формула Клеро с поправкой на высоту.
inline double normalGravity(double lat, double h)
{
    const double g0 = G_EQ * (1.0 + GRAVITY_CONSTANT * sin(lat) * sin(lat));
    const double scale = R_EARTH / (R_EARTH + h);
    return g0 * scale * scale;
}
