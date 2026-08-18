// position_calc.h — Счисление координат и скоростей в навигационной СК.
//
// Модуль выполняет:
//   - Расчёт угловой скорости навигационной СК (ω_н)
//   - Расчёт вредных ускорений (Кориолис + центробежное + гравитация)
//   - Интегрирование уравнений скоростей (метод трапеций)
//   - Интегрирование координат (широта, долгота, высота)

#pragma once

#include <cmath>

#include "../math_lib/interpolation.h"
#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "../utils/types.h"

namespace nav
{

// Результат интегрирования координат.
struct Position
{
    double lat = 0;      // широта, рад
    double lon = 0;      // долгота, рад
    double alt = 0;      // высота, м
    double lat_dot = 0;  // производная широты, рад/с
    double lon_dot = 0;  // производная долготы, рад/с
};

// Угловая скорость навигационной СК:
//   ω_N = U_EARTH·cos(φ) + V_E / R_h
//   ω_H = U_EARTH·sin(φ) + V_E·tan(φ) / R_h
//   ω_E = -V_N / R_h
inline Vector navAngularRate(double lat, const Vector &V, double Rh)
{
    const double V_N = V[0];
    const double V_E = V[2];

    const double W_N = U_EARTH * cos(lat) + V_E / Rh;
    const double W_H = U_EARTH * sin(lat) + (V_E * tan(lat)) / Rh;
    const double W_E = -V_N / Rh;
    return {W_N, W_H, W_E};
}

// Вредные ускорения: Кориолис (2ω × V) + центробежное (ω̂ × V) + гравитация.
inline Vector harmfulAccel(double lat, double alt, const Vector &V, double lat_dot, double lon_dot)
{
    // Кориолисово ускорение: 2·ω_Земля × V
    const Vector U2 = {2 * U_EARTH * cos(lat), 2 * U_EARTH * sin(lat), 0};
    const Vector Ac = vector_product(U2, V);

    // Центробежное ускорение: ω̂ × V, где ω̂ — угловая скорость вращения СК
    const Vector w_shtr = {lon_dot * cos(lat), lon_dot * sin(lat), -lat_dot};

    const Vector A_harm = vector_sum(Ac, vector_product(w_shtr, V));
    // Гравитация (нормальная сила тяжести по формуле Клеро)
    const Vector g = {0, normalGravity(lat, alt), 0};
    return vector_sum(A_harm, g);
}

// Производная скорости: V̇ = C_б^н·f_б − a_вред
inline Vector velocityDot(const Vector &n_nav, const Vector &a_harm)
{
    return vector_diff(n_nav, a_harm);
}

// Интегрирование скоростей методом трапеций.
inline Vector integrateVelocity(const Vector &V_dot, const Vector &V_prev,
                                const Vector &V_dot_prev, double dt)
{
    return v_integral(V_dot, V_prev, V_dot_prev, dt);
}

// Интегрирование координат (широта, долгота, высота).
// Широта и долгота: φ̇ = V_N / R_h,  λ̇ = V_E / (R_h·cos(φ))
inline Position integratePosition(double lat, double lon, double alt, const Vector &V,
                                  double lat_dot_prev, double lon_dot_prev,
                                  double alt_dot_prev, double dt)
{
    const double Rh = R_EARTH + alt;

    Position pos;
    pos.lat_dot = V[0] / Rh;
    pos.lat = std::max(-M_PI / 2.0, std::min(M_PI / 2.0, v_integral(pos.lat_dot, lat, lat_dot_prev, dt)));

    pos.lon_dot = V[2] / (Rh * fmax(fabs(cos(pos.lat)), 1e-10));
    pos.lon = normalize_angle(v_integral(pos.lon_dot, lon, lon_dot_prev, dt));

    pos.alt = v_integral(V[1], alt, alt_dot_prev, dt);
    return pos;
}

} // namespace nav
