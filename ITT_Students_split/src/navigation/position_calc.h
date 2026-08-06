#pragma once

// Расчёт скоростей и координат в географической СК (N, H, E).

#include <cmath>

#include "../math_lib/interpolation.h"
#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "../utils/types.h"

namespace nav
{

// координаты и их производные на текущем шаге
struct Position
{
    double lat = 0;     // φ, рад
    double lon = 0;     // λ, рад
    double alt = 0;     // h, м
    double lat_dot = 0; // φ̇, рад/с
    double lon_dot = 0; // λ̇, рад/с
};

// абсолютная угловая скорость географического трёхгранника
inline Vector navAngularRate(double lat, const Vector &V, double Rh)
{
    const double V_N = V[0];
    const double V_E = V[2];

    const double W_N = U_EARTH * cos(lat) + V_E / Rh;
    const double W_H = U_EARTH * sin(lat) + (V_E * tan(lat)) / Rh;
    const double W_E = -V_N / Rh;
    return {W_N, W_H, W_E};
}

// Вредные ускорения a^k: кориолисово + переносное (3.15) плюс сила тяжести.
inline Vector harmfulAccel(double lat, const Vector &V, double lat_dot, double lon_dot)
{
    const Vector U2 = {2 * U_EARTH * cos(lat), 2 * U_EARTH * sin(lat), 0};
    const Vector Ac = vector_product(U2, V);

    //  ω' = λ̇ cosφ · i + λ̇ sinφ · j − φ̇ · k
    const Vector w_shtr = {lon_dot * cos(lat), lon_dot * sin(lat), -lat_dot};

    const Vector A_harm = vector_sum(Ac, vector_product(w_shtr, V));
    const Vector g = {0, G_ACC, 0};
    return vector_sum(A_harm, g);
}

// (3.16): V̇ = n − a^k
inline Vector velocityDot(const Vector &n_nav, const Vector &a_harm)
{
    return vector_diff(n_nav, a_harm);
}

// интегрирование скоростей методом трапеций
inline Vector integrateVelocity(const Vector &V_dot, const Vector &V_prev,
                                const Vector &V_dot_prev, double dt)
{
    return v_integral(V_dot, V_prev, V_dot_prev, dt);
}

// интегрирование координат; V — уже проинтегрированные скорости текущего шага
inline Position integratePosition(double lat, double lon, double alt, const Vector &V,
                                  double lat_dot_prev, double lon_dot_prev,
                                  double alt_dot_prev, double dt)
{
    const double Rh = R_EARTH + alt;

    Position pos;
    pos.lat_dot = V[0] / Rh;
    pos.lat = normolize_angle(v_integral(pos.lat_dot, lat, lat_dot_prev, dt));

    pos.lon_dot = V[2] / (Rh * cos(pos.lat));
    pos.lon = normolize_angle(v_integral(pos.lon_dot, lon, lon_dot_prev, dt));

    pos.alt = v_integral(V[1], alt, alt_dot_prev, dt);
    return pos;
}

} // namespace nav
