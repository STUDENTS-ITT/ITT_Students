#pragma once

// Интегрирование и интерполяция данных.

#include <cstddef>

#include "../utils/types.h"

// интеграл методом трапеций за один шаг:
// x(t+dt) = x(t) + (f_текущая + f_предыдущая) / 2 * dt
inline double v_integral(double v_now_dot, double V0, double v_pred_dot, double dt)
{
    return V0 + ((v_now_dot + v_pred_dot) / 2) * dt;
}

// то же самое покомпонентно для вектора
inline Vector v_integral(const Vector &v_now_dot, const Vector &V0, const Vector &v_pred_dot, double dt)
{
    Vector res(V0.size());
    for (std::size_t i = 0; i < V0.size(); i++)
    {
        res[i] = v_integral(v_now_dot[i], V0[i], v_pred_dot[i], dt);
    }
    return res;
}
