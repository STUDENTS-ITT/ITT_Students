// interpolation.h — Метод трапеций для интегрирования скорости и координат.
//
// Используется в ins_filter.h на каждом такте ИМУ для перехода
// от ускорения к скорости (v_integral) и от скорости к координатам.

#pragma once

#include <cstddef>

#include "../utils/types.h"

// Одномерная интеграция: V = V0 + (a_now + a_pred)/2 * dt (метод трапеций).
inline double v_integral(double v_now_dot, double V0, double v_pred_dot, double dt)
{
    return V0 + ((v_now_dot + v_pred_dot) / 2) * dt;
}

// Векторная интеграция поэлементно: интегрирует каждый компонент ускорения.
inline Vector v_integral(const Vector &v_now_dot, const Vector &V0, const Vector &v_pred_dot, double dt)
{
    Vector res(V0.size());
    for (std::size_t i = 0; i < V0.size(); i++)
    {
        res[i] = v_integral(v_now_dot[i], V0[i], v_pred_dot[i], dt);
    }
    return res;
}
