#pragma once

// Преобразования координат и углов.

#include <cmath>

#include "../utils/constants.h"
#include "../utils/types.h"
#include "matrix_ops.h"

// нормирование углов от -pi до pi
inline double normolize_angle(double a)
{
    while (a > PI)
    {
        a -= 2 * PI;
    }
    while (a < -PI)
    {
        a += 2 * PI;
    }
    return a;
}

inline double rad_to_deg(double a)
{
    return a * RAD_TO_DEG;
}

inline double deg_to_rad(double a)
{
    return a * DEG_TO_RAD;
}

// Матрица направляющих косинусов C: связанная СК -> географическая СК.
// Порядок осей связанной СК по ГОСТ: X вперёд, Y вверх, Z вправо.
// Строки результата соответствуют осям N, H, E.
inline Matrix bodyToNavMatrix(double heading, double pitch, double roll)
{
    Matrix C(3 * 3, 0);
    const double c_vals[3][3] =
        {
            {cos(pitch) * cos(heading), -cos(roll) * cos(heading) * sin(pitch) + sin(roll) * sin(heading), sin(roll) * cos(heading) * sin(pitch) + cos(roll) * sin(heading)},
            {sin(pitch), cos(roll) * cos(pitch), -sin(roll) * cos(pitch)},
            {-cos(pitch) * sin(heading), cos(roll) * sin(heading) * sin(pitch) + sin(roll) * cos(heading), -sin(roll) * sin(heading) * sin(pitch) + cos(roll) * cos(heading)}};

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            at(C, r, c, 3) = c_vals[r][c];
        }
    }
    return C;
}

// пересчёт вектора из связанной СК в географическую
inline Vector bodyToNav(const Matrix &C, const Vector &v_body)
{
    return multiply_m(C, v_body, 3);
}

// пересчёт вектора из географической СК в связанную
inline Vector navToBody(const Matrix &C, const Vector &v_nav)
{
    return multiply_m(transpose_m(C, 3), v_nav, 3);
}
