// transformations.h — Переходы между СК тела и навигационной СК.
//
// Содержит:
//   - normalize_angle — приведение угла в диапазон [-π, π]
//   - rad_to_deg / deg_to_rad — перевод единиц
//   - bodyToNavMatrix — матрица перехода C_б^н (Эйлера Z-Y-X)
//   - bodyToNav — ускорение/скорость из СК тела → навигационную СК
//   - navToBody — обратное преобразование (умножение на C^T)

#pragma once

#include <cmath>

#include "../utils/constants.h"
#include "../utils/types.h"
#include "matrix_ops.h"

// Приведение угла к диапазону [-π, π].
inline double normalize_angle(double a)
{
    a = std::fmod(a, 2.0 * PI);
    while (a > PI)
    {
        a -= 2.0 * PI;
    }
    while (a <= -PI)
    {
        a += 2.0 * PI;
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

// Матрица перехода от СК тела к навигационной СК (последовательность Z-Y-X).
// heading (ψ) — поворот вокруг оси Z, pitch (θ) — вокруг Y, roll (φ) — вокруг X.
//
// Навигационная СК — (Север, Вверх, Восток), курс отсчитывается от Севера по
// часовой стрелке (к Востоку): при ψ = 90° продольная ось тела смотрит на восток.
inline Matrix bodyToNavMatrix(double heading, double pitch, double roll)
{
    Matrix C(3 * 3, 0);
    const double c_vals[3][3] =
        {
            {cos(pitch) * cos(heading), -cos(roll) * cos(heading) * sin(pitch) - sin(roll) * sin(heading), sin(roll) * cos(heading) * sin(pitch) - cos(roll) * sin(heading)},
            {sin(pitch), cos(roll) * cos(pitch), -sin(roll) * cos(pitch)},
            {cos(pitch) * sin(heading), -cos(roll) * sin(heading) * sin(pitch) + sin(roll) * cos(heading), sin(roll) * sin(heading) * sin(pitch) + cos(roll) * cos(heading)}};

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            at(C, r, c, 3) = c_vals[r][c];
        }
    }
    return C;
}

// Перевод вектора из СК тела в навигационную: v_nav = C_б^н · v_body.
inline Vector bodyToNav(const Matrix &C, const Vector &v_body)
{
    return multiply_m(C, v_body, 3);
}

// Перевод вектора из навигационной СК в СК тела: v_body = (C_б^н)^T · v_nav.
inline Vector navToBody(const Matrix &C, const Vector &v_nav)
{
    return multiply_m(transpose_m(C, 3), v_nav, 3);
}
