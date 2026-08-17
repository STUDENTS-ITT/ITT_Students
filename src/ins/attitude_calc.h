// attitude_calc.h — Расчёт углов ориентации по угловым скоростям гироскопа.
//
// Модуль выполняет:
//   - Вычисление относительной угловой скорости (вычитание вращения Земли)
//   - Преобразование угловой скорости тела в скорости Эйлера (ψ̇, θ̇, φ̇)
//   - Интегрирование угловых скоростей методом трапеций
//   - Определение начального.heading по компонентам вращения Земли

#pragma once

#include <cmath>

#include "../math_lib/interpolation.h"
#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "../utils/types.h"

namespace ins
{

// Углы ориентации (курс, тангаж, крен) в радианах.
struct Attitude
{
    double heading = 0;
    double pitch = 0;
    double roll = 0;
};

// Производные углов ориентации (курс·, тангаж·, крен·) в рад/с.
struct AttitudeRates
{
    double heading_dot = 0;
    double pitch_dot = 0;
    double roll_dot = 0;
};

// Относительная угловая скорость: ω_отн = ω_гиро - (C_б^н)^T · ω_Земля.
// Вычитание вращения Земли из показаний гироскопа.
inline Vector relativeRate(const Vector &w_abs, const Vector &w_nav, const Matrix &C)
{
    return vector_diff(w_abs, navToBody(C, w_nav));
}

// Преобразование относительной угловой скорости тела в скорости Эйлера.
// Обратная матрица связывающей матрицы Эйлера (сингулярность при θ = ±90°).
inline AttitudeRates eulerRates(const Vector &w_rel, double pitch, double roll)
{
    const double wx = w_rel[0];
    const double wy = w_rel[1];
    const double wz = w_rel[2];

    AttitudeRates rates;
    double cos_pitch = fmax(fabs(cos(pitch)), 1e-10);
    rates.heading_dot = (wy * cos(roll) - wz * sin(roll)) / cos_pitch;
    rates.pitch_dot = wy * sin(roll) + wz * cos(roll);
    rates.roll_dot = wx - tan(pitch) * (wy * cos(roll) - wz * sin(roll));
    return rates;
}

// Интегрирование углов ориентации методом трапеций.
// prev_rates — производные на предыдущем такте, now — на текущем.
inline Attitude integrate(const Attitude &prev, const AttitudeRates &now,
                          const AttitudeRates &prev_rates, double dt)
{
    Attitude att;
    att.heading = normalize_angle(v_integral(now.heading_dot, prev.heading, prev_rates.heading_dot, dt));
    att.pitch = normalize_angle(v_integral(now.pitch_dot, prev.pitch, prev_rates.pitch_dot, dt));
    att.roll = normalize_angle(v_integral(now.roll_dot, prev.roll, prev_rates.roll_dot, dt));
    return att;
}

// Начальный курс из компонент вращения Земли: ψ = atan2(-ω_z, ω_x).
// Используется как вспомогательная оценка при выставке.
inline double headingFromEarthRate(double wx, double wz)
{
    return normalize_angle(atan2(-wz, wx));
}

} // namespace ins
