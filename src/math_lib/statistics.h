// statistics.h — Статистические функции для выставки.
//
// Содержит:
//   - Average — среднее арифметическое с переводом рад → град (для логов)
//   - Mean — среднее арифметическое без перевода
//   - circularMean — среднее по единичной окружности (для углов, корректно
//     на стыке ±π)
//   - movingAverage — скользящее среднее (фильтр для сглаживания шума ИМУ)

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "../utils/constants.h"
#include "../utils/types.h"

// Среднее арифметическое элементов вектора, результат в градусах.
inline double Average(const Vector &angle)
{
    double sum = 0;
    std::size_t begin_sec = angle.size();
    if (begin_sec == 0) return 0.0;
    for (std::size_t k = 0; k < begin_sec; k++)
    {
        sum += angle[k];
    }
    return (sum / static_cast<double>(begin_sec)) * RAD_TO_DEG;
}

// Среднее арифметическое элементов вектора (без перевода единиц).
inline double Mean(const Vector &values)
{
    double sum = 0;
    if (values.size() == 0) return 0.0;
    for (std::size_t k = 0; k < values.size(); k++)
    {
        sum += values[k];
    }
    return sum / static_cast<double>(values.size());
}

// Средний угол по единичной окружности: atan2(mean(sin), mean(cos)).
// Корректно работает на стыке 0/2π (в отличие от простого среднего).
inline double circularMean(const Vector &angles)
{
    double sum_sin = 0;
    double sum_cos = 0;
    for (std::size_t k = 0; k < angles.size(); k++)
    {
        sum_sin += sin(angles[k]);
        sum_cos += cos(angles[k]);
    }
    return atan2(sum_sin, sum_cos);
}

// Скользящее среднее с окном window (нечётное, несмещённое).
// Используется в aligner.hpp для сглаживания показаний ИМУ при выставке.
inline Vector movingAverage(const Vector &data, int window)
{
    const int n = static_cast<int>(data.size());
    Vector smoothed(data.size(), 0.0);
    if (n == 0 || window <= 0)
    {
        return smoothed;
    }
    if (window > n)
    {
        window = n;
    }
    if (window % 2 == 0)
    {
        window++;
    }
    const int half = window / 2;

    // Префиксная сумма для O(n) вычисления суммы на отрезке.
    Vector prefix(static_cast<std::size_t>(n) + 1, 0.0);
    for (int i = 0; i < n; i++)
    {
        prefix[i + 1] = prefix[i] + data[i];
    }
    for (int i = 0; i < n; i++)
    {
        const int l = std::max(0, i - half);
        const int r = std::min(n - 1, i + half);
        smoothed[i] = (prefix[r + 1] - prefix[l]) / (r - l + 1);
    }
    return smoothed;
}
