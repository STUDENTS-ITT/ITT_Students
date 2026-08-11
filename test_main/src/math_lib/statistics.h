#pragma once

// Статистические методы.

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "../utils/constants.h"
#include "../utils/types.h"

// среднее арифметическое выборки углов, рад -> град
inline double Average(const Vector &angle)
{
    double sum = 0;
    std::size_t begin_sec = angle.size();
    for (std::size_t k = 0; k < begin_sec; k++)
    {
        sum += angle[k];
    }
    return (sum / static_cast<double>(begin_sec)) * RAD_TO_DEG;
}

// среднее арифметическое без перевода единиц
inline double Mean(const Vector &values)
{
    double sum = 0;
    for (std::size_t k = 0; k < values.size(); k++)
    {
        sum += values[k];
    }
    return sum / static_cast<double>(values.size());
}

// Среднее направление выборки углов (рад). Обычное среднее здесь непригодно:
// на скачке ±π оно даёт значение, не связанное с выборкой.
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

// Скользящее среднее по окну window отсчётов; чётное окно увеличивается до
// нечётного, у краёв выборки окно усекается по её границам.
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
