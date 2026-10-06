// gps_processor.h — Один отсчёт эталона СНС (gps.dat + angle.dat).
//
// SnsSample содержит координаты, скорости и углы ориентации из СНС.
// measurement() — упаковка в вектор из 9 компонент для фильтра Калмана
// (порядок: lat, lon, alt, vn, vh, ve, heading, pitch, roll).
// maxSpeed() — модуль максимальной скорости (для определения начала движения).

#pragma once

#include <cmath>

#include "../utils/types.h"

namespace nav
{

// Один отсчёт эталона СНС.
struct SnsSample
{
    double time = 0;
    double lat = 0;     // широта, рад
    double lon = 0;     // долгота, рад
    double alt = 0;     // высота, м
    double vn = 0;      // скорость на север, м/с
    double vh = 0;      // вертикальная скорость, м/с
    double ve = 0;      // скорость на восток, м/с

    double heading = 0; // курс, рад
    double roll = 0;    // крен, рад
    double pitch = 0;   // тангаж, рад

    // Упаковка измерений в вектор для фильтра Калмана (порядок z[0..8]).
    Vector measurement() const
    {
        return {lat, lon, alt,
                vn, vh, ve,
                heading, pitch, roll};
    }

    // Модуль максимальной компоненты скорости (для порога движения).
    double maxSpeed() const
    {
        return fmax(fabs(vn), fmax(fabs(vh), fabs(ve)));
    }
};

} // namespace nav
