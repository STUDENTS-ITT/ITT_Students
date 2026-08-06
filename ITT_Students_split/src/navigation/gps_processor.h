#pragma once

// Отсчёт данных СНС (gps.dat) и эталонных углов (angle.dat).

#include <cmath>

#include "../utils/types.h"

namespace nav
{

// Один отсчёт внешней системы: строка gps.dat вместе с одноимённой строкой
// angle.dat. Файлы читаются построчно параллельно с imu.dat, поэтому вся
// запись в памяти не хранится.
struct SnsSample
{
    double time = 0; // с
    double lat = 0;  // рад
    double lon = 0;  // рад
    double alt = 0;  // м
    double vn = 0;   // м/с
    double vh = 0;   // м/с
    double ve = 0;   // м/с

    double heading = 0; // рад
    double roll = 0;    // рад
    double pitch = 0;   // рад

    // вектор измерений z = [φ, λ, h, Vn, Vh, Ve, ψ, θ, γ]
    Vector measurement() const
    {
        return {lat, lon, alt,
                vn, vh, ve,
                heading, pitch, roll};
    }

    // модуль наибольшей из проекций скорости — признак движения
    double maxSpeed() const
    {
        return fmax(fabs(vn), fmax(fabs(vh), fabs(ve)));
    }
};

} // namespace nav
