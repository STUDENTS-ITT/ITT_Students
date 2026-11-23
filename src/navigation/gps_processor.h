// gps_processor.h - обработка данных СНС (gps.dat + angle.dat).
//
// SnsSample - структура измерений, передаваемых на вход коррекции фильтра.
// measurement() - возвращает вектор из 9 элементов для вектора измерений Z
// (формат: lat, lon, alt, vn, vh, ve, heading, pitch, roll).
// maxSpeed() - максимальная абсолютная скорость по проекциям.

#pragma once

#include <cmath>

#include "../common/types.h"

namespace nav
{

// Статус измерения СНС для gnss_guard.
enum class SnsStatus
{
    VALID = 0,
    INVALID = 1,
    STALE = 2
};

// Причина отбраковки кадра.
enum class SnsReject
{
    NONE = 0,
    NONFINITE = 1,
    RANGE = 2,
    ZERO_FIX = 3,
    SPEED = 4,
    ALTITUDE = 5,
    TIME = 6,
    JUMP_POS = 7,
    JUMP_VEL = 8
};

// Структура измерений СНС.
struct SnsSample
{
    double time = 0;
    double lat = 0;     // широта, рад
    double lon = 0;     // долгота, рад
    double alt = 0;     // высота, м
    double vn = 0;      // северная скорость, м/с
    double vh = 0;      // вертикальная скорость, м/с
    double ve = 0;      // восточная скорость, м/с

    double heading = 0; // курс, рад
    double roll = 0;    // крен, рад
    double pitch = 0;   // тангаж, рад

    double sig_scale = 1.0; // масштаб сигм измерений (резерв, используется VO)

    // Формирование вектора измерений Z для входа коррекции (z[0..8]).
    Vector measurement() const
    {
        return {lat, lon, alt,
                vn, vh, ve,
                heading, pitch, roll};
    }

    // Максимальная абсолютная скорость по проекциям.
    double maxSpeed() const
    {
        return fmax(fabs(vn), fmax(fabs(vh), fabs(ve)));
    }
};

} // namespace nav
