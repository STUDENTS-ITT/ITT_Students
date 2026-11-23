// gravity.h — модели силы тяжести.

#pragma once

#include <cmath>

#include "../common/constants.h"

// Нормальная сила тяжести на широте lat (рад) и высоте h (м).
// Формула Клеро с поправкой на высоту.
inline double normalGravity(double lat, double h)
{
    const double g0 = G_EQ * (1.0 + GRAVITY_CONSTANT * sin(lat) * sin(lat));
    const double scale = R_EARTH / (R_EARTH + h);
    return g0 * scale * scale;
}
