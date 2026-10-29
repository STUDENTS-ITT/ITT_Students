// gps_processor.h — Один отсчёт эталона СНС (gps.dat + angle.dat).
//
// SnsSample содержит координаты, скорости и углы ориентации из СНС.
// measurement() — упаковка в вектор из 9 компонент для фильтра Калмана
// (порядок: lat, lon, alt, vn, vh, ve, heading, pitch, roll).
// maxSpeed() — модуль максимальной скорости (для определения начала движения).
//
// SnsStatus/SnsReject — статус отсчёта и причина отклонения; их вычисляет
// защита СНС (gnss_guard.h, уровни 1-2).

#pragma once

#include <cmath>

#include "utils/types.h"

namespace nav
{

// Статус отсчёта СНС (docs/gnss_guard_design.md, раздел 4, А).
// Свойство конкретного кадра, а не пары «кадр ↔ состояние фильтра»:
// булева признака недостаточно, чтобы отличить «СНС молчит» от «СНС
// отвечает мусором» — это разные отказы с разной реакцией.
enum class SnsStatus
{
    VALID = 0,    // самодостаточные проверки пройдены
    INVALID = 1,  // данные есть, но кадр непригоден
    STALE = 2     // корректных отсчётов нет дольше таймаута
};

// Причина отклонения кадра на уровне 1 (для лога и разбора).
enum class SnsReject
{
    NONE = 0,
    NONFINITE,   // нечисловое значение
    RANGE,       // |lat| > 90° или |lon| > 180°
    ZERO_FIX,    // координаты нулевые («нет решения»)
    SPEED,       // недостижимая скорость
    ALTITUDE,    // выход из полосы высоты
    KINEMATICS,  // расхождение с последним принятым фиксом
    TIME         // немонотонное или слишком большое dt
};

// Имя статуса для лога.
inline const char *snsStatusName(SnsStatus s)
{
    switch (s)
    {
    case SnsStatus::VALID: return "VALID";
    case SnsStatus::INVALID: return "INVALID";
    case SnsStatus::STALE: return "STALE";
    }
    return "?";
}

// Имя причины отклонения для лога.
inline const char *snsRejectName(SnsReject r)
{
    switch (r)
    {
    case SnsReject::NONE: return "-";
    case SnsReject::NONFINITE: return "nonfinite";
    case SnsReject::RANGE: return "range";
    case SnsReject::ZERO_FIX: return "zero_fix";
    case SnsReject::SPEED: return "speed";
    case SnsReject::ALTITUDE: return "altitude";
    case SnsReject::KINEMATICS: return "kinematics";
    case SnsReject::TIME: return "time";
    }
    return "?";
}

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
