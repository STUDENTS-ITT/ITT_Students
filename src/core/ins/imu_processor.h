// imu_processor.h — Разбор строк imu.dat и извлечение компонентов ИМУ.
//
// Формат imu.dat (tab-separated):
//   time_s  timestamp_ns  wx  wy  wz  ax  ay  az
//
// Колонки:
//   0 — время (с)
//   1 — временная метка (нс)
//   2-4 — угловые скорости гироскопа (рад/с)
//   5-7 — ускорения акселерометра (м/с²)

#pragma once

#include <cstddef>
#include <vector>

#include "../utils/types.h"

namespace ins
{

// Индексы столбцов в imu.dat.
constexpr int IMU_COL_TIME = 0;
constexpr int IMU_COL_GYRO = 2;
constexpr int IMU_COL_ACCEL = 5;

// Минимальное число столбцов для валидной строки.
constexpr std::size_t IMU_MIN_COLS = 8;

// Проверка, что строка содержит все необходимые столбцы.
inline bool isValidRow(const std::vector<double> &row)
{
    return row.size() >= IMU_MIN_COLS;
}

// Время отсчёта (секунды).
inline double sampleTime(const std::vector<double> &row)
{
    return row[IMU_COL_TIME];
}

// Ускорение акселерометра с вычитанием смещения ba.
inline Vector accel(const std::vector<double> &row, const Vector &ba)
{
    return {row[IMU_COL_ACCEL + 0] - ba[0],
            row[IMU_COL_ACCEL + 1] - ba[1],
            row[IMU_COL_ACCEL + 2] - ba[2]};
}

// Угловая скорость гироскопа с вычитанием смещения bg.
inline Vector gyro(const std::vector<double> &row, const Vector &bg)
{
    return {row[IMU_COL_GYRO + 0] - bg[0],
            row[IMU_COL_GYRO + 1] - bg[1],
            row[IMU_COL_GYRO + 2] - bg[2]};
}

} // namespace ins
