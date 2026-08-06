#pragma once

// Обработка сырых данных ИМУ (imu.dat).

#include <cstddef>
#include <vector>

#include "../utils/types.h"

namespace ins
{

// Раскладка колонок в imu.dat.
constexpr int IMU_COL_TIME = 0;  // время, с
constexpr int IMU_COL_GYRO = 2;  // wx, wy, wz — рад/с
constexpr int IMU_COL_ACCEL = 5; // ax, ay, az — м/с^2

// минимальное число колонок в корректной строке
constexpr std::size_t IMU_MIN_COLS = 8;

inline bool isValidRow(const std::vector<double> &row)
{
    return row.size() >= IMU_MIN_COLS;
}

inline double sampleTime(const std::vector<double> &row)
{
    return row[IMU_COL_TIME];
}

// imu.dat уже в связанной СК ГОСТ: X вперёд, Y вверх, Z вправо (ay ≈ +g).
// Из измерений вычитаются оценки смещений акселерометров x[9..11].
inline Vector accel(const std::vector<double> &row, const Vector &x)
{
    return {row[IMU_COL_ACCEL + 0] - x[9],
            row[IMU_COL_ACCEL + 1] - x[10],
            row[IMU_COL_ACCEL + 2] - x[11]};
}

// абсолютная угловая скорость за вычетом оценок дрейфов гироскопов x[12..14]
inline Vector gyro(const std::vector<double> &row, const Vector &x)
{
    return {row[IMU_COL_GYRO + 0] - x[12],
            row[IMU_COL_GYRO + 1] - x[13],
            row[IMU_COL_GYRO + 2] - x[14]};
}

} // namespace ins
