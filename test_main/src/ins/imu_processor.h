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
// Вычитается накопленная оценка смещений акселерометров (3 элемента).
inline Vector accel(const std::vector<double> &row, const Vector &ba)
{
    return {row[IMU_COL_ACCEL + 0] - ba[0],
            row[IMU_COL_ACCEL + 1] - ba[1],
            row[IMU_COL_ACCEL + 2] - ba[2]};
}

// абсолютная угловая скорость за вычетом накопленных дрейфов гироскопов
inline Vector gyro(const std::vector<double> &row, const Vector &bg)
{
    return {row[IMU_COL_GYRO + 0] - bg[0],
            row[IMU_COL_GYRO + 1] - bg[1],
            row[IMU_COL_GYRO + 2] - bg[2]};
}

} // namespace ins
