#pragma once

// Общие физические и модельные константы проекта.

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// число пи
constexpr double PI = 3.14159265358979323846;

// угловая скорость вращения Земли, рад/с
constexpr double U_EARTH = 7.292115e-5;

// радиус Земли, м
constexpr double R_EARTH = 6371e3;

// ускорение свободного падения, м/с^2
constexpr double G_ACC = 9.81;

// частота выдачи данных ИМУ, Гц
constexpr double IMU_RATE_HZ = 197.0;

// шаг дискретизации ИМУ, с
constexpr double IMU_PERIOD = 1.0 / IMU_RATE_HZ;

// коррекция от СНС выполняется раз в IMU_RATE отсчётов (1 Гц)
constexpr int SNS_DECIMATION = 197;

constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
