// dataset_generator.h — Синтез тестовых наборов данных БИНС/СНС.
//
// Генератор строит аналитическую траекторию по профилю (сегменты: длительность,
// горизонтальная/вертикальная скорость, скорость разворота) и по ней формирует
// физически согласованные показания ИМУ (удельная сила f_b и угловая скорость
// ω_b во входной СК программы), измерения СНС (gps.dat, angle.dat) и истину
// (truth.txt в формате reference.txt).
//
// Модель полностью согласована со счислением ядра (см. trajectory.h):
//   - C = bodyToNavMatrix(heading, pitch, roll), f_body = Cᵀ·(V̇ + a_harm),
//     ω_gyro = ω_nb^b + (C^н^б)ᵀ·ω_н, где ω_н = navAngularRate(...).
//   - Поэтому прогон «чистых» (без шума) показаний через имитатор уходит от
//     истины лишь на ошибку дискретизации интеграторов (метод трапеций).
//
// Смещения/шумы, частота и случайность задаются через NoiseConfig и seed.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../math_lib/transformations.h"
#include "../navigation/position_calc.h"
#include "../utils/constants.h"
#include "../utils/types.h"

namespace dataset
{

// Один сегмент профиля траектории.
struct ProfileSegment
{
    double dur_s = 0;        // длительность, с
    double v_ms = 0;         // целевая горизонтальная скорость, м/с
    double v_up_ms = 0;      // целевая вертикальная скорость, м/с
    double turn_deg_s = 0;   // командная скорость разворота, град/с
};

// Параметры шумов и ошибок измерений.
struct NoiseConfig
{
    // Гироскоп.
    double gyro_sigma = 0.0016;                      // белый шум, рад/с
    std::vector<double> gyro_bias_deg_h = {20.0, -15.0, 10.0};   // смещение, град/ч
    double gyro_bias_rw_deg_h_sqrt_s = 0.0;          // блуждание смещения, град/ч на √с

    // Акселерометр.
    double accel_sigma = 0.043;                      // белый шум, м/с²
    std::vector<double> accel_bias_mg = {1.5, -2.0, 1.0};       // смещение, мg
    double accel_bias_rw_mg_sqrt_s = 0.0;            // блуждание смещения, мg на √с

    // СНС: координаты/скорости.
    double gps_pos_h_sigma = 1.5;                    // горизонт, м
    double gps_pos_v_sigma = 2.5;                    // высота, м
    double gps_vel_sigma = 0.05;                     // скорости, м/с
    std::vector<double> gps_pos_offset = {0.0, 0.0, 0.0};      // N, Вверх, E, м
    double gps_pos_corr_sigma = 0.8;                 // коррелированная ошибка, м
    double gps_pos_corr_tau_s = 90.0;                // время корреляции, с

    // СНС: углы.
    std::vector<double> angle_sigma_deg = {0.05, 0.05, 0.2};   // roll, pitch, yaw

    // Метки времени и пропуски.
    double imu_jitter_s = 0.0015;                    // дрожание ИМУ, с
    double imu_dropout_p = 0.003;                    // доля пропусков отсчётов ИМУ
    double sns_jitter_s = 0.006;                     // дрожание СНС, с
};

// Полная конфигурация синтеза набора данных.
struct DatasetConfig
{
    std::string name;                       // имя набора (для отчёта)
    std::string out_dir;                    // каталог для записи файлов
    double imu_hz = 200.0;                  // частота ИМУ, Гц
    double gps_hz = 10.0;                   // частота СНС (координаты/скорости), Гц
    double angle_hz = 10.0;                 // частота СНС (углы), Гц

    // Начальные условия.
    double lat_deg = 47.641468;
    double lon_deg = -122.1401651;
    double alt_m = 116.975;
    double heading_deg = 5.0;               // курс на старте
    double pitch_deg = -0.33;               // установочный тангаж корпуса
    double roll_deg = 0.0;                  // установочный крен корпуса
    double align_time_s = 120.0;            // длительность выставки

    // Сглаживание команд между сегментами (постоянные времени, с).
    double tau_speed_s = 4.0;               // горизонтальная скорость
    double tau_climb_s = 4.0;               // вертикальная скорость
    double tau_turn_s = 2.0;                // скорость разворота
    double tau_roll_s = 1.5;                // крен в развороте

    std::vector<ProfileSegment> profile;    // профиль траектории

    NoiseConfig noise;
    uint64_t seed = 42;
};

// Сводка сгенерированного набора.
struct DatasetReport
{
    std::size_t imu_rows = 0;
    std::size_t gps_rows = 0;
    std::size_t angle_rows = 0;
    std::size_t truth_rows = 0;
    double duration_s = 0;
    double final_alt_m = 0;      // фактическая высота в конце
    double traveled_m = 0;       // пройденный путь по горизонтали
};

// Синтез набора данных: пишет imu.dat, gps.dat, angle.dat (если gps_hz > 0),
// StartupNav.ini и truth.txt в cfg.out_dir. Возвращает true при успехе.
bool generateDataset(const DatasetConfig &cfg, DatasetReport &report,
                     std::string *error);

} // namespace dataset