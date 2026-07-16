#ifndef SINS_ALGORITHM_H
#define SINS_ALGORITHM_H

#include "types.h"
#include "math_utils.h"

class SINS_Algorithm {
private:
    // --- Состояние БИНС (инициализация по умолчанию) ---
    NavState state{};       // {} автоматически зануляет все поля структуры
    double last_T_sys = 0.0;

    // --- Вспомогательные методы ---

    // Расчет эффективного g (гравитация + центробежная сила)
    inline double calc_g_eff(double lat, double alt) {
        double R = R_EQ + alt;
        double g_grav = GM_EARTH / (R * R);
        double g_centr = (U_EARTH * U_EARTH) * R * cos(lat) * cos(lat);
        return g_grav - g_centr;
    }

    // Вычисление проекций переносной угловой скорости географического трехгранника (стр. 120, ф-ла 3.6)
    inline Vec3 calc_omega_e() {
        double lat = state.pos.x;           // lat
        double V_n = state.vel.x;           // V_north
        double V_e = state.vel.z;           // V_east
        double R = R_EQ + state.pos.z;      // alt

        return {
            U_EARTH * cos(lat) + V_e / R,                // omega_xg
            U_EARTH * sin(lat) + (V_e / R) * tan(lat),   // omega_yg
            -V_n / R                                     // omega_zg
        };
    }

public:
    // --- Метод начальной выставки (стр. 162, ф-ла 3.164) ---

    inline void initializeAlignment(double T_sys, double T_reg, double T_rem, double T_nav,
                                    double nav_start_T,
                                    const Vec3& accel_meas, const Vec3& gyro_meas,
                                    double init_lat, double init_lon, double init_alt) {
        
        state.T_sys = T_sys;
        state.T_reg = T_reg;
        state.T_rem = T_rem;
        state.T_nav = nav_start_T;
        state.accel = accel_meas;
        state.gyro = gyro_meas;
        last_T_sys = nav_start_T;

        state.pos = {init_lat, init_lon, init_alt};
        double g_eff = calc_g_eff(state.pos.x, state.pos.z);

        // Вычисление начальных углов по акселерометрам (в покое)
        state.euler.y = asin(accel_meas.x / g_eff); // pitch (тангаж)
        state.euler.z = atan2(-accel_meas.z, accel_meas.y); // roll (крен)
        state.euler.x = 0.0; // yaw (рыскание) - не определяется по акселерометрам

        state.C = buildRotationMatrix(state.euler.x, state.euler.y, state.euler.z);
        state.vel = {0.0, 0.0, 0.0};
    }

    // --- Основной навигационный цикл ---

    inline void updateNavigation(double current_T_sys, const Vec3& accel_meas, const Vec3& gyro_meas) {
        if (current_T_sys < state.T_nav) return;

        state.T_sys = current_T_sys;
        state.accel = accel_meas;
        state.gyro = gyro_meas;

        double dt = state.T_sys - last_T_sys;
        if (dt <= 0.0) dt = 0.001;
        last_T_sys = state.T_sys;

        // --- 1. ОРИЕНТАЦИЯ ---

        // Получаем переносную угловую скорость географического трехгранника (omega_e)
        Vec3 omega_e = calc_omega_e();
        
        // Проектируем её на оси связанной системы (используя C^T)
        Mat3 C_T = transposeMat3(state.C);
        Vec3 omega_e_body = matMulVec(C_T, omega_e);

        // Вычитаем переносную скорость из показаний ДУС -> относительная угловая скорость (omega_r)
        Vec3 omega_r;
        omega_r.x = gyro_meas.x - omega_e_body.x;
        omega_r.y = gyro_meas.y - omega_e_body.y;
        omega_r.z = gyro_meas.z - omega_e_body.z;

        // Интегрирование уравнений Эйлера-Крылова (стр. 127, ф-ла 3.30)
        double yaw   = state.euler.x;
        double pitch = state.euler.y;
        double roll  = state.euler.z;

        double cos_p = cos(pitch);
        if (fabs(cos_p) < 1e-6) return; // Сингулярность при пикировании 90°

        double dot_yaw   = (omega_r.y * cos(roll) - omega_r.z * sin(roll)) / cos_p;
        double dot_pitch = omega_r.y * sin(roll) + omega_r.z * cos(roll);
        double dot_roll  = omega_r.x - tan(pitch) * (omega_r.y * cos(roll) - omega_r.z * sin(roll));

        state.euler.x += dot_yaw * dt;
        state.euler.y += dot_pitch * dt;
        state.euler.z += dot_roll * dt;

        state.C = buildRotationMatrix(state.euler.x, state.euler.y, state.euler.z);

        // --- 2. НАВИГАЦИЯ (СКОРОСТИ И КООРДИНАТЫ) ---

        // Пересчет ускорений в географическую систему (n_g = C * n)
        Vec3 n_g = matMulVec(state.C, accel_meas);

        double V_n = state.vel.x;
        double V_e = state.vel.z;
        double R = R_EQ + state.pos.z;
        double lat = state.pos.x;
        double g_eff = calc_g_eff(lat, state.pos.z);

        // Компенсация вредных ускорений (Кориолис)
        double a_k_x = (V_e / R) * (2 * U_EARTH * sin(lat) + (V_e / R) * tan(lat));
        double a_k_z = -(V_n / R) * (2 * U_EARTH * sin(lat) + (V_e / R) * tan(lat));

        // Истинные горизонтальные ускорения
        double dot_V_n = n_g.x - a_k_x;
        double dot_V_e = n_g.z - a_k_z;
        double dot_V_u = n_g.y - g_eff; // Вертикальное (для справки, мы его блокируем)

        // Интегрирование скоростей
        state.vel.x += dot_V_n * dt;
        state.vel.z += dot_V_e * dt;
        
        // Вертикальный канал заблокирован (наземная машина)
        state.vel.y = 0.0; 
        state.pos.z = 0.0;

        // Интегрирование координат (стр. 123, ф-ла 3.18)
        state.pos.x += (state.vel.x / R) * dt;                     // Широта
        state.pos.y += (state.vel.z / (R * cos(lat))) * dt;        // Долгота
    }

    // --- Получение текущего состояния ---

    inline NavState getCurrentState() const {
        return state;
    }
}; // <--- Обратите внимание на точку с запятой здесь!

#endif // SINS_ALGORITHM_H