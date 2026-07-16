#ifndef SINS_ALGORITHM_H
#define SINS_ALGORITHM_H

// =============================================================================
// SINS_Algorithm — алгоритм бесплатформенной инерциальной навигационной системы (БИНС)
//
// Реализация навигационного алгоритма с углами Эйлера–Крылова для наземных объектов.
// Основные уравнения: см. "Теория_БИНС_с_углами_Эйлера-Крылова.md"
//
// Системы координат:
//   Связанная (OXYZ):    OX — нос, OY — вверх, OZ — правый борт
//   Географическая (NUE): X — Север, Y — Вверх, Z — Восток
//
// Допущения:
//   1. Вертикальный канал заблокирован (V_up = 0, Alt = 0) — наземный объект
//   2. Начальная скорость = 0 (объект в покое при выставке)
//   3. Начальное рыскание = 0 (нет датчика курса)
//   4. Метод интегрирования — Эйлер первого порядка
// =============================================================================

#include "types.h"
#include "math_utils.h"

class SINS_Algorithm {
private:
    NavState state{};       // Текущее состояние навигации (координаты, скорость, ориентация)
    double last_T_sys = 0.0; // Время предыдущего шага интегрирования (для вычисления dt)

    // =========================================================================
    // Вспомогательные методы
    // =========================================================================

    /**
     * Расчет эффективного ускорения свободного падения (g_eff).
     *
     * Формула (из теории, ф-лы 3.12–3.13):
     *   g_eff = g_gravitation − g_centrifugal = GM/R² − Ω²·R·cos²(φ)
     *
     * Где:
     *   GM = 3.986004418×10¹⁴ м³/с² — гравитационный параметр Земли
     *   R  = R_eq + alt              — расстояние от центра Земли (м)
     *   Ω  = 7.292115×10⁻⁵ рад/с   — угловая скорость вращения Земли
     *   φ  — широта (рад)
     *
     * @param lat  текущая широта в радианах
     * @param alt  текущая высота в метрах
     * @return     эффективное g в м/с²
     */
    inline double calc_g_eff(double lat, double alt) {
        double R = R_EQ + alt;
        double g_grav = GM_EARTH / (R * R);
        double g_centr = (U_EARTH * U_EARTH) * R * cos(lat) * cos(lat);
        return g_grav - g_centr;
    }

    /**
     * Проекции переносной угловой скорости географического трёхгранника.
     *
     * Формула 3.6 из теории (в системе координат NUE кода):
     *   ω_north = Ω·cos(φ) + V_east/R
     *   ω_up    = Ω·sin(φ) + (V_east/R)·tg(φ)
     *   ω_east  = −V_north/R
     *
     * Обусловлена вращением Земли (слагаемые с Ω) и движением объекта
     * вдоль поверхности Земли (слагаемые с V/R).
     *
     * @return Vec3 {ω_north, ω_up, ω_east} в рад/с
     */
    inline Vec3 calc_omega_e() {
        double lat = state.pos.x;
        double V_n = state.vel.x;
        double V_e = state.vel.z;
        double R = R_EQ + state.pos.z;

        return {
            U_EARTH * cos(lat) + V_e / R,
            U_EARTH * sin(lat) + (V_e / R) * tan(lat),
            -V_n / R
        };
    }

public:
    // =========================================================================
    // Начальная выставка (alignment)
    // =========================================================================

    /**
     * Инициализация БИНС по данным выставки.
     *
     * Объект находится в покое. Акселерометры измеряют вектор g в связанной системе.
     *
     * Определение углов:
     *   θ₀ = arcsin(Ax / g_eff)    — тангаж
     *   γ₀ = atan2(−Az, Ay)        — крен
     *   ψ₀ = 0                     — рыскание (не определяется по акселерометрам)
     *
     * @param T_sys         текущее время системы (с)
     * @param T_reg         время регулировки (с)
     * @param T_rem         время до конца выставки (с)
     * @param T_nav         время перехода в навигацию (с)
     * @param nav_start_T   T_sys на границе выставка→навигация
     * @param accel_meas    усреднённые показания акселерометров {Ax, Ay, Az} м/с²
     * @param gyro_meas     усреднённые показания гироскопов {Wx, Wy, Wz} рад/с
     * @param init_lat      начальная широта (рад)
     * @param init_lon      начальная долгота (рад)
     * @param init_alt      начальная высота (м)
     */
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

        state.euler.y = asin(accel_meas.x / g_eff);           // pitch (тангаж)
        state.euler.z = atan2(-accel_meas.z, accel_meas.y);   // roll (крен)
        state.euler.x = 0.0;                                   // yaw (рыскание = 0)

        state.C = buildRotationMatrix(state.euler.x, state.euler.y, state.euler.z);
        state.vel = {0.0, 0.0, 0.0};
    }

    // =========================================================================
    // Основной навигационный цикл
    // =========================================================================

    /**
     * Один шаг навигационного алгоритма БИНС.
     *
     * Порядок:
     *   1. Ориентация: вычисление ω_e → ω_relative → интегрирование углов (ф-лы 3.30)
     *   2. Навигация: пересчёт ускорений → компенсация → интегрирование V и координат
     *      (ф-лы 3.15–3.18)
     *
     * @param current_T_sys  текущее время (с)
     * @param accel_meas     акселерометр {Ax, Ay, Az} м/с² (связанная система)
     * @param gyro_meas      гироскоп {Wx, Wy, Wz} рад/с (связанная система)
     */
    inline void updateNavigation(double current_T_sys, const Vec3& accel_meas, const Vec3& gyro_meas) {
        if (current_T_sys < state.T_nav) return;

        state.T_sys = current_T_sys;
        state.accel = accel_meas;
        state.gyro = gyro_meas;

        double dt = state.T_sys - last_T_sys;
        if (dt <= 0.0) dt = 0.001;
        last_T_sys = state.T_sys;

        // =====================================================================
        // БЛОК 1: ОРИЕНТАЦИЯ
        // =====================================================================

        // Переносная угловая скорость географического трёхгранника (ф-ла 3.6)
        Vec3 omega_e = calc_omega_e();
        
        // Пересчёт ω_e из географической в связанную систему: ω_e_body = C · ω_e
        Vec3 omega_e_body = matMulVec(state.C, omega_e);

        // Относительная угловая скорость: ω_r = ω_gyro − ω_e_body
        Vec3 omega_r;
        omega_r.x = gyro_meas.x - omega_e_body.x;
        omega_r.y = gyro_meas.y - omega_e_body.y;
        omega_r.z = gyro_meas.z - omega_e_body.z;

        // Уравнения Эйлера–Крылова (ф-лы 3.30)
        double yaw   = state.euler.x;
        double pitch = state.euler.y;
        double roll  = state.euler.z;

        double cos_p = cos(pitch);
        if (fabs(cos_p) < 1e-6) return; // Сингулярность при θ ≈ ±90°

        double dot_yaw   = (omega_r.y * cos(roll) - omega_r.z * sin(roll)) / cos_p;
        double dot_pitch = omega_r.y * sin(roll) + omega_r.z * cos(roll);
        double dot_roll  = omega_r.x - tan(pitch) * (omega_r.y * cos(roll) - omega_r.z * sin(roll));

        state.euler.x += dot_yaw * dt;
        state.euler.y += dot_pitch * dt;
        state.euler.z += dot_roll * dt;

        state.C = buildRotationMatrix(state.euler.x, state.euler.y, state.euler.z);

        // =====================================================================
        // БЛОК 2: НАВИГАЦИЯ
        // =====================================================================

        // Пересчёт ускорений из связанной в географическую: n_g = C · a_body
        Vec3 n_g = matMulVec(state.C, accel_meas);

        double V_n = state.vel.x;
        double V_e = state.vel.z;
        double R = R_EQ + state.pos.z;
        double lat = state.pos.x;
        double g_eff = calc_g_eff(lat, state.pos.z);

        // Компенсация вредных ускорений (ф-ла 3.15, при V_up = 0):
        //
        // Северный канал:
        //   a_k_n = V_e²/R · tg(φ) + 2·U·V_e·sin(φ)
        //   (центростремительное от параллели + Кориолис, проекция на Север)
        //
        // Восточный канал:
        //   a_k_e = −V_e²/R − V_n²/R − 2·U·V_e·cos(φ)
        //   (центростремительные + Кориолис, проекция на Восток)
        double a_k_n = (V_e * V_e / R) * tan(lat) + 2.0 * U_EARTH * V_e * sin(lat);
        double a_k_e = -(V_e * V_e / R) - (V_n * V_n / R) - 2.0 * U_EARTH * V_e * cos(lat);

        // Ускорения относительного движения (ф-ла 3.16)
        double dot_V_n = n_g.x - a_k_n;
        double dot_V_e = n_g.z - a_k_e;

        // Интегрирование горизонтальных скоростей
        state.vel.x += dot_V_n * dt;
        state.vel.z += dot_V_e * dt;
        
        // Вертикальный канал заблокирован (наземный объект)
        state.vel.y = 0.0;
        state.pos.z = 0.0;

        // Интегрирование координат (ф-лы 3.18)
        state.pos.x += (state.vel.x / R) * dt;                     // Широта
        state.pos.y += (state.vel.z / (R * cos(lat))) * dt;        // Долгота
    }

    inline NavState getCurrentState() const {
        return state;
    }
};

#endif // SINS_ALGORITHM_H
