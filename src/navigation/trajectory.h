// trajectory.h — Основной цикл счисления БИНС с фильтром Калмана.
//
// Содержит:
//   - NavState — полное состояние навигации (координаты, скорости, углы,
//     смещения ДУС/акселя, вектор ошибок фильтра Калмана)
//   - makeResult / makeReference — формирование строк для записи
//   - step — один такт счисления: интегрирование + коррекция по СНС

#pragma once

#include <vector>

#include "../data_io/data_writer.h"
#include "../ins/attitude_calc.h"
#include "../ins/imu_processor.h"
#include "../ins/ins_filter.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "../utils/types.h"
#include "gps_processor.h"
#include "position_calc.h"

namespace nav
{

// Полное состояние навигационной системы.
struct NavState
{
    // Координаты и ориентация (интегрированные по ИМУ).
    double lat = 0;
    double lon = 0;
    double alt = 0;
    ins::Attitude att;
    Vector V = Vector(3, 0.0);  // [Vn, Vh, Ve]

    // Производные на предыдущем такте (для метода трапеций).
    Vector V_dot_prev = Vector(3, 0.0);
    double lat_dot_prev = 0;
    double lon_dot_prev = 0;
    double alt_dot_prev = 0;
    ins::AttitudeRates rates_prev;

    double time_prev = 0;

    // Смещения датчиков (корректируются фильтром Калмана).
    Vector ba = Vector(3, 0.0);  // смещение акселерометра
    Vector bg = Vector(3, 0.0);  // смещение гироскопа

    // 15-компонентный вектор ошибок и ковариационная матрица фильтра Калмана.
    Vector x = Vector(ins::KF_STATE, 0.0);
    Matrix P = Matrix(ins::KF_STATE * ins::KF_STATE, 0.0);

    // Статическое смещение гироскопа (для hdg_true).
    Vector bg_static = {0.0, 0.0, 0.0};
};

// Формирование строки результата: БИНС с вычитанием ошибок Калмана.
inline data_io::NavResult makeResult(double time, const NavState &st)
{
    data_io::NavResult r;
    r.time = time;
    r.lon = (st.lon - st.x[1]) * RAD_TO_DEG;
    r.lat = (st.lat - st.x[0]) * RAD_TO_DEG;
    r.alt = st.alt - st.x[2];
    r.heading = normalize_angle(st.att.heading - st.x[6]) * RAD_TO_DEG;
    r.pitch = normalize_angle(st.att.pitch - st.x[7]) * RAD_TO_DEG;
    r.roll = normalize_angle(st.att.roll - st.x[8]) * RAD_TO_DEG;
    r.vn = st.V[0] - st.x[3];
    r.vh = st.V[1] - st.x[4];
    r.ve = st.V[2] - st.x[5];
    return r;
}

// Формирование строки эталона СНС.
inline data_io::NavReference makeReference(double time, const SnsSample &ref)
{
    data_io::NavReference r;
    r.time = time;
    r.lon = ref.lon * RAD_TO_DEG;
    r.lat = ref.lat * RAD_TO_DEG;
    r.alt = ref.alt;
    r.heading = ref.heading * RAD_TO_DEG;
    r.pitch = ref.pitch * RAD_TO_DEG;
    r.roll = ref.roll * RAD_TO_DEG;
    r.vn = ref.vn;
    r.vh = ref.vh;
    r.ve = ref.ve;
    return r;
}

// Один такт счисления БИНС.
// На каждом шаге:
//   1. Интегрирование скоростей, координат, ориентации по данным ИМУ.
//   2. Раз в SNS_DECIMATION отсчётов (1 Гц) — коррекция по эталону СНС.
//   3. Запись результатов в файл.
inline void step(int i, const std::vector<double> &row, const SnsSample &ref,
                 NavState &st, data_io::NavLogger &log)
{
    const double time_s = ins::sampleTime(row);
    double dt = time_s - st.time_prev;
    if (dt <= 0.0)
        dt = IMU_PERIOD;

    // Матрица направляющих косинусов СК тела → навигационная СК.
    const Matrix C = bodyToNavMatrix(st.att.heading, st.att.pitch, st.att.roll);

    // Ускорение в навигационной СК: f_н = C_б^н · (f_б − ba)
    const Vector n_nav = bodyToNav(C, ins::accel(row, st.ba));

    // Предсказание фильтра Калмана: x = F·x, P = F·P·F^T + Q
    ins::predict(dt, st.lat, st.alt, C, n_nav, st.att, st.x, st.P);

    // Интегрирование скоростей (с вредными ускорениями).
    const Vector a_harm = harmfulAccel(st.lat, st.alt, st.V, st.lat_dot_prev, st.lon_dot_prev);
    const Vector V_dot = velocityDot(n_nav, a_harm);
    const Vector V = integrateVelocity(V_dot, st.V, st.V_dot_prev, dt);

    // Интегрирование координат.
    const Position pos = integratePosition(st.lat, st.lon, st.alt, V,
                                           st.lat_dot_prev, st.lon_dot_prev, st.alt_dot_prev, dt);

    // Интегрирование углов ориентации.
    const Vector w_nav = navAngularRate(st.lat, V, R_EARTH + st.alt);
    const Vector w_rel = ins::relativeRate(ins::gyro(row, st.bg), w_nav, C);
    const ins::AttitudeRates rates = ins::eulerRates(w_rel, st.att.pitch, st.att.roll);
    const ins::Attitude att = ins::integrate(st.att, rates, st.rates_prev, dt);

    // Обновление состояния.
    st.V = V;
    st.lat = pos.lat;
    st.lon = pos.lon;
    st.alt = pos.alt;
    st.att = att;

    // Сохранение производных для следующего такта.
    st.V_dot_prev = V_dot;
    st.lat_dot_prev = pos.lat_dot;
    st.lon_dot_prev = pos.lon_dot;
    st.alt_dot_prev = V[1];
    st.rates_prev = rates;
    st.time_prev = time_s;

    // Коррекция по СНС (раз в SNS_DECIMATION = 200 отсчётов, 1 Гц).
    if (i % SNS_DECIMATION == 0)
    {
        // Текущее решение БИНС (9 компонент) для вектора инновации.
        const Vector bins = {st.lat, st.lon, st.alt,
                             st.V[0], st.V[1], st.V[2],
                             st.att.heading, st.att.pitch, st.att.roll};

        // Эталон СНС: из ref берем только координаты, скорости и курс.
        // Тангаж и крен берём из текущего состояния БИНС (инновация = 0 → коррекции нет).
        const Vector sns = {ref.lat, ref.lon, ref.alt,
                            ref.vn, ref.vh, ref.ve,
                            ref.heading, st.att.pitch, st.att.roll};

        // Коррекция: x += K·(bins − sns), P = (I − K·H)·P
        ins::correct(bins, sns, st.x, st.P);

        // Запись ошибок фильтра (до обнуления).
        log.writeErrors(time_s, st.x);

        // Применение коррекций фильтра к состоянию.
        st.lat -= st.x[0];
        st.lon -= st.x[1];
        st.alt -= st.x[2];
        st.V[0] -= st.x[3];
        st.V[1] -= st.x[4];
        st.V[2] -= st.x[5];
        st.att.heading = normalize_angle(st.att.heading - st.x[6]);
        st.att.pitch = normalize_angle(st.att.pitch - st.x[7]);
        st.att.roll = normalize_angle(st.att.roll - st.x[8]);
        for (int k = 0; k < 3; k++)
        {
            st.ba[k] += st.x[9 + k];
            st.bg[k] += st.x[12 + k];
        }
        // Обнуление вектора ошибок (поправки применены).
        for (int k = 0; k < ins::KF_STATE; k++)
            st.x[k] = 0.0;

        // Запись результата и эталона.
        log.writeResult(makeResult(time_s, st));
        log.writeReference(makeReference(time_s, ref));
    }
}

} // namespace nav
