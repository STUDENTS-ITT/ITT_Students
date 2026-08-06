#pragma once

// Построение траектории: свободное счисление БИНС и оценка его ошибки
// фильтром Калмана.
//
// Схема разомкнутая: поправки в счисление не вводятся, вектор x накапливает
// полную ошибку решения и прогнозируется через Φ каждый такт. В файл пишется
// решение за вычетом накопленной оценки. Обратная связь остаётся только по
// смещениям датчиков x[9..14] — они вычитаются из показаний ИМУ.

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

// Полное состояние счисления. Поля *_prev хранят значения предыдущего шага,
// необходимые для интегрирования методом трапеций.
struct NavState
{
    double lat = 0;
    double lon = 0;
    double alt = 0;
    ins::Attitude att;
    Vector V = Vector(3, 0.0);

    Vector V_dot_prev = Vector(3, 0.0);
    double lat_dot_prev = 0;
    double lon_dot_prev = 0;
    double alt_dot_prev = 0;
    ins::AttitudeRates rates_prev;

    double time_prev = 0;

    // оценка вектора ошибок и её ковариация
    Vector x = Vector(ins::KF_STATE, 0.0);
    Matrix P = Matrix(ins::KF_STATE * ins::KF_STATE, 0.0);
};

// Строка результатов: свободное решение БИНС за вычетом накопленной оценки
// ошибки, рядом — соответствующий отсчёт эталона.
inline data_io::NavRecord makeRecord(double time, const NavState &st, const SnsSample &ref,
                                     double hdg_true, double lat_bins)
{
    data_io::NavRecord rec;
    rec.time = time;

    rec.lon = st.lon - st.x[1];
    rec.lat = st.lat - st.x[0];
    rec.alt = st.alt - st.x[2];
    rec.heading = normolize_angle(st.att.heading - st.x[6]);
    rec.pitch = normolize_angle(st.att.pitch - st.x[7]);
    rec.roll = normolize_angle(st.att.roll - st.x[8]);
    rec.vn = st.V[0] - st.x[3];
    rec.vh = st.V[1] - st.x[4];
    rec.ve = st.V[2] - st.x[5];

    rec.lon_sns = ref.lon;
    rec.lat_sns = ref.lat;
    rec.alt_sns = ref.alt;
    rec.hdg_sns = ref.heading;
    rec.roll_sns = ref.roll;
    rec.pitch_sns = ref.pitch;
    rec.vn_sns = ref.vn;
    rec.vh_sns = ref.vh;
    rec.ve_sns = ref.ve;

    rec.hdg_true = hdg_true;
    rec.lat_bins = lat_bins;
    return rec;
}

// Один такт работы алгоритма по очередной строке imu.dat и парному ей
// отсчёту эталона.
inline void step(int i, const std::vector<double> &row, const SnsSample &ref,
                 NavState &st, data_io::NavLogger &log)
{
    const double time_s = ins::sampleTime(row);
    double dt = time_s - st.time_prev;
    if (dt <= 0.0)
        dt = IMU_PERIOD;

    const Matrix C = bodyToNavMatrix(st.att.heading, st.att.pitch, st.att.roll);

    // прогноз ошибок — каждый такт, на точке линеаризации начала такта
    ins::predict(dt, st.lat, C, st.x, st.P);

    // кажущееся ускорение в географической СК
    const Vector n_nav = bodyToNav(C, ins::accel(row, st.x));

    // скорости
    const Vector a_harm = harmfulAccel(st.lat, st.V, st.lat_dot_prev, st.lon_dot_prev);
    const Vector V_dot = velocityDot(n_nav, a_harm);
    const Vector V = integrateVelocity(V_dot, st.V, st.V_dot_prev, dt);

    // координаты
    const Position pos = integratePosition(st.lat, st.lon, st.alt, V,
                                           st.lat_dot_prev, st.lon_dot_prev, st.alt_dot_prev, dt);

    // ориентация
    const Vector w_nav = navAngularRate(st.lat, V, R_EARTH + st.alt);
    const Vector w_rel = ins::relativeRate(ins::gyro(row, st.x), w_nav, C);
    const ins::AttitudeRates rates = ins::eulerRates(w_rel, st.att.pitch, st.att.roll);
    const ins::Attitude att = ins::integrate(st.att, rates, st.rates_prev, dt);

    // счисление идёт свободно: поправки фильтра в него не вводятся
    st.V = V;
    st.lat = pos.lat;
    st.lon = pos.lon;
    st.alt = pos.alt;
    st.att = att;

    st.V_dot_prev = V_dot;
    st.lat_dot_prev = pos.lat_dot;
    st.lon_dot_prev = pos.lon_dot;
    st.alt_dot_prev = V[1];
    st.rates_prev = rates;
    st.time_prev = time_s;

    if (i % SNS_DECIMATION == 0)
    {
        const Vector bins = {st.lat, st.lon, st.alt,
                             st.V[0], st.V[1], st.V[2],
                             st.att.heading, st.att.pitch, st.att.roll};

        ins::correct(bins, ref.measurement(), st.x, st.P);

        const double hdg_true = ins::headingFromEarthRate(row[ins::IMU_COL_GYRO], st.att.pitch, st.lat);
        log.writeErrors(time_s, st.x);
        log.write(makeRecord(time_s, st, ref, hdg_true, bins[0]));
    }
}

} // namespace nav
