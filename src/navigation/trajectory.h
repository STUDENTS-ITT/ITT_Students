// trajectory.h — Основной цикл счисления БИНС с фильтром Калмана.
//
// Содержит:
//   - NavState — полное состояние навигации (координаты, скорости, углы,
//     смещения ДУС/акселя, вектор ошибок фильтра Калмана)
//   - makeResult / makeReference — формирование строк для записи
//   - step — один такт счисления: интегрирование + коррекция по СНС

#pragma once

#include <vector>

#include "data_io/data_writer.h"
#include "ins/attitude_calc.h"
#include "ins/imu_processor.h"
#include "ins/ins_filter.h"
#include "math_lib/transformations.h"
#include "utils/constants.h"
#include "utils/types.h"
#include "gnss_guard.h"
#include "gps_processor.h"
#include "maneuver_tilt.h"
#include "position_calc.h"
#include "aligner.hpp"

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

    // Защита СНС (уровни 1-2, docs/gnss_guard_design.md). Хранит
    // последний принятый отсчёт, счётчики и статус текущего кадра.
    SnsGuard sns_guard;

    // 15-компонентный вектор ошибок и ковариационная матрица фильтра Калмана.
    Vector x = Vector(ins::KF_STATE, 0.0);
    Matrix P = Matrix(ins::KF_STATE * ins::KF_STATE, 0.0);

    // Статическое смещение гироскопа (для hdg_true).
    Vector bg_static = {0.0, 0.0, 0.0};

    // Время последнего |ω| > gyro_off (пауза перед акселем).
    double tilt_maneuver_time = -1e9;

    // Этап 1 (R1): поле TiltTrimState tilt_trim УДАЛЕНО — скрытый оцениватель
    // дрейфа тангажа/крена вырезан из шага счисления. См. комментарий в
    // maneuver_tilt.h.
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

// Применение коррекции KF только по pitch/roll (после correctAccel).
//
// Обслуживает аксель-апдейт (200 Гц): инъекция ошибок тангажа/крена и
// смещений акселя/гироскопа.  Имя сохранено исторически (контур «аксель как
// горизонт»), смысл тот же: приведение номинала к отсчёту акселерометра.
inline void applyTiltKalmanCorrections(NavState &st)
{
    st.att.pitch = normalize_angle(st.att.pitch - st.x[7]);
    st.att.roll = normalize_angle(st.att.roll - st.x[8]);
    for (int k = 0; k < 3; k++)
    {
        st.ba[k] += st.x[9 + k];
        st.bg[k] += st.x[12 + k];
    }
    for (int k = 7; k < ins::KF_STATE; k++)
        st.x[k] = 0.0;

    // ESKF-reset ковариации: G·P·Gᵀ (G=I для аддитивных групп),
    // плюс симметризация и страховка диагонали.
    ins::apply_reset_covariance(st.P);
}

// Полное применение коррекции KF (кадр СНС, 1 Гц).
inline void applyKalmanCorrections(NavState &st)
{
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
    for (int k = 0; k < ins::KF_STATE; k++)
        st.x[k] = 0.0;

    // ESKF-reset ковариации: G·P·Gᵀ (G=I для аддитивных групп),
    // плюс симметризация и страховка диагонали.
    ins::apply_reset_covariance(st.P);
}

// Один такт счисления БИНС.
// На каждом шаге:
//   1. Интегрирование скоростей, координат, ориентации по данным ИМУ (200 Гц).
//   2. Коррекция pitch/roll по акселю как горизонту (200 Гц, correctAccel).
//   3. Если do_correction — коррекция по СНС (1 Гц): координаты, скорости, курс.
//   4. Запись результатов в приёмник sink.
inline void step(const std::vector<double> &row, const SnsSample &ref,
                 NavState &st, data_io::NavSink &sink, bool do_correction,
                 bool has_heading_ref)
{
    const double time_s = ins::sampleTime(row);
    double dt = time_s - st.time_prev;
    if (dt <= 0.0)
        dt = IMU_PERIOD;

    // Матрица направляющих косинусов СК тела → навигационная СК.
    const Matrix C = bodyToNavMatrix(st.att.heading, st.att.pitch, st.att.roll);

    // Ускорение тела (компенсированное смещением ba) и в навигационной СК:
    // f_н = C_б^н · (f_б − ba).
    const Vector f_body = ins::accel(row, st.ba);
    const Vector n_nav = bodyToNav(C, f_body);

    // Интегрирование скоростей (с вредными ускорениями).
    const Vector a_harm = harmfulAccel(st.lat, st.alt, st.V, st.lat_dot_prev, st.lon_dot_prev);
    const Vector V_dot = velocityDot(n_nav, a_harm);
    const Vector V = integrateVelocity(V_dot, st.V, st.V_dot_prev, dt);

    // Интегрирование координат.
    const Position pos = integratePosition(st.lat, st.lon, st.alt, V,
                                           st.lat_dot_prev, st.lon_dot_prev, st.alt_dot_prev, dt);

    const Vector V_dot_body = navToBody(C, V_dot);
    const Vector gyro_raw = ins::gyro(row, st.bg);
    const double gyro_mag = sqrt(gyro_raw[0] * gyro_raw[0] +
                                 gyro_raw[1] * gyro_raw[1] +
                                 gyro_raw[2] * gyro_raw[2]);

    // Пороги сведения — умолчания TiltManeuverPolicy: консольная версия
    // не читает настройки из файла (тюнера и пресетов нет).
    const TiltManeuverPolicy TILT_POL{};

    if (gyro_mag > TILT_POL.gyro_off_rad_s || isBrakingManeuver(V_dot_body, TILT_POL))
        st.tilt_maneuver_time = time_s;

    // Интегрирование углов ориентации (гиро).
    const Vector w_nav = navAngularRate(st.lat, V, R_EARTH + st.alt);
    const Vector w_rel = ins::relativeRate(gyro_raw, w_nav, C);
    ins::AttitudeRates rates = ins::eulerRates(w_rel, st.att.pitch, st.att.roll);
    // Этап 1 (R1): вычет трима из скоростей Эйлера (applyTiltRateBias) УДАЛЁН.
    // Это был скрытый оцениватель дрейфа вне фильтра: конкурировал с
    // x[12..14], не имел ковариации и не выводился ни в телеметрию, ни в
    // runs.db. Дрейф уходит через номинал bg, а limitTiltDivergence остаётся
    // единственным ограничителем интегрирования.
    limitTiltDivergence(rates, st.att, f_body, gyro_mag, V_dot_body, TILT_POL);
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

    // Предсказание фильтра Калмана (П5): после интегрирования номинала за такт
    // — по свежему состоянию. Раньше predict шёл по состоянию прошлого такта
    // (до интеграции), что давало систематический сдвиг на dt между predict и
    // коррекцией, где инновация считается против только что проинтегрированного
    // номинала.
    const Matrix C_cur = bodyToNavMatrix(st.att.heading, st.att.pitch, st.att.roll);
    const Vector n_nav_cur = bodyToNav(C_cur, f_body);
    ins::predict(dt, st.lat, st.alt, C_cur, n_nav_cur, st.att, st.x, st.P);
    // Аксель-апдейт (Этап 2, R2): согласованное измерение удельной силы.
    //
    //   h(x̂) = Ĉ_б^нᵀ·a_вред̂ + b̂a   — модель (V̇ ≈ 0),
    //   y     = f_б^изм               — сырой отсчёт акселя (со смещением),
    //   z     = h − y.
    //
    // Слагаемое V̇ в модели не берётся из БИНС: там V̇ сам построен по f_б,
    // и подстановка обращает z в тождественный нуль (вывод — §5
    // docs/bias_estimation_stages.md).  Отброшенное V̇ ограничивается гейтом,
    // а в само измерение идут только оси X и Z тела — перпендикулярные
    // тяжести, то есть горизонт (см. H_accel_matrix).  Ось Y (модуль
    // удельной силы) не используется: её единственный сигнал — b̂a_Y, и
    // неучтённое V̇_Y уходит туда целиком.
    const Vector f_meas = vector_sum(f_body, st.ba);  // f_body уже без b̂a
    const Vector z_accel = vector_diff(
        vector_sum(navToBody(C_cur, a_harm), st.ba), f_meas);
    const double z_accel_mag = sqrt(z_accel[0] * z_accel[0] +
                                    z_accel[1] * z_accel[1] +
                                    z_accel[2] * z_accel[2]);

    if (shouldApplyAccelHorizon(time_s, st.tilt_maneuver_time, gyro_mag,
                                z_accel_mag, V_dot_body, TILT_POL))
    {
        ins::correctAccel(C_cur, a_harm, st.att, z_accel,
                          ins::kalman_cfg.sig_a * sqrt(dt), st.x, st.P);
        applyTiltKalmanCorrections(st);
    }
    else
        clearTiltKalmanErrors(st.x);

    // Защита СНС (уровни 1-2, docs/gnss_guard_design.md): самодостаточная
    // валидация отсчёта и детектор молчания. Наблюдение ведётся на каждом
    // такте, независимо от того, разрешена ли коррекция (принцип 3, черновик).
    const bool sns_ok = st.sns_guard.update(ref, time_s, ins::kalman_cfg);
    const bool manual = st.sns_guard.is_new_frame && !do_correction;
    const bool apply_correction = do_correction && sns_ok;

    // Коррекция по СНС (1 Гц, только при условии, что кадр допустим защитой).
    if (apply_correction)
    {
        const Vector bins = {st.lat, st.lon, st.alt,
                             st.V[0], st.V[1], st.V[2],
                             st.att.heading, st.att.pitch, st.att.roll};

        // Координаты, скорости, курс — из СНС; pitch/roll уже на ИМУ-частоте.
        // Курс корректируется только при наличии angle.dat (эталона курса).
        // Иначе ref.heading == 0 → не даём фильтру тянуть истинный курс к нулю,
        // а замер курса считаем отсутствующим (инновация = 0).
        const Vector sns = {ref.lat, ref.lon, ref.alt,
                            ref.vn, ref.vh, ref.ve,
                            has_heading_ref ? ref.heading : st.att.heading,
                            st.att.pitch, st.att.roll};

        // Этап 4 плана (R4): без эталона курса строка курса фиктивна, но её
        // обновление при обычном R сжимало бы P[6][6] до σ_hdg, т.е. «доказы-
        // вало» бы достоверность курса, которой нет: P0(курс) приходился бы на
        // один кадр, и состоянием дальше нельзя было бы двигать. R раздуваем
        // так же, как для тангажа/крена, — курс остаётся с тем неопределённым
        // приором, что дала выставка.
        const double SIG_HDG = has_heading_ref ? ins::kalman_cfg.sig_hdg
                                                : 1e6 * DEG_TO_RAD;
        const double SIG_TILT_IGNORE = 1e6 * DEG_TO_RAD;

        // Диагностика согласованности (NIS) считается только если её просит
        // приёмник результатов: иначе лишних обращений матриц нет.
        const bool want_consistency = sink.wantsConsistency();
        ins::CorrectionStats stats;
        ins::correct(bins, sns, st.x, st.P, SIG_HDG, SIG_TILT_IGNORE, SIG_TILT_IGNORE,
                     want_consistency ? &stats : nullptr);
        sink.writeErrors(time_s, st.x);

        // Строка согласованности — только на кадрах фактической коррекции
        // (SNS под 1 Гц) и только по запросу приёмника.
        if (want_consistency && stats.valid)
        {
            data_io::ConsistencyRecord crec;
            crec.time = time_s;
            crec.nis = stats.nis;
            crec.dof = stats.dof;
            sink.writeConsistency(crec);
        }

        applyKalmanCorrections(st);
        sink.writeReference(makeReference(time_s, ref));
    }

    // Лог защиты СНС (уровни 1-2). Записывается даже если коррекция не
    // применена — соблюдаем наблюдаемость (принцип 3, черновик).
    if (ins::kalman_cfg.gnss_guard_log > 0.5)
    {
        data_io::SnsGuardRecord grecord;
        grecord.time = time_s;
        grecord.status = st.sns_guard.status;
        grecord.reject = st.sns_guard.reject;
        grecord.new_frame = st.sns_guard.is_new_frame;
        grecord.applied = apply_correction;
        grecord.manual = manual;
        grecord.dt = st.sns_guard.last_dt;
        grecord.dpos = st.sns_guard.last_dpos;
        grecord.dvel = st.sns_guard.last_dvel;
        grecord.accepted = st.sns_guard.accepted_frames;
        grecord.rejected = st.sns_guard.rejected_frames;
        sink.writeGuard(grecord);
    }

    // Запись результата на каждом такте (200 Гц).
    sink.writeResult(makeResult(time_s, st));
}

} // namespace nav
