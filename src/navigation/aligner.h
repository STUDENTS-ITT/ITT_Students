// aligner.h — Формирование начального состояния навигации.
//
// Содержит:
//   - initialCovariance — начальная ковариационная матрица фильтра Калмана P₀
//   - referenceAttitude — начальная ориентация из первого отсчёта СНС
//   - initialAlignment — формирование NavState: координаты из СНС,
//     углы — либо из СНС, либо из автономной выставки (перегрузка по yaw, pitch, roll)

#pragma once

#include "ins/attitude_calc.h"
#include "ins/ins_filter.h"
#include "math_lib/matrix_ops.h"
#include "utils/constants.h"
#include "averaging.h"
#include "gps_processor.h"
#include "trajectory.h"

namespace nav
{

// Начальная ковариационная матрица P₀.
// C11: P0 больше не захардкожен — калибруется по настроенным σ из kalman_cfg:
//   pos = 2·σ_pos, alt = 2·σ_h, а курс = σ_bg / Ω_h (неснимаемое смещение гиро
//   искажает проекцию скорости вращения Земли, Ω_h = ω_E·cos(lat)), т.е. курс
//   определяется точностью курсовой выставки, а не свободным масштабом.
//
// Этап 2 (R3): диагональные σ смещений приведены к паспортным значениям
// наборов — были занижены на порядок (ba: 0.31 мг против 2…8 мг,
// bg: 20.6 °/ч против 20…150 °/ч). Занижённый приоритет не давал фильтру
// сдвинуть оценку смещения, а из-за этого «застревали» и углы.
//
// Туда же добавлена физическая корреляция «наклон ↔ горизонтальное смещение
// акселя», которую вносит выставка. Выравнивание по акселю кладёт в тангаж и
// крен не истинный наклон, а сумму наклона и ba_гор/g (аксель видит тяжесть
// плюс смещение), поэтому ошибки углов и смещений коррелированы:
//   x[7] = тангаж − истина = +ba_X/g, x[9] = ba_X → P[7][9] = +σ_ba²/g,
//   x[8] = крен   − истина = −ba_Z/g, x[11] = ba_Z → P[8][11] = −σ_ba²/g.
// По отдельности угол и смещение аксель-апдейтом неразличимы (в H они дают
// одну и ту же ось тела); эта связь — то, что позволяет перенести наблюдение
// из угла в смещение.
//
// Явная конфигурация передаётся аргументом (для экспорта/документации).
//
// sig_heading — σ начального курса, рад. План (Этап 4, R5): при курсе из
// эталона СНС (angle.dat) это шум эталона σ_hdg, а не неопределённость
// вращения Земли; значение <= 0 означает «курс из gyro-compassing» и
// оставляет прежнюю формулу σ_bg/Ω_h (честный приор для выставки, см.
// комментарий выше). Фактический шум эталона оценивается по данным в
// simulation.cpp и передаётся как max(σ_hdg, σ̂_шума).
inline Matrix initialCovariance(const ins::KalmanConfig &cfg, double lat_rad,
                                double sig_heading = -1.0)
{
    // Паспортные неопределённости смещений на момент выставки.
    const double sig_ba = 5.0e-2;  // σ(ba) = 5 мг (наборы 2…8 мг)
    const double sig_bg = 5.0e-4;  // σ(bg) = 103 °/ч (наборы 20…150 °/ч)

    // Курс: либо заданный (эталон СНС), либо σ_bg/Ω_h под выставку по
    // вращению Земли. Ω_h = ω_E·cos(lat); 1e-10 — защита на полюсе.
    const double sig_hdg0 = (sig_heading > 0.0)
                                ? sig_heading
                                : cfg.sig_bg / fmax(fabs(cos(lat_rad)) * U_EARTH, 1e-10);

    const double sigma[ins::KF_STATE] = {
        2.0 * cfg.sig_pos, 2.0 * cfg.sig_pos, 2.0 * cfg.sig_h,
        2.5, 2.5, 2.5,
        sig_hdg0, 1.0 * DEG_TO_RAD, 1.0 * DEG_TO_RAD,
        sig_ba, sig_ba, sig_ba,
        sig_bg, sig_bg, sig_bg};

    const double pdiag[ins::KF_STATE] = {
        sigma[0] * sigma[0], sigma[1] * sigma[1], sigma[2] * sigma[2],
        sigma[3] * sigma[3], sigma[4] * sigma[4], sigma[5] * sigma[5],
        sigma[6] * sigma[6], sigma[7] * sigma[7], sigma[8] * sigma[8],
        sigma[9] * sigma[9], sigma[10] * sigma[10], sigma[11] * sigma[11],
        sigma[12] * sigma[12], sigma[13] * sigma[13], sigma[14] * sigma[14]};

    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = pdiag[i];

    // Корреляции «наклон ↔ ba_гор» (вывод в комментарии выше). g берём на
    // уровне моря: высота в этот момент неизвестна точно, а её вклад в g —
    // доли процента.
    const double g = normalGravity(lat_rad, 0.0);
    at(P0, 7, 9, ins::KF_STATE) = pdiag[9] / g;
    at(P0, 9, 7, ins::KF_STATE) = pdiag[9] / g;
    at(P0, 8, 11, ins::KF_STATE) = -pdiag[11] / g;
    at(P0, 11, 8, ins::KF_STATE) = -pdiag[11] / g;

    return P0;
}

// P₀ по текущей (глобальной) конфигурации — поведение без изменений
// (сигнальный курс из gyro-compassing).
inline Matrix initialCovariance(double lat_rad)
{
    return initialCovariance(ins::kalman_cfg, lat_rad);
}

inline ins::Attitude referenceAttitude(const SnsSample &first)
{
    ins::Attitude att;
    att.heading = first.heading;
    att.roll = first.roll;
    att.pitch = first.pitch;
    return att;
}

inline NavState initialAlignment(const SnsSample &first, const ins::Attitude &att,
                                 double sig_heading = -1.0)
{
    NavState st;

    st.att = att;
    st.lat = first.lat;
    st.lon = first.lon;
    st.alt = first.alt;
    st.V = {first.vn, first.vh, first.ve};
    st.P = initialCovariance(ins::kalman_cfg, st.lat, sig_heading);
    return st;
}

inline NavState initialAlignment(const SnsSample &first, double yaw, double pitch, double roll)
{
    ins::Attitude att;
    att.heading = yaw;
    att.pitch = pitch;
    att.roll = roll;
    return initialAlignment(first, att);
}

// ba_init — начальное смещение акселерометра из выставки;
// sig_heading — σ начального курса, рад (<= 0 — формула выставки σ_bg/Ω_h).
// Этап 4 плана (R5): при курсе из эталона СНС передаётся его шум.
inline NavState initialAlignment(double lat_rad, double lon_rad, double alt,
                                 double yaw, double pitch, double roll,
                                 const Vector &ba_init = {0.0, 0.0, 0.0},
                                 double sig_heading = -1.0)
{
    NavState st;

    st.att.heading = yaw;
    st.att.pitch = pitch;
    st.att.roll = roll;
    st.lat = lat_rad;
    st.lon = lon_rad;
    st.alt = alt;
    st.V = {0.0, 0.0, 0.0};
    st.ba = ba_init;
    st.bg = {0.0, 0.0, 0.0};
    st.P = initialCovariance(ins::kalman_cfg, st.lat, sig_heading);
    return st;
}

// ----------------------------------------------------------------------
// Быстрая выставка (окно 1 с): P₀ и x₀ из итогов AveragingWindow
// ----------------------------------------------------------------------
//
// Отличия от initialCovariance выше (docs/fast_alignment_plan.md, §4.4):
//
//   1. Масштаб σ берётся по источнику номинала. Номинал позиции/скорости/
//      курса — последний отсчёт СНС, поэтому в P₀ идёт σ̂_отсчёта; номинал
//      наклона и смещений — средние окна, поэтому там σ̂_среднего.
//
//   2. Полы (паспортные значения) обязательны: в наборе square_noisefree
//      все максимумы отклонения нулевые, и без полов P₀ = 0 ⇒ K = 0 ⇒
//      фильтр перестаёт корректироваться.
//
//   3. Наклон получает структурную составляющую ba_гор/g: выставка по
//      акселю не отличает истинный наклон от горизонтального смещения
//      акселя. Поэтому σ(наклон) ≥ 2·σ_ba_гор/g (пол 0.573° при 5 мг), а
//      корреляции «наклон ↔ ba_гор» переносят наблюдение между ними.
//
//   4. Вертикальное смещение акселя наблюдаемо по модулю f̄_Y, поэтому
//      σ(ba_Y) — шумовая оценка окна с полом σ_ba/10. Горизонтальный дрейф
//      гиро виден по уходу от вращения Земли (пол σ_bg/5), вертикальный за
//      1 с не наблюдаем — паспортное σ_bg.
//
//   5. sigma(v) = 2.5 из initialCovariance заменена на СКО шума СНС: при
//      окне 1 с курс и координаты уже известны на уровне шума эталона.
//
// sig_heading > 0 — σ начального курса из эталона (angle.dat); <= 0 —
// gyro-compassing, σ = σ_bg/Ω_h (как в initialCovariance).
inline Matrix initialCovarianceFromAveraging(const ins::KalmanConfig &cfg,
                                             const AveragingResult &avg,
                                             double lat_rad, double sig_heading)
{
    // Окно не признано стоянкой: статистики окна недостоверны, берём
    // прежний паспортный приор (консервативно, с предупреждением у вызова).
    if (!avg.rest_ok)
    {
        return initialCovariance(cfg, lat_rad, sig_heading);
    }

    const double sig_ba_hor = 5.0e-2;   // паспорт: 5 мг, м/с²
    const double sig_bg_pass = 5.0e-4;  // паспорт: 103 °/ч, рад/с

    const double g = normalGravity(lat_rad, 0.0);

    // 1/cos(lat) — как в Rj_matrix: ограничение у полюсов 0.05.
    double cos_lat = fabs(cos(lat_rad));
    if (cos_lat < 0.05) cos_lat = 0.05;

    // Позиция: номинал — последний отсчёт ⇒ σ̂_отсчёта; долгота — в тех же
    // метрах, что и широта, поэтому в радианах делится ещё на cos(lat).
    const double sig_pos_lat = fmax(avg.sig_pos_sample / R_EARTH, cfg.sig_pos);
    const double sig_pos_lon = fmax(avg.sig_pos_lon_sample / (R_EARTH * cos_lat),
                                    cfg.sig_pos / cos_lat);
    const double sig_h = fmax(avg.sig_alt_sample, cfg.sig_h);
    const double sig_v = fmax(avg.sig_v_sample, cfg.sig_v);
    const double sig_hdg = (sig_heading > 0.0)
                               ? sig_heading
                               : cfg.sig_bg / (cos_lat * U_EARTH);

    // Наклон: шумовая часть (σ_ā_гор/g) + структурная (σ_ba_гор/g), пол
    // 2·σ_ba_гор/g. Горизонтальные оси тела — X (индекс 0) и Z (индекс 2).
    const double sig_a_hor = fmax(avg.sig_a_mean[0], avg.sig_a_mean[2]);
    const double sig_tilt = fmax(sqrt(sig_ba_hor * sig_ba_hor
                                      + sig_a_hor * sig_a_hor) / g,
                                 2.0 * sig_ba_hor / g);

    // Вертикальное смещение акселя: шум среднего, пол σ_ba/10.
    const double sig_ba_vert = fmax(avg.sig_a_mean[1], sig_ba_hor / 10.0);

    // Дрейф гиро: горизонтальные оси — оценка окна с полом σ_bg/5,
    // вертикальная — паспорт (за 1 с не наблюдаема).
    const double sig_w_hor = fmax(avg.sig_w_mean[0], avg.sig_w_mean[2]);
    const double sig_bg_hor = fmax(sig_w_hor, sig_bg_pass / 5.0);
    const double sig_bg_vert = sig_bg_pass;

    const double sigma[ins::KF_STATE] = {
        sig_pos_lat, sig_pos_lon, sig_h,
        sig_v, sig_v, sig_v,
        sig_hdg, sig_tilt, sig_tilt,
        sig_ba_hor, sig_ba_vert, sig_ba_hor,
        sig_bg_hor, sig_bg_vert, sig_bg_hor};

    Matrix P0(ins::KF_STATE * ins::KF_STATE, 0);
    for (int i = 0; i < ins::KF_STATE; i++)
        at(P0, i, i, ins::KF_STATE) = sigma[i] * sigma[i];

    // Корреляции «наклон ↔ горизонтальное смещение акселя» — тот же вывод,
    // что в initialCovariance: x[7] = +ba_X/g, x[8] = −ba_Z/g.
    at(P0, 7, 9, ins::KF_STATE) = sigma[9] * sigma[9] / g;
    at(P0, 9, 7, ins::KF_STATE) = sigma[9] * sigma[9] / g;
    at(P0, 8, 11, ins::KF_STATE) = -sigma[11] * sigma[11] / g;
    at(P0, 11, 8, ins::KF_STATE) = -sigma[11] * sigma[11] / g;

    return P0;
}

// Начальные ошибки x₀ из итогов окна (docs/fast_alignment_plan.md, §4.3).
//
// Номинал позиции/высоты/скоростей/курса — последний отсчёт окна, поэтому
// ошибка номинала (номинал − истина, знак состояния x[0..8]) есть разность
// «последний отсчёт − среднее окна»; среднее окна и есть оценка истины.
// Ошибки наклона и смещения акселя по построению нулевые: номинал уже
// содержит оценку уровня и ba, а их неопределённость задана в P₀
// (структурная часть + корреляции). bg в номинале нулевой, поэтому оценка
// уходит в x[12..14] = истина − номинал и применяется инъекцией.
//
// apply = false — окно не признано стоянкой: средние окна не являются
// оценкой истины (в них входит движение), поэтому x₀ = 0 и приор берётся
// паспортный (initialCovarianceFromAveraging вырождается в initialCovariance).
inline Vector initialErrorsFromAveraging(const AveragingResult &avg, bool apply)
{
    Vector x(ins::KF_STATE, 0.0);

    if (!apply)
    {
        return x;
    }

    if (avg.has_sns)
    {
        x[0] = avg.sns_last.lat - avg.lat_mean;
        x[1] = avg.sns_last.lon - avg.lon_mean;
        x[2] = avg.sns_last.alt - avg.alt_mean;
        x[3] = avg.sns_last.vn - avg.vn_mean;
        x[4] = avg.sns_last.vh - avg.vh_mean;
        x[5] = avg.sns_last.ve - avg.ve_mean;
    }

    if (avg.has_heading)
    {
        x[6] = normalize_angle(avg.heading_last - avg.heading_circ_mean);
    }

    for (int k = 0; k < 3; k++)
        x[12 + k] = avg.bg_est[k];

    return x;
}

} // namespace nav
