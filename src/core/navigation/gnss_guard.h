// gnss_guard.h — Защита комплексирования от отказов и аномалий СНС.
//
// Реализованы уровни 1-2 по docs/gnss_guard_design.md:
//   1) самодостаточная валидация отсчёта (диапазоны, нулевой фикс, скорость,
//      высота, кинематика, монотонность времени) — статус VALID/INVALID;
//   2) детектор молчания СНС по времени ИМУ — статус STALE.
// Уровень 0 (устойчивый разбор строк) выполнен в data_io/data_reader.cpp.
// Уровни 3-6 (гейт по инновации, детектор скачка, конечная блокировка с
// backoff, мягкое пересопряжение) не реализованы.
//
// Известное ограничение: повтор времени (Δt = 0) не наблюдается — кадр
// считается новым по изменению времени (см. update()), поэтому строка с
// прежним временем не валидируется и в лог как отказ не попадает. Разбор
// ограничения и варианты закрытия — docs/gnss_guard_design.md, раздел 14.2.
//
// Принципы (раздел 3 черновика):
//   1) самодостаточные проверки идут первыми и не смягчаются ростом P;
//   3) каждое решение наблюдаемо — счётчики и статус отдаются в лог;
//   4) решение принимается там, где есть номинал фильтра, — в nav::step().
//
// Ядро Qt-независимо: только STL и заголовки ядра.

#pragma once

#include <cmath>
#include <cstddef>

#include "../ins/ins_filter.h"   // ins::KalmanConfig, ins::kalman_cfg
#include "../utils/constants.h"  // R_EARTH, DEG_TO_RAD
#include "gps_processor.h"       // SnsSample, SnsStatus, SnsReject

namespace nav
{

// Пороги уровня 1, не вынесенные в KalmanConfig (черновик, раздел 5.2).
// Полоса высоты и предельный интервал времени — не «настройки подбора»:
// это величины-предохранители, выход за них означает отказ датчика.
constexpr double SNS_ALT_BAND_M = 5000.0;      // полоса высоты вокруг первого фикса, м
constexpr double SNS_ZERO_FIX_EPS_RAD = 1e-7;  // «нулевой фикс»: |lat|,|lon| меньше, рад
constexpr double SNS_DT_MAX_S = 3600.0;        // предельный интервал между кадрами, с

// Горизонтальное расстояние между двумя точками (рад) по локальной
// плоскости, м. Применяется к малым расхождениям соседних отсчётов.
inline double snsHorizontalDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double dlat = lat2 - lat1;
    const double dlon = (lon2 - lon1) * cos(0.5 * (lat1 + lat2));
    return R_EARTH * sqrt(dlat * dlat + dlon * dlon);
}

// Состояние защиты СНС. Живёт в NavState и переживает такты: опора
// кинематической проверки — последний принятый фикс, а не последний
// прочитанный (иначе мусор принимается за опору).
struct SnsGuard
{
    // Опора проверок 6-7: координаты, скорости и время последнего VALID.
    SnsSample last_accepted{};
    bool has_accepted = false;

    // Полоса высоты отсчитывается от первого кадра, прошедшего проверки 1-4.
    double alt0 = 0.0;
    bool has_alt0 = false;

    // Время последнего увиденного кадра и первого кадра вообще — для
    // детектора молчания, когда ни один кадр ещё не принят.
    double last_seen_time = 0.0;
    double first_seen_time = 0.0;
    bool has_seen = false;

    // Решение по текущему кадру.
    SnsStatus status = SnsStatus::STALE;
    SnsReject reject = SnsReject::NONE;
    bool is_new_frame = false;

    // Метрики последнего нового кадра (для лога, черновик раздел 8).
    double last_dt = 0.0;
    double last_dpos = 0.0;
    double last_dvel = 0.0;

    // Счётчики наблюдаемости: сколько кадров принято и отклонено. Сводка —
    // в NavLogger::close(); она читает эти числа из записи лога (SnsGuardRecord),
    // поэтому причины отклонений и время в STALE логгер считает сам, по строкам,
    // и второй копии этих счётчиков здесь не нужно.
    std::size_t accepted_frames = 0;
    std::size_t rejected_frames = 0;

    // Обновить состояние по текущему (удерживаемому) отсчёту и времени ИМУ.
    // Возвращает true, если коррекцию по этому отсчёту применять разрешено.
    // Повторные вызовы с тем же отсчётом решение не пересчитывают: кадр
    // оценивается один раз, дальше отдаётся сохранённый статус.
    bool update(const SnsSample &s, double imu_time, const ins::KalmanConfig &cfg)
    {
        is_new_frame = !has_seen || s.time != last_seen_time;
        if (is_new_frame)
        {
            if (!has_seen) first_seen_time = s.time;
            has_seen = true;
            last_seen_time = s.time;
            validate(s, cfg);
        }

        // Уровень 2: молчание считается по времени ИМУ, а не по числу кадров,
        // поэтому пауза в данных и обрыв связи дают одинаковую реакцию.
        // Статус только что пришедшего кадра не перекрывается: status — свойство
        // конкретной строки (раздел 4.А), INVALID значит «данные есть, но
        // непригодны», STALE — «данных нет дольше таймаута». Иначе в логе не
        // отличить «приёмник не отвечает» от «отвечает мусором» (раздел 5.3).
        if (has_seen && !is_new_frame)
        {
            const double t_ref = has_accepted ? last_accepted.time : first_seen_time;
            if (imu_time - t_ref > cfg.gnss_silence_timeout_s)
                status = SnsStatus::STALE;
        }

        // С выключенной защитой разрешение прежнее (поведение не меняется),
        // но наблюдение продолжается — это позволяет сравнить прогоны.
        return cfg.gnss_guard_on <= 0.5 || status == SnsStatus::VALID;
    }

private:
    void rejectFrame(SnsReject why)
    {
        status = SnsStatus::INVALID;
        reject = why;
        rejected_frames++;
    }

    // Уровень 1: самодостаточные проверки (черновик, раздел 5.2).
    void validate(const SnsSample &s, const ins::KalmanConfig &cfg)
    {
        reject = SnsReject::NONE;

        // Метрики кадра сбрасываются в начале: иначе отклонённый на первых
        // проверках кадр унёс бы в лог метрики предыдущего (раздел 8).
        last_dt = 0.0;
        last_dpos = 0.0;
        last_dvel = 0.0;

        // 1. Конечность всех девяти компонент измерения.
        const double measured[9] = {s.lat, s.lon, s.alt, s.vn, s.vh, s.ve,
                                    s.heading, s.roll, s.pitch};
        for (const double v : measured)
        {
            if (!std::isfinite(v))
            {
                rejectFrame(SnsReject::NONFINITE);
                return;
            }
        }
        if (!std::isfinite(s.time))
        {
            rejectFrame(SnsReject::NONFINITE);
            return;
        }

        // 2. Диапазоны координат.
        if (fabs(s.lat) > 90.0 * DEG_TO_RAD || fabs(s.lon) > 180.0 * DEG_TO_RAD)
        {
            rejectFrame(SnsReject::RANGE);
            return;
        }

        // 3. Нулевой фикс — стандартный признак «решения нет».
        if (fabs(s.lat) < SNS_ZERO_FIX_EPS_RAD && fabs(s.lon) < SNS_ZERO_FIX_EPS_RAD)
        {
            rejectFrame(SnsReject::ZERO_FIX);
            return;
        }

        // 4. Предельная скорость носителя.
        if (s.maxSpeed() > cfg.gnss_vel_plaus)
        {
            rejectFrame(SnsReject::SPEED);
            return;
        }

        // 5. Полоса высоты вокруг первого правдоподобного фикса.
        if (has_alt0 && fabs(s.alt - alt0) > SNS_ALT_BAND_M)
        {
            rejectFrame(SnsReject::ALTITUDE);
            return;
        }

        // 6-7. Кинематика и монотонность времени относительно последнего
        // принятого фикса. Одно правило закрывает заморозку решения,
        // телепорт и правдоподобный, но неверный фикс.
        // Повтор времени (dt = 0) сюда не доходит: строка с прежним временем
        // не считается новым кадром в update() — см. раздел 14.2 черновика.
        if (has_accepted)
        {
            const double dt = s.time - last_accepted.time;
            // dt пишется в лог и при нарушении монотонности/предела — иначе
            // причина TIME в логе осталась бы без числовой опоры.
            last_dt = dt;
            if (!(dt > 0.0) || dt > SNS_DT_MAX_S)
            {
                rejectFrame(SnsReject::TIME);
                return;
            }

            last_dpos = snsHorizontalDistance(last_accepted.lat, last_accepted.lon,
                                              s.lat, s.lon);
            if (last_dpos > cfg.gnss_vel_plaus * dt + cfg.gnss_acc_bound)
            {
                rejectFrame(SnsReject::KINEMATICS);
                return;
            }

            const double dvn = s.vn - last_accepted.vn;
            const double dve = s.ve - last_accepted.ve;
            last_dvel = sqrt(dvn * dvn + dve * dve);
        }

        // Кадр принят: обновляем опору кинематики и полосу высоты.
        if (!has_alt0)
        {
            alt0 = s.alt;
            has_alt0 = true;
        }
        last_accepted = s;
        has_accepted = true;
        status = SnsStatus::VALID;
        accepted_frames++;
    }
};

} // namespace nav
