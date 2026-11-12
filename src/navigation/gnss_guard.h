// gnss_guard.h - защита измерений СНС (уровни 1-2) по образцу NoQT.
//
// Проверки уровня 1: не конечные значения, диапазоны, zero-fix, правдоподобие скорости,
// полоса по высоте, проверка dt, скачки позиции/скорости.
// Уровень 2: детект пропадания СНС -> переход в STALE по таймауту.
//
// При выключенном gnss_guard_on (<= 0.5) все кадры считаются допустимыми.

#pragma once

#include <cmath>
#include <cstddef>

#include "../ins/ins_filter.h"
#include "../utils/constants.h"
#include "gps_processor.h"

namespace nav
{

inline double snsHorizontalDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double dlat = lat2 - lat1;
    const double dlon = (lon2 - lon1) * cos(0.5 * (lat1 + lat2));
    return R_EARTH * sqrt(dlat * dlat + dlon * dlon);
}

struct SnsGuard
{
    SnsSample last_accepted{};
    bool has_accepted = false;

    double alt0 = 0.0;
    bool has_alt0 = false;

    double last_seen_time = 0.0;
    double first_seen_time = 0.0;
    bool has_seen = false;

    SnsStatus status = SnsStatus::STALE;
    SnsReject reject = SnsReject::NONE;
    bool is_new_frame = false;

    double last_dt = 0.0;
    double last_dpos = 0.0;
    double last_dvel = 0.0;

    std::size_t accepted_frames = 0;
    std::size_t rejected_frames = 0;

    bool update(const SnsSample &s, double imu_time, const ins::KalmanConfig &cfg)
    {
        is_new_frame = !has_seen || s.time != last_seen_time;
        if (is_new_frame)
        {
            if (!has_seen)
                first_seen_time = s.time;
            has_seen = true;
            last_seen_time = s.time;
            validate(s, cfg);
        }

        if (has_seen && !is_new_frame)
        {
            const double t_ref = has_accepted ? last_accepted.time : first_seen_time;
            if (imu_time - t_ref > cfg.gnss_silence_timeout_s)
                status = SnsStatus::STALE;
        }

        return cfg.gnss_guard_on <= 0.5 || status == SnsStatus::VALID;
    }

private:
    void rejectFrame(SnsReject why)
    {
        status = SnsStatus::INVALID;
        reject = why;
        rejected_frames++;
    }

    void validate(const SnsSample &s, const ins::KalmanConfig &cfg)
    {
        reject = SnsReject::NONE;

        last_dt = 0.0;
        last_dpos = 0.0;
        last_dvel = 0.0;

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

        if (std::fabs(s.lat) > 90.0 * DEG_TO_RAD || std::fabs(s.lon) > 180.0 * DEG_TO_RAD)
        {
            rejectFrame(SnsReject::RANGE);
            return;
        }

        if (std::fabs(s.lat) < 1e-7 && std::fabs(s.lon) < 1e-7)
        {
            rejectFrame(SnsReject::ZERO_FIX);
            return;
        }

        if (s.maxSpeed() > cfg.gnss_vel_plaus)
        {
            rejectFrame(SnsReject::SPEED);
            return;
        }

        if (!has_alt0)
        {
            alt0 = s.alt;
            has_alt0 = true;
        }
        else if (std::fabs(s.alt - alt0) > cfg.gnss_alt_band_m)
        {
            rejectFrame(SnsReject::ALTITUDE);
            return;
        }

        if (has_accepted)
        {
            const double dt = s.time - last_accepted.time;
            last_dt = dt;
            if (!(dt > 0.0) || dt > cfg.gnss_dt_max_s)
            {
                rejectFrame(SnsReject::TIME);
                return;
            }

            const double dpos = snsHorizontalDistance(s.lat, s.lon, last_accepted.lat, last_accepted.lon);
            const double dalt = std::fabs(s.alt - last_accepted.alt);
            const double dpos3 = std::sqrt(dpos * dpos + dalt * dalt);
            last_dpos = dpos3;
            if (dpos3 > cfg.gnss_jump_pos_m)
            {
                rejectFrame(SnsReject::JUMP_POS);
                return;
            }

            const double dvn = s.vn - last_accepted.vn;
            const double dvh = s.vh - last_accepted.vh;
            const double dve = s.ve - last_accepted.ve;
            const double dvel = std::sqrt(dvn * dvn + dvh * dvh + dve * dve);
            last_dvel = dvel;
            if (dvel > cfg.gnss_jump_vel_ms)
            {
                rejectFrame(SnsReject::JUMP_VEL);
                return;
            }
        }

        status = SnsStatus::VALID;
        reject = SnsReject::NONE;
        last_accepted = s;
        has_accepted = true;
        accepted_frames++;
    }
};

} // namespace nav
