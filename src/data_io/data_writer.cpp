#include "data_writer.h"

#include <iomanip>

#include "utils/constants.h"
#include "io_error.h"

namespace data_io
{
namespace
{
constexpr int COL = 15;
} // namespace

bool NavLogger::open(const std::string &result_path, const std::string &reference_path,
                     const std::string &error_path)
{
    result_file_.open(result_path);
    reference_file_.open(reference_path);
    error_file_.open(error_path);

    // Лог защиты кладётся рядом с result.txt: имя файла фиксировано
    // конвенцией, каталог берётся у результата.
    const std::size_t slash = result_path.find_last_of("/\\");
    const std::string dir = (slash == std::string::npos ? std::string()
                                                        : result_path.substr(0, slash + 1));
    guard_path_ = dir + "gnss_guard.txt";
    consistency_path_ = dir + "consistency.txt";

    if (!result_file_.is_open())
    {
        reportOpenError(result_path);
        return false;
    }
    if (!reference_file_.is_open())
    {
        reportOpenError(reference_path);
        return false;
    }
    if (!error_file_.is_open())
    {
        reportOpenError(error_path);
        return false;
    }
    return true;
}

void NavLogger::writeHeader()
{
    result_file_ << std::left
                 << std::setw(COL) << "time"
                 << std::setw(COL) << "lon"
                 << std::setw(COL) << "lat"
                 << std::setw(COL) << "alt"
                 << std::setw(COL) << "heading"
                 << std::setw(COL) << "pitch"
                 << std::setw(COL) << "roll"
                 << std::setw(COL) << "vn"
                 << std::setw(COL) << "vh"
                 << std::setw(COL) << "ve"
                 << std::endl;

    reference_file_ << std::left
                    << std::setw(COL) << "time"
                    << std::setw(COL) << "lon"
                    << std::setw(COL) << "lat"
                    << std::setw(COL) << "alt"
                    << std::setw(COL) << "heading"
                    << std::setw(COL) << "pitch"
                    << std::setw(COL) << "roll"
                    << std::setw(COL) << "vn"
                    << std::setw(COL) << "vh"
                    << std::setw(COL) << "ve"
                    << std::endl;

    error_file_ << std::left
                << std::setw(COL) << "time"
                << std::setw(COL) << "x0"
                << std::setw(COL) << "x1"
                << std::setw(COL) << "x2"
                << std::setw(COL) << "x3"
                << std::setw(COL) << "x4"
                << std::setw(COL) << "x5"
                << std::setw(COL) << "x6"
                << std::setw(COL) << "x7"
                << std::setw(COL) << "x8"
                << std::setw(COL) << "x9"
                << std::setw(COL) << "x10"
                << std::setw(COL) << "x11"
                << std::setw(COL) << "x12"
                << std::setw(COL) << "x13"
                << std::setw(COL) << "x14"
                << std::endl;
}

void NavLogger::writeResult(const NavResult &r)
{
    result_file_ << std::left << std::fixed << std::setprecision(8)
                 << std::setw(COL) << r.time
                 << std::setw(COL) << r.lon
                 << std::setw(COL) << r.lat
                 << std::setw(COL) << r.alt
                 << std::setw(COL) << r.heading
                 << std::setw(COL) << r.pitch
                 << std::setw(COL) << r.roll
                 << std::setw(COL) << r.vn
                 << std::setw(COL) << r.vh
                 << std::setw(COL) << r.ve
                 << std::endl;
}

void NavLogger::writeReference(const NavReference &r)
{
    reference_file_ << std::left << std::fixed << std::setprecision(8)
                    << std::setw(COL) << r.time
                    << std::setw(COL) << r.lon
                    << std::setw(COL) << r.lat
                    << std::setw(COL) << r.alt
                    << std::setw(COL) << r.heading
                    << std::setw(COL) << r.pitch
                    << std::setw(COL) << r.roll
                    << std::setw(COL) << r.vn
                    << std::setw(COL) << r.vh
                    << std::setw(COL) << r.ve
                    << std::endl;
}

void NavLogger::writeErrors(double time, const Vector &x)
{
    error_file_ << std::left << std::fixed << std::setprecision(4) << std::setw(COL) << time;
    for (std::size_t k = 0; k < x.size(); k++)
    {
        error_file_ << std::setw(COL) << x[k];
    }
    error_file_ << std::endl;
}

bool NavLogger::openGuardFile()
{
    guard_tried_ = true;
    guard_file_.open(guard_path_);
    if (!guard_file_.is_open())
    {
        // Лог защиты — диагностика, а не результат: прогон не прерываем.
        reportOpenError(guard_path_);
        return false;
    }

    guard_file_ << "# Лог защиты СНС (docs/gnss_guard_design.md, раздел 8): уровни 1-2.\n"
                << "# Строка на каждый такт ИМУ. gate=ENABLED: гейт по инновации "
                   "(уровни 3-5) не реализован.\n"
                << "# status — статус отсчёта; apply — применена ли коррекция; "
                   "manual — активно ручное окно outage_*;\n"
                << "# new — на такте пришёл новый кадр; reject — причина отклонения; "
                   "accepted/rejected — счётчики кадров.\n"
                << "# dt/dpos/dvel — метрики последнего нового кадра (на удержании "
                   "повторяются, new=0).\n"
                << std::left
                << std::setw(COL) << "time"
                << std::setw(COL) << "status"
                << std::setw(COL) << "gate"
                << std::setw(COL) << "apply"
                << std::setw(COL) << "manual"
                << std::setw(COL) << "new"
                << std::setw(COL) << "reject"
                << std::setw(COL) << "dt"
                << std::setw(COL) << "dpos"
                << std::setw(COL) << "dvel"
                << std::setw(COL) << "accepted"
                << std::setw(COL) << "rejected"
                << std::endl;
    return true;
}

bool NavLogger::openConsistencyFile()
{
    consistency_tried_ = true;
    consistency_file_.open(consistency_path_);
    if (!consistency_file_.is_open())
    {
        // Диагностика, а не результат: прогон не прерываем.
        reportOpenError(consistency_path_);
        return false;
    }

    consistency_file_ << "# Согласованность фильтра на кадрах коррекции по СНС\n"
                         "# (FUTURE_PROPOSALS.md, раздел 12; dof = KF_MEAS = 9).\n"
                         "# NIS = zᵀ·S⁻¹·z: в согласованном фильтре среднее ≈ dof.\n"
                         "# NEES не выводится: zᵀ·P_meas⁻¹·z численно недостоверен\n"
                         "# (docs/accuracy_reserve_diagnostics.md, раздел 6).\n"
                      << std::left
                      << std::setw(COL) << "time"
                      << std::setw(COL) << "nis"
                      << std::setw(COL) << "dof"
                      << std::endl;
    return true;
}

void NavLogger::writeGuard(const SnsGuardRecord &r)
{
    if (!guard_tried_ && !openGuardFile()) return;
    if (!guard_file_.is_open()) return;

    // Сводка: отрезки STALE считаем по времени тактов, причины отклонений —
    // по новым кадрам (иначе удержанный кадр считался бы на каждом такте).
    if (guard_records_ == 0) guard_first_time_ = r.time;
    guard_last_time_ = r.time;

    if (r.status == nav::SnsStatus::STALE)
    {
        if (guard_stale_last_ < 0.0) guard_stale_last_ = r.time;
    }
    else if (guard_stale_last_ >= 0.0)
    {
        guard_stale_seconds_ += r.time - guard_stale_last_;
        guard_stale_last_ = -1.0;
    }

    if (r.manual) guard_manual_frames_++;
    if (r.applied) guard_applied_++;
    if (r.new_frame && r.reject != nav::SnsReject::NONE)
        guard_rejected_by_[static_cast<int>(r.reject)]++;
    guard_accepted_ = r.accepted;
    guard_rejected_ = r.rejected;
    guard_records_++;

    guard_file_ << std::left << std::fixed << std::setprecision(6)
                << std::setw(COL) << r.time
                << std::setw(COL) << nav::snsStatusName(r.status)
                << std::setw(COL) << "ENABLED"
                << std::setw(COL) << (r.applied ? 1 : 0)
                << std::setw(COL) << (r.manual ? 1 : 0)
                << std::setw(COL) << (r.new_frame ? 1 : 0)
                << std::setw(COL) << nav::snsRejectName(r.reject)
                << std::setw(COL) << r.dt
                << std::setw(COL) << r.dpos
                << std::setw(COL) << r.dvel
                << std::setw(COL) << r.accepted
                << std::setw(COL) << r.rejected
                << std::endl;
}

void NavLogger::writeConsistency(const ConsistencyRecord &r)
{
    // Единственный переключатель — флаг: без него файл не создаётся вовсе.
    if (!consistency_enabled_) return;
    if (!consistency_tried_ && !openConsistencyFile()) return;
    if (!consistency_file_.is_open()) return;

    consistency_records_++;
    consistency_file_ << std::left << std::fixed << std::setprecision(6)
                      << std::setw(COL) << r.time
                      << std::setw(COL) << r.nis
                      << std::setw(COL) << r.dof
                      << std::endl;
}

void NavLogger::close()
{
    if (consistency_file_.is_open())
    {
        consistency_file_ << "# Кадров коррекции " << consistency_records_ << ".\n";
        consistency_file_.close();
    }

    if (guard_file_.is_open())
    {
        if (guard_stale_last_ >= 0.0)
        {
            guard_stale_seconds_ += guard_last_time_ - guard_stale_last_;
            guard_stale_last_ = -1.0;
        }

        guard_file_ << "# Сводка: кадров принято " << guard_accepted_
                    << ", отклонено " << guard_rejected_ << " (нечисловое "
                    << guard_rejected_by_[static_cast<int>(nav::SnsReject::NONFINITE)]
                    << ", диапазон " << guard_rejected_by_[static_cast<int>(nav::SnsReject::RANGE)]
                    << ", нулевой фикс " << guard_rejected_by_[static_cast<int>(nav::SnsReject::ZERO_FIX)]
                    << ", скорость " << guard_rejected_by_[static_cast<int>(nav::SnsReject::SPEED)]
                    << ", высота " << guard_rejected_by_[static_cast<int>(nav::SnsReject::ALTITUDE)]
                    << ", кинематика " << guard_rejected_by_[static_cast<int>(nav::SnsReject::KINEMATICS)]
                    << ", время " << guard_rejected_by_[static_cast<int>(nav::SnsReject::TIME)] << ");\n"
                    << "# тактов " << guard_records_ << " (с " << guard_first_time_
                    << " по " << guard_last_time_ << " с); время в STALE "
                    << guard_stale_seconds_ << " с; коррекций применено "
                    << guard_applied_ << "; тактов в ручном окне "
                    << guard_manual_frames_ << ".\n";
        guard_file_.close();
    }

    result_file_.close();
    reference_file_.close();
    error_file_.close();
}

} // namespace data_io
