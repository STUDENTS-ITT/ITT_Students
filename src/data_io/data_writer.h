// data_writer.h — Запись результатов счисления в файлы.
//
// NavResult — строка результата БИНС (исправленный Калманом).
// NavReference — строка эталона СНС.
// NavLogger — запись в три файла: result.txt, reference.txt, errors.txt;
//   плюс лениво создаваемые gnss_guard.txt — лог защиты СНС (уровни 1-2) —
//   и consistency.txt — согласованность фильтра (NIS) на кадрах коррекции.

#pragma once

#include <cstddef>
#include <fstream>
#include <string>

#include "navigation/gps_processor.h"
#include "utils/types.h"

namespace data_io
{

// Результат БИНС (исправленный фильтром Калмана): 10 колонок.
struct NavResult
{
    double time = 0;
    double lon = 0;       // град
    double lat = 0;       // град
    double alt = 0;       // м
    double heading = 0;   // град
    double pitch = 0;     // град
    double roll = 0;      // град
    double vn = 0;        // м/с
    double vh = 0;        // м/с
    double ve = 0;        // м/с
};

// Эталон СНС: 10 колонок.
struct NavReference
{
    double time = 0;
    double lon = 0;       // град
    double lat = 0;       // град
    double alt = 0;       // м
    double heading = 0;   // град
    double pitch = 0;     // град
    double roll = 0;      // град
    double vn = 0;        // м/с
    double vh = 0;        // м/с
    double ve = 0;        // м/с
};

// Строка лога защиты СНС (docs/gnss_guard_design.md, раздел 8).
//
// Колонки `gate`, `nis`, `sig_pos_pred`, `thresh`, `infl` из раздела 8 пока
// не записываются: гейт по инновации (уровни 3-5) не реализован. Поле
// `gate` присутствует и всегда принимает единственное существующее значение,
// чтобы формат лога не менялся при внедрении уровней 3-5.
struct SnsGuardRecord
{
    double time = 0;                    // время такта ИМУ, с
    nav::SnsStatus status = nav::SnsStatus::STALE;  // статус отсчёта
    nav::SnsReject reject = nav::SnsReject::NONE;   // причина отклонения кадра
    bool new_frame = false;             // на такте пришёл новый кадр СНС
    bool applied = false;               // коррекция применена фактически
    bool manual = false;                // активно ручное окно outage_*
    double dt = 0;                      // интервал до последнего принятого фикса, с
    double dpos = 0;                    // горизонтальное расхождение с ним, м
    double dvel = 0;                    // расхождение скоростей, м/с
    std::size_t accepted = 0;           // принято кадров (накопительно)
    std::size_t rejected = 0;           // отклонено кадров (накопительно)
};

// Строка согласованности фильтра (FUTURE_PROPOSALS.md, раздел 12).
//
// Пишется только на кадрах коррекции по СНС (~1 Гц). NEES намеренно не
// выводится: величина `zᵀ·P_meas⁻¹·z` численно недостоверна (docs/
// accuracy_reserve_diagnostics.md, раздел 6: средний NEES на square_650
// = -1.01e6, 99.8 % кадров отрицательны), поэтому согласованность ведётся
// по NIS, ожидаемое значение — dof.
struct ConsistencyRecord
{
    double time = 0;  // время такта, с
    double nis = 0;   // innovᵀ·S⁻¹·innov
    int dof = 9;      // число степеней свободы (размерность H)
};

// Абстрактный приёмник результатов счисления.
//
// nav::step() принимает NavSink&, поэтому один и тот же расчёт может писать
// либо в файлы (NavLogger, консольный режим), либо в память (GUI), либо
// одновременно в оба (через TeeSink).
class NavSink
{
public:
    virtual ~NavSink() = default;

    virtual void writeHeader() {}
    virtual void writeResult(const NavResult &r) = 0;
    virtual void writeReference(const NavReference &r) = 0;
    virtual void writeErrors(double time, const Vector &x) = 0;
    // Лог защиты СНС. По умолчанию ничего не делает: пишут только те
    // приёмники, которым он нужен (файловый NavLogger).
    virtual void writeGuard(const SnsGuardRecord &r) { (void)r; }
    // Согласованность на кадре коррекции. По умолчанию — заглушка.
    virtual void writeConsistency(const ConsistencyRecord &r) { (void)r; }
    // Нужна ли согласованность этому приёмнику: если ни один приёмник её не
    // просит, ядро не считает NIS/NEES и результаты не меняются.
    virtual bool wantsConsistency() const { return false; }
    virtual void close() {}
};

// Набор файлов для записи результатов.
// Лог защиты СНС (gnss_guard.txt) открывается лениво, при первой записи, и
// кладётся рядом с result.txt; если открыть не удалось — решения защиты
// просто не пишутся, прогон не прерывается.
class NavLogger : public NavSink
{
public:
    bool open(const std::string &result_path, const std::string &reference_path,
              const std::string &error_path);

    void writeHeader() override;
    void writeResult(const NavResult &r) override;
    void writeReference(const NavReference &r) override;
    void writeErrors(double time, const Vector &x) override;
    void writeGuard(const SnsGuardRecord &r) override;
    // Согласованность (NIS) — только по явному запросу (флаг консоли).
    void setConsistencyEnabled(bool on) { consistency_enabled_ = on; }
    bool wantsConsistency() const override { return consistency_enabled_; }
    void writeConsistency(const ConsistencyRecord &r) override;
    void close() override;

private:
    bool openGuardFile();
    bool openConsistencyFile();

    std::ofstream result_file_;
    std::ofstream reference_file_;
    std::ofstream error_file_;

    std::string guard_path_;      // путь к gnss_guard.txt (из пути result.txt)
    std::ofstream guard_file_;
    bool guard_tried_ = false;    // попытка открытия уже была

    bool consistency_enabled_ = false;      // пишем ли consistency.txt
    std::string consistency_path_;          // путь (из пути result.txt)
    std::ofstream consistency_file_;
    bool consistency_tried_ = false;        // попытка открытия уже была
    std::size_t consistency_records_ = 0;   // записано кадров коррекции

    // Сводка по логу (черновик, раздел 8).
    std::size_t guard_records_ = 0;
    std::size_t guard_accepted_ = 0;
    std::size_t guard_rejected_ = 0;
    std::size_t guard_rejected_by_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double guard_stale_seconds_ = 0.0;
    double guard_stale_last_ = -1.0;   // время начала текущего отрезка STALE
    double guard_first_time_ = 0.0;
    double guard_last_time_ = 0.0;
    std::size_t guard_applied_ = 0;
    std::size_t guard_manual_frames_ = 0;
};

// Дублирование потока в два приёмника (файлы + память GUI).
class TeeSink : public NavSink
{
public:
    TeeSink(NavSink &a, NavSink &b) : a_(a), b_(b) {}

    void writeHeader() override
    {
        a_.writeHeader();
        b_.writeHeader();
    }
    void writeResult(const NavResult &r) override
    {
        a_.writeResult(r);
        b_.writeResult(r);
    }
    void writeReference(const NavReference &r) override
    {
        a_.writeReference(r);
        b_.writeReference(r);
    }
    void writeErrors(double time, const Vector &x) override
    {
        a_.writeErrors(time, x);
        b_.writeErrors(time, x);
    }
    void writeGuard(const SnsGuardRecord &r) override
    {
        a_.writeGuard(r);
        b_.writeGuard(r);
    }
    void writeConsistency(const ConsistencyRecord &r) override
    {
        a_.writeConsistency(r);
        b_.writeConsistency(r);
    }
    bool wantsConsistency() const override
    {
        return a_.wantsConsistency() || b_.wantsConsistency();
    }
    void close() override
    {
        a_.close();
        b_.close();
    }

private:
    NavSink &a_;
    NavSink &b_;
};

} // namespace data_io
