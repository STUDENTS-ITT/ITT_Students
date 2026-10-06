// telemetry_buffer.h — Потокобезопасный буфер телеметрии между рабочим потоком
// расчёта и GUI.
//
// Расчёт идёт с частотой ИМУ (200 Гц) и выдаёт по 15 диагоналей P, инновацию и
// результат на каждый такт. Гонять это через сигналы Qt дорого, поэтому рабочий
// поток пишет в буфер под QMutex, а GUI забирает пакетами с ограничением
// частоты перерисовки (20–30 Гц).

#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include "core/data_io/data_writer.h"
#include "core/ins/ins_filter.h"
#include "core/navigation/simulation.h"

namespace gui
{

// Один отсчёт телеметрии: результат БИНС + диагностика фильтра.
struct TelemetrySample
{
    double time = 0;
    data_io::NavResult result{};
    data_io::NavReference reference{};
    Vector p_diag = Vector(ins::KF_STATE, 0.0);
    Vector innovation;        // z (9) — заполняется на кадрах коррекции
    Vector errors = Vector(ins::KF_STATE, 0.0);  // x до применения коррекции
    double nis = 0.0;         // нормированная инновация (кадр коррекции)
    double nees = 0.0;        // нормированная ошибка состояния
    int nis_dof = ins::KF_MEAS;
    Vector ba = Vector(3, 0.0);
    Vector bg = Vector(3, 0.0);
    bool corrected = false;   // был кадр коррекции по СНС на этом такте
    bool has_reference = false;    // заполнен reference (кадр коррекции)
    bool has_innovation = false;   // заполнена innovation (кадр коррекции)
    int step_index = 0;
};

// Накопитель телеметрии с ограничением размера.
class TelemetryBuffer
{
public:
    explicit TelemetryBuffer(std::size_t capacity = 50000) : capacity_(capacity) {}

    // Вызывается из рабочего потока.
    void push(const TelemetrySample &s)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (data_.size() >= capacity_)
        {
            // Отбрасываем самые старые: память ограничена, а для графиков важнее
            // свежие данные. Размер буфера заведомо больше пачки между
            // двумя отрисовками GUI.
            data_.pop_front();
        }
        data_.push_back(s);
        ++total_;
    }

    // Забрать все накопленные отсчёты (GUI-поток).
    std::vector<TelemetrySample> drain()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<TelemetrySample> out;
        out.reserve(data_.size());
        out.insert(out.end(), data_.begin(), data_.end());
        data_.clear();
        return out;
    }

    // Отсчёты, оставшиеся в буфере (для финальной отрисовки).
    std::size_t pending() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.size();
    }

    std::size_t total() const { return total_; }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.clear();
        total_ = 0;
    }

private:
    mutable std::mutex mutex_;
    std::deque<TelemetrySample> data_;
    std::size_t capacity_;
    std::size_t total_ = 0;
};

// Ряды, накопленные за весь прогон, для графиков и таблиц (только GUI-поток).
struct RunSeries
{
    struct Row
    {
        double time = 0;
        double x[ins::KF_STATE] = {0};
        double p[ins::KF_STATE] = {0};
        double result[10] = {0};  // t, lon, lat, alt, hdg, pitch, roll, vn, vh, ve
        double reference[10] = {0};
        bool has_reference = false;
        double innovation[ins::KF_MEAS] = {0};
        bool has_innovation = false;
        double nis = 0.0;     // NIS/NEES — только на кадрах коррекции
        double nees = 0.0;
        int nis_dof = ins::KF_MEAS;
        bool corrected = false;
        int step_index = 0;
    };

    std::vector<Row> rows;

    // Кадры коррекции СНС — отдельный (разреженный) ряд для графиков ошибок.
    struct CorrectionRow
    {
        double time = 0;
        double innovation[ins::KF_MEAS] = {0};
        double x[ins::KF_STATE] = {0};
        double nis = 0.0;
        double nees = 0.0;
    };
    std::vector<CorrectionRow> corrections;

    // Согласованность фильтра (NIS/NEES) — тоже только на кадрах коррекции.
    struct ConsistencyRow
    {
        double time = 0;
        double nis = 0.0;
        double nees = 0.0;
        int dof = ins::KF_MEAS;
    };
    std::vector<ConsistencyRow> consistency;

    bool isEmpty() const { return rows.empty(); }

    void clear()
    {
        rows.clear();
        corrections.clear();
        consistency.clear();
    }

    std::size_t size() const { return rows.size(); }

    // Границы времени, с.
    double timeMin() const;
    double timeMax() const;

    // Индексы полей в Row::result / Row::reference.
    enum ResultField
    {
        Time = 0, Lon, Lat, Alt, Heading, Pitch, Roll, Vn, Vh, Ve
    };
};

} // namespace gui