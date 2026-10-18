// bias_log.h — Журнал оценок смещений ДУС/акселя (bias.txt).
//
// Этап 0 плана docs/bias_estimation_plan.md: измеримая baseline по смещениям
// без изменения математики фильтра. Пишется по флагу RunOptions::write_bias_log,
// поэтому result.txt / reference.txt / errors.txt не меняются и test_parity
// сохраняется.
//
// Единицы СИ: ba — м/с², bg — рад/с (как в params.json, раздел noise_si),
// чтобы отчёт сравнивал оценку с истиной без пересчёта.

#pragma once

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

namespace nav
{

// Один кадр журнала смещений.
struct BiasSample
{
    double time = 0.0;          // время такта, с
    double ba[3] = {0, 0, 0};   // оценка смещения акселерометра, м/с²
    double bg[3] = {0, 0, 0};   // оценка смещения гироскопа, рад/с
    double sig_ba[3] = {0, 0, 0};  // σ(P_ba) — стандартное отклонение, м/с²
    double sig_bg[3] = {0, 0, 0};  // σ(P_bg), рад/с
    double trim_bias_pitch = 0.0;  // скрытый трим тангажа (R1), рад/с
    double trim_bias_roll = 0.0;   // скрытый трим крена (R1), рад/с
};

// Запись bias.txt. Открывается только при write_bias_log, поэтому на обычном
// прогоне накладных расходов нет.
class BiasLogWriter
{
public:
    // Создаёт (перезаписывает) файл path. Возвращает false при ошибке открытия.
    bool open(const std::string &path);
    void write(const BiasSample &s);
    void close();
    bool isOpen() const;

private:
    std::ofstream out_;
};

// Чтение bias.txt обратно (для --bias-report и автотестов).
// При ошибке возвращает false и, если error не nullptr, заполняет её текстом.
bool readBiasLog(const std::string &path, std::vector<BiasSample> &out,
                 std::string *error = nullptr);

} // namespace nav
