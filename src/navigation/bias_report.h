// bias_report.h — Сводная таблица по журналу смещений (Этап 0 плана).
//
// Считает «истина / выставка / конец / σ(P) / нормированная ошибка» для ba/bg
// на окнах 1, 10, 30, 60, 120, 300 с. Истина берётся из params.json набора
// (раздел noise_si), выставка — из первого кадра журнала.
//
// Нормированная ошибка: (оценка − истина) / σ(P). По плану за «сходится»
// считается |норм. ошибка| ≤ 2 там, где компонента наблюдаема.

#pragma once

#include <string>
#include <vector>

#include "bias_log.h"

namespace nav
{

// Истинные смещения набора из params.json (СИ: м/с² и рад/с).
struct BiasTruth
{
    bool valid = false;
    std::string name;
    double align_time_s = 0.0;
    double ba[3] = {0, 0, 0};   // noise_si.accel_bias_m_s2
    double bg[3] = {0, 0, 0};   // noise_si.gyro_bias_rad_s
    // Наблюдаемые компоненты (план, раздел 2). На стоянках и на прямой без
    // поворота горизонтальные оси неотличимы от наклона — помечаются false,
    // чтобы отчёт не выдавал ложную точность.
    bool observable[6] = {true, true, true, true, true, true};  // ba×3, bg×3
    std::string observability_note;
};

// Разбор params.json. Возвращает false, если файл не читается или ключи
// отсутствуют (bias_truth.valid == false). error — необязательно.
bool loadBiasTruth(const std::string &params_json_path, BiasTruth &out,
                   std::string *error = nullptr);

// Строка отчёта для одной компоненты на одном окне.
struct BiasReportRow
{
    std::string component;   // ba_X / ba_Y / ba_Z / bg_X / bg_Y / bg_Z
    double window_s = 0.0;   // окно от конца выставки, с
    double truth = 0.0;      // истина, СИ
    double estimate = 0.0;   // оценка в конце окна, СИ
    double sigma = 0.0;      // σ(P) в конце окна, СИ
    double norm_error = 0.0; // (оценка − истина) / σ; NaN, если σ == 0
    bool observable = true;  // наблюдаема ли компонента (по params.json)
};

// Построение таблицы: журнал + истина. Окна берутся из windows_s, если они
// заданы, иначе — набор по умолчанию 1/10/30/60/120/300 с.
std::vector<BiasReportRow> buildBiasReport(const std::vector<BiasSample> &log,
                                           const BiasTruth &truth,
                                           const std::vector<double> &windows_s = {});

// Оконный формат отчёта для консоли/GUI: таблица + итог «сходится/нет».
std::string formatBiasReport(const std::vector<BiasReportRow> &rows,
                             const BiasTruth &truth);
} // namespace nav
