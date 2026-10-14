// matrices_exporter.h — Экспорт статических матриц фильтра ESKF (Q/R/P0/H).
//
// Формирует и сохраняет матрицы фильтра 15-го порядка для текущего датасета:
// шум процесса Q, шум измерений R, начальную ковариацию P0 и матрицу
// наблюдения H. Динамические F/K/P зависят от прогона и здесь не считаются
// (диагонали P доступны в телеметрии и на вкладке «Ковариация»).
//
// Формулы берутся из ядра (ins::Qj_matrix/Rj_matrix, nav::initialCovariance,
// H_matrix) с явно переданной конфигурацией — единый источник истины,
// дублирования арифметики нет.

#pragma once

#include <QString>

#include "core/ins/ins_filter.h"
#include "core/navigation/filter_settings.h"

namespace gui
{

// Набор статических матриц фильтра и сопутствующих величин.
struct FilterMatrices
{
    QString dataset;             // имя набора данных (каталог)
    double lat_deg = 0.0;        // широта начальной точки, град
    double T = IMU_PERIOD;       // шаг дискретизации, с
    double sig_hdg = 0.0;        // курс СНС в R, рад
    double sig_pitch = 0.0;      // тангаж в R, рад (в СНС-коррекции — «игнор»)
    double sig_roll = 0.0;       // крен в R, рад (в СНС-коррекции — «игнор»)
    ins::KalmanConfig cfg;       // настроечные σ (8)
    Matrix Q;                    // 15×15
    Matrix R;                    // 9×9
    Matrix P0;                   // 15×15
    Matrix H;                    // 9×15
};

// Собрать матрицы из текущих настроек фильтра и параметров датасета.
// lat_deg — широта начальной точки, T — шаг дискретизации (с).
FilterMatrices buildFilterMatrices(const nav::FilterSettings &settings,
                                   const QString &dataset, double lat_deg, double T);

// Прочитать параметры датасета: широта (град, из StartupNav.ini) и шаг
// дискретизации T (медиана интервалов между отсчётами ИМУ; иначе IMU_PERIOD).
// Возвращает false, если ни StartupNav.ini, ни imu.dat недоступны.
bool readDatasetParameters(const QString &data_dir, double *lat_deg, double *T);

// Текстовые представления набора матриц.
QString formatMatricesCsv(const FilterMatrices &m);
QString formatMatricesMarkdown(const FilterMatrices &m);
QString formatMatricesJson(const FilterMatrices &m);

// Записать три файла в каталог dir: <base>.csv, <base>.md, <base>.json.
// base — имя без расширения (обычно «matrices_<датасет>»).
bool writeFilterMatrices(const FilterMatrices &m, const QString &dir,
                         const QString &base, QString *error = nullptr);

} // namespace gui
