#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

// =============================================================================
// signal_processing.h — обработка сигналов IMU для этапа выставки
//
// Содержит:
//   - progressiveMean: кумулятивное (прогрессивное) среднее
//   - getFinalMean: извлечение финального среднего
//   - deg2rad: перевод градусов в радианы
//   - extractAccelAxis: извлечение одной оси акселерометра из буфера
//   - extractGyroAxis: извлечение одной оси гироскопа из буфера
//
// Используется на этапе выставки для усреднения шума IMU-датчиков
// и определения статических показаний (вектор g для акселерометров).
// =============================================================================

#include <vector>
#include <cmath>
#include "types.h"
#include "file_loader.h"

/**
 * Прогрессивное (кумулятивное) среднее.
 *
 * Для каждого индекса i вычисляет среднее всех значений от 0 до i:
 *   mean[i] = (value[0] + value[1] + ... + value[i]) / (i + 1)
 *
 * Финальное среднее (последний элемент массива) — это среднее
 * по всему входному массиву. Промежуточные значения позволяют
 * отследить сходимость усреднения.
 *
 * @param in   входной массив значений
 * @param out  выходной массив прогрессивных средних (размер = in.size())
 */
inline void progressiveMean(const std::vector<double>& in, std::vector<double>& out) {
    if (in.empty()) { out.clear(); return; }
    int n = (int)in.size();
    out.resize(n);
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        sum += in[i];
        out[i] = sum / (i + 1);
    }
}

/**
 * Извлечение финального среднего — последний элемент прогрессивного среднего.
 * @param data  массив прогрессивных средних
 * @return      среднее по всему массиву (или 0.0 если массив пуст)
 */
inline double getFinalMean(const std::vector<double>& data) {
    if (data.empty()) return 0.0;
    return data.back();
}

/**
 * Перевод градусов в радианы.
 * Используется для конвертации показаний гироскопов (град/с → рад/с).
 */
inline double deg2rad(double deg) { return deg * PI / 180.0; }

/**
 * Извлечение одной оси акселерометра из буфера IMU-записей.
 *
 * @param records  буфер записей IMU
 * @param axis     номер оси: 0=Ax, 1=Ay, 2=Az
 * @param start    начальный индекс в буфере
 * @param count    количество извлекаемых сэмплов
 * @return         вектор значений выбранной оси
 */
inline std::vector<double> extractAccelAxis(const std::vector<IMU_Record>& records, int axis, int start, int count) {
    std::vector<double> res;
    res.reserve(count);
    for (int i = start; i < start + count && i < (int)records.size(); ++i) {
        switch(axis) {
            case 0: res.push_back(records[i].Ax); break;
            case 1: res.push_back(records[i].Ay); break;
            case 2: res.push_back(records[i].Az); break;
        }
    }
    return res;
}

/**
 * Извлечение одной оси гироскопа из буфера IMU-записей.
 *
 * @param records  буфер записей IMU
 * @param axis     номер оси: 0=Wx, 1=Wy, 2=Wz
 * @param start    начальный индекс в буфере
 * @param count    количество извлекаемых сэмплов
 * @return         вектор значений выбранной оси (град/с)
 */
inline std::vector<double> extractGyroAxis(const std::vector<IMU_Record>& records, int axis, int start, int count) {
    std::vector<double> res;
    res.reserve(count);
    for (int i = start; i < start + count && i < (int)records.size(); ++i) {
        switch(axis) {
            case 0: res.push_back(records[i].Wx); break;
            case 1: res.push_back(records[i].Wy); break;
            case 2: res.push_back(records[i].Wz); break;
        }
    }
    return res;
}

#endif // SIGNAL_PROCESSING_H
