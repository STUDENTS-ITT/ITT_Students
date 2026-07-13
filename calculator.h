#ifndef CALCULATOR_H
#define CALCULATOR_H

#include "imu_data.h"
#include <cmath>
#include <vector>

using namespace std;

#define G_CONST 6.67430e-11
#define EARTH_MASS 5.97219e24
#define EARTH_RADIUS 6371000.0
#define EARTH_OMEGA 7.292115e-5

inline double calculateGravity(double latitude_rad) {
    double grav = G_CONST * EARTH_MASS / (EARTH_RADIUS * EARTH_RADIUS);
    double centri = EARTH_OMEGA * EARTH_OMEGA * EARTH_RADIUS
                  * cos(latitude_rad) * cos(latitude_rad);
    return grav - centri;
}

inline double toDegrees(double radians) {
    return radians * 180.0 / PI;
}

inline double toRadians(double degrees) {
    return degrees * PI / 180.0;
}

// ========== 1. Boxcar (равномерное окно, O(n)) ==========
inline void movingAverage(const vector<double>& in, vector<double>& out, int win) {
    int n = (int)in.size();
    int half = win / 2;
    out.resize(n);

    vector<double> padded(n + 2 * half);
    for (int i = 0; i < half; i++) {
        padded[i] = in[half - 1 - i];
        padded[n + half + i] = in[n - 1 - i];
    }
    for (int i = 0; i < n; i++)
        padded[half + i] = in[i];

    vector<double> prefix(padded.size() + 1, 0.0);
    for (int i = 0; i < (int)padded.size(); i++)
        prefix[i + 1] = prefix[i] + padded[i];

    for (int i = 0; i < n; i++)
        out[i] = (prefix[i + win] - prefix[i]) / win;
}

// ========== 2. Gaussian (весовое окно, O(n*win)) ==========
// Предварительное вычисление весов Гаусса
inline vector<double> gaussianKernel(int win) {
    int half = win / 2;
    double sigma = win / 6.0; // ±3σ укладывается в окно
    vector<double> kernel(win);
    double sum = 0;
    for (int i = 0; i < win; i++) {
        int d = i - half;
        kernel[i] = exp(-0.5 * (d * d) / (sigma * sigma));
        sum += kernel[i];
    }
    for (int i = 0; i < win; i++)
        kernel[i] /= sum;
    return kernel;
}

// Свёртка с гауссовским ядром
inline void gaussianFilter(const vector<double>& in, vector<double>& out,
                           const vector<double>& kernel) {
    int n = (int)in.size();
    int win = (int)kernel.size();
    int half = win / 2;
    out.resize(n);

    // отражение границ
    vector<double> padded(n + 2 * half);
    for (int i = 0; i < half; i++) {
        padded[i] = in[half - 1 - i];
        padded[n + half + i] = in[n - 1 - i];
    }
    for (int i = 0; i < n; i++)
        padded[half + i] = in[i];

    // взвешенная свёртка
    for (int i = 0; i < n; i++) {
        double sum = 0;
        for (int j = 0; j < win; j++)
            sum += kernel[j] * padded[i + j];
        out[i] = sum;
    }
}

// ========== 3. Progressive (расширяющееся окно, O(n)) ==========
// На каждом шаге использует ВСЕ данные от 0 до i — мин. дисперсия, макс. гладкость
inline void progressiveMean(const vector<double>& in, vector<double>& out) {
    int n = (int)in.size();
    out.resize(n);
    double sum = 0;
    for (int i = 0; i < n; i++) {
        sum += in[i];
        out[i] = sum / (i + 1);
    }
}

// ========== 4. Hybrid: Boxcar для начала, Progressive для основной части ==========
// Первые win отсчётов — Boxcar с отражением (без скачков),
// затем плавный переход на прогрессивное среднее.
inline void hybridSmooth(const vector<double>& in, vector<double>& out, int win) {
    int n = (int)in.size();
    out.resize(n);

    // Boxcar для ВСЕХ индексов
    vector<double> boxcar;
    movingAverage(in, boxcar, win);

    // Progressive для ВСЕХ индексов
    vector<double> progressive;
    progressiveMean(in, progressive);

    // i < win: Boxcar
    // win <= i < 2*win: линейная интерполяция
    // i >= 2*win: Progressive
    for (int i = 0; i < n; i++) {
        if (i < win) {
            out[i] = boxcar[i];
        } else if (i < 2 * win) {
            double w = (double)(i - win) / win;
            out[i] = (1.0 - w) * boxcar[i] + w * progressive[i];
        } else {
            out[i] = progressive[i];
        }
    }
}

// Расчёт углов по данным акселерометров и гироскопов
inline void calculateAngles(ImuData& data, double g) {
    double ratio = data.ax / g;
    if (ratio > 1.0) ratio = 1.0;
    if (ratio < -1.0) ratio = -1.0;
    data.pitch = asin(ratio);
    data.roll = atan2(-data.az, data.ay);
    data.yaw = atan2(data.wz, data.wx);
}

#endif
