#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <vector>
#include <cmath>
#include "types.h"  // <--- ДОБАВЛЕНО! Теперь компилятор знает структуры
#include "file_loader.h"

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

inline double getFinalMean(const std::vector<double>& data) {
    if (data.empty()) return 0.0;
    return data.back();
}

inline double deg2rad(double deg) { return deg * PI / 180.0; }

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