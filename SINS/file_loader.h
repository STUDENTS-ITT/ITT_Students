#ifndef FILE_LOADER_H
#define FILE_LOADER_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include "types.h"  // <--- ДОБАВЛЕНО! Теперь компилятор знает структуры

/**
 * Потоковый загрузчик файла IMU.txt.
 */
class IMU_Stream {
private:
    std::ifstream file;
    bool is_open = false;

public:
    IMU_Stream() = default;

    bool open(const std::string& filepath) {
        file.open(filepath);
        if (!file.is_open()) return false;
        std::string dummy;
        std::getline(file, dummy); // Пропуск заголовков
        std::getline(file, dummy); // Пропуск единиц измерения
        is_open = true;
        return true;
    }

    bool readNext(IMU_Record& rec) {
        if (!is_open) return false;
        std::string line;
        if (!std::getline(file, line)) return false;
        if (line.empty()) return readNext(rec);
        std::stringstream ss(line);
        ss >> rec.Time >> rec.Ax >> rec.Ay >> rec.Az >> rec.Wx >> rec.Wy >> rec.Wz;
        return !ss.fail();
    }

    void close() { if (is_open) { file.close(); is_open = false; } }
    ~IMU_Stream() { close(); }
};

// Загрузчик Nav.dat — читает всё, находит границу фаз выставка→навигация
inline bool loadNav(const std::string& filepath, Nav_Record& record) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    std::string line;
    std::getline(file, line); // Заголовки
    std::getline(file, line); // Единицы

    bool found_first = false;
    double prev_T_sys = 0.0;
    record.alignment_end_T = 0.0;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        double T_sys, T_reg, T_rem, T_nav, lon, lat, alt;
        ss >> T_sys >> T_reg >> T_rem >> T_nav >> lon >> lat >> alt;
        if (ss.fail()) break;

        if (!found_first) {
            record.T_sys = T_sys;
            record.T_reg = T_reg;
            record.T_rem = T_rem;
            record.T_nav = T_nav;
            record.Lon = lon;
            record.Lat = lat;
            record.Alt = alt;
            found_first = true;
        }

        // Граница: первая строка, где T_nav > 0 → конец выставки = предыдущий T_sys
        if (T_nav > 0.0) {
            record.alignment_end_T = prev_T_sys;
            break;
        }

        prev_T_sys = T_sys;
    }

    // Если перехода не было — выставка до конца файла
    if (record.alignment_end_T == 0.0 && found_first) {
        record.alignment_end_T = prev_T_sys;
    }

    return found_first;
}

#endif // FILE_LOADER_H