#ifndef FILE_LOADER_H
#define FILE_LOADER_H

// =============================================================================
// file_loader.h — загрузчики данных IMU и навигации
//
// Содержит:
//   - IMU_Stream: потоковый чтение IMU.txt (без загрузки всего файла в память)
//   - loadNav: загрузка Nav.dat с определением границы выставка→навигация
//
// Формат IMU.txt:
//   Строка 1: заголовки (пропускается)
//   Строка 2: единицы измерения (пропускаются)
//   Строка 3+: Time Ax Ay Az Wx Wy Wz
//     Time — время (с), начинается с ~20.0
//     Ax, Ay, Az — акселерометр (м/с²)
//     Wx, Wy, Wz — гироскоп (град/с)
//
// Формат Nav.dat:
//   Строка 1: заголовки (пропускается)
//   Строка 2: единицы измерения (пропускаются)
//   Строка 3+: T_sys T_reg T_rem T_nav Lon Lat Alt
//     T_nav = 0 → этап выставки
//     T_nav > 0 → этап навигации (граница определяется автоматически)
// =============================================================================

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include "types.h"

// =============================================================================
// Потоковый загрузчик IMU.txt
// =============================================================================

/**
 * Класс IMU_Stream — последовательное чтение данных IMU из файла.
 *
 * Преимущество: файл не загружается целиком в память (важно для больших файлов
 * ~4 млн строк). Чтение идёт построчно через std::ifstream.
 *
 * Использование:
 *   IMU_Stream stream;
 *   stream.open("IMU.txt");
 *   IMU_Record rec;
 *   while (stream.readNext(rec)) { ... }
 *   stream.close(); // или деструктор
 */
class IMU_Stream {
private:
    std::ifstream file;
    bool is_open = false;

public:
    IMU_Stream() = default;

    /**
     * Открытие файла и пропуск двух строк заголовка.
     * @return true если файл успешно открыт
     */
    bool open(const std::string& filepath) {
        file.open(filepath);
        if (!file.is_open()) return false;
        std::string dummy;
        std::getline(file, dummy); // Пропуск строки заголовков
        std::getline(file, dummy); // Пропуск строки единиц измерения
        is_open = true;
        return true;
    }

    /**
     * Чтение следующей строки и разбор в структуру IMU_Record.
     * Пустые строки пропускаются рекурсивно.
     *
     * @param rec [out] заполняемая запись IMU
     * @return true если строка успешно прочитана и разобрана
     */
    bool readNext(IMU_Record& rec) {
        if (!is_open) return false;
        std::string line;
        if (!std::getline(file, line)) return false;
        if (line.empty()) return readNext(rec);
        std::stringstream ss(line);
        ss >> rec.Time >> rec.Ax >> rec.Ay >> rec.Az >> rec.Wx >> rec.Wy >> rec.Wz;
        return !ss.fail();
    }

    /** Закрытие файла. */
    void close() { if (is_open) { file.close(); is_open = false; } }

    /** Деструктор — автоматическое закрытие. */
    ~IMU_Stream() { close(); }
};

// =============================================================================
// Загрузчик Nav.dat
// =============================================================================

/**
 * Чтение всего файла Nav.dat и определение границы выставка→навигация.
 *
 * Алгоритм:
 *   1. Читает файл построчно, пропуская заголовки
 *   2. Первая строка данных → начальные координаты и временные метки
 *   3. Ищет первую строку с T_nav > 0 → это начало навигации
 *   4. alignment_end_T = T_sys предыдущей строки (конец выставки)
 *
 * @param filepath  путь к файлу Nav.dat
 * @param record    [out] заполняемая запись навигации
 * @return true если файл прочитан и содержит данные
 */
inline bool loadNav(const std::string& filepath, Nav_Record& record) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    std::string line;
    std::getline(file, line); // Заголовки
    std::getline(file, line); // Единицы измерения

    bool found_first = false;
    double prev_T_sys = 0.0;
    record.alignment_end_T = 0.0;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        double T_sys, T_reg, T_rem, T_nav, lon, lat, alt;
        ss >> T_sys >> T_reg >> T_rem >> T_nav >> lon >> lat >> alt;
        if (ss.fail()) break;

        // Первая строка данных → сохраняем как начальные условия
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

        // Граница: первая строка с T_nav > 0 → конец выставки = предыдущий T_sys
        if (T_nav > 0.0) {
            record.alignment_end_T = prev_T_sys;
            break;
        }

        prev_T_sys = T_sys;
    }

    // Если переход не найден — выставка до конца файла
    if (record.alignment_end_T == 0.0 && found_first) {
        record.alignment_end_T = prev_T_sys;
    }

    return found_first;
}

#endif // FILE_LOADER_H
