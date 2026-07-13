#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;

inline bool fileExists(const string& path) {
    ifstream file(path);
    return file.good();
}

inline bool skipLines(ifstream& file, int count) {
    string line;
    for (int i = 0; i < count; i++)
        if (!getline(file, line)) return false;
    return true;
}

// Чтение широты из Nav.dat (лат. — 6-е число на 3-й строке)
inline double readLatitudeFromNav(const string& folder) {
    string path = folder + "/Nav.dat";
    ifstream file(path);
    if (!file.is_open()) {
        cerr << "Ошибка: не удалось открыть " << path << endl;
        return 45.0;
    }

    // Пропускаем 2 строки заголовка
    string line;
    for (int i = 0; i < 2; i++)
        if (!getline(file, line)) {
            cerr << "Ошибка: Nav.dat слишком короткий" << endl;
            return 45.0;
        }

    // Парсим 6-е число
    if (!getline(file, line)) {
        cerr << "Ошибка: нет строки данных в Nav.dat" << endl;
        return 45.0;
    }

    istringstream iss(line);
    double val;
    double lat = 45.0;
    for (int i = 0; i < 6; i++) {
        if (!(iss >> val)) {
            cerr << "Ошибка: не удалось прочитать колонку " << i + 1 << " в Nav.dat" << endl;
            return 45.0;
        }
        if (i == 5) lat = val; // колонка 6 — широта
    }
    return lat;
}

inline void printError(const string& message) {
    cout << "Ошибка: " << message << endl;
}

inline void printStatus(const string& message) {
    cout << "[СТАТУС] " << message << endl;
}

#endif
