// =============================================================================
// main.cpp — точка входа программы навигации БИНС
//
// Алгоритм работы:
//   1. Загрузить начальные условия из Nav.dat (координаты, фазы выставка/навигация)
//   2. Открыть потоковый чтение IMU.txt (данные акселерометров и гироскопов)
//   3. Этап выставки: накопить IMU-сэмплы до T_reg, усреднить, определить начальные углы
//   4. Этап навигации: пошагово обработать оставшиеся сэмплы через алгоритм БИНС
//   5. Вывести первые 20 состояний для визуального контроля
//
// Входные данные:
//   INS_Data/<folder>/Nav.dat  — навигационная запись (фазы, координаты)
//   INS_Data/<folder>/IMU.txt  — сырые данные IMU (400 Гц, Time Ax Ay Az Wx Wy Wz)
//
// Запуск: ./SINS [folder_number]
// =============================================================================

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "types.h"              // Базовые типы: Vec3, Mat3, NavState, IMU_Record
#include "file_loader.h"        // Загрузчики файлов: IMU_Stream, loadNav
#include "signal_processing.h"  // Усреднение: progressiveMean, extractAccelAxis
#include "sins_algorithm.h"     // Алгоритм БИНС: SINS_Algorithm

/**
 * Вывод текущего состояния навигации в консоль.
 *
 * @param s     текущее состояние БИНС (координаты, углы, скорости)
 * @param time  текущее время Т_sys (с)
 */
void printNavState(const NavState& s, double time) {
    // Перевод из радиан в градусы для наглядного вывода
    double lat_deg = s.pos.x * 180.0 / PI;
    double lon_deg = s.pos.y * 180.0 / PI;
    double yaw_deg  = s.euler.x * 180.0 / PI;
    double pitch_deg = s.euler.y * 180.0 / PI;
    double roll_deg  = s.euler.z * 180.0 / PI;

    std::cout << "T=" << time << "s | "
              << "Lat=" << lat_deg << "° Lon=" << lon_deg << "° | "
              << "Yaw=" << yaw_deg << "° Pitch=" << pitch_deg << "° Roll=" << roll_deg << "° | "
              << "Vn=" << s.vel.x << " Ve=" << s.vel.z << " m/s" << std::endl;
}

int main(int argc, char* argv[]) {
    // ---------------------------------------------------------------------
    // 1. Определение пути к данным
    // ---------------------------------------------------------------------
    // По умолчанию используется набор данных "1" (INS_Data/1/).
    // Можно задать другой номер через аргумент командной строки.
    std::string folder = "1";
    if (argc > 1) folder = argv[1];

    std::string imu_path = "INS_Data/" + folder + "/IMU.txt";
    std::string nav_path = "INS_Data/" + folder + "/Nav.dat";

    // ---------------------------------------------------------------------
    // 2. Загрузка начальных условий из Nav.dat
    // ---------------------------------------------------------------------
    // loadNav читает весь Nav.dat, находит границу фаз выставка→навигация
    // (первый момент, где T_nav > 0), извлекает начальные координаты.
    Nav_Record nav_rec;
    if (!loadNav(nav_path, nav_rec)) {
        std::cerr << "Ошибка: Не удалось загрузить Nav.dat" << std::endl;
        return -1;
    }

    // ---------------------------------------------------------------------
    // 3. Открытие потокового чтения IMU.txt
    // ---------------------------------------------------------------------
    // IMU_Stream читает файл построчно, пропуская заголовки (2 строки).
    // Формат строк: Time Ax Ay Az Wx Wy Wz (400 Гц, Time ≥ 20.0 с)
    IMU_Stream imu_stream;
    if (!imu_stream.open(imu_path)) {
        std::cerr << "Ошибка: Не удалось открыть IMU.txt" << std::endl;
        return -1;
    }

    // ---------------------------------------------------------------------
    // 4. Этап выставки: накопление и усреднение IMU-данных
    // ---------------------------------------------------------------------
    // Все сэмплы с Time ≤ alignment_end_T (T_sys = 332 с) используются
    // для определения начальной ориентации объекта.
    // During alignment the object is stationary, so averaged accelerometer
    // readings give the gravity vector in the body frame → pitch & roll.
    std::vector<IMU_Record> reg_buffer;
    IMU_Record rec;
    while (imu_stream.readNext(rec) && rec.Time <= nav_rec.alignment_end_T) {
        reg_buffer.push_back(rec);
    }

    if (reg_buffer.empty()) {
        std::cerr << "Ошибка: Недостаточно данных для T_reg" << std::endl;
        return -1;
    }

    std::cout << "Этап выставки: " << reg_buffer.size() << " сэмплов усреднено" << std::endl;

    // ---------------------------------------------------------------------
    // 5. Усреднение данных выставки по каждой оси
    // ---------------------------------------------------------------------
    // progressiveMean вычисляет кумулятивное среднее от начала до каждого момента.
    // Финальное среднее — последний элемент массива (getFinalMean).
    // Гироскопы перевод из град/с в рад/с (deg2rad).
    std::vector<double> ax_mean, ay_mean, az_mean;
    std::vector<double> wx_mean, wy_mean, wz_mean;

    progressiveMean(extractAccelAxis(reg_buffer, 0, 0, reg_buffer.size()), ax_mean);
    progressiveMean(extractAccelAxis(reg_buffer, 1, 0, reg_buffer.size()), ay_mean);
    progressiveMean(extractAccelAxis(reg_buffer, 2, 0, reg_buffer.size()), az_mean);

    progressiveMean(extractGyroAxis(reg_buffer, 0, 0, reg_buffer.size()), wx_mean);
    progressiveMean(extractGyroAxis(reg_buffer, 1, 0, reg_buffer.size()), wy_mean);
    progressiveMean(extractGyroAxis(reg_buffer, 2, 0, reg_buffer.size()), wz_mean);

    Vec3 accel_init = {getFinalMean(ax_mean), getFinalMean(ay_mean), getFinalMean(az_mean)};
    Vec3 gyro_init  = {deg2rad(getFinalMean(wx_mean)), 
                       deg2rad(getFinalMean(wy_mean)), 
                       deg2rad(getFinalMean(wz_mean))};

    // ---------------------------------------------------------------------
    // 6. Инициализация алгоритма БИНС
    // ---------------------------------------------------------------------
    // Передаём усреднённые данные выставки и начальные координаты из Nav.dat.
    // Алгоритм вычислит начальные углы (pitch, roll), зафиксирует yaw = 0,
    // установит скорость = 0 и сформирует начальную матрицу C.
    SINS_Algorithm sins;
    sins.initializeAlignment(nav_rec.T_sys, nav_rec.T_reg, nav_rec.T_rem, nav_rec.T_nav,
                             nav_rec.alignment_end_T,
                             accel_init, gyro_init,
                             nav_rec.Lat * PI / 180.0,   // градусы → радианы
                             nav_rec.Lon * PI / 180.0,
                             nav_rec.Alt);

    std::cout << "БИНС инициализирована. Начало навигации..." << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;

    // ---------------------------------------------------------------------
    // 7. Этап навигации: пошаговая обработка IMU-данных
    // ---------------------------------------------------------------------
    // Потоковый цикл: читаем каждый сэмпл IMU, передаём в алгоритм БИНС.
    // На каждом шаге выполняется интегрирование ориентации, скоростей и координат.
    // Выводим первые 20 состояний для контроля (далее — только обработка).
    int counter = 0;
    while (imu_stream.readNext(rec)) {
        // Перевод гироскопов из град/с в рад/с (акселерометры уже в м/с²)
        Vec3 gyro = {deg2rad(rec.Wx), deg2rad(rec.Wy), deg2rad(rec.Wz)};
        Vec3 accel = {rec.Ax, rec.Ay, rec.Az};

        // Один шаг навигационного алгоритма (ориентация + навигация)
        sins.updateNavigation(rec.Time, accel, gyro);

        if (++counter <= 20) {
            printNavState(sins.getCurrentState(), rec.Time);
        }
    }

    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "Обработка завершена." << std::endl;
    return 0;
}
