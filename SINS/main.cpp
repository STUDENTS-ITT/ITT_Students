#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "types.h"          // 1. Самая база
#include "file_loader.h"    // 2. Загрузчики (знают про types.h)
#include "signal_processing.h" // 3. Обработка (знает про types.h и file_loader.h)
#include "sins_algorithm.h" // 4. Алгоритм (знает про types.h и math_utils.h)

void printNavState(const NavState& s, double time) {
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
    std::string folder = "1";
    if (argc > 1) folder = argv[1];

    std::string imu_path = "INS_Data/" + folder + "/IMU.txt";
    std::string nav_path = "INS_Data/" + folder + "/Nav.dat";

    // 1. Загружаем начальные условия (только первую строку)
    Nav_Record nav_rec;
    if (!loadNav(nav_path, nav_rec)) {
        std::cerr << "Ошибка: Не удалось загрузить Nav.dat" << std::endl;
        return -1;
    }

    // 2. Открываем поток IMU (не загружаем всё в память!)
    IMU_Stream imu_stream;
    if (!imu_stream.open(imu_path)) {
        std::cerr << "Ошибка: Не удалось открыть IMU.txt" << std::endl;
        return -1;
    }

    // 3. Буфер для этапа выставки (до T_reg)
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

    // 4. Усредняем буфер по всем 6 осям
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

    // 5. Инициализация алгоритма БИНС
    SINS_Algorithm sins;
    sins.initializeAlignment(nav_rec.T_sys, nav_rec.T_reg, nav_rec.T_rem, nav_rec.T_nav,
                             nav_rec.alignment_end_T,
                             accel_init, gyro_init,
                             nav_rec.Lat * PI / 180.0,
                             nav_rec.Lon * PI / 180.0,
                             nav_rec.Alt);

    std::cout << "БИНС инициализирована. Начало навигации..." << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;

    // 6. Потоковая обработка оставшихся данных (начиная с того места, где остановились)
    int counter = 0;
    while (imu_stream.readNext(rec)) {
        Vec3 gyro = {deg2rad(rec.Wx), deg2rad(rec.Wy), deg2rad(rec.Wz)};
        Vec3 accel = {rec.Ax, rec.Ay, rec.Az};

        sins.updateNavigation(rec.Time, accel, gyro);

        if (++counter <= 20) {
            printNavState(sins.getCurrentState(), rec.Time);
        }
    }

    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "Обработка завершена." << std::endl;
    return 0;
}