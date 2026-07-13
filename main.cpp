#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include "imu_data.h"
#include "calculator.h"
#include "file_utils.h"

using namespace std;
namespace fs = std::filesystem;

// 0 = Boxcar (равномерное, быстрое O(n))
// 1 = Gaussian (весовое, точное O(n*win))
// 2 = Progressive (расширяющееся, макс. точность O(n))
#define USE_GAUSSIAN 2

int main() {
    cout << "=============================================================\n";
    cout << "  IMU Data Processing - Выставка углов\n";
    cout << "=============================================================\n\n";

    // Автоматическое обнаружение подпапок в INS_Data/
    vector<string> folders;
    string dataDir = "INS_Data";
    if (!fs::exists(dataDir)) {
        printError("Директория " + dataDir + " не найдена");
        return 1;
    }
    for (const auto& entry : fs::directory_iterator(dataDir)) {
        if (entry.is_directory()) {
            folders.push_back(entry.path().string());
        }
    }
    sort(folders.begin(), folders.end());

    if (folders.empty()) {
        printError("Нет поддиректорий в " + dataDir);
        return 1;
    }

    const int PRINT_LIMIT = 20;
    const int WINDOW = 1000; // окно сглаживания (отсчётов)

    for (const string& folder : folders) {
        cout << "========== ОБРАБОТКА ПАПКИ " << folder << " ==========\n\n";

        // 1. Широта и g
        double lat = readLatitudeFromNav(folder);
        double lat_rad = lat * PI / 180.0;
        double g = calculateGravity(lat_rad);
        cout << "Широта: " << fixed << setprecision(6) << lat << " deg\n";
        const char* filterName = (USE_GAUSSIAN == 2) ? "Progressive" :
                            (USE_GAUSSIAN == 1) ? "Gaussian" : "Boxcar";
    cout << "g: " << g << " m/s² | Окно " << filterName
         << ": " << WINDOW << " отсчётов\n\n";

        // 2. Открытие IMU.txt
        string imu_path = folder + "/IMU.txt";
        ifstream file(imu_path);
        if (!file.is_open()) {
            printError("Не удалось открыть " + imu_path);
            continue;
        }

        if (!skipLines(file, 2)) {
            printError("Файл IMU.txt слишком короткий");
            file.close();
            continue;
        }

        // 3. Чтение всех данных в векторы
        vector<double> t, ax, ay, az, wx, wy, wz;
        ImuData data;
        string line;

        while (getline(file, line)) {
            if (!data.parseFromString(line)) continue;
            if (data.time >= 2000.0) break;
            t.push_back(data.time);
            ax.push_back(data.ax);
            ay.push_back(data.ay);
            az.push_back(data.az);
            wx.push_back(data.wx);
            wy.push_back(data.wy);
            wz.push_back(data.wz);
        }
        file.close();

        int n = (int)t.size();
        if (n == 0) {
            printError("Нет данных в IMU.txt");
            continue;
        }

        // 4. Сглаживание для ускорений и угловых скоростей
        vector<double> ax_sm, ay_sm, az_sm, wx_sm, wy_sm, wz_sm;
#if USE_GAUSSIAN == 2
        hybridSmooth(ax, ax_sm, WINDOW);
        hybridSmooth(ay, ay_sm, WINDOW);
        hybridSmooth(az, az_sm, WINDOW);
        hybridSmooth(wx, wx_sm, WINDOW);
        hybridSmooth(wy, wy_sm, WINDOW);
        hybridSmooth(wz, wz_sm, WINDOW);
#elif USE_GAUSSIAN == 1
        vector<double> kernel = gaussianKernel(WINDOW);
        gaussianFilter(ax, ax_sm, kernel);
        gaussianFilter(ay, ay_sm, kernel);
        gaussianFilter(az, az_sm, kernel);
        gaussianFilter(wx, wx_sm, kernel);
        gaussianFilter(wy, wy_sm, kernel);
        gaussianFilter(wz, wz_sm, kernel);
#else
        movingAverage(ax, ax_sm, WINDOW);
        movingAverage(ay, ay_sm, WINDOW);
        movingAverage(az, az_sm, WINDOW);
        movingAverage(wx, wx_sm, WINDOW);
        movingAverage(wy, wy_sm, WINDOW);
        movingAverage(wz, wz_sm, WINDOW);
#endif

        // 5. Расчёт углов (raw и smoothed)
        vector<double> pitch_r(n), roll_r(n), yaw_r(n);
        vector<double> pitch_s(n), roll_s(n), yaw_s(n);

        double sum_pr = 0, sum_rr = 0, sum_yr = 0;
        double sum_ps = 0, sum_rs = 0, sum_ys = 0;

        for (int i = 0; i < n; i++) {
            // raw — по исходным данным
            data.time = t[i];
            data.ax = ax[i]; data.ay = ay[i]; data.az = az[i];
            data.wx = wx[i]; data.wy = wy[i]; data.wz = wz[i];
            calculateAngles(data, g);
            pitch_r[i] = data.pitch;
            roll_r[i]  = data.roll;
            yaw_r[i]   = data.yaw;
            sum_pr += pitch_r[i];
            sum_rr += roll_r[i];
            sum_yr += yaw_r[i];

            // smoothed — по сглаженным данным
            data.ax = ax_sm[i]; data.ay = ay_sm[i]; data.az = az_sm[i];
            data.wx = wx_sm[i]; data.wy = wy_sm[i]; data.wz = wz_sm[i];
            calculateAngles(data, g);
            pitch_s[i] = data.pitch;
            roll_s[i]  = data.roll;
            yaw_s[i]   = data.yaw;
            sum_ps += pitch_s[i];
            sum_rs += roll_s[i];
            sum_ys += yaw_s[i];
        }

        double mean_p_r = sum_pr / n;
        double mean_r_r = sum_rr / n;
        double mean_y_r = sum_yr / n;
        double mean_p_s = sum_ps / n;
        double mean_r_s = sum_rs / n;
        double mean_y_s = sum_ys / n;

        // 6. Вывод таблицы в консоль (первые PRINT_LIMIT строк)
        cout << string(142, '=') << "\n";
        cout << "  Time     Ax      Ay      Az      Wx      Wy      Wz     "
                "Pitch_r Roll_r  Yaw_r  "
                "Ax_sm   Ay_sm   Az_sm   Wx_sm   Wy_sm   Wz_sm  "
                "Pitch_s Roll_s  Yaw_s\n";
        cout << string(142, '=') << "\n";

        int show = (n < PRINT_LIMIT) ? n : PRINT_LIMIT;
        for (int i = 0; i < show; i++) {
            cout << fixed << setprecision(4)
                 << setw(8) << t[i] << " "
                 << setprecision(3)
                 << setw(7) << ax[i] << " " << setw(7) << ay[i] << " "
                 << setw(7) << az[i] << " "
                 << setw(7) << wx[i] << " " << setw(7) << wy[i] << " "
                 << setw(7) << wz[i] << " "
                 << setprecision(2)
                 << setw(8) << pitch_r[i] << " "
                 << setw(7) << roll_r[i] << " "
                 << setw(7) << yaw_r[i] << "  "
                 << setprecision(3)
                 << setw(7) << ax_sm[i] << " " << setw(7) << ay_sm[i] << " "
                 << setw(7) << az_sm[i] << " "
                 << setw(7) << wx_sm[i] << " " << setw(7) << wy_sm[i] << " "
                 << setw(7) << wz_sm[i] << " "
                 << setprecision(2)
                 << setw(8) << pitch_s[i] << " "
                 << setw(7) << roll_s[i] << " "
                 << setw(7) << yaw_s[i] << "\n";
        }

        if (n > PRINT_LIMIT)
            cout << "  ... (" << (n - PRINT_LIMIT) << " строк пропущено)\n";

        cout << string(142, '=') << "\n";
        cout << "\nВсего строк: " << n << "\n";
        cout << "Средние углы RAW:     Pitch = " << setprecision(4) << mean_p_r
             << " rad  Roll = " << mean_r_r << " rad  Yaw = " << mean_y_r << " rad\n";
        cout << "Средние углы SMOOTH:  Pitch = " << mean_p_s
             << " rad  Roll = " << mean_r_s << " rad  Yaw = " << mean_y_s << " rad\n\n";

        // 7. Сохранение в файл
        string out_name = "output_" + fs::path(folder).filename().string() + ".txt";
        ofstream fout(out_name);
        if (fout.is_open()) {
            fout << "Time Ax Ay Az Wx Wy Wz "
                    "Pitch_raw Roll_raw Yaw_raw "
                    "Ax_sm Ay_sm Az_sm Wx_sm Wy_sm Wz_sm "
                    "Pitch_sm Roll_sm Yaw_sm\n";
            fout << "s m/s2 m/s2 m/s2 deg/s deg/s deg/s "
                    "rad rad rad "
                    "m/s2 m/s2 m/s2 deg/s deg/s deg/s "
                    "rad rad rad\n";
            for (int i = 0; i < n; i++) {
                fout << fixed << setprecision(4) << t[i]
                     << setprecision(6)
                     << " " << ax[i] << " " << ay[i] << " " << az[i]
                     << " " << wx[i] << " " << wy[i] << " " << wz[i]
                     << setprecision(4)
                     << " " << pitch_r[i]
                     << " " << roll_r[i]
                     << " " << yaw_r[i]
                     << setprecision(6)
                     << " " << ax_sm[i] << " " << ay_sm[i] << " " << az_sm[i]
                     << " " << wx_sm[i] << " " << wy_sm[i] << " " << wz_sm[i]
                     << setprecision(4)
                     << " " << pitch_s[i]
                     << " " << roll_s[i]
                     << " " << yaw_s[i] << "\n";
            }
            fout << "\n# Latitude = " << lat << " deg\n";
            fout << "# g = " << g << " m/s2\n";
            fout << "# Window = " << WINDOW << " samples\n";
            fout << "# Mean Pitch raw = " << mean_p_r << " rad\n";
            fout << "# Mean Roll raw  = " << mean_r_r << " rad\n";
            fout << "# Mean Yaw raw   = " << mean_y_r << " rad\n";
            fout << "# Mean Pitch sm  = " << mean_p_s << " rad\n";
            fout << "# Mean Roll sm   = " << mean_r_s << " rad\n";
            fout << "# Mean Yaw sm    = " << mean_y_s << " rad\n";
            fout.close();
            cout << "Результаты сохранены в " << out_name << "\n\n";
        }
    }

    cout << string(60, '=') << "\n";
    cout << "  Обработка всех папок завершена.\n";
    cout << string(60, '=') << "\n";
    return 0;
}
