// main.cpp — Точка входа: автономная выставка + счисление БИНС с фильтром Калмана.
//
// Поток выполнения:
//   1. Чтение конфигурации (StartupNav.ini) — координаты и время выставки.
//   2. Автономная выставка (Median + EMA фильтры) — начальные углы ориентации.
//   3. Формирование начального состояния: углы из выставки, координаты из конфига,
//      скорости нулевые.
//   4. Основной цикл: на каждом такте ИМУ — интегрирование БИНС, запись
//      результатов (400 Гц), коррекция pitch/roll по акселю (400 Гц),
//      коррекция по СНС при обновлении gps (1 Гц).

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "data_io/data_reader.h"
#include "data_io/data_writer.h"
#include "ins/imu_processor.h"
#include "navigation/aligner.h"
#include "navigation/gps_processor.h"
#include "navigation/trajectory.h"
#include "navigation/aligner.hpp"
#include "utils/constants.h"
#include "utils/paths.h"

namespace
{

bool readStartupNav(const std::string &path, double &lon_deg, double &lat_deg, double &alt, double &time_s)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        return false;
    }

    std::string line;

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &lon_deg);

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &lat_deg);

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &alt);

    if (!std::getline(f, line)) return false;
    sscanf(line.c_str(), "%lf", &time_s);

    return true;
}

} // namespace

int main(int argc, char **argv)
{
    const std::string data_dir = "../data/raw";

    const std::filesystem::path tools_dir = utils::toolsDir(argv[0]);
    const std::string result_file = (tools_dir / "result.txt").string();
    const std::string reference_file = (tools_dir / "reference.txt").string();
    const std::string err_file = (tools_dir / "errors.txt").string();

    const std::string imu_file = data_dir + "/imu.dat";
    const std::string gps_file = data_dir + "/gps.dat";
    const std::string angle_file = data_dir + "/angle.dat";
    const std::string startup_file = data_dir + "/StartupNav.ini";

    const bool has_angle = std::filesystem::exists(angle_file);
    std::cout << "angle.dat: " << (has_angle ? "found" : "not found (angles will be zero)") << std::endl;

    const auto start_time = std::chrono::high_resolution_clock::now();

    double start_lon = 0.0, start_lat = 0.0, start_alt = 0.0, align_time = 120.0;

    if (readStartupNav(startup_file, start_lon, start_lat, start_alt, align_time))
    {
        std::cout << "StartupNav: lon=" << start_lon << " lat=" << start_lat
                  << " alt=" << start_alt << " time=" << align_time << std::endl;
    }
    else
    {
        std::cout << "StartupNav not found, reading from gps.dat..." << std::endl;
        data_io::SnsReader sns_tmp;
        if (!sns_tmp.open(gps_file, has_angle ? angle_file : ""))
        {
            std::cerr << "no reference data" << std::endl;
            return 1;
        }
        nav::SnsSample first;
        sns_tmp.next(first);
        sns_tmp.close();

        start_lon = first.lon * RAD_TO_DEG;
        start_lat = first.lat * RAD_TO_DEG;
        start_alt = first.alt;
        align_time = 120.0;
    }

    double Yaw_0 = 0.0, Pitch_0 = 0.0, Roll_0 = 0.0;
    double ba_x = 0.0, ba_y = 0.0, ba_z = 0.0;

    std::cout << "=== Alignment ===" << std::endl;
    get_angle_start(&Yaw_0, &Pitch_0, &Roll_0, &ba_x, &ba_y, &ba_z,
                    imu_file.c_str(),
                    start_lat, start_alt, align_time);
    std::cout << "[Degree] Yaw: " << Yaw_0 * 180.0 / PI
              << ", Pitch: " << Pitch_0 * 180.0 / PI
              << ", Roll: " << Roll_0 * 180.0 / PI << std::endl;
    std::cout << "ba: " << ba_x << ", " << ba_y << ", " << ba_z << std::endl;

    const Vector ba0 = {ba_x, ba_y, ba_z};

    nav::NavState state = nav::initialAlignment(start_lat * DEG_TO_RAD, start_lon * DEG_TO_RAD, start_alt,
                                                Yaw_0, Pitch_0, Roll_0, ba0);
    std::cout << "[Rad] Yaw: " << state.att.heading << ", Pitch: " << state.att.pitch << ", Roll: "
              << state.att.roll << std::endl;
    std::cout << "Output: " << tools_dir.string() << std::endl;

    data_io::ImuReader imu;
    if (!imu.open(imu_file))
    {
        return 1;
    }
    data_io::SnsReader sns;
    if (!sns.open(gps_file, has_angle ? angle_file : ""))
    {
        return 1;
    }

    data_io::NavLogger log;
    if (!log.open(result_file, reference_file, err_file))
    {
        return 1;
    }
    log.writeHeader();

    std::vector<double> row;
    nav::SnsSample ref;
    nav::SnsSample last_ref;
    nav::SnsSample hold_ref;
    bool has_ref = false;
    bool hold_valid = false;

    while (imu.next(row))
    {
        if (!ins::isValidRow(row))
        {
            continue;
        }

        const double imu_time = row[0];
        const double prev_gps_time = hold_valid ? hold_ref.time : -1.0;

        while (!has_ref || last_ref.time <= imu_time)
        {
            if (has_ref && last_ref.time <= imu_time)
            {
                hold_ref = last_ref;
                hold_valid = true;
            }
            if (!sns.next(last_ref))
            {
                break;
            }
            has_ref = true;
            if (last_ref.time > imu_time)
            {
                break;
            }
        }

        if (!hold_valid)
        {
            continue;
        }

        const bool do_correction = (hold_ref.time != prev_gps_time);

        ref = hold_ref;
        nav::step(row, ref, state, log, do_correction, has_angle);
    }

    imu.close();
    sns.close();
    log.close();

    const auto end_time = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << elapsed.count() << " s" << std::endl;

    return 0;
}
