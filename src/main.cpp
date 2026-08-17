// main.cpp — Точка входа: автономная выставка + счисление БИНС с фильтром Калмана.
//
// Поток выполнения:
//   1. Разведочный проход по эталону (gps.dat + angle.dat) — первый отсчёт
//      и момент начала движения.
//   2. Автономная выставка (Median + EMA фильтры) — начальные углы ориентации.
//   3. Формирование начального состояния: углы из выставки, координаты и
//      скорости — из первого отсчёта СНС.
//   4. Начальный участок (180 с): положение не меняется, данные пишутся,
//      но фильтр Калмана ещё не работает.
//   5. Основной цикл: на каждом такте ИМУ — интегрирование БИНС, коррекция
//      по СНС каждые 200 отсчётов (1 Гц), запись результатов.

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "data_io/data_reader.h"
#include "data_io/data_writer.h"
#include "ins/imu_processor.h"
#include "navigation/aligner.h"
#include "navigation/bins_alignment.h"
#include "navigation/gps_processor.h"
#include "navigation/trajectory.h"
#include "navigation/aligner.hpp"
#include "utils/constants.h"
#include "utils/paths.h"

namespace
{

// Количество отсчётов ИМУ на начальном участке (180 с при 200 Гц).
constexpr int START_SAMPLES = 180 * SNS_DECIMATION;

// Формирование строки для записи на начальном участке: Калман ещё не
// работает, пишутся стартовые значения и текущий отсчёт эталона.
data_io::NavRecord startRecord(double time, const nav::NavState &st, const nav::SnsSample &ref)
{
    return nav::makeRecord(time, st, ref, st.att.heading, st.lat);
}

} // namespace

int main(int argc, char **argv)
{
    // Каталог с входными данными (относительно рабочего каталога).
    const std::string data_dir = "../data/raw";

    // Имена файлов данных и результатов.
    const std::string imu_file = data_dir + "/imu.dat";
    const std::string gps_file = data_dir + "/gps.dat";
    const std::string angle_file = data_dir + "/angle.dat";
    const std::string out_file = "kalman15_line2.txt";  // траектория
    const std::string err_file = "d_1.txt";              // ошибки фильтра

    const auto start_time = std::chrono::high_resolution_clock::now();

    // === Этап 1: разведочный проход по эталону ===
    // Один проход по gps.dat + angle.dat: запоминаем первый отсчёт
    // (координаты, скорость) и время начала движения (граница выставки).
    const nav::SnsScan scan = nav::scanSns(gps_file, angle_file);
    if (!scan.ok)
    {
        std::cerr << "no reference data" << std::endl;
        return 1;
    }

    // === Этап 2: автономная выставка (Median + EMA фильтры) ===
    // Широта и высота — из первого отсчёта gps.dat.
    // Время окончания выставки — из scanSns (момент начала движения).
    double Yaw_0 = 0.0, Pitch_0 = 0.0, Roll_0 = 0.0;

    std::cout << "=== Alignment ===" << std::endl;
    get_angle_start(&Yaw_0, &Pitch_0, &Roll_0,
                    imu_file.c_str(),
                    scan.first.lat * RAD_TO_DEG,
                    scan.first.alt,
                    scan.motion_start);
    std::cout << "Yaw: " << Yaw_0 * 180.0 / PI
              << ", Pitch: " << Pitch_0 * 180.0 / PI
              << ", Roll: " << Roll_0 * 180.0 / PI << std::endl;

    // === Этап 3: формирование начального состояния ===
    // Углы — из автономной выставки, координаты и скорости — из первого отсчёта СНС.
    nav::NavState state = nav::initialAlignment(scan.first, Yaw_0, Pitch_0, Roll_0);
    std::cout << state.att.heading << "        " << state.att.roll << "        "
              << state.att.pitch << std::endl;

    // === Этап 4: открытие потоков данных ===
    // ImuReader — последовательное чтение imu.dat (заголовок пропускается).
    // SnsReader — параллельное чтение gps.dat и angle.dat (синхронно построчно).
    data_io::ImuReader imu;
    if (!imu.open(imu_file))
    {
        return 1;
    }
    data_io::SnsReader sns;
    if (!sns.open(gps_file, angle_file))
    {
        return 1;
    }

    // NavLogger — два выходных файла: траектория и ошибки фильтра.
    data_io::NavLogger log;
    if (!log.open(out_file, err_file))
    {
        return 1;
    }
    log.writeHeader();

    std::vector<double> row;   // строка imu.dat
    nav::SnsSample ref;        // отсчёт эталона (gps + angle)
    int i = 0;                 // счётчик отсчётов ИМУ

    // Начальный участок (180 с): положение неизменным.
    // Фильтр Калмана не работает, пишутся только стартовые значения.
    while (i < START_SAMPLES && imu.next(row))
    {
        if (!ins::isValidRow(row))
        {
            continue;
        }
        if (!sns.next(ref))
        {
            break;
        }
        if (i % SNS_DECIMATION == 0)
        {
            log.write(startRecord(ins::sampleTime(row), state, ref));
        }
        state.time_prev = ins::sampleTime(row);
        i++;
    }

    // Основной цикл счисления с фильтром Калмана.
    // На каждом такте: интегрирование скоростей, координат, ориентации.
    // Раз в 200 отсчётов (1 Гц): коррекция по эталону.
    while (imu.next(row))
    {
        if (!ins::isValidRow(row))
        {
            continue;
        }
        if (!sns.next(ref))
        {
            break;
        }
        nav::step(i, row, ref, state, log);
        i++;
    }

    imu.close();
    sns.close();
    log.close();

    const auto end_time = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << elapsed.count() << " s" << std::endl;

    return 0;
}
