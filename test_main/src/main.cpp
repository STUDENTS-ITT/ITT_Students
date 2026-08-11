// Точка входа: чтение исходных данных, автономная начальная выставка,
// счисление БИНС с коррекцией по СНС и запись результатов.

#include <chrono>
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
#include "utils/constants.h"
#include "utils/paths.h"

namespace
{

const std::string IMU_FILE = "imu.dat";
const std::string GPS_FILE = "gps.dat";
const std::string ANGLE_FILE = "angle.dat";
const std::string OUT_FILE = "kalman15_line2.txt";
const std::string ERR_FILE = "d_1.txt";
const std::string ALIGN_FILE = "Angles.dat";

// начальный участок без счисления, отсчётов ИМУ (180 с)
constexpr int START_SAMPLES = 180 * SNS_DECIMATION;

// Строка результатов на начальном участке: решение БИНС ещё не считается,
// пишутся стартовые значения и текущий отсчёт эталона.
data_io::NavRecord startRecord(double time, const nav::NavState &st, const nav::SnsSample &ref)
{
    return nav::makeRecord(time, st, ref, st.att.heading, st.lat);
}

} // namespace

int main(int argc, char **argv)
{
    const std::filesystem::path dir = utils::dataDir(argc > 0 ? argv[0] : nullptr, IMU_FILE);

    const auto start = std::chrono::high_resolution_clock::now();

    // разведочный проход по эталону: стартовый отсчёт и граница неподвижности
    const nav::SnsScan scan = nav::scanSns((dir / GPS_FILE).string(), (dir / ANGLE_FILE).string());
    if (!scan.ok)
    {
        std::cerr << "no reference data" << std::endl;
        return 1;
    }

    const nav::AlignmentResult align = nav::alignBins((dir / IMU_FILE).string(), scan);
    if (!align.ok)
    {
        std::cerr << "alignment failed, falling back to angle.dat" << std::endl;
    }
    const ins::Attitude att0 = align.ok ? align.att : nav::referenceAttitude(scan.first);

    nav::NavState state = nav::initialAlignment(scan.first, att0);
    std::cout << state.att.heading << "        " << state.att.roll << "        "
              << state.att.pitch << std::endl;

    if (align.ok)
    {
        const ins::Attitude ref = nav::referenceAttitude(scan.first);
        std::cout << "alignment: " << align.samples << " samples over " << align.duration
                  << " s, g = " << align.g << std::endl;
        std::cout << "reference: " << ref.heading << "        " << ref.roll << "        "
                  << ref.pitch << std::endl;
        data_io::writeAlignment((dir / ALIGN_FILE).string(),
                                state.att.heading, state.att.pitch, state.att.roll);
    }

    // основные потоки: imu.dat и эталон читаются построчно синхронно
    data_io::ImuReader imu;
    if (!imu.open((dir / IMU_FILE).string()))
    {
        return 1;
    }
    data_io::SnsReader sns;
    if (!sns.open((dir / GPS_FILE).string(), (dir / ANGLE_FILE).string()))
    {
        return 1;
    }

    data_io::NavLogger log;
    if (!log.open((dir / OUT_FILE).string(), (dir / ERR_FILE).string()))
    {
        return 1;
    }
    log.writeHeader();

    std::vector<double> row;
    nav::SnsSample ref;
    int i = 0;

    // Начальный участок: положение считается неизменным. Отсчёт эталона
    // забирается только под годную строку ИМУ, иначе потоки разъедутся.
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

    // основной цикл счисления
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

    const auto end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed = end - start;
    std::cout << elapsed.count() << std::endl;

    return 0;
}
