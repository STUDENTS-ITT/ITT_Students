// main.cpp — Консольная точка входа: автономная выставка + счисление БИНС
// с фильтром Калмана.
//
// Вся математика вынесена в esfcore (nav::runSimulation), main отвечает только
// за разбор аргументов, поиск каталогов и вывод. Тот же расчёт выполняет
// Qt-приложение imitator_gui, поэтому файлы результатов совпадают.
//
// Использование:
//   imitator [каталог_данных] [settings.ini]
//   каталог_данных по умолчанию — ../data/raw
//   результаты пишутся в tools/ рядом с исполняемым файлом

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "core/navigation/simulation.h"
#include "core/utils/constants.h"
#include "core/utils/paths.h"

int main(int argc, char **argv)
{
    // argv[1] — каталог данных (по умолчанию ../data/raw), argv[2] — settings.ini.
    const std::string data_dir = (argc > 1) ? argv[1] : "../data/raw";
    const std::string settings_file = (argc > 2) ? argv[2] : "";

    // Совместимость со старым params.ini (без секций) обеспечивает сам runner:
    // nav::runSimulation определяет формат файла и читает ключи соответственно.

    const std::filesystem::path tools_dir = utils::toolsDir(argv[0]);

    nav::RunOptions options;
    options.data_dir = data_dir;
    options.output_dir = tools_dir.string();
    options.settings_file = settings_file;
    options.write_files = true;
    options.capture_telemetry = false;

    nav::RunSummary summary;
    const bool ok = nav::runSimulation(options, summary, nullptr, nullptr, &std::cout);

    if (!ok)
    {
        if (summary.error.empty() || summary.error == "cancelled")
        {
            std::cerr << "imitator: run failed" << std::endl;
        }
        return 1;
    }

    std::cout << summary.wall_seconds << " s" << std::endl;

    return 0;
}