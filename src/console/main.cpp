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
//
// Дополнительные флаги (Этап 0 плана docs/bias_estimation_plan.md):
//   --bias-log     писать bias.txt — журнал оценок смещений ДУС/акселя
//   --bias-report  прогнать набор и вывести сводную таблицу по смещениям
//                  (истина из params.json / оценка / σ(P) / норм. ошибка)

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "core/navigation/bias_report.h"
#include "core/navigation/simulation.h"
#include "core/utils/constants.h"
#include "core/utils/paths.h"

int main(int argc, char **argv)
{
    // Разбор аргументов: флаги --bias-* идёт перед позиционными, чтобы не
    // сломать существующий вызов `imitator <каталог> <settings.ini>`.
    bool write_bias_log = false;
    bool bias_report = false;
    std::vector<std::string> positional;

    for (int i = 1; i < argc; i++)
    {
        if (std::strcmp(argv[i], "--bias-log") == 0) write_bias_log = true;
        else if (std::strcmp(argv[i], "--bias-report") == 0) bias_report = true;
        else positional.push_back(argv[i]);
    }

    // argv[1] — каталог данных (по умолчанию ../data/raw), argv[2] — settings.ini.
    const std::string data_dir = positional.empty() ? "../data/raw" : positional[0];
    const std::string settings_file = positional.size() > 1 ? positional[1] : "";

    // Совместимость со старым params.ini (без секций) обеспечивает сам runner:
    // nav::runSimulation определяет формат файла и читает ключи соответственно.

    const std::filesystem::path tools_dir = utils::toolsDir(argv[0]);

    nav::RunOptions options;
    options.data_dir = data_dir;
    options.output_dir = tools_dir.string();
    options.settings_file = settings_file;
    options.write_files = true;
    options.write_bias_log = write_bias_log;
    options.capture_telemetry = false;

    // Режим --bias-report: прогон с журналом смещений + сводная таблица.
    // result/reference/errors пишутся как обычно, добавляется bias.txt.
    if (bias_report)
    {
        nav::RunSummary report_summary;
        std::string report;
        std::string error;
        const bool ok = nav::runBiasReport(options, report_summary, report, error);
        std::cout << report;
        if (!ok)
        {
            std::cerr << "imitator: bias-report failed: " << error << std::endl;
            return 1;
        }
        return 0;
    }

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

    if (summary.invalid_imu_rows > 0)
    {
        std::cout << "WARNING: skipped " << summary.invalid_imu_rows
                  << " malformed imu.dat rows" << std::endl;
    }

    std::cout << summary.wall_seconds << " s" << std::endl;

    return 0;
}