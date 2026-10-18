// main.cpp — Точка входа imitator_gui.
//
// Приложение не содержит математики: только Qt-обвязка над ядром esfcore.
// Фильтр (15 состояний), матричные операции, чтение/запись данных и основной
// цикл счисления выполняются кодом консольного imitator.

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QLocale>
#include <QTextStream>
#include <QTimer>
#include <QTranslator>

#include "MainWindow.h"
#include "core/data_io/dataset_generator.h"
#include "core/navigation/bias_report.h"
#include "core/navigation/simulation.h"
#include "export/tuning_report.h"
#include "settings/app_settings.h"

int main(int argc, char *argv[])
{
    // --self-test должен работать без окна: платформа задаётся до создания
    // QApplication, иначе Qt успеет выбрать графическую платформу.
    bool self_test = false;
    int self_test_timeout_s = 600;
    for (int i = 1; i < argc; i++)
    {
        if (qstrcmp(argv[i], "--self-test") == 0)
        {
            self_test = true;
            qputenv("QT_QPA_PLATFORM", "offscreen");
        }
        else if (qstrncmp(argv[i], "--self-test-generate", 20) == 0)
        {
            qputenv("QT_QPA_PLATFORM", "offscreen");
        }
        else if (qstrncmp(argv[i], "--export-matrices", 17) == 0)
        {
            qputenv("QT_QPA_PLATFORM", "offscreen");
        }
        else if (qstrncmp(argv[i], "--tuning-report", 15) == 0)
        {
            qputenv("QT_QPA_PLATFORM", "offscreen");
        }
        else if (qstrncmp(argv[i], "--bias-report", 13) == 0)
        {
            qputenv("QT_QPA_PLATFORM", "offscreen");
        }
        else if (qstrncmp(argv[i], "--self-test-timeout=", 20) == 0)
        {
            self_test_timeout_s = QString::fromLatin1(argv[i] + 20).toInt();
        }
    }

    // У offscreen-платформы (её выбирают режимы --self-test/--tuning-report)
    // нет собственной базы шрифтов: без каталога шрифтов Qt рисует текст в
    // PDF-отчётах «квадратами». Подсказываем системный каталог шрифтов, если
    // он не задан извне (можно переопределить через QT_QPA_FONTDIR).
    if (qgetenv("QT_QPA_PLATFORM") == QByteArrayLiteral("offscreen") &&
        qEnvironmentVariableIsEmpty("QT_QPA_FONTDIR"))
    {
        const QString fonts =
            QDir(qEnvironmentVariable("WINDIR", QStringLiteral("C:/Windows")))
                .filePath(QStringLiteral("Fonts"));
        if (QFileInfo::exists(fonts))
            qputenv("QT_QPA_FONTDIR", fonts.toLocal8Bit());
    }

    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("imitator_gui"));
    QApplication::setApplicationVersion(QStringLiteral("1.0"));
    QApplication::setOrganizationName(QStringLiteral("ITT"));
    QApplication::setOrganizationDomain(QStringLiteral("itt.local"));

    // Числа в русской локали показываем с точкой в качестве разделителя:
    // формат файлов (result.txt, settings.ini) от этого не зависит.
    QLocale::setDefault(QLocale::c());

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QCoreApplication::translate(
            "main",
            "Qt-обвязка над БИНС/СНС-комплексом с фильтром Калмана (ESKF, 15 состояний).\n"
            "Математика выполняется в общем с консольным imitator ядре."));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption data_opt({"d", "data"},
                                 QCoreApplication::translate("main", "Каталог с imu.dat / gps.dat / angle.dat / StartupNav.ini."),
                                 QCoreApplication::translate("main", "dir"));
    QCommandLineOption out_opt({"o", "output"},
                               QCoreApplication::translate("main", "Каталог для result.txt / reference.txt / errors.txt."),
                               QCoreApplication::translate("main", "dir"));
    QCommandLineOption settings_opt({"s", "settings"},
                                    QCoreApplication::translate("main", "Файл параметров фильтра."),
                                    QCoreApplication::translate("main", "file"));
    QCommandLineOption dark_opt({"t", "dark"},
                                QCoreApplication::translate("main", "Тёмная тема оформления."));
    QCommandLineOption autostart_opt("autostart",
                                     QCoreApplication::translate("main", "Запустить расчёт сразу при открытии окна."));
    QCommandLineOption self_test_opt("self-test",
                                     QCoreApplication::translate("main",
                                         "Без окна: выполнить расчёт и выйти с кодом 0 (успех) или 1 (ошибка). Для CI."));
    QCommandLineOption self_test_timeout_opt("self-test-timeout",
                                             QCoreApplication::translate("main",
                                                 "Предельное время ожидания в режиме --self-test, с (по умолчанию 600)."),
                                             QCoreApplication::translate("main", "sec"),
                                             "600");
    QCommandLineOption self_test_report_opt("self-test-report",
                                            QCoreApplication::translate("main",
                                                "Файл для отчёта о прогоне в режиме --self-test (с логом)."),
                                            QCoreApplication::translate("main", "file"));
    QCommandLineOption self_test_generate_opt("self-test-generate",
                                              QCoreApplication::translate("main",
                                                  "Сгенерировать тестовый набор данных в указанный каталог и выйти (для CI)."),
                                              QCoreApplication::translate("main", "dir"));
    QCommandLineOption export_matrices_opt("export-matrices",
                                           QCoreApplication::translate("main",
                                               "Без окна: экспортировать статические матрицы фильтра "
                                               "(Q/R/P0/H) в указанный каталог (csv/md/json) и выйти. Для CI."),
                                           QCoreApplication::translate("main", "dir"));
    QCommandLineOption tuning_report_opt("tuning-report",
                                         QCoreApplication::translate("main",
                                             "Без окна: собрать PDF-отчёт о подборе по конфигу тюнера "
                                             "(tuner_config*.json): best_params.txt и eval.json из его work_root."),
                                         QCoreApplication::translate("main", "config.json"));
    QCommandLineOption tuning_report_out_opt("tuning-report-out",
                                             QCoreApplication::translate("main",
                                                 "Куда сохранить отчёт о подборе "
                                                 "(по умолчанию — reports/<набор>/tuning_<набор>.pdf)."),
                                             QCoreApplication::translate("main", "file.pdf"));
    QCommandLineOption bias_report_opt("bias-report",
                                       QCoreApplication::translate("main",
                                           "Без окна: прогнать набор и вывести сводную таблицу по смещениям ИМУ "
                                           "(истина из params.json / оценка / σ(P) / нормированная ошибка) — "
                                           "этап 0 docs/bias_estimation_plan.md. Каталог набора берётся из --data."));

    parser.addOption(data_opt);
    parser.addOption(out_opt);
    parser.addOption(settings_opt);
    parser.addOption(dark_opt);
    parser.addOption(autostart_opt);
    parser.addOption(self_test_opt);
    parser.addOption(self_test_timeout_opt);
    parser.addOption(self_test_report_opt);
    parser.addOption(self_test_generate_opt);
    parser.addOption(export_matrices_opt);
    parser.addOption(tuning_report_opt);
    parser.addOption(tuning_report_out_opt);
    parser.addOption(bias_report_opt);
    parser.process(app);

    QString report_file;
    if (parser.isSet(self_test_report_opt))
        report_file = QFileInfo(parser.value(self_test_report_opt)).absoluteFilePath();

    if (parser.isSet(self_test_timeout_opt))
        self_test_timeout_s = parser.value(self_test_timeout_opt).toInt();

    // Пользовательские настройки читаем до создания окна, чтобы применить
    // переданные командной строкой значения.
    gui::AppSettings settings = gui::AppSettings::load();

    if (parser.isSet(data_opt))
    {
        settings.data_dir = QDir(parser.value(data_opt)).absolutePath();
    }
    if (parser.isSet(out_opt))
    {
        settings.output_dir = QDir(parser.value(out_opt)).absolutePath();
    }
    if (parser.isSet(settings_opt))
    {
        settings.settings_file = QFileInfo(parser.value(settings_opt)).absoluteFilePath();
    }
    if (parser.isSet(dark_opt))
    {
        settings.dark_theme = true;
    }

    // В режиме --self-test-generate приложение не создаёт окно, а только
    // синтезирует набор данных (для smoke-теста, чтобы не зависеть от
    // датасета в репозитории) и выходит.
    if (parser.isSet(self_test_generate_opt))
    {
        dataset::DatasetConfig cfg;
        cfg.name = "gui_smoke";
        cfg.out_dir = QDir(parser.value(self_test_generate_opt)).absolutePath().toStdString();
        cfg.profile = {
            {cfg.align_time_s, 0.0, 0.0, 0.0},   // выставка (покой)
            {20.0, 10.0, 0.0, 0.0},
            {40.0, 10.0, 0.0, 1.5},
            {20.0, 10.0, 0.0, 0.0},
            {20.0, 0.0, 0.0, 0.0},
        };

        dataset::DatasetReport report;
        std::string gen_error;
        const bool gen_ok = dataset::generateDataset(cfg, report, &gen_error);

        QTextStream out(stdout);
        if (gen_ok)
        {
            out << QStringLiteral("self-test-generate: IMU=%1, GPS=%2, ANGLE=%3, TRUTH=%4 (%5 с) -> %6\n")
                       .arg(report.imu_rows)
                       .arg(report.gps_rows)
                       .arg(report.angle_rows)
                       .arg(report.truth_rows)
                       .arg(report.duration_s)
                       .arg(QString::fromStdString(cfg.out_dir));
        }
        else
        {
            out << QStringLiteral("self-test-generate: ошибка: %1\n")
                       .arg(QString::fromStdString(gen_error));
        }
        out.flush();
        return gen_ok ? 0 : 1;
    }

    // Режим --export-matrices: без окна собираем статические матрицы фильтра
    // (Q/R/P0/H) для текущих настроек/датасета и сохраняем csv/md/json.
    if (parser.isSet(export_matrices_opt))
    {
        const QString dir = QDir(parser.value(export_matrices_opt)).absolutePath();
        gui::MainWindow window(settings, /*autostart=*/false);

        QString error;
        const bool ok = window.exportMatricesTo(dir, &error);

        QTextStream out(stdout);
        if (ok)
            out << QStringLiteral("export-matrices: ok -> %1\n").arg(dir);
        else
            out << QStringLiteral("export-matrices: ошибка: %1\n").arg(error);
        out.flush();
        return ok ? 0 : 1;
    }

    // Режим --tuning-report: без окна собираем PDF-отчёт о подборе по конфигу
    // тюнера (best_params.txt и eval.json из его work_root). Приложение только
    // читает артефакты подбора и верстает PDF — математики здесь нет.
    if (parser.isSet(tuning_report_opt))
    {
        const QString config =
            QFileInfo(parser.value(tuning_report_opt)).absoluteFilePath();

        QString out_path;
        if (parser.isSet(tuning_report_out_opt))
        {
            out_path = QFileInfo(parser.value(tuning_report_out_opt)).absoluteFilePath();
        }
        else
        {
            // По умолчанию — в репозиторный каталог отчётов:
            // reports/<набор>/tuning_<набор>.pdf.
            QString dataset = gui::TuningReport::datasetFor(config);
            if (dataset.isEmpty()) dataset = QStringLiteral("dataset");
            out_path = QDir(QDir(gui::AppSettings::defaultReportsDir()).filePath(dataset))
                           .filePath(QStringLiteral("tuning_%1.pdf").arg(dataset));
        }

        QString error;
        const bool ok = gui::TuningReport::build(config, out_path, &error);

        QTextStream out(stdout);
        if (ok)
            out << QStringLiteral("tuning-report: ok -> %1\n").arg(out_path);
        else
            out << QStringLiteral("tuning-report: ошибка: %1\n").arg(error);
        out.flush();
        return ok ? 0 : 1;
    }

    // Режим --bias-report (этап 0 плана оценивания смещений): без окна
    // прогоняем набор с журналом смещений и печатаем сводную таблицу
    // «истина / оценка / σ(P) / нормированная ошибка» на окнах 1…300 с.
    // Математика целиком в esfcore (nav::runBiasReport), поэтому результат
    // совпадает с консольным imitator --bias-report.
    //
    // imitator_gui — приложение WIN32-подсистемы, его stdout на Windows не
    // виден в консоли (это проверено: существующие --self-test-* режимы тоже
    // не выводят текст), поэтому таблица пишется в файл, а в stdout идёт путь —
    // тот же приём, что у --tuning-report (он пишет PDF и печатает «ok ->»).
    if (parser.isSet(bias_report_opt))
    {
        nav::RunOptions options;
        options.data_dir = settings.data_dir.toStdString();
        options.output_dir = settings.output_dir.toStdString();
        options.settings_file = settings.settings_file.toStdString();
        options.write_files = true;      // result/reference/errors пишутся как обычно
        options.write_bias_log = true;   // + bias.txt (принудительно ради отчёта)

        nav::RunSummary summary;
        std::string report, error;
        bool ok = nav::runBiasReport(options, summary, report, error);

        QTextStream out(stdout);
        if (ok)
        {
            // Отчёт рядом с журналом смещений (bias.txt) — в каталоге вывода.
            const QString report_path =
                QDir(QString::fromStdString(options.output_dir))
                    .filePath(QStringLiteral("bias_report.txt"));

            // UTF-8 с BOM: в отчёте есть кириллица, а без BOM PowerShell 5.1
            // читает файл как ANSI (см. reports/*.txt — те тоже с BOM).
            static const char kUtf8Bom[] = {'\xEF', '\xBB', '\xBF'};
            const std::string payload =
                std::string(kUtf8Bom, sizeof(kUtf8Bom)) + report;

            QFile f(report_path);
            // WriteOnly у QFile обрезает существующий файл (эквивалент Trunc).
            const bool written =
                f.open(QIODevice::WriteOnly | QIODevice::Text) &&
                f.write(payload.data(), static_cast<qint64>(payload.size())) ==
                    static_cast<qint64>(payload.size());
            f.close();

            if (written)
            {
                out << QStringLiteral("bias-report: ok -> %1\n").arg(report_path);
            }
            else
            {
                // Отчёт не записан — режим не выполнил свою задачу, код 1.
                out << QStringLiteral("bias-report: не удалось записать %1\n")
                           .arg(report_path);
                ok = false;
            }
        }
        else
        {
            out << QStringLiteral("bias-report: ошибка: %1\n")
                       .arg(QString::fromStdString(error));
        }
        out.flush();
        return ok ? 0 : 1;
    }

    // В режиме --self-test расчёт стартует сразу, окно не показывается, а после
    // завершения (или таймаута) приложение выходит с ненулевым кодом при ошибке.
    gui::MainWindow window(settings, /*autostart=*/self_test);
    if (!self_test) window.show();

    if (!self_test) return app.exec();

    QEventLoop loop;
    bool finished = false;
    bool ok = false;
    QString message;

    QObject::connect(&window, &gui::MainWindow::runFinished,
                     [&](bool run_ok, const QString &msg) {
                         finished = true;
                         ok = run_ok;
                         message = msg;
                         loop.quit();
                     });

    QTimer timeout;
    timeout.setSingleShot(true);
    QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
    timeout.start(self_test_timeout_s * 1000);

    QElapsedTimer timer;
    timer.start();
    loop.exec();

    // Дополнительно проверяем чтение сохранённого прогона из SQLite —
    // это самый недоступный для ручного тестирования участок GUI.
    QString history_note;
    if (finished && ok)
    {
        const qint64 run_id = window.currentRunId();
        const int rows = run_id > 0 ? window.openRun(run_id) : 0;
        if (rows > 0)
        {
            history_note = QString("history: run %1 загружен, строк %2")
                               .arg(run_id)
                               .arg(rows);
        }
        else
        {
            history_note = QString("history: сохранённый прогон прочитать не удалось");
            ok = false;
        }

        // Сводный PDF-отчёт: собираем во временный файл рядом с отчётом о
        // self-test и проверяем, что файл получился непустым и начинается с
        // сигнатуры PDF. Путь всегда свой, чтобы не затереть текстовый отчёт.
        if (ok)
        {
            QString pdf = QDir(QDir::tempPath()).filePath(QStringLiteral("imitator_self_test.pdf"));
            if (!report_file.isEmpty())
            {
                pdf = QFileInfo(report_file).absolutePath() +
                      QStringLiteral("/imitator_self_test.pdf");
            }

            QString report_error;
            if (window.exportReportTo(pdf, &report_error) && QFileInfo(pdf).size() > 1000)
            {
                QFile probe(pdf);
                if (probe.open(QIODevice::ReadOnly) && probe.read(4) == QByteArray("%PDF"))
                {
                    history_note += QString("; report: %1 байт").arg(QFileInfo(pdf).size());
                }
                else
                {
                    history_note += QString("; report: нет сигнатуры PDF");
                    ok = false;
                }
            }
            else
            {
                history_note += QString("; report: %1")
                                    .arg(report_error.isEmpty()
                                             ? QStringLiteral("сборка не удалась")
                                             : report_error);
                ok = false;
            }
        }

        // И сравнение прогонов: тот же прогон ставится вторым, проверяется, что
        // ряды добавились на графики и разность считается по ненулевой сетке.
        // Отчёт выше уже собран, поэтому сравнение в PDF не попадает — отчёт
        // строится только по истине.
        if (ok && run_id > 0)
        {
            const int cmp_rows = window.openCompareRun(run_id);
            if (cmp_rows > 0 && window.hasComparison())
            {
                history_note += QString("; compare: второй прогон %1, строк %2")
                                    .arg(run_id)
                                    .arg(cmp_rows);
            }
            else
            {
                history_note += QString("; compare: не удалось");
                ok = false;
            }
        }

        // Экспорт статических матриц фильтра (Q/R/P0/H): проверяем, что
        // в отдельном каталоге появились все три файла (csv/md/json).
        if (ok)
        {
            QString mdir = QDir(QDir::tempPath())
                               .filePath(QStringLiteral("imitator_self_test_matrices"));
            if (!report_file.isEmpty())
            {
                mdir = QFileInfo(report_file).absolutePath() +
                       QStringLiteral("/imitator_self_test_matrices");
            }
            QDir().mkpath(mdir);

            QString matrices_error;
            const bool matrices_ok = window.exportMatricesTo(mdir, &matrices_error);
            const int files = QDir(mdir).entryList(QDir::Files).size();
            if (matrices_ok && files >= 3)
            {
                history_note += QString("; matrices: %1 файлов").arg(files);
            }
            else
            {
                history_note += QString("; matrices: %1")
                                    .arg(matrices_error.isEmpty()
                                             ? QStringLiteral("файлы не созданы")
                                             : matrices_error);
                ok = false;
            }
        }
    }

    // Консольный вывод у GUI-приложения на Windows теряется при перенаправлении,
    // поэтому отчёт пишем в файл (--self-test-report), если он задан.
    QString report;
    if (!finished)
    {
        report = QString("self-test: таймаут %1 с\n---\n%2")
                     .arg(self_test_timeout_s)
                     .arg(window.logText());
    }
    else
    {
        report = QString("self-test: %1 за %2 мс: %3\n%4\n---\n%5")
                     .arg(ok ? "успех" : "ошибка")
                     .arg(timer.elapsed())
                     .arg(message)
                     .arg(history_note)
                     .arg(window.logText());
    }

    if (!report_file.isEmpty())
    {
        QFile f(report_file);
        if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        {
            f.write(report.toUtf8());
            f.close();
        }
    }
    QTextStream out(stdout);
    out << report;
    out.flush();

    if (!finished) return 2;
    return ok ? 0 : 1;
}