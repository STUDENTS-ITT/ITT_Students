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
#include <QFileInfo>
#include <QLocale>
#include <QTextStream>
#include <QTimer>
#include <QTranslator>

#include "MainWindow.h"
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
        else if (qstrncmp(argv[i], "--self-test-timeout=", 20) == 0)
        {
            self_test_timeout_s = QString::fromLatin1(argv[i] + 20).toInt();
        }
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

    parser.addOption(data_opt);
    parser.addOption(out_opt);
    parser.addOption(settings_opt);
    parser.addOption(dark_opt);
    parser.addOption(autostart_opt);
    parser.addOption(self_test_opt);
    parser.addOption(self_test_timeout_opt);
    parser.addOption(self_test_report_opt);
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

        // И сравнение прогонов: тот же прогон ставится вторым, проверяется, что
        // ряды добавились на графики и разность считается по ненулевой сетке.
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