// app_settings.h — Пользовательские настройки GUI (QSettings) и работа с
// settings.ini.
//
// QSettings хранит состояние окна, выбранные каталоги и путь к базе прогонов
// (в QStandardPaths::AppDataLocation), а параметры фильтра — в обычный INI,
// который читает ядро.

#pragma once

#include <QMainWindow>
#include <QString>

class QSettings;

namespace gui
{

struct AppSettings
{
    // Состояние окна.
    QByteArray window_geometry;
    QByteArray window_state;

    // Каталоги и файлы.
    QString data_dir;       // imu.dat / gps.dat / angle.dat / StartupNav.ini
    QString output_dir;     // result.txt / reference.txt / errors.txt
    QString settings_file;  // settings.ini
    QString database_file;  // история прогонов (SQLite)

    // Поведение прогона.
    bool write_files = true;
    bool capture_telemetry = true;
    bool save_to_history = true;
    bool dark_theme = false;

    // Окно графика.
    int export_width = 1280;
    int export_height = 800;

    static AppSettings load();
    void save() const;

    // Путь базы по умолчанию: <AppDataLocation>/runs.db.
    static QString defaultDatabaseFile();
    // Каталог данных по умолчанию: <каталог приложения>/data/raw.
    static QString defaultDataDir();
    // Каталог результатов по умолчанию: <каталог приложения>/tools.
    static QString defaultOutputDir();
    // Файл настроек по умолчанию: <каталог приложения>/settings.ini.
    static QString defaultSettingsFile();

    void saveWindowGeometry(const QMainWindow *window) const;
    void restoreWindowGeometry(QMainWindow *window) const;
};

} // namespace gui