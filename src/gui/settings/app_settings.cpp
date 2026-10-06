// app_settings.cpp — Пользовательские настройки GUI (QSettings).

#include "settings/app_settings.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

namespace gui
{
namespace
{
const char *kGeometry = "window/geometry";
const char *kState = "window/state";
const char *kDataDir = "paths/data_dir";
const char *kOutputDir = "paths/output_dir";
const char *kSettingsFile = "paths/settings_file";
const char *kDatabaseFile = "paths/database_file";
const char *kWriteFiles = "run/write_files";
const char *kTelemetry = "run/capture_telemetry";
const char *kSaveHistory = "run/save_to_history";
const char *kDarkTheme = "view/dark_theme";
const char *kExportW = "export/width";
const char *kExportH = "export/height";

// Каталог рядом с exe (…/build) и один уровень выше (корень проекта).
QString appRootDir()
{
    QDir dir(QCoreApplication::applicationDirPath());
    if (dir.exists("data/raw")) return dir.absolutePath();
    if (dir.exists("../data/raw")) return dir.absoluteFilePath("../data");
    return dir.absolutePath();
}
} // namespace

QString AppSettings::defaultDataDir()
{
    QDir root(appRootDir());
    return root.filePath("data/raw");
}

QString AppSettings::defaultOutputDir()
{
    QDir root(appRootDir());
    return root.filePath("tools");
}

QString AppSettings::defaultSettingsFile()
{
    QDir root(appRootDir());
    return root.filePath("settings.ini");
}

QString AppSettings::defaultDatabaseFile()
{
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if (dir.isEmpty()) return QDir(appRootDir()).filePath("runs.db");
    QDir().mkpath(dir);
    return QDir(dir).filePath("runs.db");
}

AppSettings AppSettings::load()
{
    QSettings s;

    AppSettings a;
    a.window_geometry = s.value(QLatin1String(kGeometry)).toByteArray();
    a.window_state = s.value(QLatin1String(kState)).toByteArray();

    a.data_dir = s.value(QLatin1String(kDataDir), defaultDataDir()).toString();
    a.output_dir = s.value(QLatin1String(kOutputDir), defaultOutputDir()).toString();
    a.settings_file = s.value(QLatin1String(kSettingsFile), defaultSettingsFile()).toString();
    a.database_file = s.value(QLatin1String(kDatabaseFile), defaultDatabaseFile()).toString();

    a.write_files = s.value(QLatin1String(kWriteFiles), true).toBool();
    a.capture_telemetry = s.value(QLatin1String(kTelemetry), true).toBool();
    a.save_to_history = s.value(QLatin1String(kSaveHistory), true).toBool();
    a.dark_theme = s.value(QLatin1String(kDarkTheme), false).toBool();

    a.export_width = s.value(QLatin1String(kExportW), 1280).toInt();
    a.export_height = s.value(QLatin1String(kExportH), 800).toInt();
    return a;
}

void AppSettings::save() const
{
    QSettings s;

    s.setValue(QLatin1String(kGeometry), window_geometry);
    s.setValue(QLatin1String(kState), window_state);
    s.setValue(QLatin1String(kDataDir), data_dir);
    s.setValue(QLatin1String(kOutputDir), output_dir);
    s.setValue(QLatin1String(kSettingsFile), settings_file);
    s.setValue(QLatin1String(kDatabaseFile), database_file);
    s.setValue(QLatin1String(kWriteFiles), write_files);
    s.setValue(QLatin1String(kTelemetry), capture_telemetry);
    s.setValue(QLatin1String(kSaveHistory), save_to_history);
    s.setValue(QLatin1String(kDarkTheme), dark_theme);
    s.setValue(QLatin1String(kExportW), export_width);
    s.setValue(QLatin1String(kExportH), export_height);
    s.sync();
}

void AppSettings::saveWindowGeometry(const QMainWindow *window) const
{
    if (window == nullptr) return;
    AppSettings copy = *this;
    copy.window_geometry = window->saveGeometry();
    copy.window_state = window->saveState();
    copy.save();
}

void AppSettings::restoreWindowGeometry(QMainWindow *window) const
{
    if (window == nullptr) return;
    if (!window_geometry.isEmpty()) window->restoreGeometry(window_geometry);
    if (!window_state.isEmpty()) window->restoreState(window_state);
}

} // namespace gui