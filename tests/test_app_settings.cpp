// test_app_settings.cpp — пути по умолчанию GUI-настроек.
//
// Проверяет, что appRootDir() находит корень проекта подъёмом по дереву от
// каталога тестового exe (build/tests): это фикс дефекта §14, от которого
// зависит и каталог данных, и каталог отчётов reports/.
//
// Тест пропускается (QSKIP), если в репозитории нет data/raw — как test_parity.

#include <QtTest>

#include <QDir>
#include <QFileInfo>

#include "gui/settings/app_settings.h"

namespace
{
// Корень проекта считается найденным, если рядом с каталогом данных есть imu.dat.
bool projectRootAvailable()
{
    return QFileInfo::exists(
        QDir(gui::AppSettings::defaultDataDir()).filePath(QStringLiteral("imu.dat")));
}
} // namespace

class TestAppSettings : public QObject
{
    Q_OBJECT

private slots:
    void defaultsResolveToProjectRoot();
    void reportsDirIsSiblingOfData();
};

void TestAppSettings::defaultsResolveToProjectRoot()
{
    if (!projectRootAvailable()) QSKIP("В репозитории нет data/raw/imu.dat");

    // Корень восстанавливаем как родителя каталога отчётов (reports/ лежит
    // в корне проекта) и сверяем с ним остальные пути по умолчанию.
    const QString root =
        QFileInfo(gui::AppSettings::defaultReportsDir()).absolutePath();

    QCOMPARE(QDir::cleanPath(gui::AppSettings::defaultDataDir()),
             QDir::cleanPath(root + QStringLiteral("/data/raw")));
    QCOMPARE(QDir::cleanPath(gui::AppSettings::defaultPresetsDir()),
             QDir::cleanPath(root + QStringLiteral("/presets")));
    QCOMPARE(QDir::cleanPath(gui::AppSettings::defaultSettingsFile()),
             QDir::cleanPath(root + QStringLiteral("/settings.ini")));
}

void TestAppSettings::reportsDirIsSiblingOfData()
{
    if (!projectRootAvailable()) QSKIP("В репозитории нет data/raw/imu.dat");

    const QString reports = gui::AppSettings::defaultReportsDir();
    QCOMPARE(QFileInfo(reports).fileName(), QStringLiteral("reports"));

    // Каталог отчётов — рядом с каталогом данных в корне проекта.
    const QDir root(QFileInfo(reports).absolutePath());
    QVERIFY(QFileInfo::exists(root.filePath(QStringLiteral("data/raw/imu.dat"))));
}

QTEST_MAIN(TestAppSettings)
#include "test_app_settings.moc"
