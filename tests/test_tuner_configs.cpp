// test_tuner_configs.cpp — проверка перечня конфигов подбора (вкладка «Подбор»).
//
// Тест не запускает тюнер: раскладывает в QTemporaryDir синтетический каталог
// с tuner_config*.json и truth_cfg*/cfg_*.json и проверяет, что в список
// попадают только нужные файлы в нужном порядке, а служебные (smoke) и прочие
// подкаталоги (temp_tune_*) игнорируются.

#include <QtTest>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include "settings/tuner_configs.h"

namespace
{

void writeFile(const QString &path, const QByteArray &data = QByteArray("{}"))
{
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    QVERIFY2(f.open(QIODevice::WriteOnly | QIODevice::Truncate), qPrintable(path));
    f.write(data);
    f.close();
}

} // namespace

class TestTunerConfigs : public QObject
{
    Q_OBJECT

private Q_SLOTS:
    // Список собирает корневые tuner_config*.json и truth_cfg*/cfg_*.json,
    // пропускает smoke и не заходит в чужие подкаталоги.
    void listsRootAndTruthConfigs()
    {
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        const QString root = dir.path();

        // Штатные конфиги наборов в корне тюнера.
        writeFile(root + "/tuner_config.json");
        writeFile(root + "/tuner_config.static300.json");
        writeFile(root + "/tuner_config.smoke.json");     // служебный — пропускаем
        writeFile(root + "/tuner_config.bak.txt");        // не .json — не наш
        writeFile(root + "/notes.json");                  // не tuner_config* — не наш

        // Конфиги экспериментов с истиной (13A/13B/13G).
        writeFile(root + "/truth_cfg/cfg_square650.json");
        writeFile(root + "/truth_cfg/cfg_static300.json");
        writeFile(root + "/truth_cfg_13b/cfg_square650.json");
        writeFile(root + "/truth_cfg_13g/cfg_square650.json");
        writeFile(root + "/truth_cfg_13g/other.json");    // не cfg_* — не наш
        QDir().mkpath(root + "/truth_cfg_empty");         // пустой каталог

        // Рабочие каталоги прогонов: не должны попадать в список.
        writeFile(root + "/temp_tune_13g_square650/eval.json");

        const QStringList got = gui::listTunerConfigs(root);
        const QStringList expected = {
            QStringLiteral("tuner_config.json"),
            QStringLiteral("tuner_config.static300.json"),
            QStringLiteral("truth_cfg/cfg_square650.json"),
            QStringLiteral("truth_cfg/cfg_static300.json"),
            QStringLiteral("truth_cfg_13b/cfg_square650.json"),
            QStringLiteral("truth_cfg_13g/cfg_square650.json"),
        };
        QCOMPARE(got, expected);
    }

    // Пустой или несуществующий каталог — пустой список.
    void emptyAndMissingDir()
    {
        QVERIFY(gui::listTunerConfigs(QString()).isEmpty());

        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        QVERIFY(gui::listTunerConfigs(dir.path()).isEmpty());
        QVERIFY(gui::listTunerConfigs(dir.path() + "/no_such_dir").isEmpty());
    }
};

QTEST_MAIN(TestTunerConfigs)
#include "test_tuner_configs.moc"
