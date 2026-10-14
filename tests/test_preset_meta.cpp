// test_preset_meta.cpp — проверка «паспорта набора» в пресете (секция [meta]).
//
// Тест не запускает подбор: раскладывает в QTemporaryDir синтетические
// пресеты (с секцией [meta], без неё, с чужим набором) и проверяет, что
// разбор и сверка имени набора работают, а лишние секции игнорируются.

#include <QtTest>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include "settings/preset_meta.h"

namespace
{

void writeUtf8(const QString &path, const QByteArray &data)
{
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    QVERIFY2(f.open(QIODevice::WriteOnly | QIODevice::Truncate), qPrintable(path));
    f.write(data);
    f.close();
}

// Пресет нового формата: паспорт набора + рабочие секции.
const char *kPresetWithMeta =
    "# Пресет подобранных коэффициентов КФ.\n"
    "\n"
    "[meta]\n"
    "dataset = square_650\n"
    "data_dir = C:/ITT_Students_Qt_Version/dataset_shum/square_650\n"
    "error_ref = truth.txt\n"
    "\n"
    "[filter]\n"
    "sigma_g = 0.1\n"
    "sigma_a = 3.0\n";

// Пресет старого формата: секции [meta] нет.
const char *kPresetLegacy =
    "# Пресет подобранных коэффициентов КФ.\n"
    "\n"
    "[filter]\n"
    "sigma_g = 0.1\n"
    "sigma_a = 3.0\n";

} // namespace

class TestPresetMeta : public QObject
{
    Q_OBJECT

private slots:
    void readsMeta();
    void sectionIsCaseInsensitive();
    void missingMeta();
    void missingFile();
    void matchKeepsSilent();
    void mismatchWarns();
    void emptyDataDirSkips();
    void keysOutsideMetaIgnored();
    void commentsIgnored();
};

void TestPresetMeta::readsMeta()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/tuned_square_650.ini";
    writeUtf8(path, QByteArray(kPresetWithMeta));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QVERIFY(meta.present);
    QCOMPARE(meta.dataset, QStringLiteral("square_650"));
    QCOMPARE(meta.data_dir,
             QStringLiteral("C:/ITT_Students_Qt_Version/dataset_shum/square_650"));
    QCOMPARE(meta.error_ref, QStringLiteral("truth.txt"));
}

void TestPresetMeta::sectionIsCaseInsensitive()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/preset.ini";
    writeUtf8(path, QByteArray("[Meta]\nDATASET = spiral_climb\n\n[filter]\nsigma_g = 0.1\n"));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QVERIFY(meta.present);
    QCOMPARE(meta.dataset, QStringLiteral("spiral_climb"));
}

void TestPresetMeta::missingMeta()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/tuned.ini";
    writeUtf8(path, QByteArray(kPresetLegacy));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QVERIFY(!meta.present);
    QVERIFY(meta.dataset.isEmpty());
    // Старый пресет: сверять нечего — применение не предупреждается.
    QVERIFY(gui::presetMatchesDataset(meta, dir.path() + "/square_650"));
}

void TestPresetMeta::missingFile()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const gui::PresetMeta meta = gui::readPresetMeta(dir.path() + "/no_such.ini");
    QVERIFY(!meta.present);
}

void TestPresetMeta::matchKeepsSilent()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/tuned.ini";
    writeUtf8(path, QByteArray(kPresetWithMeta));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QString warning = QStringLiteral("stale");
    QVERIFY(gui::presetMatchesDataset(meta, QStringLiteral("C:/data/square_650"), &warning));
    QVERIFY(warning.isEmpty());
}

void TestPresetMeta::mismatchWarns()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/tuned.ini";
    writeUtf8(path, QByteArray(kPresetWithMeta));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QString warning;
    QVERIFY(!gui::presetMatchesDataset(meta, QStringLiteral("C:/data/static_300"), &warning));
    QVERIFY(warning.contains(QStringLiteral("square_650")));
    QVERIFY(warning.contains(QStringLiteral("static_300")));
}

void TestPresetMeta::emptyDataDirSkips()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/tuned.ini";
    writeUtf8(path, QByteArray(kPresetWithMeta));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QVERIFY(gui::presetMatchesDataset(meta, QString(), nullptr));
}

void TestPresetMeta::keysOutsideMetaIgnored()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/preset.ini";
    // Ключ dataset в [filter] не должен попасть в паспорт.
    writeUtf8(path, QByteArray("[filter]\ndataset = wrong\nsigma_g = 0.1\n"));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QVERIFY(!meta.present);
    QVERIFY(meta.dataset.isEmpty());
}

void TestPresetMeta::commentsIgnored()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.path() + "/preset.ini";
    writeUtf8(path, QByteArray("# dataset = commented_out\n"
                               "; dataset = also_commented\n"
                               "[meta]\n"
                               "# dataset = still_comment\n"
                               "dataset = high_dynamics\n"));

    const gui::PresetMeta meta = gui::readPresetMeta(path);
    QVERIFY(meta.present);
    QCOMPARE(meta.dataset, QStringLiteral("high_dynamics"));
}

QTEST_MAIN(TestPresetMeta)
#include "test_preset_meta.moc"
