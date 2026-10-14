// test_parity.cpp — бит-в-бит паритет консольного imitator и imitator_gui.
//
// Один каталог данных и один явный settings.ini, изолированные рабочие
// каталоги. Ожидается, что result.txt/reference.txt/errors.txt совпадают
// байт-в-байт: ими пользуются и тюнер, и GUI-графики.
//
// Важно: settings.ini передаётся явно. Неявные дефолты консоли и GUI могут
// отличаться (GUI берёт значения из настроек приложения), поэтому прогон без
// -s сравнивать нельзя.

#include <fstream>
#include <string>

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QTemporaryDir>
#include <QtTest>

#include "core/navigation/filter_settings.h"

class TestParity : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void fixedSettingsProduceIdenticalFiles();

private:
    QString dataDir_;
    QString imitator_;
    QString imitatorGui_;
};

namespace
{

QString sha256(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly))
        return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    hash.addData(&f);
    return QString::fromLatin1(hash.result().toHex());
}

bool runToCompletion(QProcess &p, int timeout_ms)
{
    return p.waitForFinished(timeout_ms) && p.exitStatus() == QProcess::NormalExit;
}

} // namespace

void TestParity::initTestCase()
{
    dataDir_ = QString::fromUtf8(ITT_PARITY_DATA_DIR);
    imitator_ = QString::fromUtf8(ITT_IMITATOR_EXE);
    imitatorGui_ = QString::fromUtf8(ITT_IMITATOR_GUI_EXE);

    QVERIFY2(QFileInfo::exists(dataDir_ + "/imu.dat"),
             qPrintable("нет данных для паритета: " + dataDir_));
    QVERIFY2(QFileInfo::exists(imitator_), qPrintable("нет imitator: " + imitator_));
    QVERIFY2(QFileInfo::exists(imitatorGui_), qPrintable("нет imitator_gui: " + imitatorGui_));
}

void TestParity::fixedSettingsProduceIdenticalFiles()
{
    QTemporaryDir scratch;
    QVERIFY(scratch.isValid());

    const QString conDir = scratch.filePath("con");
    const QString guiDir = scratch.filePath("gui");
    QVERIFY(QDir().mkpath(conDir));
    QVERIFY(QDir().mkpath(guiDir));

    // Единый settings.ini из канонических значений по умолчанию.
    const nav::FilterSettings s;
    const std::string iniText = nav::dumpFilterSettings(s);
    const QString settingsFile = scratch.filePath("settings.ini");
    {
        std::ofstream out(settingsFile.toStdString(), std::ios::binary);
        QVERIFY2(out.is_open(), "не удалось создать settings.ini");
        out << iniText;
    }

    // Консоль: результаты — <cwd>/tools. Рабочий каталог изолирован, чтобы
    // консоль не нашла стороннюю папку tools рядом.
    {
        QProcess p;
        p.setWorkingDirectory(conDir);
        p.start(imitator_, {dataDir_, settingsFile});
        QVERIFY2(runToCompletion(p, 300000), qPrintable(p.errorString()));
        QCOMPARE(p.exitCode(), 0);
    }
    // GUI: headless self-test, результаты — -o <guiDir>.
    {
        QProcess p;
        p.setWorkingDirectory(scratch.path());
        p.start(imitatorGui_,
                {"--self-test", "--self-test-timeout=300",
                 "-d", dataDir_, "-o", guiDir,
                 "-s", settingsFile,
                 "--self-test-report", scratch.filePath("gui_report.txt")});
        QVERIFY2(runToCompletion(p, 360000), qPrintable(p.errorString()));
        QCOMPARE(p.exitCode(), 0);
    }

    const QString files[] = {"result.txt", "reference.txt", "errors.txt"};
    for (const QString &file : files)
    {
        const QString fromConsole = conDir + "/tools/" + file;
        const QString fromGui = guiDir + "/" + file;

        QVERIFY2(QFileInfo::exists(fromConsole),
                 qPrintable("консоль не создала файл: " + fromConsole));
        QVERIFY2(QFileInfo::exists(fromGui),
                 qPrintable("GUI не создал файл: " + fromGui));

        const QString consoleHash = sha256(fromConsole);
        const QString guiHash = sha256(fromGui);
        QVERIFY2(consoleHash == guiHash,
                 qPrintable(QString("ПАРИТЕТ НАРУШЕН: %1 различается между консолью и GUI\n"
                                    "  консоль sha256 = %2\n"
                                    "  gui     sha256 = %3")
                                .arg(file, consoleHash, guiHash)));
    }
}

QTEST_MAIN(TestParity)
#include "test_parity.moc"