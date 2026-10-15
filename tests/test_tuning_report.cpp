// test_tuning_report.cpp — сборка PDF-отчёта о подборе из артефактов тюнера.
//
// Тест не запускает сам подбор: он раскладывает рядом с конфигом синтетические
// best_params.txt и eval.json и проверяет, что отчёт собирается приложением.

#include <QtTest>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include "export/tuning_report.h"

namespace
{

void writeUtf8(const QString &path, const QString &text)
{
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        f.write(text.toUtf8());
        f.close();
    }
}

// Минимальный, но представительный best_params.txt (как пишет tune_kalman.py).
QString bestParamsText()
{
    return QString::fromUtf8(
        "=== Подобранные параметры фильтра Калмана ===\n"
        "данные            : C:/data/dataset_shum/static_300\n"
        "эталон ошибки     : truth.txt (наземная истина)\n"
        "J_train           : 1.2394 (этап 1: 1.5464)\n"
        "J_valid           : 2.4825\n"
        "reg (штраф u^2)   : 0.1043\n"
        "\n"
        "--- breakdown по компонентам (норм. = P95/допуск) ---\n"
        "train pos:  0.644 (RAW 3.219 м / допуск 5.0 м)\n"
        "valid pos:  0.631 (RAW 3.156 м)\n"
        "tail pos (P99.9@200Гц) train:  0.474 (RAW 4.741 м)\n"
        "\n"
        "--- sigma (СИ) ---\n"
        "параметр               sigma  u=log10(s/ref)\n"
        "sigma_g         1.135357e-01          2.5244\n"
        "sigma_a         3.050000e+00          3.0000\n"
        "\n"
        "--- E1b: центроид плато ---\n"
        "точек в плато     : 37\n"
        "J_min плато       : 1.2329\n"
        "\n"
        "--- чувствительность J к параметрам (шаг ±0.15 по u) ---\n"
        "база: J_train=1.2394, J_valid=2.4825\n"
        "sigma_g      sign=+1 J_train=1.2000 J_valid=2.4000\n");
}

QString configText()
{
    return QString::fromUtf8(R"JSON({
  "data_dir": "C:/data/dataset_shum/static_300",
  "work_root": "work",
  "params": [
    {"name": "sigma_g", "sigma_ref": 3.394e-4, "u_span": 2.7},
    {"name": "sigma_a", "sigma_ref": 3.05e-3,  "u_span": 3.0}
  ],
  "metric": {"tol_pos": 5.0, "tol_vel": 0.5, "tol_hdg": 1.0, "tie_weight": 0.05},
  "horizon": {"skip_first_s": 130.0, "train_frac": 0.6},
  "optim": {"n_global": 64, "n_local": 80, "seed": 42},
  "output": {"preset": "presets/tuned_static300.ini"}
})JSON");
}

} // namespace

class TestTuningReport : public QObject
{
    Q_OBJECT

private slots:
    void buildsPdfFromArtifacts();
    void missingConfigFails();
    void workRootIsRelativeToConfig();
    void workRootIsRelativeToTunerDir();
    void datasetNameFromConfig();
};

void TestTuningReport::buildsPdfFromArtifacts()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString root = dir.path();

    const QString config = root + "/tuner_config.static300.json";
    writeUtf8(config, configText());
    writeUtf8(root + "/work/best_params.txt", bestParamsText());

    // Несколько оценок для графика сходимости.
    for (int i = 0; i < 4; ++i)
    {
        writeUtf8(root + QString("/work/global_%1/eval.json").arg(i, 3, 10, QLatin1Char('0')),
                  QString::fromUtf8(R"({"u": [0.1, 0.2], "J_train": %1, "J_valid": %2})")
                      .arg(1.5 + i * 0.1)
                      .arg(2.5 + i * 0.2));
    }

    const QString pdf = root + "/out/tuning_report.pdf";
    QString error;
    QVERIFY2(gui::TuningReport::build(config, pdf, &error), qPrintable(error));
    QVERIFY(error.isEmpty());

    QFile f(pdf);
    QVERIFY(f.exists());
    QVERIFY(f.size() > 1000);
    QVERIFY(f.open(QIODevice::ReadOnly));
    QCOMPARE(f.read(5), QByteArray("%PDF-"));
}

void TestTuningReport::missingConfigFails()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    QString error;
    const bool ok = gui::TuningReport::build(dir.path() + "/nope.json",
                                             dir.path() + "/out.pdf", &error);
    QVERIFY(!ok);
    QVERIFY(!error.isEmpty());
}

void TestTuningReport::workRootIsRelativeToConfig()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString config = dir.path() + "/tuner_config.json";
    writeUtf8(config, configText());

    // Маркера каталога тюнера рядом нет — путь относительно каталога конфига.
    const QString work = gui::TuningReport::workRootFor(config);
    QCOMPARE(QDir::cleanPath(work), QDir::cleanPath(dir.path() + "/work"));
}

void TestTuningReport::workRootIsRelativeToTunerDir()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString tuner = dir.path() + "/tools/kalman_tuner";

    // Маркер каталога тюнера: именно так work_root трактует tune_kalman.py.
    writeUtf8(tuner + "/tune_kalman.py", QString());
    writeUtf8(tuner + "/temp_tune_13g_square650/best_params.txt",
              bestParamsText());

    // Конфиг лежит в подкаталоге и указывает work_root в каталог тюнера —
    // так же, как конфиги в tools/kalman_tuner/truth_cfg_13g/.
    const QString config = tuner + "/truth_cfg_13g/cfg_square650.json";
    writeUtf8(config, configText().replace(
                          QStringLiteral("\"work_root\": \"work\""),
                          QStringLiteral("\"work_root\": \"temp_tune_13g_square650\"")));

    const QString work = gui::TuningReport::workRootFor(config);
    QCOMPARE(QDir::cleanPath(work),
             QDir::cleanPath(tuner + "/temp_tune_13g_square650"));

    // Отчёт по такому конфигу собирается (раньше был пустым).
    const QString pdf = dir.path() + "/out/tuning.pdf";
    QString error;
    QVERIFY2(gui::TuningReport::build(config, pdf, &error), qPrintable(error));
    QFile f(pdf);
    QVERIFY(f.exists());
    QVERIFY(f.size() > 1000);
}

void TestTuningReport::datasetNameFromConfig()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString config = dir.path() + "/tuner_config.static300.json";
    writeUtf8(config, configText());

    // Имя набора — последний компонент data_dir из конфига.
    QCOMPARE(gui::TuningReport::datasetFor(config), QStringLiteral("static_300"));

    // Нечитаемый конфиг — пустая строка (вызывающий подставит запасное имя).
    QVERIFY(gui::TuningReport::datasetFor(dir.path() + "/nope.json").isEmpty());
}

QTEST_MAIN(TestTuningReport)
#include "test_tuning_report.moc"
