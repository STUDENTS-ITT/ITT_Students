// test_settings.cpp — INI-ридер и настройки фильтра.

#include <cmath>
#include <cstdio>
#include <string>

#include <QtTest>

#include "core/navigation/filter_settings.h"
#include "core/utils/ini_parser.h"

class TestSettings : public QObject
{
    Q_OBJECT

private slots:
    void parsesSectionsAndKeys();
    void ignoresCommentsAndBlanks();
    void keysAreCaseInsensitive();
    void keepsDefaultsForMissingKeys();
    void rejectsMalformedNumbers();
    void loadsSettingsFromFile();
    void settingsDefaultsMatchFilterDefaults();
    void appliesSettingsToGlobalConfig();
    void dumpsAndReloadsSettings();
    void descriptorsCoverAllKeys();
    void detectsSectionsFormat();
    void appliesLegacyFlatKeys();
};

namespace
{

// Записать текст в файл и вернуть путь как std::string.
std::string writeTempIni(const QString &path, const char *text)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) return std::string();
    f.write(text);
    f.close();
    return path.toStdString();
}

} // namespace

void TestSettings::detectsSectionsFormat()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());

    const std::string sections = writeTempIni(dir.filePath("sections.ini"),
                                               "; комментарий\n"
                                               "\n"
                                               "[filter]\n"
                                               "sigma_g = 0.5\n");
    QVERIFY(!sections.empty());
    QVERIFY(nav::settingsFileHasSections(sections));

    const std::string flat = writeTempIni(dir.filePath("flat.ini"),
                                          "sigma_g = 0.5\nsigma_a = 1.5\n");
    QVERIFY(!flat.empty());
    QVERIFY(!nav::settingsFileHasSections(flat));

    // Нечитаемый и пустой файлы считаем секционными: парсер сам сообщит,
    // с какими ключами он работал.
    QVERIFY(nav::settingsFileHasSections(dir.filePath("missing.ini").toStdString()));
    const std::string empty = writeTempIni(dir.filePath("empty.ini"), "; только комментарий\n");
    QVERIFY(!empty.empty());
    QVERIFY(nav::settingsFileHasSections(empty));
}

void TestSettings::appliesLegacyFlatKeys()
{
    // Старый params.ini без секций: loadFilterSettings() не находит ключей и
    // оставляет значения по умолчанию, поэтому плоские ключи применяются
    // отдельным проходом (иначе они затираются дефолтами в runner).
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const std::string flat = writeTempIni(dir.filePath("params.ini"),
                                          "sigma_g = 0.5\n"
                                          "sigma_pos = 0.0002\n"
                                          "outage_start_s = 30\n"
                                          "outage_end_s = 31\n");
    QVERIFY(!flat.empty());

    nav::FilterSettings s;
    QVERIFY(nav::loadFilterSettings(flat, s));
    QCOMPARE(s.filter.sig_g, ins::KalmanConfig{}.sig_g);  // секций нет — дефолт

    nav::applySettings(s);
    nav::applyLegacyParamsFile(flat, nullptr);
    QCOMPARE(ins::kalman_cfg.sig_g, 0.5);
    QCOMPARE(ins::kalman_cfg.sig_pos, 0.0002);
    QCOMPARE(ins::kalman_cfg.outage_start_s, 30.0);
    QCOMPARE(ins::kalman_cfg.outage_end_s, 31.0);
    // Не перечисленные ключи остаются дефолтными.
    QCOMPARE(ins::kalman_cfg.sig_a, ins::KalmanConfig{}.sig_a);

    const nav::FilterSettings defaults;
    nav::applySettings(defaults);
}

void TestSettings::parsesSectionsAndKeys()
{
    utils::IniFile ini;
    ini.loadFromString("[filter]\nsigma_g = 0.5\nsigma_a=1.5\n");

    QVERIFY(ini.has("filter/sigma_g"));
    QCOMPARE(ini.value("filter/sigma_g", -1.0), 0.5);
    QCOMPARE(ini.value("filter/sigma_a", -1.0), 1.5);
    QCOMPARE(ini.value("filter/missing", 42.0), 42.0);
}

void TestSettings::ignoresCommentsAndBlanks()
{
    utils::IniFile ini;
    ini.loadFromString("; комментарий\n"
                       "# тоже комментарий\n"
                       "\n"
                       "   \n"
                       "[filter]\n"
                       "  sigma_g = 0.25  \n");

    QCOMPARE(ini.value("filter/sigma_g", -1.0), 0.25);
    QCOMPARE(ini.keys().size(), std::size_t(1));
}

void TestSettings::keysAreCaseInsensitive()
{
    utils::IniFile ini;
    ini.loadFromString("[Filter]\nSIGMA_G = 0.75\n");

    QVERIFY(ini.has("filter/sigma_g"));
    QVERIFY(ini.has("FILTER/SIGMA_G"));
    QCOMPARE(ini.value("filter/sigma_g", -1.0), 0.75);
}

void TestSettings::keepsDefaultsForMissingKeys()
{
    utils::IniFile ini;
    ini.loadFromString("[filter]\nsigma_g = 0.5\n");

    // Отсутствующий ключ не перетирает значение по умолчанию.
    QCOMPARE(ini.value("filter/sigma_a", 1.25), 1.25);
    QCOMPARE(ini.text("filter/sigma_a", "1.25"), "1.25");
}

void TestSettings::rejectsMalformedNumbers()
{
    utils::IniFile ini;
    ini.loadFromString("[filter]\nsigma_g = abc\nsigma_a = 0.5abc\n");

    QCOMPARE(ini.value("filter/sigma_g", 9.0), 9.0);
    QCOMPARE(ini.value("filter/sigma_a", 9.0), 9.0);
}

void TestSettings::loadsSettingsFromFile()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("settings_test.ini");

    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("[filter]\nsigma_g = 1.5e-2\noutage_start_s = 100\noutage_end_s = 200\n"
            "[tilt]\nsettle_s = 0.5\n"
            "[run]\nalign_fallback_s = 60\n");
    f.close();

    nav::FilterSettings s;
    QVERIFY(nav::loadFilterSettings(path.toStdString(), s));

    QCOMPARE(s.filter.sig_g, 1.5e-2);
    QCOMPARE(s.filter.outage_start_s, 100.0);
    QCOMPARE(s.filter.outage_end_s, 200.0);
    QCOMPARE(s.tilt.settle_s, 0.5);
    QCOMPARE(s.align_fallback_s, 60.0);

    // Не переопределённые поля сохраняют значения по умолчанию.
    QCOMPARE(s.filter.sig_a, ins::KalmanConfig{}.sig_a);
    QCOMPARE(s.tilt.gyro_off_rad_s, nav::TiltManeuverPolicy{}.gyro_off_rad_s);
}

void TestSettings::settingsDefaultsMatchFilterDefaults()
{
    // Значения по умолчанию FilterSettings обязаны совпадать с теми, что были
    // захардкожены в ins::KalmanConfig — иначе прогоны «поедут».
    const nav::FilterSettings s;
    const ins::KalmanConfig k;
    const nav::TiltManeuverPolicy t;

    QCOMPARE(s.filter.sig_g, k.sig_g);
    QCOMPARE(s.filter.sig_a, k.sig_a);
    QCOMPARE(s.filter.sig_bg, k.sig_bg);
    QCOMPARE(s.filter.sig_ba, k.sig_ba);
    QCOMPARE(s.filter.sig_pos, k.sig_pos);
    QCOMPARE(s.filter.sig_h, k.sig_h);
    QCOMPARE(s.filter.sig_v, k.sig_v);
    QCOMPARE(s.filter.sig_hdg, k.sig_hdg);
    QCOMPARE(s.filter.outage_start_s, k.outage_start_s);
    QCOMPARE(s.filter.outage_end_s, k.outage_end_s);

    QCOMPARE(s.tilt.gyro_off_rad_s, t.gyro_off_rad_s);
    QCOMPARE(s.tilt.settle_s, t.settle_s);
    QCOMPARE(s.tilt.brake_body_accel, t.brake_body_accel);
    QCOMPARE(s.tilt.tau_fast_s, t.tau_fast_s);
    QCOMPARE(s.tilt.tau_slow_s, t.tau_slow_s);
    QCOMPARE(s.tilt.fast_err_rad, t.fast_err_rad);
    QCOMPARE(s.tilt.level_acc_rad, t.level_acc_rad);
    QCOMPARE(s.tilt.bias_gain, t.bias_gain);
    QCOMPARE(s.tilt.bias_max_rad_s, t.bias_max_rad_s);
    QCOMPARE(s.tilt.divergence_deadband_rad, t.divergence_deadband_rad);
}

void TestSettings::appliesSettingsToGlobalConfig()
{
    nav::FilterSettings s;
    s.filter.sig_g = 0.123;
    s.tilt.settle_s = 0.9;
    nav::applySettings(s);

    QCOMPARE(ins::kalman_cfg.sig_g, 0.123);
    QCOMPARE(nav::tiltPolicy().settle_s, 0.9);

    // Возвращаем значения по умолчанию, чтобы не влиять на другие тесты.
    const nav::FilterSettings defaults;
    nav::applySettings(defaults);
    QCOMPARE(ins::kalman_cfg.sig_g, defaults.filter.sig_g);
}

void TestSettings::dumpsAndReloadsSettings()
{
    nav::FilterSettings s;
    s.filter.sig_g = 0.777;
    s.tilt.bias_gain = 0.5;
    s.align_fallback_s = 45.0;

    const std::string text = nav::dumpFilterSettings(s);
    QVERIFY(text.find("[filter]") != std::string::npos);
    QVERIFY(text.find("[tilt]") != std::string::npos);

    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString path = dir.filePath("dump.ini");

    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly));
    f.write(text.data(), static_cast<qint64>(text.size()));
    f.close();

    nav::FilterSettings loaded;
    QVERIFY(nav::loadFilterSettings(path.toStdString(), loaded));
    QCOMPARE(loaded.filter.sig_g, 0.777);
    QCOMPARE(loaded.tilt.bias_gain, 0.5);
    QCOMPARE(loaded.align_fallback_s, 45.0);
    QCOMPARE(loaded.tilt.settle_s, s.tilt.settle_s);
}

void TestSettings::descriptorsCoverAllKeys()
{
    const auto descriptors = nav::filterSettingDescriptors();
    QVERIFY(!descriptors.empty());

    nav::FilterSettings s;
    const std::string text = nav::dumpFilterSettings(s);

    for (const auto &d : descriptors)
    {
        QVERIFY(!d.key.empty());
        QVERIFY(!d.label.empty());
        QVERIFY(d.has_range);
        QVERIFY(d.min_value <= d.max_value);

        // В INI ключ записывается внутри секции, без префикса "group/".
        std::string key = d.key;
        const std::size_t slash = key.find('/');
        std::string group;
        if (slash != std::string::npos)
        {
            group = key.substr(0, slash);
            key.erase(0, slash + 1);
        }
        QVERIFY(!key.empty());
        if (!group.empty())
        {
            QVERIFY2(text.find("[" + group + "]") != std::string::npos, group.c_str());
        }
        QVERIFY2(text.find(key) != std::string::npos, key.c_str());
    }
}

QTEST_MAIN(TestSettings)
#include "test_settings.moc"