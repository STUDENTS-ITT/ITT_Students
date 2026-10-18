// test_bias_estimate.cpp — Этап 0 плана docs/bias_estimation_plan.md.
//
// Проверяется журнал смещений (bias.txt) и сводный отчёт по нему:
//   1. журнал пишется только по флагу write_bias_log — на result/reference/errors
//      и test_parity не влияет;
//   2. формат: 15 колонок, конечные числа, монотонное время;
//   3. вертикальная компонента ba_Y и курсовая bg_Y сходятся в пределах 2σ —
//      они наблюдаемы и сегодня;
//   4. горизонтальные ba_X/ba_Z НЕ сходятся — это воспроизводимый дефект R2
//      (скрытый трим и аксель-апдейт без столбца ba в H). Тест фиксирует его
//      как ожидаемый провал: когда этапы 1–4 исправят структуру, Qt сообщит
//      «Unexpected Pass» и маркер нужно будет снять.
//
// Тест сам генерирует набор, поэтому не зависит от данных в репозитории.

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include <QtTest>

#include "core/data_io/dataset_generator.h"
#include "core/navigation/aligner.hpp"
#include "core/navigation/bias_log.h"
#include "core/navigation/bias_report.h"
#include "core/navigation/simulation.h"
#include "core/utils/constants.h"

namespace
{

// Истина смещений из конфига генератора: mg → м/с², °/ч → рад/с.
// Коэффициенты сверены с params.json наборов Katya (раздел noise_si).
nav::BiasTruth makeTruth(const dataset::DatasetConfig &cfg)
{
    nav::BiasTruth t;
    t.valid = true;
    t.name = cfg.name;
    t.align_time_s = cfg.align_time_s;
    // g по широте/высоте выставки — тот же расчёт, что в выставке.
    const double g = calculate_g(cfg.lat_deg * DEG_TO_RAD, cfg.alt_m);
    for (int k = 0; k < 3; k++)
    {
        t.ba[k] = cfg.noise.accel_bias_mg[k] * 1e-3 * g;   // мг → м/с²
        t.bg[k] = cfg.noise.gyro_bias_deg_h[k] * DEG_TO_RAD / 3600.0;  // °/ч → рад/с
    }
    // Манёвренный профиль (есть поворот) — наблюдаемы все шесть компонент.
    for (int k = 0; k < 6; k++) t.observable[k] = true;
    t.observability_note = "maneuvering profile (test)";
    return t;
}

// Конфиг: 40 с выставки + разгон/поворот/торможение, 130 с всего.
// Есть поворот, поэтому горизонтальные смещения наблюдаемы в принципе.
dataset::DatasetConfig makeConfig(const QString &out_dir, quint64 seed)
{
    dataset::DatasetConfig cfg;
    cfg.name = "bias_estimate";
    cfg.out_dir = out_dir.toStdString();
    cfg.imu_hz = 200.0;
    cfg.gps_hz = 10.0;
    cfg.angle_hz = 10.0;
    cfg.lat_deg = 47.641468;
    cfg.lon_deg = -122.1401651;
    cfg.alt_m = 116.975;
    cfg.heading_deg = 5.0;
    cfg.align_time_s = 40.0;
    cfg.profile = {
        {40.0, 0.0, 0.0, 0.0},   // выставка
        {20.0, 10.0, 0.0, 0.0},  // разгон
        {30.0, 10.0, 0.0, 1.5},  // поворот — делает ba_гор/bg_гор наблюдаемыми
        {20.0, 10.0, 0.0, 0.0},  // прямо
        {20.0, 0.0, 0.0, 0.0},   // торможение
    };
    // Смещения по умолчанию конфига (mg / °/ч) — ненулевые, чтобы сходимость
    // была отличима от нуля.
    cfg.seed = seed;
    return cfg;
}

// Последний кадр журнала (финальная оценка смещений).
const nav::BiasSample &lastSample(const std::vector<nav::BiasSample> &log)
{
    return log.back();
}

} // namespace

class TestBiasEstimate : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();

    // Журнал не создаётся без флага: паритет result/reference/errors сохраняется.
    void noLogByDefault();

    // Флаг включает журнал, формат корректен.
    void logWrittenAndParseable();

    // Наблюдаемые компоненты сходятся в пределах 2σ.
    void observableComponentsConverge();

    // Дефект R2: горизонтальные смещения не сходятся (ожидаемый провал).
    void horizontalBaStuck_documentsR2();

    // Сводный отчёт строится и содержит итоговую строку.
    void reportBuilds();

private:
    QTemporaryDir dir_;
    dataset::DatasetConfig cfg_;
    nav::BiasTruth truth_;
    std::vector<nav::BiasSample> log_;
    bool have_log_ = false;
    std::string log_error_;
};

void TestBiasEstimate::initTestCase()
{
    QVERIFY2(dir_.isValid(), "не удалось создать временный каталог");

    cfg_ = makeConfig(dir_.path(), 20261018);
    truth_ = makeTruth(cfg_);

    dataset::DatasetReport report;
    std::string error;
    QVERIFY2(dataset::generateDataset(cfg_, report, &error), error.c_str());

    // Прогон с журналом смещений: output_dir — тот же временный каталог.
    nav::RunOptions options;
    options.data_dir = dir_.path().toStdString();
    options.output_dir = dir_.path().toStdString();
    options.write_files = true;
    options.write_bias_log = true;

    nav::RunSummary summary;
    std::string report_text;
    std::string run_error;
    QVERIFY2(nav::runBiasReport(options, summary, report_text, run_error),
             run_error.c_str());
    QVERIFY(summary.ok);
    QVERIFY(summary.steps > 0);

    log_error_.clear();
    const std::string bias_path = (std::filesystem::path(options.output_dir) /
                                   "bias.txt").string();
    have_log_ = nav::readBiasLog(bias_path, log_, &log_error_);
    QVERIFY2(have_log_, qPrintable(QString::fromStdString(log_error_)));
}

void TestBiasEstimate::noLogByDefault()
{
    // Отдельный прогон без флага: bias.txt не должен появиться.
    QTemporaryDir plain_dir;
    QVERIFY(plain_dir.isValid());

    dataset::DatasetConfig plain_cfg = makeConfig(plain_dir.path(), 7);
    dataset::DatasetReport report;
    std::string error;
    QVERIFY2(dataset::generateDataset(plain_cfg, report, &error), error.c_str());

    nav::RunOptions options;
    options.data_dir = plain_dir.path().toStdString();
    options.output_dir = plain_dir.path().toStdString();
    options.write_files = true;
    options.write_bias_log = false;  // по умолчанию

    nav::RunSummary summary;
    QVERIFY(nav::runSimulation(options, summary, nullptr, nullptr, nullptr));

    const QString bias_path = plain_dir.path() + "/bias.txt";
    QVERIFY2(!QFile::exists(bias_path),
             "bias.txt не должен создаваться без write_bias_log");

    // Ключевые файлы на месте — паритет не задет.
    QVERIFY(QFile::exists(plain_dir.path() + "/result.txt"));
    QVERIFY(QFile::exists(plain_dir.path() + "/reference.txt"));
    QVERIFY(QFile::exists(plain_dir.path() + "/errors.txt"));
}

void TestBiasEstimate::logWrittenAndParseable()
{
    QVERIFY(have_log_);
    QVERIFY(log_.size() > 100);

    // Время строго возрастает (журнал пишется по тактам ИМУ).
    for (std::size_t i = 1; i < log_.size(); i++)
        QVERIFY2(log_[i].time > log_[i - 1].time,
                 qPrintable(QString("время не возрастает на строке %1").arg(i)));

    // readBiasLog уже отвергает NaN/Inf и неверное число колонок, поэтому
    // здесь достаточно проверить конечность и физичность сигм.
    for (const nav::BiasSample &s : log_)
    {
        for (int k = 0; k < 3; k++)
        {
            QVERIFY(std::isfinite(s.ba[k]));
            QVERIFY(std::isfinite(s.bg[k]));
            // σ(P) — стандартное отклонение, неотрицательно и конечно.
            QVERIFY(s.sig_ba[k] >= 0.0 && std::isfinite(s.sig_ba[k]));
            QVERIFY(s.sig_bg[k] >= 0.0 && std::isfinite(s.sig_bg[k]));
        }
    }

    // Журнал покрывает весь прогон: последний кадр близок к концу данных.
    QVERIFY(log_.back().time > cfg_.align_time_s);
}

void TestBiasEstimate::observableComponentsConverge()
{
    QVERIFY(have_log_);

    const nav::BiasSample &last = lastSample(log_);

    // ba_Y — вертикальная ось: гравитация даёт сильный сигнал, смещение
    // наблюдаемо на любом профиле. Ошибка в пределах 2σ(P).
    const double err_bay = last.ba[1] - truth_.ba[1];
    const double norm_bay = (last.sig_ba[1] > 0.0) ? err_bay / last.sig_ba[1] : 0.0;
    QVERIFY2(fabs(norm_bay) <= 2.0,
             qPrintable(QString("ba_Y: |оц-ист|/σ = %1 > 2 (оц %2, истина %3, σ %4)")
                            .arg(norm_bay).arg(last.ba[1])
                            .arg(truth_.ba[1]).arg(last.sig_ba[1])));

    // bg_Y — курсовая ось: смещение гироскопа по курсу наблюдаемо через
    // вращение Земли и повороты. Ошибка в пределах 2σ(P).
    const double err_bgy = last.bg[1] - truth_.bg[1];
    const double norm_bgy = (last.sig_bg[1] > 0.0) ? err_bgy / last.sig_bg[1] : 0.0;
    QVERIFY2(fabs(norm_bgy) <= 2.0,
             qPrintable(QString("bg_Y: |оц-ист|/σ = %1 > 2 (оц %2, истина %3, σ %4)")
                            .arg(norm_bgy).arg(last.bg[1])
                            .arg(truth_.bg[1]).arg(last.sig_bg[1])));
}

void TestBiasEstimate::horizontalBaStuck_documentsR2()
{
    QVERIFY(have_log_);

    const nav::BiasSample &last = lastSample(log_);

    // R2 (docs/bias_estimation_plan.md, раздел 4): correctTilt подаёт аксель
    // как абсолютный горизонт (H = e7,e8 без столбца ba), поэтому горизонтальные
    // смещения физически не могут вернуться из угла в оценку и сидят на ~0.
    //
    // Проверка зафиксирована как ОЖИДАЕМЫЙ провал: она документирует дефект.
    // После этапов 1–2 (снятие трима и штатный аксель-апдейт) условие должно
    // начать выполняться — Qt сообщит «Unexpected Pass», и маркер снимается.
    QEXPECT_FAIL("", "Дефект R2: ba_X/ba_Z не сходятся, пока не сделаны этапы 1–2",
                 Continue);

    const double err_x = last.ba[0] - truth_.ba[0];
    const double err_z = last.ba[2] - truth_.ba[2];
    const double norm_x = (last.sig_ba[0] > 0.0) ? err_x / last.sig_ba[0] : 0.0;
    const double norm_z = (last.sig_ba[2] > 0.0) ? err_z / last.sig_ba[2] : 0.0;

    QVERIFY2(fabs(norm_x) <= 2.0 && fabs(norm_z) <= 2.0,
             qPrintable(QString("ba_X/ba_Z: |оц-ист|/σ = %1 / %2 (должны быть <= 2)")
                            .arg(norm_x).arg(norm_z)));
}

void TestBiasEstimate::reportBuilds()
{
    QVERIFY(have_log_);

    const std::vector<nav::BiasReportRow> rows =
        nav::buildBiasReport(log_, truth_);
    QVERIFY(!rows.empty());

    // 6 компонент × 6 окон по умолчанию.
    QCOMPARE(rows.size(), std::size_t(36));

    const std::string text = nav::formatBiasReport(rows, truth_);
    QVERIFY(text.find("=== Bias estimate report ===") != std::string::npos);
    QVERIFY(text.find(truth_.name) != std::string::npos);
    // Итоговый блок в инженерных единицах (мг, °/ч) обязателен.
    QVERIFY(text.find("final state") != std::string::npos);

    // Истина в таблице должна совпадать с заданной (проверка разбора окон).
    bool found_bay = false;
    for (const nav::BiasReportRow &r : rows)
    {
        if (r.component == "ba_Y")
        {
            found_bay = true;
            QCOMPARE(r.truth, truth_.ba[1]);
        }
    }
    QVERIFY(found_bay);
}

QTEST_MAIN(TestBiasEstimate)
#include "test_bias_estimate.moc"
