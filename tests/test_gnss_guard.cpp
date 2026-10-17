// test_gnss_guard.cpp — Модульные тесты защиты СНС (уровни 1-2).
//
// Проверяются самодостаточная валидация отсчёта (диапазоны, нулевой фикс,
// скорость, полоса высоты, кинематика, монотонность времени), детектор
// молчания и выключатель gnss_guard_on.

#include <cmath>

#include <QtTest>

#include "core/ins/ins_filter.h"
#include "core/navigation/gnss_guard.h"
#include "core/utils/constants.h"

namespace
{

// Отсчёт СНС с нулевыми углами ориентации (внутри — радианы).
nav::SnsSample makeSample(double t, double lat_deg, double lon_deg, double alt_m,
                          double vn, double ve, double vh = 0.0)
{
    nav::SnsSample s;
    s.time = t;
    s.lat = lat_deg * DEG_TO_RAD;
    s.lon = lon_deg * DEG_TO_RAD;
    s.alt = alt_m;
    s.vn = vn;
    s.vh = vh;
    s.ve = ve;
    s.heading = 0.0;
    s.pitch = 0.0;
    s.roll = 0.0;
    return s;
}

} // namespace

class TestGnssGuard : public QObject
{
    Q_OBJECT

private slots:
    void validFramePasses();
    void nonFiniteRejected();
    void rangeRejected();
    void zeroFixRejected();
    void speedRejected();
    void altitudeBandRejected();
    void kinematicsRejected();
    void timeGoesBackwardsRejected();
    void timeTooLargeRejected();
    void staleAfterSilence();
    void validFrameAfterGapAccepted();
    void guardOffAllows();
    void kinematicsAllowsNoise();
};

void TestGnssGuard::validFramePasses()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(g.update(makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::VALID);
}

void TestGnssGuard::nonFiniteRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    auto s = makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0);
    s.lat = std::nan("");
    QVERIFY(!g.update(s, 0.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::NONFINITE);
}

void TestGnssGuard::rangeRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(!g.update(makeSample(0.0, 95.0, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::RANGE);
}

void TestGnssGuard::zeroFixRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(!g.update(makeSample(0.0, 0.0, 0.0, 100.0, 0.0, 0.0), 0.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::ZERO_FIX);
}

void TestGnssGuard::speedRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(!g.update(makeSample(0.0, 47.64, 122.14, 100.0, 80.0, 0.0), 0.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::SPEED);
}

void TestGnssGuard::altitudeBandRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(g.update(makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    // Полоса высоты ±SNS_ALT_BAND_M вокруг первого принятого фикса: сам край
    // полосы ещё допустим, поэтому берём выход за него с запасом.
    QVERIFY(!g.update(makeSample(0.1, 47.64, 122.14, 100.0 + nav::SNS_ALT_BAND_M + 1.0,
                                 0.0, 0.0), 0.1, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::ALTITUDE);
}

void TestGnssGuard::kinematicsRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(g.update(makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    QVERIFY(!g.update(makeSample(0.1, 47.74, 122.14, 100.0, 0.0, 0.0), 0.1, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::KINEMATICS);
}

void TestGnssGuard::timeGoesBackwardsRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(g.update(makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    QVERIFY(!g.update(makeSample(-0.1, 47.64, 122.14, 100.0, 0.0, 0.0), -0.1, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::TIME);
}

void TestGnssGuard::timeTooLargeRejected()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(g.update(makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    QVERIFY(!g.update(makeSample(4000.0, 47.64, 122.14, 100.0, 0.0, 0.0), 4000.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::TIME);
}

void TestGnssGuard::staleAfterSilence()
{
    ins::KalmanConfig cfg;
    cfg.gnss_silence_timeout_s = 0.5;
    nav::SnsGuard g;
    const auto s = makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0);
    QVERIFY(g.update(s, 0.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::VALID);

    // Тот же (удерживаемый) отсчёт, но время ИМУ ушло дальше таймаута.
    QVERIFY(!g.update(s, 0.6, cfg));
    QCOMPARE(g.status, nav::SnsStatus::STALE);
}

void TestGnssGuard::validFrameAfterGapAccepted()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(g.update(makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    // Возврат СНС после паузы: допуск растёт вместе с dt, кадр принимается.
    QVERIFY(g.update(makeSample(5.0, 47.64, 122.1395, 100.0, 0.0, 0.0), 5.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::VALID);
}

void TestGnssGuard::guardOffAllows()
{
    ins::KalmanConfig cfg;
    cfg.gnss_guard_on = 0.0;
    nav::SnsGuard g;
    // С выключенной защитой решение прежнее (коррекция разрешена),
    // но наблюдение продолжается: статус и причина видны.
    QVERIFY(g.update(makeSample(0.0, 0.0, 0.0, 100.0, 0.0, 0.0), 0.0, cfg));
    QCOMPARE(g.status, nav::SnsStatus::INVALID);
    QCOMPARE(g.reject, nav::SnsReject::ZERO_FIX);
}

void TestGnssGuard::kinematicsAllowsNoise()
{
    ins::KalmanConfig cfg;
    nav::SnsGuard g;
    QVERIFY(g.update(makeSample(0.0, 47.64, 122.14, 100.0, 0.0, 0.0), 0.0, cfg));
    // Шум СНС порядка десятков метров на интервале ~0.09 с должен проходить.
    QVERIFY(g.update(makeSample(0.09, 47.64, 122.1398, 100.0, 0.0, 0.0), 0.09, cfg));
    QCOMPARE(g.status, nav::SnsStatus::VALID);
}

QTEST_MAIN(TestGnssGuard)
#include "test_gnss_guard.moc"
