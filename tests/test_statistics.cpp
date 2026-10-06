// test_statistics.cpp — Статистические функции выставки и интегрирования.

#include <cmath>

#include <QtTest>

#include "core/math_lib/interpolation.h"
#include "core/math_lib/statistics.h"
#include "core/math_lib/transformations.h"
#include "core/utils/constants.h"

class TestStatistics : public QObject
{
    Q_OBJECT

private slots:
    void averageDegrees();
    void mean();
    void circularMeanAcrossBoundary();
    void circularMeanSimple();
    void movingAverageConstant();
    void movingAverageImpulse();
    void movingAverageEdgeCases();
    void integrationTrapezoid();
};

void TestStatistics::averageDegrees()
{
    // Average переводит радианы в градусы.
    QCOMPARE(Average({}), 0.0);
    QVERIFY(qAbs(Average({0.0, 0.0, 0.0})) < 1e-12);
    QVERIFY(qAbs(Average({PI / 2.0}) - 90.0) < 1e-12);
    QVERIFY(qAbs(Average({0.0, PI / 2.0}) - 45.0) < 1e-12);
}

void TestStatistics::mean()
{
    QCOMPARE(Mean({}), 0.0);
    QVERIFY(qAbs(Mean({1.0, 2.0, 3.0}) - 2.0) < 1e-15);
    QVERIFY(qAbs(Mean({-1.0, 1.0}) - 0.0) < 1e-15);
}

void TestStatistics::circularMeanAcrossBoundary()
{
    // Углы по разные стороны стыка ±π: наивное средние даёт ≈0, здесь — ≈π.
    const Vector a = {PI - 0.05, -PI + 0.05};
    const double m = circularMean(a);
    QVERIFY(qAbs(normalize_angle(m) - normalize_angle(PI)) < 1e-9);
}

void TestStatistics::circularMeanSimple()
{
    QVERIFY(qAbs(circularMean({0.0, 0.0, 0.0})) < 1e-12);
    QVERIFY(qAbs(circularMean({0.0, 0.1, -0.1})) < 1e-12);
    QVERIFY(qAbs(circularMean({PI / 2.0, PI / 2.0}) - PI / 2.0) < 1e-12);
}

void TestStatistics::movingAverageConstant()
{
    const Vector v(11, 2.5);
    const Vector s = movingAverage(v, 5);
    QCOMPARE(s.size(), v.size());
    for (double x : s)
    {
        QVERIFY(qAbs(x - 2.5) < 1e-12);
    }
}

void TestStatistics::movingAverageImpulse()
{
    // Одиночный выброс в центре размазывается, но не пропадает полностью.
    Vector v(9, 0.0);
    v[4] = 1.0;
    const Vector s = movingAverage(v, 5);

    QCOMPARE(s.size(), v.size());
    QVERIFY(s[4] < 1.0);
    QVERIFY(s[4] > 0.0);
    // Симметрия относительно выброса.
    QVERIFY(qAbs(s[3] - s[5]) < 1e-12);
    QVERIFY(s[0] < 1e-12);
}

void TestStatistics::movingAverageEdgeCases()
{
    QCOMPARE(movingAverage({}, 5).size(), std::size_t(0));
    QCOMPARE(movingAverage({1.0, 2.0}, 0).size(), std::size_t(2));
    QCOMPARE(movingAverage({1.0, 2.0}, 0)[0], 0.0);

    // Окно больше длины массива и чётное окно не ломают результат.
    const Vector v = {1.0, 2.0, 3.0};
    const Vector s = movingAverage(v, 100);
    QCOMPARE(s.size(), v.size());
    for (const double x : s) QVERIFY(std::isfinite(x));
}

void TestStatistics::integrationTrapezoid()
{
    // v = v0 + (a_now + a_prev)/2 · dt
    QCOMPARE(v_integral(2.0, 1.0, 0.0, 0.5), 1.0 + (2.0 + 0.0) / 2.0 * 0.5);

    const Vector res = v_integral({2.0, 4.0}, {0.0, 0.0}, {0.0, 0.0}, 1.0);
    QCOMPARE(res.size(), std::size_t(2));
    QCOMPARE(res[0], 1.0);
    QCOMPARE(res[1], 2.0);
}

QTEST_MAIN(TestStatistics)
#include "test_statistics.moc"