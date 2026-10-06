// test_transformations.cpp — Переходы между СК тела и навигационной СК.

#include <cmath>

#include <QtTest>

#include "core/math_lib/transformations.h"
#include "core/utils/constants.h"

class TestTransformations : public QObject
{
    Q_OBJECT

private slots:
    void normalizeAngleRange();
    void normalizeAngleWraps();
    void unitConversion();
    void identityOrientation();
    void headingNinetyPointsEast();
    void bodyNavAreInverse();
    void matrixIsRotation();
    void tiltedOrientationRoundTrip();
};

void TestTransformations::normalizeAngleRange()
{
    QVERIFY(normalize_angle(0.0) == 0.0);
    QVERIFY(qAbs(normalize_angle(0.5) - 0.5) < 1e-15);
    QVERIFY(normalize_angle(2.0 * PI - 0.1) < 0.0);
    QVERIFY(normalize_angle(-2.0 * PI + 0.1) > 0.0);

    for (double a = -20.0; a < 20.0; a += 0.37)
    {
        const double n = normalize_angle(a);
        QVERIFY(n > -PI - 1e-12 && n <= PI + 1e-12);
    }
}

void TestTransformations::normalizeAngleWraps()
{
    // Несколько оборотов сводятся к тому же углу.
    QVERIFY(qAbs(normalize_angle(3.0 * PI) - normalize_angle(PI)) < 1e-12);
    QVERIFY(qAbs(normalize_angle(5.0 * PI / 2.0) - PI / 2.0) < 1e-12);

    // Разность углов, кратная 2π, обнуляется (важно для инновации по углам).
    QVERIFY(qAbs(normalize_angle(0.3 + 2.0 * PI) - 0.3) < 1e-12);
    QVERIFY(qAbs(normalize_angle(0.3 - 2.0 * PI) - 0.3) < 1e-12);
}

void TestTransformations::unitConversion()
{
    QVERIFY(qAbs(rad_to_deg(PI) - 180.0) < 1e-12);
    QVERIFY(qAbs(deg_to_rad(180.0) - PI) < 1e-12);
    QVERIFY(qAbs(rad_to_deg(deg_to_rad(37.5)) - 37.5) < 1e-12);
}

void TestTransformations::identityOrientation()
{
    const Matrix C = bodyToNavMatrix(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            QVERIFY(qAbs(at(C, i, j, 3) - (i == j ? 1.0 : 0.0)) < 1e-12);
        }
    }

    const Vector v = {1.0, 2.0, 3.0};
    const Vector n = bodyToNav(C, v);
    for (int i = 0; i < 3; i++) QVERIFY(qAbs(n[i] - v[i]) < 1e-12);
}

void TestTransformations::headingNinetyPointsEast()
{
    // Продольная ось тела при ψ = 90° смотрит на восток: (0, 0, 1).
    const Matrix C = bodyToNavMatrix(PI / 2.0, 0.0, 0.0);
    const Vector n = bodyToNav(C, {1.0, 0.0, 0.0});
    QVERIFY(qAbs(n[0]) < 1e-12); // север
    QVERIFY(qAbs(n[1]) < 1e-12); // верх
    QVERIFY(qAbs(n[2] - 1.0) < 1e-12); // восток
}

void TestTransformations::bodyNavAreInverse()
{
    const double angles[][3] = {
        {0.0, 0.0, 0.0},
        {0.7, -0.2, 0.15},
        {-2.5, 0.4, -0.9},
        {3.0, -0.6, 1.2},
    };

    const Vector v = {12.0, -3.0, 7.5};

    for (const auto &a : angles)
    {
        const Matrix C = bodyToNavMatrix(a[0], a[1], a[2]);

        const Vector nav = bodyToNav(C, v);
        const Vector back = navToBody(C, nav);
        for (int i = 0; i < 3; i++)
        {
            QVERIFY(qAbs(back[i] - v[i]) < 1e-9);
        }
    }
}

void TestTransformations::matrixIsRotation()
{
    // Cᵀ·C = I для любой ориентации.
    const Matrix C = bodyToNavMatrix(0.9, 0.3, -0.45);
    const Matrix CT = transpose_m(C, 3);
    const Matrix G = multiply_matrix(CT, C, 3, 3);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            QVERIFY(qAbs(at(G, i, j, 3) - (i == j ? 1.0 : 0.0)) < 1e-12);
        }
    }
}

void TestTransformations::tiltedOrientationRoundTrip()
{
    // Ненулевая ориентация: вертикаль тела не совпадает с вертикалью мира,
    // но обратное преобразование возвращает исходный вектор.
    const double yaw = 1.1, pitch = 0.25, roll = -0.35;
    const Matrix C = bodyToNavMatrix(yaw, pitch, roll);

    const Vector v_body = {5.0, 1.0, -2.0};
    const Vector v_nav = bodyToNav(C, v_body);
    const Vector v_back = navToBody(C, v_nav);

    for (int i = 0; i < 3; i++)
    {
        QVERIFY(qAbs(v_back[i] - v_body[i]) < 1e-9);
    }
}

QTEST_MAIN(TestTransformations)
#include "test_transformations.moc"