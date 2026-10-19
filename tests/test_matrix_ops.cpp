// test_matrix_ops.cpp — Проверка конвенций линейной алгебры.
//
// Особое внимание к двум ошибкам, которые компилируются без предупреждений,
// но дают мусор (см. раздел «Критичные конвенции»):
//   - transpose_m(A, cols): второй аргумент — число СТОЛБЦОВ A.
//   - multiply_matrix(A, B, cols_A, cols_B): внутренняя размерность (строки B)
//     должна совпадать с cols_A.

#include <cmath>

#include <QtTest>

#include "core/math_lib/matrix_ops.h"

class TestMatrixOps : public QObject
{
    Q_OBJECT

private slots:
    void atIndexing();
    void multiplyMatrixInnerDimension();
    void transposeUsesColumnCount();
    void transposeRoundTrip();
    void sumDiff();
    void inverseIdentity();
    void inverseNonTrivial();
    void inverseSingularFallback();
    void solveSpdTinyScale();
    void solveSpdCorrelatedBlocks();
    void solveSpdRealScalesOfFilter();
    void solveSpdRejectsNonPositive();
    void identityAndObservation();
    void vectorOps();
};

void TestMatrixOps::atIndexing()
{
    // Матрица 2×3, хранится построчно: a00 a01 a02 a10 a11 a12
    Matrix A = {1, 2, 3, 4, 5, 6};

    QCOMPARE(at(A, 0, 0, 3), 1.0);
    QCOMPARE(at(A, 1, 2, 3), 6.0);
    QCOMPARE(getRows(A, 3), 2);

    at(A, 1, 0, 3) = 40.0;
    QCOMPARE(A[3], 40.0);
}

void TestMatrixOps::multiplyMatrixInnerDimension()
{
    // A: 2×3, B: 3×2 → результат 2×2.
    Matrix A = {1, 2, 3, 4, 5, 6};
    Matrix B = {7, 8, 9, 10, 11, 12};
    const Matrix C = multiply_matrix(A, B, 3, 2);

    QCOMPARE(C.size(), std::size_t(4));
    QCOMPARE(at(C, 0, 0, 2), 58.0);  // 1*7 + 2*9 + 3*11
    QCOMPARE(at(C, 0, 1, 2), 64.0);  // 1*8 + 2*10 + 3*12
    QCOMPARE(at(C, 1, 0, 2), 139.0); // 4*7 + 5*9 + 6*11
    QCOMPARE(at(C, 1, 1, 2), 154.0);

    // Умножение на вектор той же размерности.
    const Vector v = {1, 1, 1};
    const Vector r = multiply_m(A, v, 3);
    QCOMPARE(r.size(), std::size_t(2));
    QCOMPARE(r[0], 6.0);
    QCOMPARE(r[1], 15.0);
}

void TestMatrixOps::transposeUsesColumnCount()
{
    // Несимметричная матрица 2×3: транспонирование обязано дать 3×2.
    Matrix A = {1, 2, 3, 4, 5, 6};
    const Matrix AT = transpose_m(A, 3);

    QCOMPARE(AT.size(), std::size_t(6));
    QCOMPARE(getRows(AT, 2), 3);
    // AT[i][j] == A[j][i]
    QCOMPARE(at(AT, 0, 0, 2), 1.0);
    QCOMPARE(at(AT, 0, 1, 2), 4.0); // A[1][0] — ушёл вправо
    QCOMPARE(at(AT, 1, 0, 2), 2.0); // A[0][1] — ушёл вниз
    QCOMPARE(at(AT, 2, 1, 2), 6.0);

    // Двойное транспонирование возвращает исходную матрицу.
    const Matrix A2 = transpose_m(AT, 2);
    QCOMPARE(A2.size(), A.size());
    for (std::size_t i = 0; i < A.size(); i++)
    {
        QVERIFY(qFuzzyCompare(A2[i] + 1.0, A[i] + 1.0));
    }
}

void TestMatrixOps::transposeRoundTrip()
{
    // A·Aᵀ = A·Aᵀ — проверяем через произведение с транспонированной.
    Matrix A = {1, 2, 3, 4, 5, 6};
    const Matrix AT = transpose_m(A, 3);
    const Matrix G = multiply_matrix(A, AT, 3, 2);
    QCOMPARE(getRows(G, 2), 2);
    // Диагональ — суммы квадратов строк.
    QCOMPARE(at(G, 0, 0, 2), 14.0);
    QCOMPARE(at(G, 1, 1, 2), 77.0);
    QCOMPARE(at(G, 0, 1, 2), at(G, 1, 0, 2));
}

void TestMatrixOps::sumDiff()
{
    Matrix A = {1, 2, 3, 4, 5, 6};
    Matrix B = {1, 1, 1, 2, 2, 2};

    const Matrix S = matrix_sum(A, B, 3);
    QCOMPARE(at(S, 1, 1, 3), 7.0);
    const Matrix D = matrix_diff(A, B, 3);
    QCOMPARE(at(D, 1, 1, 3), 3.0);
    QCOMPARE(at(D, 1, 0, 3), 2.0);
}

void TestMatrixOps::inverseIdentity()
{
    const Matrix E = E_matrix(4);
    const Matrix I = return_matrix(E, 4);
    QCOMPARE(I.size(), E.size());
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            QCOMPARE(at(I, i, j, 4), (i == j) ? 1.0 : 0.0);
        }
    }
}

void TestMatrixOps::inverseNonTrivial()
{
    // A = [[4,7],[2,6]] → A⁻¹ = [[0.6,-0.7],[-0.2,0.4]]
    Matrix A = {4, 7, 2, 6};
    const Matrix I = return_matrix(A, 2);
    QVERIFY(qAbs(at(I, 0, 0, 2) - 0.6) < 1e-12);
    QVERIFY(qAbs(at(I, 0, 1, 2) + 0.7) < 1e-12);
    QVERIFY(qAbs(at(I, 1, 0, 2) + 0.2) < 1e-12);
    QVERIFY(qAbs(at(I, 1, 1, 2) - 0.4) < 1e-12);

    // A·A⁻¹ = I.
    const Matrix P = multiply_matrix(A, I, 2, 2);
    QVERIFY(qAbs(at(P, 0, 0, 2) - 1.0) < 1e-12);
    QVERIFY(qAbs(at(P, 0, 1, 2)) < 1e-12);
    QVERIFY(qAbs(at(P, 1, 0, 2)) < 1e-12);
    QVERIFY(qAbs(at(P, 1, 1, 2) - 1.0) < 1e-12);
}

void TestMatrixOps::inverseSingularFallback()
{
    // Вырожденная матрица: возвращается единичная (страховка от NaN).
    Matrix A = {1, 2, 2, 4}; // det = 0
    const Matrix I = return_matrix(A, 2);
    QCOMPARE(I.size(), std::size_t(4));
    QVERIFY(std::isfinite(I[0]));
    QCOMPARE(at(I, 0, 0, 2), 1.0);
}

void TestMatrixOps::solveSpdTinyScale()
{
    // Диагональная S с «фильтровыми» масштабами: координаты — рад² (~1e-16),
    // высота — м² (~25). Точное решение диагональной системы — y_i = b_i/S_ii.
    //
    // Почему нужен отдельный решатель: в return_matrix есть АБСОЛЮТНЫЙ порог
    // 1e-15 (выбор ведущего элемента и пропуск строки при исключении), поэтому
    // на координатных строках он возвращает единичную матрицу и квадратичная
    // форма вырождается в ‖z‖² вместо zᵀ·S⁻¹·z.
    const int n = 3;
    Matrix S(n * n, 0.0);
    at(S, 0, 0, n) = 1.0e-16;
    at(S, 1, 1, n) = 4.0e-16;
    at(S, 2, 2, n) = 25.0;

    const Vector b = {1.0e-8, 2.0e-8, 5.0};

    Vector y;
    QVERIFY(cholesky_solve(S, n, b, y));
    QCOMPARE(y.size(), std::size_t(n));

    const double expect[3] = {1.0e8, 5.0e7, 0.2};
    for (int i = 0; i < n; i++)
        QVERIFY(std::fabs(y[i] - expect[i]) <= 1e-12 * std::fabs(expect[i]));

    // Квадратичная форма точная: 1 + 1 + 1 = 3.
    double nis = 0.0;
    for (int i = 0; i < n; i++) nis += b[i] * y[i];
    QVERIFY(std::fabs(nis - 3.0) < 1e-12);
}

void TestMatrixOps::solveSpdCorrelatedBlocks()
{
    // Два коррелированных блока: «координатный» (масштаб 1e-14, корреляция 0.5)
    // и «скоростной» (масштаб ~1, корреляция -0.25). Эталон — формула Крамера
    // для 2×2, т. е. независимый путь решения.
    const int n = 4;
    Matrix S(n * n, 0.0);
    const double a = 1.0e-14, b = 4.0e-14, r = 0.5;
    at(S, 0, 0, n) = a;
    at(S, 0, 1, n) = r * std::sqrt(a * b);
    at(S, 1, 0, n) = at(S, 0, 1, n);
    at(S, 1, 1, n) = b;
    const double c = 6.5, d = 0.22, q = -0.25;
    at(S, 2, 2, n) = c;
    at(S, 2, 3, n) = q * std::sqrt(c * d);
    at(S, 3, 2, n) = at(S, 2, 3, n);
    at(S, 3, 3, n) = d;

    const Vector z = {-3.0e-7, 5.0e-7, 0.02, -0.1};

    Vector y;
    QVERIFY(cholesky_solve(S, n, z, y));

    const double detA = a * b - at(S, 0, 1, n) * at(S, 1, 0, n);
    const double detB = c * d - at(S, 2, 3, n) * at(S, 3, 2, n);
    const double expect[4] = {(z[0] * b - at(S, 0, 1, n) * z[1]) / detA,
                              (-at(S, 1, 0, n) * z[0] + a * z[1]) / detA,
                              (z[2] * d - at(S, 2, 3, n) * z[3]) / detB,
                              (-at(S, 3, 2, n) * z[2] + c * z[3]) / detB};
    for (int i = 0; i < n; i++)
        QVERIFY(std::fabs(y[i] - expect[i]) <= 1e-9 * std::fabs(expect[i]));

    // Невязка S·y − z мала: проверяется решение, а не только форма.
    const Vector Sy = multiply_m(S, y, n);
    for (int i = 0; i < n; i++)
        QVERIFY(std::fabs(Sy[i] - z[i]) <= 1e-9 * std::fabs(z[i]));
}

void TestMatrixOps::solveSpdRealScalesOfFilter()
{
    // Реальный блок S фильтра БИНС (static_300, кадр 0, состояния 0, 1, 3, 5:
    // долгота, широта, vn, ve) — тот самый разброс масштабов 1e-15…1.
    // Точный эталон NIS = 45.418800197008167 получен в рациональной арифметике
    // (Python, fractions.Fraction). На этом блоке return_matrix даёт -15231.9
    // (пропуск строк из-за абсолютного порога 1e-15), а Холецкий совпадает
    // с точным значением.
    const int n = 4;
    Matrix S(n * n, 0.0);
    const double rows[4][4] = {
        {5.471497657032136e-15, 0.0, 1.1113345628629727e-07, 1.3567873223770755e-12},
        {0.0, 7.8483996030276816e-15, -2.0137314623469908e-12, 1.6494326984939553e-07},
        {1.1113345628629727e-07, -2.0137314623469908e-12, 6.5170171494234959, 3.7302627358305197e-09},
        {1.3567873223770755e-12, 1.6494326984939553e-07, 3.7302627358305197e-09, 6.517017140093186}};
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) at(S, i, j, n) = rows[i][j];

    const Vector z = {-7.7262174547065854e-08, 4.01931416593726e-07,
                      -0.015416804946266204, 0.038531268138590292};

    Vector y;
    QVERIFY(cholesky_solve(S, n, z, y));

    double nis = 0.0;
    for (int i = 0; i < n; i++) nis += z[i] * y[i];
    QVERIFY(std::fabs(nis - 45.418800197008167) <= 1e-9 * 45.418800197008167);
}

void TestMatrixOps::solveSpdRejectsNonPositive()
{
    // Нулевая диагональ и знаконеопределённая матрица — отказ (false), а не
    // молчаливо неверное решение.
    Vector y;
    Matrix zero = {0.0, 0.0, 0.0, 1.0};
    QVERIFY(!cholesky_solve(zero, 2, Vector{1.0, 1.0}, y));

    Matrix indefinite = {1.0, 0.0, 0.0, -1.0};
    QVERIFY(!cholesky_solve(indefinite, 2, Vector{1.0, 1.0}, y));
}

void TestMatrixOps::identityAndObservation()
{
    const Matrix E = E_matrix(3);
    QCOMPARE(E.size(), std::size_t(9));
    QCOMPARE(at(E, 2, 2, 3), 1.0);
    QCOMPARE(at(E, 0, 1, 3), 0.0);

    // H 9×15: единицы по главной диагонали (измеряем первые 9 состояний).
    const Matrix H = H_matrix(9, 15);
    QCOMPARE(H.size(), std::size_t(135));
    QCOMPARE(getRows(H, 15), 9);
    for (int i = 0; i < 9; i++)
    {
        QCOMPARE(at(H, i, i, 15), 1.0);
    }
    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 15; j++)
        {
            if (i != j) QCOMPARE(at(H, i, j, 15), 0.0);
        }
    }

    // H·x = первые 9 компонент x.
    Vector x(15);
    for (int i = 0; i < 15; i++) x[i] = i + 1.0;
    const Vector z = multiply_m(H, x, 15);
    QCOMPARE(z.size(), std::size_t(9));
    for (int i = 0; i < 9; i++) QCOMPARE(z[i], i + 1.0);
}

void TestMatrixOps::vectorOps()
{
    const Vector a = {1, 0, 0};
    const Vector b = {0, 1, 0};

    const Vector c = vector_product(a, b);
    QCOMPARE(c[0], 0.0);
    QCOMPARE(c[1], 0.0);
    QCOMPARE(c[2], 1.0);

    const Vector d = vector_sum(a, {1, 2, 3});
    QCOMPARE(d[2], 3.0);
    const Vector e = vector_diff(a, {1, 1, 1});
    QCOMPARE(e[0], 0.0);
    QCOMPARE(e[1], -1.0);
}

QTEST_MAIN(TestMatrixOps)
#include "test_matrix_ops.moc"