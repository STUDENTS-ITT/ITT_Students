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