// matrix_ops.h — Минимальная библиотека линейной алгебры для фильтра Калмана.
//
// Матрица хранится в одномерном векторе построчно.
// Функции принимают число столбцов отдельным аргументом.
//
// Содержит:
//   - Умножение матрицы на вектор и матрицы на матрицу
//   - Транспонирование, сложение, вычитание
//   - Обратная матрица (метод Гаусса-Жордана с выбором ведущего элемента)
//   - Решение симметричной положительно определённой системы (Холецкий)
//   - Единичная матрица, матрица наблюдения H
//   - Векторное произведение и арифметика векторов

#pragma once

#include <cmath>
#include <cstddef>
#include <iostream>
#include <utility>

#include "../utils/types.h"

// Доступ к элементу матрицы по индексам (i, j) и числу столбцов.
inline double &at(Matrix &A, int i, int j, int cols)
{
    return A[i * cols + j];
}

inline const double &at(const Matrix &A, int i, int j, int cols)
{
    return A[i * cols + j];
}

// Число строк матрицы по размеру вектора и числу столбцов.
inline int getRows(const Matrix &A, int cols)
{
    return static_cast<int>(A.size()) / cols;
}

// Умножение матрицы A (n×m) на вектор v (m).
inline Vector multiply_m(const Matrix &A, const Vector &v, int cols_A)
{
    int n = getRows(A, cols_A);
    int m = cols_A;
    Vector result(n, 0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            result[i] += at(A, i, j, cols_A) * v[j];
        }
    }
    return result;
}

// Умножение матрицы A (n×p) на матрицу B (p×m).
inline Matrix multiply_matrix(const Matrix &A, const Matrix &B, int cols_A, int cols_B)
{
    int n = getRows(A, cols_A);
    int m = cols_B;
    int p = getRows(B, cols_B);
    Matrix result(n * m, 0);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            double sum = 0;
            for (int k = 0; k < p; k++)
            {
                sum += at(A, i, k, cols_A) * at(B, k, j, cols_B);
            }
            at(result, i, j, m) = sum;
        }
    }
    return result;
}

// Транспонирование матрицы A (n×m) → A^T (m×n).
inline Matrix transpose_m(const Matrix &A, int cols)
{
    int n = getRows(A, cols);
    int m = cols;
    Matrix AT(m * n, 0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            at(AT, j, i, n) = at(A, i, j, cols);
        }
    }
    return AT;
}

// Покоординатное сложение матриц A + B.
inline Matrix matrix_sum(const Matrix &A, const Matrix &B, int cols)
{
    int n = getRows(A, cols);
    int m = cols;

    Matrix result(n * m, 0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            at(result, i, j, m) = at(A, i, j, m) + at(B, i, j, m);
        }
    }
    return result;
}

// Покоординатное вычитание матриц A - B.
inline Matrix matrix_diff(const Matrix &A, const Matrix &B, int cols)
{
    int n = getRows(A, cols);
    int m = cols;

    Matrix result(n * m, 0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            at(result, i, j, m) = at(A, i, j, m) - at(B, i, j, m);
        }
    }
    return result;
}

// Обратная матрица методом Гаусса-Жордана (partial pivoting).
// Принимает квадратную матрицу A (n×n), возвращает A⁻¹.
inline Matrix return_matrix(const Matrix &A, int cols)
{
    int n = getRows(A, cols);
    Matrix M(n * (2 * n), 0);

    // Формирование расширенной матрицы [A | I].
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            at(M, i, j, 2 * n) = at(A, i, j, n);
            at(M, i, j + n, 2 * n) = (i == j) ? 1.0 : 0.0;
        }
    }

    // Прямой ход: выбор ведущего элемента и исключение.
    for (int i = 0; i < n; i++)
    {
        int max_row = i;
        for (int k = i + 1; k < n; k++)
        {
            if (fabs(at(M, k, i, 2 * n)) > fabs(at(M, max_row, i, 2 * n)))
            {
                max_row = k;
            }
        }

        if (max_row != i)
        {
            for (int j = 0; j < 2 * n; j++)
            {
                std::swap(at(M, i, j, 2 * n), at(M, max_row, j, 2 * n));
            }
        }
        double pivot = at(M, i, i, 2 * n);
        if (fabs(pivot) < 1e-15)
        {
            Matrix I(n * n, 0);
            for (int r = 0; r < n; r++)
                at(I, r, r, n) = 1.0;
            return I;
        }

        // Нормализация строки.
        for (int j = 0; j < 2 * n; j++)
        {
            at(M, i, j, 2 * n) /= pivot;
        }
        // Исключение столбца во всех остальных строках.
        for (int k = 0; k < n; k++)
        {
            if (k == i)
                continue;
            double factor = at(M, k, i, 2 * n);
            if (fabs(factor) < 1e-15)
                continue;
            for (int j = 0; j < 2 * n; j++)
            {
                at(M, k, j, 2 * n) -= factor * at(M, i, j, 2 * n);
            }
        }
    }

    // Извлечение правой половины [I | A⁻¹] → A⁻¹.
    Matrix result(n * n, 0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            at(result, i, j, n) = at(M, i, j + n, 2 * n);
        }
    }
    return result;
}

// Решение S·y = b для симметричной положительно определённой S (n×n) методом
// Холецкого: S = L·Lᵀ, затем L·u = b (прямая подстановка) и Lᵀ·y = u (обратная).
// S задаётся полной матрицей, используется симметризованная полусумма
// (S_ij + S_ji)/2: после обновлений ковариации в форме Джозефа отклонения от
// симметрии — только численные.
//
// Зачем отдельно от return_matrix. Элементы S в фильтре различаются на
// порядки (координаты — рад² ~ 1e-16, высота — м² ~ 25, скорости — м²/с² ~ 1,
// углы — рад², «игнорируемые» строки тангажа/крена — ~1e8), поэтому обращение
// Гауссом-Жорданом с частичным выбором ведущего элемента теряет в почти
// вырожденных направлениях почти всю точность и даёт для заведомо
// положительной квадратичной формы zᵀ·S⁻¹·z отрицательные значения
// (docs/accuracy_reserve_diagnostics.md, раздел 6). Холецкий для
// положительно определённой S не требует выбора ведущего элемента.
//
// Сверка с точной арифметикой (рациональные числа) на реальных S фильтра
// (static_300, 2991 кадр коррекции): Холецкий совпадает до 4e-14 по
// относительной ошибке на каждом кадре и всегда даёт NIS > 0. Диагональное
// предусловие D = diag(1/√S_ii) (казалось бы, естественное для такого
// разброса единиц) здесь, наоборот, разрушает точность: на почти вырожденной
// S единичная диагональ делает пивоты шумом, и квадратичная форма уезжает на
// порядки (проверено: кадр 0 static_300 — 31.0 вместо точного 45.5124), при
// этом преобразование подобия точное. Поэтому предусловие сознательно не
// применяется.
//
// Возвращает false, если S не положительно определена (неположительный
// диагональный элемент или нулевая диагональ у S); в этом случае y не
// заполняется.
inline bool cholesky_solve(const Matrix &S, int n, const Vector &b, Vector &y)
{
    for (int i = 0; i < n; i++)
    {
        const double dii = at(S, i, i, n);
        if (!(dii > 0.0) || !std::isfinite(dii))
            return false;
    }

    Matrix L(n * n, 0.0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            double sum = 0.5 * (at(S, i, j, n) + at(S, j, i, n));
            for (int k = 0; k < j; k++)
                sum -= at(L, i, k, n) * at(L, j, k, n);

            if (i == j)
            {
                if (!(sum > 0.0) || !std::isfinite(sum))
                    return false;
                at(L, i, j, n) = std::sqrt(sum);
            }
            else
            {
                at(L, i, j, n) = sum / at(L, j, j, n);
            }
        }
    }

    Vector u(n, 0.0);
    for (int i = 0; i < n; i++)
    {
        double sum = b[i];
        for (int k = 0; k < i; k++)
            sum -= at(L, i, k, n) * u[k];
        u[i] = sum / at(L, i, i, n);
    }

    y.assign(n, 0.0);
    for (int i = n - 1; i >= 0; i--)
    {
        double sum = u[i];
        for (int k = i + 1; k < n; k++)
            sum -= at(L, k, i, n) * y[k];
        y[i] = sum / at(L, i, i, n);
    }
    return true;
}

// Единичная матрица n×n.
inline Matrix E_matrix(std::size_t n)
{
    Matrix E(n * n, 0);
    for (std::size_t i = 0; i < n; i++)
    {
        for (std::size_t j = 0; j < n; j++)
        {
            at(E, static_cast<int>(i), static_cast<int>(j), static_cast<int>(n)) = (i == j) ? 1.0 : 0.0;
        }
    }
    return E;
}

// Матрица наблюдения H (m×n): единицы на главной диагонали.
inline Matrix H_matrix(std::size_t m, std::size_t n)
{
    Matrix H(m * n, 0);
    for (std::size_t i = 0; i < m; i++)
    {
        at(H, static_cast<int>(i), static_cast<int>(i), static_cast<int>(n)) = 1;
    }
    return H;
}

// Векторное произведение a × b.
inline Vector vector_product(const Vector &a, const Vector &b)
{
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]};
}

// Разность векторов a - b.
inline Vector vector_diff(const Vector &a, const Vector &b)
{
    if (a.size() != b.size())
    {
        std::cerr << "err size vec" << std::endl;
        return {};
    }
    Vector res(a.size());
    for (std::size_t i = 0; i < a.size(); i++)
    {
        res[i] = a[i] - b[i];
    }
    return res;
}

// Сумма векторов a + b.
inline Vector vector_sum(const Vector &a, const Vector &b)
{
    if (a.size() != b.size())
    {
        std::cerr << "err size vec" << std::endl;
        return {};
    }
    Vector res(a.size());
    for (std::size_t i = 0; i < a.size(); i++)
    {
        res[i] = a[i] + b[i];
    }
    return res;
}
