#pragma once

// Операции с матрицами и векторами.
// Матрица A размера n x cols хранится построчно: A[i * cols + j].

#include <cmath>
#include <cstddef>
#include <iostream>
#include <utility>

#include "../utils/types.h"

// доступ к элементам матрицы, версия для записи
inline double &at(Matrix &A, int i, int j, int cols)
{
    return A[i * cols + j];
}

// версия для чтения
inline const double &at(const Matrix &A, int i, int j, int cols)
{
    return A[i * cols + j];
}

// число строк матрицы
inline int getRows(const Matrix &A, int cols)
{
    return static_cast<int>(A.size()) / cols;
}

// умножение матрицы на вектор
inline Vector multiply_m(const Matrix &A, const Vector &v, int cols_A)
{
    int n = getRows(A, cols_A); // строки
    int m = cols_A;             // столбцы
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

// умножение матриц
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

// транспонированная матрица
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

// сумма матриц
inline Matrix matrux_sum(const Matrix &A, const Matrix &B, int cols)
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

// разность матриц
inline Matrix matrux_diff(const Matrix &A, const Matrix &B, int cols)
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

// обратная матрица методом Гаусса–Жордана с выбором ведущего элемента
inline Matrix return_matrix(const Matrix &A, int cols)
{
    int n = getRows(A, cols);
    Matrix M(n * (2 * n), 0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            at(M, i, j, 2 * n) = at(A, i, j, n);
            at(M, i, j + n, 2 * n) = (i == j) ? 1.0 : 0.0;
        }
    }
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

        for (int j = 0; j < 2 * n; j++)
        {
            at(M, i, j, 2 * n) /= pivot;
        }
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

// единичная матрица n x n
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

// матрица наблюдения m x n вида [E | 0]
inline Matrix H_matrix(std::size_t m, std::size_t n)
{
    Matrix H(m * n, 0);
    for (std::size_t i = 0; i < m; i++)
    {
        at(H, static_cast<int>(i), static_cast<int>(i), static_cast<int>(n)) = 1;
    }
    return H;
}

// векторное произведение векторов 3*3
inline Vector vector_product(const Vector &a, const Vector &b)
{
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]};
}

// разность векторов
inline Vector vector_diff(const Vector &a, const Vector &b)
{
    if (a.size() != b.size())
    {
        std::cerr << "err size vec" << std::endl;
    }
    Vector res(a.size());
    for (std::size_t i = 0; i < a.size(); i++)
    {
        res[i] = a[i] - b[i];
    }
    return res;
}

// сумма векторов
inline Vector vector_sum(const Vector &a, const Vector &b)
{
    if (a.size() != b.size())
    {
        std::cerr << "err size vec" << std::endl;
    }
    Vector res(a.size());
    for (std::size_t i = 0; i < a.size(); i++)
    {
        res[i] = a[i] + b[i];
    }
    return res;
}
