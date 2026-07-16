#ifndef MATH_UTILS
#define MATH_UTILS

// =============================================================================
// math_utils.h — математические операции для БИНС
//
// Содержит:
//   - matMulVec: умножение матрицы 3×3 на вектор
//   - transposeMat3: транспонирование матрицы 3×3
//   - buildRotationMatrix: построение матрицы поворота из углов Эйлера–Крылова
//
// Матрица C (направляющих косинусов) переводит векторы из географической
// системы координат (NUE: Север–Вверх–Восток) в связанную (OXYZ: нос–вверх–правый борт).
//
// Последовательность поворотов (ф-лы 3.19–3.23 из теории):
//   1. ψ (yaw/рыскание)  围绕 OY_g (Север→Восток)
//   2. θ (pitch/тангаж)  围绕 OZ'  (Вверх→нос)
//   3. γ (roll/крен)     围绕 OX'' (нос→правый борт)
// =============================================================================

#include "types.h"

/**
 * Умножение матрицы 3×3 на трёхмерный вектор: result = M · v
 *
 * @param m матрица 3×3
 * @param v вектор
 * @return  результат умножения
 */
inline Vec3 matMulVec(const Mat3& m, const Vec3& v)
{
    return
    {
        m.data[0][0] * v.x + m.data[0][1] * v.y + m.data[0][2] * v.z,
        m.data[1][0] * v.x + m.data[1][1] * v.y + m.data[1][2] * v.z,
        m.data[2][0] * v.x + m.data[2][1] * v.y + m.data[2][2] * v.z
    };
}

/**
 * Транспонирование матрицы 3×3: result = M^T
 *
 * Используется для обратного преобразования (из связанной в географическую):
 *   если C переводит geo → body, то C^T переводит body → geo
 */
inline Mat3 transposeMat3(const Mat3& m)
{
    Mat3 res;

    res.data[0][0] = m.data[0][0]; res.data[0][1] = m.data[1][0]; res.data[0][2] = m.data[2][0];
    res.data[1][0] = m.data[0][1]; res.data[1][1] = m.data[1][1]; res.data[1][2] = m.data[2][1];
    res.data[2][0] = m.data[0][2]; res.data[2][1] = m.data[1][2]; res.data[2][2] = m.data[2][2];

    return res;
}

/**
 * Построение матрицы направляющих косинусов C из углов Эйлера–Крылова.
 *
 * Матрица C = A_γ · A_θ · A_ψ (ф-ла 3.24 из теории).
 * Переводит векторы из географической системы (NUE) в связанную (OXYZ).
 *
 * @param yaw    ψ — рыскание (рад), поворот вокруг вертикали (OY_g)
 * @param pitch  θ — тангаж (рад), поворот вокруг поперечной оси (OZ')
 * @param roll   γ — крен (рад), поворот вокруг продольной оси (OX'')
 * @return       матрица C размером 3×3
 */
inline Mat3 buildRotationMatrix(double yaw, double pitch, double roll)
{
    double c_y = cos(yaw);
    double s_y = sin(yaw);

    double c_p = cos(pitch);
    double s_p = sin(pitch);

    double c_r = cos(roll);
    double s_r = sin(roll);

    Mat3 C;

    // Строка 0: проекция вектора на ось OX связанной (нос)
    C.data[0][0] = c_p * c_y;
    C.data[0][1] = -c_r * c_y * s_p + s_r * s_y;
    C.data[0][2] = s_r * c_y * s_p + c_r * s_y;

    // Строка 1: проекция вектора на ось OY связанной (вверх)
    C.data[1][0] = s_p;
    C.data[1][1] = c_r * c_p;
    C.data[1][2] = -s_r * c_p;

    // Строка 2: проекция вектора на ось OZ связанной (правый борт)
    C.data[2][0] = -c_p * s_y;
    C.data[2][1] = c_r * s_y * s_p + s_r * c_y;
    C.data[2][2] = -s_r * s_y * s_p + c_r * c_y;

    return C;
}

#endif
