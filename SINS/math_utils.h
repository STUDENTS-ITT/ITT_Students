#ifndef MATH_UTILS
#define MATH_UTILS

#include "types.h"

inline Vec3 matMulVec(const Mat3& m, const Vec3& v)
{
    return
    {
        m.data[0][0] * v.x + m.data[0][1] * v.y + m.data[0][2] * v.z,
        m.data[1][0] * v.x + m.data[1][1] * v.y + m.data[1][2] * v.z,
        m.data[2][0] * v.x + m.data[2][1] * v.y + m.data[2][2] * v.z
    };
}

inline Mat3 transposeMat3(const Mat3& m)
{
    Mat3 res;

    res.data[0][0] = m.data[0][0]; res.data[0][1] = m.data[1][0]; res.data[0][2] = m.data[2][0];
    res.data[1][0] = m.data[0][1]; res.data[1][1] = m.data[1][1]; res.data[1][2] = m.data[2][1];
    res.data[2][0] = m.data[0][2]; res.data[2][1] = m.data[1][2]; res.data[2][2] = m.data[2][2];

    return res;
}

inline Mat3 buildRotationMatrix(double yaw, double pitch, double roll)
{
    double c_y = cos(yaw);
    double s_y = sin(yaw);

    double c_p = cos(pitch);
    double s_p = sin(pitch);

    double c_r = cos(roll);
    double s_r = sin(roll);

    Mat3 C;

    C.data[0][0] = c_p * c_y;
    C.data[0][1] = -c_r * c_y * s_p + s_r * s_y;
    C.data[0][2] = s_r * c_y * s_p + c_r * s_y;

    C.data[1][0] = s_p;
    C.data[1][1] = c_r * c_p;
    C.data[1][2] = -s_r * c_p;

    C.data[2][0] = -c_p * s_y;
    C.data[2][1] = c_r * s_y * s_p + s_r * c_y;
    C.data[2][2] = -s_r * s_y * s_p + c_r * c_y;

    return C;
}

#endif