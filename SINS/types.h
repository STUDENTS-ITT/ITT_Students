#ifndef TYPES_H
#define TYPES_H

#include <cmath>

const double GM_EARTH = 3.986004418e14;
const double R_EQ = 6378137.0;
const double U_EARTH = 7.292115e-5;
const double PI = 3.14159265358979323846;

struct Vec3 { double x, y, z; };
struct Mat3 { double data[3][3]; };

struct NavState {
    double T_sys, T_reg, T_rem, T_nav;
    Vec3 pos;       // {Lat, Lon, Alt} in radians/meters
    Vec3 vel;       // {North, Up, East} in m/s
    Vec3 euler;     // {Yaw, Pitch, Roll} in radians
    Mat3 C;         // Direction Cosine Matrix
    Vec3 accel;     // {Ax, Ay, Az} in m/s^2
    Vec3 gyro;      // {Wx, Wy, Wz} in rad/s
};

struct IMU_Record {
    double Time;
    double Ax, Ay, Az;
    double Wx, Wy, Wz;
};

struct Nav_Record {
    double T_sys, T_reg, T_rem, T_nav;
    double Lon, Lat, Alt;
    double alignment_end_T;  // T_sys на границе перехода выставка→навигация
};

#endif // TYPES_H