#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <string>
#include <sstream>
#include <iomanip>

using namespace std;

#define PI 3.141592653589793

struct ImuData {
    double time;
    double ax, ay, az;
    double wx, wy, wz;
    double pitch;
    double roll;
    double yaw;

    ImuData() { clear(); }

    void clear() {
        time = 0.0;
        ax = ay = az = 0.0;
        wx = wy = wz = 0.0;
        pitch = roll = yaw = 0.0;
    }

    bool parseFromString(const string& line) {
        istringstream iss(line);
        double t, a_x, a_y, a_z, w_x, w_y, w_z;
        if (iss >> t >> a_x >> a_y >> a_z >> w_x >> w_y >> w_z) {
            time = t; ax = a_x; ay = a_y; az = a_z;
            wx = w_x; wy = w_y; wz = w_z;
            return true;
        }
        return false;
    }
};

#endif
