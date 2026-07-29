#include <iostream>
#define M_PI 3.1415


auto normolize_angle(double a)
{
    while (a > M_PI)
    {
        a -= 2 * M_PI;
    }
    while (a < -M_PI)
    {
        a += 2 * M_PI;
    }
    return a;
}