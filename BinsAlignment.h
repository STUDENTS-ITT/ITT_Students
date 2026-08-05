#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

vector<double> Smooth(const vector<double>& data, int window)
{
    vector<double> smoothed(data.size(), 0.0);
    const int n = (int)data.size();

    if (n == 0 || window <= 0)
    {
        return smoothed;
    }

    if (window > n)
    {
        window = n;
    }

    if (window % 2 == 0)
    {
        window++;
    }

    const int half = window / 2;

    vector<double> pref(n + 1, 0.0);
    for (int i = 0; i < n; i++)
    {
        pref[i + 1] = pref[i] + data[i];
    }

    for (int i = 0; i < n; i++)
    {
        const int l = max(0, i - half);
        const int r = min(n - 1, i + half);
        smoothed[i] = (pref[r + 1] - pref[l]) / (r - l + 1);
    }

    return smoothed;
}

double calculateG(double latRad, double h)
{
    double g0 = 9.780327 * (1.0 + 0.0053024 * pow(sin(latRad), 2));
    double scale = 6371000.0 / (6371000.0 + h);
    return g0 * scale * scale;
}

int alignBins()
{
    int n = 1000;
    vector<double> Time, Ax, Ay, Az, Wx, Wy, Wz;
    double time, ax, ay, az, wx, wy, wz;
    double Tnav;
    double Ts = 0.0;
    string line;
    char buffer[512];

    FILE *Nav = fopen("Nav.dat", "r");
    if (!Nav)
        return 1;
    fgets(buffer, sizeof(buffer), Nav);
    fgets(buffer, sizeof(buffer), Nav);

    double latRad = 0.0, heightM = 0.0;
    bool haveNav = false;

    while (fgets(buffer, sizeof(buffer), Nav))
    {
        int parsed = sscanf(buffer, "%lf %*s %*s %lf", &Ts, &Tnav);
        if (parsed == 2)
        {
            haveNav = true;
            if (Tnav == 0.0)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
    fclose(Nav);

    if (!haveNav)
        return 1;

    istringstream ss(buffer);
    vector<double> tokens;
    double v;
    while (ss >> v)
    {
        tokens.push_back(v);
    }
    if (tokens.size() >= 25)
    {
        latRad = tokens[23] * (M_PI / 180.0);
        heightM = tokens[24];
    }
    double g = calculateG(latRad, heightM);

    FILE *IMU = fopen("IMU.txt", "r");
    if (!IMU)
        return 1;

    fgets(buffer, sizeof(buffer), IMU);
    fgets(buffer, sizeof(buffer), IMU);

    size_t i = 0;
    while (fscanf(IMU, "%lf %lf %lf %lf %lf %lf %lf", &time, &ax, &ay, &az, &wx, &wy, &wz) == 7 && time < Ts)
    {
        Time.push_back(time);
        Ax.push_back(ax);
        Ay.push_back(ay);
        Az.push_back(az);
        Wx.push_back(wx);
        Wy.push_back(wy);
        Wz.push_back(wz);
    }
    fclose(IMU);

    if (Time.empty())
        return 1;

    Ax = Smooth(Ax, n);
    Ay = Smooth(Ay, n);
    Az = Smooth(Az, n);
    Wx = Smooth(Wx, n);
    Wy = Smooth(Wy, n);
    Wz = Smooth(Wz, n);

    vector<double> Psi, Gamma, Theta;
    double Psi_avg = 0, Gamma_avg = 0, Theta_avg = 0;
    double sumSinPsi = 0.0, sumCosPsi = 0.0;

    for (i = 0; i < Time.size(); i++)
    {
        Psi.push_back(atan2(-Wz[i], Wx[i]));
        Gamma.push_back(atan2(Az[i], Ay[i]));
        Theta.push_back(asin(max(-1.0, min(1.0, Ax[i] / g))));

        sumSinPsi += sin(Psi[i]);
        sumCosPsi += cos(Psi[i]);
        Gamma_avg += Gamma[i];
        Theta_avg += Theta[i];
    }

    Psi_avg = atan2(sumSinPsi, sumCosPsi);
    Gamma_avg /= Time.size();
    Theta_avg /= Time.size();

    cout << "Psi: " << Psi_avg * 180 / M_PI << "\n";
    cout << "Gamma: " << Gamma_avg * 180 / M_PI << "\n";
    cout << "Theta: " << Theta_avg * 180 / M_PI << "\n";

    FILE *Angles = fopen("Angles.dat", "w");
    if (!Angles)
        return 1;

    fprintf(Angles, "Psi\tGamma\ttheta\n");
    fprintf(Angles, "%.4lf\t%.4lf\t%.4lf\n", Psi_avg, Gamma_avg, Theta_avg);

    fclose(Angles);

    return 0;
}
