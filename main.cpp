#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
//#define _USE_MATH_DEFINES
#define M_PI 3.14159265358979323846
#define U_Earth 7.292115e-5
#define R 6371e3
#include <cmath>
#include <iomanip>
#include <array>
#include <algorithm>
#include <chrono>
using namespace std;

using Matrix = vector<double>;
using Vector = vector<double>;

// работа со строками
vector<double> parseLine(const string &line, vector<double> &items)
{
    items.clear();
    stringstream ss(line);
    string item;
    while (ss >> item)
    {
        items.push_back(stod(item));
    }
    return items;
}

// среднее арифметическое
double Average(const vector<double> &angle)
{
    double sum = 0;
    int begin_sec = angle.size();
    for (size_t k = 0; k < begin_sec; k++)
    {
        sum += angle[k];
    }
    double deg = 180 / M_PI;
    double beggin_angle = (sum / begin_sec) * (deg);
    return beggin_angle;
}

// доступ к элементам матрицы версия для записи
inline double &at(Matrix &A, int i, int j, int cols)
{
    return A[i * cols + j];
}

// версия для чтения
inline const double &at(const Matrix &A, int i, int j, int cols)
{
    return A[i * cols + j];
}

// получение размера матриц
inline int getRows(const Matrix &A, int cols)
{
    return A.size() / cols;
}
// умножение матрицы на вектор
Vector multiply_m(const Matrix &A, const Vector &v, int cols_A)
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

// умножение матрицы
Matrix multiply_matrix(const Matrix &A, const Matrix &B, int cols_A, int cols_B)
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

// векторное произведение векторов 3*3
Vector vector_product(const Vector &a, const Vector &b)
{
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]};
}

// разность векторов
Vector vector_diff(const Vector &a, const Vector &b)
{
    if (a.size() != b.size())
    {
        cerr << "err size vec" << endl;
    }
    Vector res(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        res[i] = a[i] - b[i];
    }
    return res;
}

// сумма векторов
Vector vector_sum(const Vector &a, const Vector &b)
{
    if (a.size() != b.size())
    {
        cerr << "err size vec" << endl;
    }
    Vector res(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        res[i] = a[i] + b[i];
    }
    return res;
}

// интеграл
double v_integral(double v_now_dot, double V0, double v_pred_dot, double dt)
{
    return V0 + ((v_now_dot + v_pred_dot) / 2) * dt;
}

// разделение на строки
vector<string> splitLine(const string &line)
{
    vector<string> elements;
    stringstream ss(line);
    string element;
    while (ss >> element)
    {
        elements.push_back(element);
    }
    return elements;
}

// транспонированная матрица
Matrix transpose_m(const Matrix &A, int cols)
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
Matrix matrux_sum(const Matrix &A, const Matrix &B, int cols)
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

// разница матриц
Matrix matrux_diff(const Matrix &A, const Matrix &B, int cols)
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

// обратная матрица
Matrix return_matrix(const Matrix &A, int cols)
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
                swap(at(M, i, j, 2 * n), at(M, max_row, j, 2 * n));
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

Matrix E_matrix(size_t n)
{
    Matrix E(n * n, 0);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            at(E, i, j, n) = (i == j) ? 1.0 : 0.0;
        }
    }
    return E;
}

Matrix H_matrix(size_t m, size_t n)
{
    Matrix H(m * n, 0);
    for (int i = 0; i < m; i++)
    {
        at(H, i, i, n) = 1;
    }
    return H;
}

// Дискретный Q: q = σ² * T (учебник §5.4, дискретизация белого шума)
// Паспорт ИМУ: σ_g, σ_bg, σ_a, σ_ba  [(ед.)/√Гц]
Matrix Qj_matrix(double T)
{
    const double sig_g = 3.394e-4;
    const double sig_bg = 3.8785e-5;
    const double sig_a = 4e-3;
    const double sig_ba = 2e-4;

    const double q_v = sig_a * sig_a * T;
    const double q_att = sig_g * sig_g * T;
    const double q_ba = sig_ba * sig_ba * T;
    const double q_bg = sig_bg * sig_bg * T;

    Matrix Qj(15 * 15, 0);
    for (int i = 0; i < 3; i++)
        at(Qj, i, i, 15) = 0.0; // φ, λ, h
    for (int i = 3; i < 6; i++)
        at(Qj, i, i, 15) = q_v; // V
    for (int i = 6; i < 9; i++)
        at(Qj, i, i, 15) = q_att; // ψ, θ, γ
    for (int i = 9; i < 12; i++)
        at(Qj, i, i, 15) = q_ba; // ba
    for (int i = 12; i < 15; i++)
        at(Qj, i, i, 15) = q_bg; // bg
    return Qj;
}

// нормирование углов от -pi до pi
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

// Дискретный ФК (§5.4): x̂⁻=Φx̂, P⁻=ΦPΦᵀ+Q, K=P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹,
// x̂⁺=x̂⁻+K(z−Hx̂⁻), P⁺=(I−KH)P⁻
// Состояние: [δφ,δλ,δh, δVn,δVh,δVe, δψ,δθ,δγ, ba_x,ba_y,ba_z, bg_x,bg_y,bg_z]
void kalman(const Vector &bins, const Vector &sns, double lat, Vector &x0, Matrix &P0, const Matrix &C)
{
    const double g = 9.81;

    const double T = 1.0 / 197.0;

    Vector zj = vector_diff(bins, sns);
    
    zj[6] = normolize_angle(zj[6]);
    zj[7] = normolize_angle(zj[7]);
    zj[8] = normolize_angle(zj[8]);

    Vector uj(15, 0.0);
    Matrix E = E_matrix(15);
    Matrix Hj1 = H_matrix(9, 15);
    Matrix Qj = Qj_matrix(T);

    // R: φ,λ в радианах → σ_поз≈5 м; h,V,углы — в своих единицах
    const double sig_pos = 10.0 / R;          // рад
    const double sig_h = 200.0;                // м
    const double sig_v = 100;                // м/с
    const double sig_ang = 10.0 * M_PI / 180; // рад (~1°)
    Matrix Rj1(9 * 9, 0);
    double rdiag[9] = {
        sig_pos * sig_pos, sig_pos * sig_pos, sig_h * sig_h,
        sig_v * sig_v, sig_v * sig_v, sig_v * sig_v,
        sig_ang * sig_ang, sig_ang * sig_ang, sig_ang * sig_ang};
    for (int i = 0; i < 9; i++)
        at(Rj1, i, i, 9) = rdiag[i];

    // Φ ≈ I + A*T (модель ошибок БИНС для ЛА)
    Matrix Fj(15 * 15, 0);
    for (int i = 0; i < 15; i++)
        at(Fj, i, i, 15) = 1.0;

    // φ̇=Vn/R, λ̇=Ve/(R cos φ), ḣ=Vh
    at(Fj, 0, 3, 15) = T / R;
    at(Fj, 1, 5, 15) = T / (R * cos(lat));
    at(Fj, 2, 4, 15) = T;

    // δV̇_N ≈ +g δθ, δV̇_E ≈ −g δγ  (горизонтальные каналы)
    at(Fj, 3, 7, 15) = T * g;
    at(Fj, 5, 8, 15) = -T * g;

    // смещения акс. в связанной СК → навигационная через C (тело→геогр.)
    for (int j = 0; j < 3; j++)
    {
        at(Fj, 3, 9 + j, 15) = T * at(C, 0, j, 3); // Vn
        at(Fj, 4, 9 + j, 15) = T * at(C, 1, j, 3); // Vh
        at(Fj, 5, 9 + j, 15) = T * at(C, 2, j, 3); // Ve
    }

    // δψ̇≈bg_y..., упрощённо: углы ← смещения гиро (связанная СК)
    at(Fj, 6, 14, 15) = T; // ψ ← bg_z (верт. канал, приближённо)
    at(Fj, 7, 12, 15) = T; // θ ← bg_x
    at(Fj, 8, 13, 15) = T; // γ ← bg_y

    Matrix FTj = transpose_m(Fj, 15);
    Matrix HTj1 = transpose_m(Hj1, 15);
    Vector xj1 = vector_sum(multiply_m(Fj, x0, 15), uj);
    Matrix Pj1 = matrux_sum(multiply_matrix(multiply_matrix(Fj, P0, 15, 15), FTj, 15, 15), Qj, 15);

    Matrix Pj1_HTj1 = multiply_matrix(Pj1, HTj1, 15, 9);
    Matrix S = matrux_sum(multiply_matrix(multiply_matrix(Hj1, Pj1, 15, 15), HTj1, 15, 9), Rj1, 9);
    Matrix Kj1 = multiply_matrix(Pj1_HTj1, return_matrix(S, 9), 9, 9);

    Vector innov = vector_diff(zj, multiply_m(Hj1, xj1, 15));
    innov[6] = normolize_angle(innov[6]);
    innov[7] = normolize_angle(innov[7]);
    innov[8] = normolize_angle(innov[8]);
    x0 = vector_sum(xj1, multiply_m(Kj1, innov, 9));

    Matrix E_KH = matrux_diff(E, multiply_matrix(Kj1, Hj1, 9, 15), 15);
    P0 = multiply_matrix(E_KH, Pj1, 15, 15);
}

void navigation(
    int i,
    const vector<double> &row,
    const vector<double> &Lat_sp,
    const vector<double> &Lon_sp,
    const vector<double> &Alt_sp,
    const vector<double> &Vn_sp,
    const vector<double> &Vh_sp,
    const vector<double> &Ve_sp,
    const vector<double> &Hdg_sp,
    const vector<double> &Pitch_sp,
    const vector<double> &Roll_sp,
    double &lat, double &lon, double &h0,
    double &Heading0, double &Pitch0, double &Roll0,
    Vector &V0, Vector &V_pred_dot,
    double &Lat_Pred_dot, double &Lon_Pred_dot, double &h_Pred_dot,
    double &Heading_dot_pred, double &Pitch_dot_pred, double &Roll_dot_pred,
    Vector &x0, Matrix &P0,
    ofstream &outFile, ofstream &out2file, double deg_rad, double &time_pred)
{
    double time_s = row[0];
    double dt = time_s - time_pred;
    if (dt <= 0.0)
        dt = 1.0/197.0;
    Matrix C(3 * 3, 0);
    double c_vals[3][3] =
        {
            {cos(Pitch0) * cos(Heading0), -cos(Roll0) * cos(Heading0) * sin(Pitch0) + sin(Roll0) * sin(Heading0), sin(Roll0) * cos(Heading0) * sin(Pitch0) + cos(Roll0) * sin(Heading0)},
            {sin(Pitch0), cos(Roll0) * cos(Pitch0), -sin(Roll0) * cos(Pitch0)},
            {-cos(Pitch0) * sin(Heading0), cos(Roll0) * sin(Heading0) * sin(Pitch0) + sin(Roll0) * cos(Heading0), -sin(Roll0) * sin(Heading0) * sin(Pitch0) + cos(Roll0) * cos(Heading0)}};

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            at(C, r, c, 3) = c_vals[r][c];
        }
    }

    // imu.dat уже в связанной СК ГОСТ: X вперёд, Y вверх, Z вправо (ay ≈ +g)
    Vector n = {row[5] - x0[9], row[6] - x0[10], row[7] - x0[11]};
    Vector result = multiply_m(C, n, 3);
    double n_N = result[0];
    double n_H = result[1];
    double n_E = result[2];

    Vector U2 = {2 * U_Earth * cos(lat), 2 * U_Earth * sin(lat), 0};

    Vector Ac = vector_product(U2, V0);
    Vector g = {0, 9.81, 0};
    //  ω' = λ̇ cosφ · i + λ̇ sinφ · j − φ̇ · k
    Vector w_shtr = {Lon_Pred_dot * cos(lat), Lon_Pred_dot * sin(lat), -Lat_Pred_dot};
    // вредные ускорения a^k без g: Кориолис + ω'×V (3.15); g отдельно
    Vector A_harm = vector_sum(Ac, vector_product(w_shtr, V0));
    Vector Ak = vector_sum(A_harm, g);
    Vector n_r = {n_N, n_H, n_E};
    // (3.16): V̇ = n − a^k
    Vector V_dot = vector_diff(n_r, Ak);

    double V_N_dot = V_dot[0];
    double V_H_dot = V_dot[1];
    double V_E_dot = V_dot[2];

    double V_N = v_integral(V_N_dot, V0[0], V_pred_dot[0], dt);
    double V_H = v_integral(V_H_dot, V0[1], V_pred_dot[1], dt);
    double V_E = v_integral(V_E_dot, V0[2], V_pred_dot[2], dt);

    double Rh = R + h0;
    double Lat_dot = V_N / Rh;                        
    double Lat = v_integral(Lat_dot, lat, Lat_Pred_dot, dt);
    Lat = normolize_angle(Lat);

    double Lon_dot = V_E / (Rh * cos(Lat));             
    double Lon = v_integral(Lon_dot, lon, Lon_Pred_dot, dt);
    Lon = normolize_angle(Lon);

    double h = v_integral(V_H, h0, h_Pred_dot, dt);

    Matrix CT = transpose_m(C, 3);
    // абсолютная угловая скорость геогр. трёхгранника 
    double W_N = U_Earth * cos(lat) + V_E / Rh;
    double W_H = U_Earth * sin(lat) + (V_E * tan(lat)) / Rh;
    double W_E = -V_N / Rh;
    Vector w = {W_N, W_H, W_E};
    Vector w_per = multiply_m(CT, w, 3);
    Vector w_abs = {row[2] - x0[12], row[3] - x0[13], row[4] - x0[14]};
    Vector w_otn = vector_diff(w_abs, w_per);

    double Wx_otn = w_otn[0];
    double Wy_otn = w_otn[1];
    double Wz_otn = w_otn[2];

    // кинематические ур-я Эйлера–Крылова (3.30); ψ — угол рыскания (против ЧС от Севера)
    double Heading_dot = (Wy_otn * cos(Roll0) - Wz_otn * sin(Roll0)) / cos(Pitch0);
    double Pitch_dot = Wy_otn * sin(Roll0) + Wz_otn * cos(Roll0);
    double Roll_dot = Wx_otn - tan(Pitch0) * (Wy_otn * cos(Roll0) - Wz_otn * sin(Roll0));

    double Heading = v_integral(Heading_dot, Heading0, Heading_dot_pred, dt);
    double Pitch = v_integral(Pitch_dot, Pitch0, Pitch_dot_pred, dt);
    double Roll = v_integral(Roll_dot, Roll0, Roll_dot_pred, dt);
    Heading = normolize_angle(Heading);
    Pitch = normolize_angle(Pitch);
    Roll = normolize_angle(Roll);

    if (i < Lat_sp.size() && i < Hdg_sp.size())
    {
        if (i % 197 == 0)
        {
            Vector bins1 = {Lat, Lon, h, V_N, V_H, V_E, Heading, Pitch, Roll};
            //double hdg_ist=atan2(-w_abs[2],w_abs[0]);
            double hdg_ist=acos((row[2]-U_Earth*sin(Pitch)*sin(Lat))/(U_Earth*cos(Lat)*cos(Pitch)));
            hdg_ist=normolize_angle(hdg_ist);
            // angle.dat: yaw в той же СК, что ψ учебника (против ЧС от N).
            
            Vector sns = {Lat_sp[i], Lon_sp[i], Alt_sp[i], Vn_sp[i], Vh_sp[i], Ve_sp[i],
                          Hdg_sp[i], Pitch_sp[i], Roll_sp[i]};

            kalman(bins1, sns, Lat, x0, P0, C);
            // out2file << row[0] << "         " << x0[6] * 180 / M_PI << endl;

            //  коррекция навигации
            V0 = {V_N - x0[3], V_H - x0[4], V_E - x0[5]};
            lat = Lat - x0[0];
            lon = Lon - x0[1];
            h0 = h - x0[2];
            Heading0 = normolize_angle(Heading - x0[6]);
            Pitch0 = normolize_angle(Pitch - x0[7]);
            Roll0 = normolize_angle(Roll - x0[8]);

            Lat_Pred_dot = 0;
            Lon_Pred_dot = 0;
            V_pred_dot = {0, 0, 0};
            Heading_dot_pred = 0;
            Pitch_dot_pred = 0;
            Roll_dot_pred = 0;
            h_Pred_dot = 0;
            out2file << left << setw(15) << row[0] << setw(15) << x0[0] << setw(15) << x0[1] << setw(15)
                    << x0[2] << setw(15) << x0[3] << setw(15) << x0[4]
                    << setw(15) << x0[5] << setw(15) << x0[6] << setw(15) << x0[7] << setw(15) << x0[8] << setw(15)
                    << x0[9] << setw(15) << x0[10] << setw(15) <<x0[11]  
                    << setw(15) <<x0[12] << setw(15) << x0[13] << setw(15) << x0[14] << endl;
            // в файл — УЖЕ скорректированные углы/координаты (иначе график heading «ломаный»)
            outFile << left << setw(15) << row[0] << setw(15) << lon * 180 / M_PI << setw(15) << lat * 180 / M_PI << setw(15)
                    << Heading0 * 180 / M_PI << setw(15) << Pitch0 * 180 / M_PI << setw(15) << Roll0 * 180 / M_PI
                    << setw(15) << V0[0] << setw(15) << V0[1] << setw(15) << V0[2] << setw(15) << h0 << setw(15)
                    << Lon_sp[i] * 180 / M_PI << setw(15) << Lat_sp[i] * 180 / M_PI << setw(15) <<Alt_sp[i]  
                    << setw(15) <<Hdg_sp[i] * 180 / M_PI << setw(15) << Roll_sp[i] * 180 / M_PI << setw(15) << Pitch_sp[i] * 180 / M_PI << setw(15) 
                    <<Vn_sp[i] << setw(15) << Vh_sp[i]<< setw(15) << Ve_sp[i]  <<setw(15)<<hdg_ist*180/M_PI<<setw(15)<<bins1[0]* 180 / M_PI<< endl;

            // сброс ошибок pos/vel/att; смещения ba/bg СОХРАНЯЕМ (иначе курс уплывает)
            for (int k = 0; k < 9; k++)
                x0[k] = 0.0;
        }
        else
        {
            V0 = {V_N, V_H, V_E};
            lat = Lat;
            lon = Lon;
            h0 = h;
            Heading0 = Heading;
            Pitch0 = Pitch;
            Roll0 = Roll;
            V_pred_dot = {V_N_dot, V_H_dot, V_E_dot};
            Lat_Pred_dot = Lat_dot;
            Lon_Pred_dot = Lon_dot;
            Heading_dot_pred = Heading_dot;
            Pitch_dot_pred = Pitch_dot;
            Roll_dot_pred = Roll_dot;
            h_Pred_dot = V_H;
        }

        time_pred = time_s;
    }
}
int main()
{
    // чтение из файлов
    ifstream file("imu.dat");
    if (!file.is_open())
    {
        cerr << "error open" << endl;
        return 1;
    }
    vector<double> row;
    string line;
    

    // чтение из файлов
    // ifstream Gfile("GraphData.DAT");
    // if (!Gfile.is_open())
    // {
    //     cerr << "error open" << endl;
    //     return 1;
    // }
    // vector<string> lines;
    // string Gline;
    // while (getline(Gfile, Gline))
    // {
    //     lines.push_back(Gline);
    // }
    // Gfile.close();
    // // обрабатываем столбцы из файла
    // vector<double> times;
    // vector<double> Heading_0;
    // vector<double> Pitch_0;
    // vector<double> Roll_0;
    // // vector<double> Wx;
    // // vector<double> Wy;
    // // vector<double> Wz;
     double deg_rad = M_PI / 180;
    // for (size_t k = 2; k < lines.size(); ++k)
    // {
    //     vector<string> elements = splitLine(lines[k]);
    //     if (!elements.empty() && elements.size() >= 7)
    //     {
    //         times.push_back(stod(elements[0]));
    //         Heading_0.push_back(stod(elements[7]));
    //         Pitch_0.push_back(stod(elements[9]));
    //         Roll_0.push_back(stod(elements[8]));
    //     }
    // }

    ifstream Afile("angle.dat");
    if (!Afile.is_open())
    {
        cerr << "error open" << endl;
        return 1;
    }
    vector<string> vals;
    string val;
    while (getline(Afile, val))
    {
        vals.push_back(val);
    }
    Afile.close();

    // vector<double> Lon_sp;
    // vector<double> Alt_sp;
    // vector<double> Vn_sp;
    // vector<double> Vh_sp;
    // vector<double> Ve_sp;
    vector<double> time_angle;
    //vector<double> Lat_sp;
    vector<double> Hdg_sp;
    vector<double> Roll_sp;
    vector<double> Pitch_sp;
    for (size_t k = 2; k < vals.size(); ++k)
    {
        vector<string> elements2 = splitLine(vals[k]);
        if (!elements2.empty())
        {
            time_angle.push_back(stod(elements2[0]));
            // Lon_sp.push_back((stod(elements1[3])) * deg_rad);
            // Lat_sp.push_back((stod(elements1[2])) * deg_rad);
            // Alt_sp.push_back(stod(elements1[4]));
            // angle.dat yaw = авиационный курс (по ЧС от N).
            // учебник (3.1): ψ рыскания — против ЧС от N ⇒ ψ = −yaw
            Hdg_sp.push_back(-stod(elements2[4]));
            Roll_sp.push_back(stod(elements2[2]));
            Pitch_sp.push_back(stod(elements2[3]));
            // Vn_sp.push_back(stod(elements1[5]));
            // Vh_sp.push_back(stod(elements1[6]));
            // Ve_sp.push_back(stod(elements1[7]));
        }
    }



    ifstream Nfile("gps.dat");
    if (!Nfile.is_open())
    {
        cerr << "error open" << endl;
        return 1;
    }
    vector<string> values;
    string value;
    while (getline(Nfile, value))
    {
        values.push_back(value);
    }
    Nfile.close();

    vector<double> Lon_sp;
    vector<double> Alt_sp;
    vector<double> Vn_sp;
    vector<double> Vh_sp;
    vector<double> Ve_sp;
    vector<double> time;
    vector<double> Lat_sp;
    // vector<double> Hdg_sp;
    // vector<double> Roll_sp;
    // vector<double> Pitch_sp;
    for (size_t k = 2; k < values.size(); ++k)
    {
        vector<string> elements1 = splitLine(values[k]);
        if (!elements1.empty())
        {
            time.push_back(stod(elements1[0]));
            Lon_sp.push_back((stod(elements1[3])) * deg_rad);
            Lat_sp.push_back((stod(elements1[2])) * deg_rad);
            Alt_sp.push_back(stod(elements1[4]));
            // Hdg_sp.push_back(0 * deg_rad);
            // Roll_sp.push_back( 0* deg_rad);
            // Pitch_sp.push_back(0 * deg_rad);
            Vn_sp.push_back(stod(elements1[5]));
            Vh_sp.push_back(stod(elements1[6]));
            Ve_sp.push_back(stod(elements1[7]));
        }
    }
    // удлиняем вектор
    // vector<double> Lon_sp;
    // vector<double> Alt_sp;
    // vector<double> Vn_sp;
    // vector<double> Vh_sp;
    // vector<double> Ve_sp;
    // vector<double> time;
    // vector<double> Lat_sp;
    // vector<double> Hdg_sp;
    // vector<double> Roll_sp;
    // vector<double> Pitch_sp;
    // vector<double> Hdg_err;

    // for (int i = 0; i < time1.size(); i++)
    // {
    //     for (int j = 0; j < 400; j++)
    //     {
    //         time.push_back(time1[i]);
    //         Lon_sp.push_back(Lon_sp1[i]);
    //         Lat_sp.push_back(Lat_sp1[i]);
    //         Alt_sp.push_back(h_sp1[i]);
    //         Hdg_sp.push_back(Hdg_sp1[i]);
    //         Roll_sp.push_back(Roll_sp1[i]);
    //         Vn_sp.push_back(Vn_sp1[i]);
    //         Vh_sp.push_back(Vh_sp1[i]);
    //         Ve_sp.push_back(Ve_sp1[i]);
    //         Pitch_sp.push_back(Pitch_sp1[i]);
    //     }
    // }

    auto start = chrono::high_resolution_clock::now();
    // начальные углы из angle.dat (рад): ψ учебника = yaw файла
    double Heading0 = Hdg_sp.empty() ? 0.0 : Hdg_sp[0];
    double Roll0 = Roll_sp.empty() ? 0.0 : Roll_sp[0];
    double Pitch0 = Pitch_sp.empty() ? 0.0 : Pitch_sp[0];
    cout << Heading0 << "        " << Roll0 << "        " << Pitch0 << endl;
    
    double lat = Lat_sp.empty() ? 0.0 : Lat_sp[0];
    double Lat_Pred_dot = 0;
    double lon =Lon_sp.empty() ? 0.0 : Lon_sp[0];
    double Lon_Pred_dot = 0;
    double h0 = Alt_sp.empty() ? 0.0 : Alt_sp[0];
    double h_Pred_dot = 0;
    double vn0 = Vn_sp.empty() ? 0.0 : Vn_sp[0];
    double vh0 = Vh_sp.empty() ? 0.0 : Vh_sp[0];
    double ve0 = Alt_sp.empty() ? 0.0 : Ve_sp[0];
    Vector V0 = {vn0, vh0, ve0};
    Vector V_pred_dot = {0, 0, 0};
    double Heading_dot_pred = 0;
    double Pitch_dot_pred = 0;
    double Roll_dot_pred = 0;
    double time_pred = 0;

    Vector x0(15, 0);

    // P0: φ,λ в рад²; ba/bg из σ_stab² = (1e-4)²
    Matrix P0(15 * 15, 0);
    const double p_pos = (50.0 / R) * (50.0 / R); // ~50 м
    const double p_h = 10.0;                      // м²
    const double p_v = 2.5;                       // (м/с)²
    const double p_att = (10.0 * deg_rad) * (10.0 * deg_rad);
    const double p_ba = 1e-8;
    const double p_bg = 1e-8;
    double pdiag[15] = {
        p_pos, p_pos, p_h,
        p_v, p_v, p_v,
        p_att, p_att, p_att,
        p_ba, p_ba, p_ba,
        p_bg, p_bg, p_bg};
    for (int i = 0; i < 15; i++)
        at(P0, i, i, 15) = pdiag[i];
   
    ofstream outFile("kalman15_line2.txt");
    ofstream out2file("d_1.txt");

     
    

    if (!outFile.is_open() || !out2file.is_open())
    {
        cerr << "error open" << endl;
        return 1;
    }
    outFile << left << setw(15) << "time" << setw(15) << "lon" << setw(15) << "lat " << setw(15)
                    << "Heading" << setw(15) <<" Pitch" << setw(15) <<" Roll"
                    << setw(15) << "V0[0]" << setw(15) << "V0[1]" << setw(15) << "V0[2]" << setw(15) << "h0" << setw(15)
                    << "Lon_sp" << setw(15) << "Lat_sp "<< setw(15) <<"Alt_sp " 
                    << setw(15) <<"Hdg_sp" << setw(15) <<" Roll_sp" << setw(15) << "Pitch_sp" << setw(15) 
                    <<"Vn_sp" << setw(15) << "Vh_sp"<< setw(15) << "Ve_sp " << endl;
    out2file << "hdg_err" << endl;

    const int TIME=180*197;
    int lineNumber = 0;

    double lat0=lat;
    double lon0=lon;
    double alt0=h0;
    double hdg0=Heading0;
    double pitch0=Pitch0;
    double roll0=Roll0;
    Vector v0=V0;

    file.clear();
    file.seekg(0, ios::beg);
    getline(file, line);
    int i = 0;
    while (getline(file, line) && i<TIME)
    {
        parseLine(line, row);
        if (row.size() < 8) continue;
        if (i % 197 == 0){
        outFile << left << setw(15) << row[0] << setw(15) << lon0 * 180 / M_PI << setw(15) << lat0 * 180 / M_PI << setw(15)
                    << hdg0 * 180 / M_PI << setw(15) << pitch0 * 180 / M_PI << setw(15) << roll0 * 180 / M_PI
                    << setw(15) << V0[0] << setw(15) << V0[1] << setw(15) << V0[2] << setw(15) << alt0 << setw(15)
                    << Lon_sp[0] * 180 / M_PI << setw(15) << Lat_sp[0] * 180 / M_PI << setw(15) <<Alt_sp[0]  
                    << setw(15) <<Hdg_sp[0] * 180 / M_PI << setw(15) << Roll_sp[0] * 180 / M_PI << setw(15) << Pitch_sp[0] * 180 / M_PI << setw(15) 
                    <<Vn_sp[0] << setw(15) << Vh_sp[0]<< setw(15) << Ve_sp[0]  <<setw(15)<<hdg0*180/M_PI<<setw(15) << lat0 * 180 / M_PI << endl;
        }
    time_pred=row[0];
        i++;
    }
    while (getline(file, line))
    {
        lineNumber++;
        parseLine(line, row);
        if (row.size() < 8)
            continue;

        navigation(
            i, row, Lat_sp, Lon_sp, Alt_sp, Vn_sp, Vh_sp, Ve_sp, Hdg_sp, Pitch_sp, Roll_sp,
            lat, lon, h0, Heading0, Pitch0, Roll0, V0, V_pred_dot,
            Lat_Pred_dot, Lon_Pred_dot, h_Pred_dot, Heading_dot_pred, Pitch_dot_pred, Roll_dot_pred,
            x0, P0, outFile, out2file, deg_rad, time_pred);

        i++;
    }
    file.close();
    outFile.close();
    out2file.close();
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = end - start;
    cout << elapsed.count() << endl;

    return 0;
}

