// data_reader.h — Последовательное чтение imu.dat, gps.dat, angle.dat.
//
// ImuReader:
//   Читает построчно, пропускает заголовок. Поддерживает два формата:
//   Старый imu.dat:  time_s  timestamp_ns  wx  wy  wz  ax  ay  az
//   Новый imu.data (dataset_1786981733): время ЧЧ:ММ:СС.дробь,
//   wx  wy  wz  ax  ay  az. Внутри строка приводится к прежнему виду.
//   Формат определяется по первому столбцу строки: ':' — новый.
//
// SnsReader:
//   Читает gps.dat/gps.data (обязательный) и angle.dat/angle.data (опциональный).
//   Старый gps.dat:  time_s  timestamp_ns  latitude  longitude  altitude  vx  vy  vz
//   Новый gps.data:  ЧЧ:ММ:СС.дробь  latitude  longitude  altitude  vx  vy  vz
//   (vx = Vn, vy = Vверх, vz = Ve)
//   Старый angle.dat: time_s  timestamp_ns  roll  pitch  yaw
//   Новый angle.data: ЧЧ:ММ:СС.дробь  x y z quat vx vy vz  roll  pitch  yaw
//   Координаты gps переводятся из градусов в радианы.
//   Углы angle трактуются как радианы (внутренняя СК — рад), стыкуются с gps по времени.
//   Если angle.dat/angle.data отсутствует — углы ориентации заполняются нулями.

#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "navigation/gps_processor.h"

namespace data_io
{

// Разбор строки на числа (разделитель — пробел/табуляция).
std::vector<double> &parseLine(const std::string &line, std::vector<double> &items);

// Разбор строки на строки (для SnsReader с ручным преобразованием типов).
std::vector<std::string> splitLine(const std::string &line);

// Разбор времени: "ЧЧ:ММ:СС.дробь" (например 00:01:01.500000000) или
// обычное число с плавающей точкой. Возвращает true при успехе.
bool parseTimestamp(const std::string &token, double &seconds);

// Чтение imu.dat: построчно, заголовок пропускается при open().
class ImuReader
{
public:
    bool open(const std::string &path);
    bool next(std::vector<double> &row);
    void close();

private:
    std::ifstream file_;
    std::string line_;
};

// Чтение gps.dat + angle.dat (опциональный): формирует SnsSample.
// Координаты переводятся в радианы.
// angle.dat стыкуется с gps по ближайшему времени (не по номеру строки).
// Если angle.dat не задан — углы orientation = 0.
class SnsReader
{
public:
    bool open(const std::string &gps_path, const std::string &angle_path = "");
    bool next(nav::SnsSample &out);
    void close();

private:
    bool readNextAngle();

    std::ifstream gps_;
    std::ifstream angle_;
    std::string gps_line_;
    std::string angle_line_;
    bool has_angle_ = false;

    bool have_ang_prev_ = false;
    bool have_ang_curr_ = false;
    double ang_t_prev_ = 0.0;
    double ang_t_curr_ = 0.0;
    double ang_roll_prev_ = 0.0;
    double ang_pitch_prev_ = 0.0;
    double ang_yaw_prev_ = 0.0;
    double ang_roll_curr_ = 0.0;
    double ang_pitch_curr_ = 0.0;
    double ang_yaw_curr_ = 0.0;
};

} // namespace data_io
