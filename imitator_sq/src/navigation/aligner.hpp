#ifndef ALIGNER_H
#define ALIGNER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <iomanip>
#include "../math_lib/constants.hpp"

/**
 * @brief Функция расчёта нормального ускорения свободного падения (Формула Клеро).
 * @param lat_rad Широта места в радианах.
 * @param h Высота над уровнем моря в метрах.
 * @return Значение ускорения свободного падения g с учётом широты и высоты.
 */
inline double calculate_g(double lat_rad, double h)
{
		double sin_lat = sin(lat_rad);	
		double g_0 = G_EQ * (1 + GRAVITY_CONSTANT * sin_lat * sin_lat); // Формула Клеро (g на уровне моря)

		double scale = NAV_R / (NAV_R + h); // Масштабный коэффициент уменьшения g с высотой

		return g_0 * scale * scale;	// Поправка на высоту
}

/**
 * @brief Функция удаления резких выбросов с помощью медианного фильтра (окно из 5 элементов).
 * @[in] input Входной массив данных.
 * @[out] output Выходной отфильтрованный массив.
 * @param n Количество элементов в массиве.
 */
inline void FastMedian(const std::vector<double>& input, std::vector<double>& output, int n)
{
		if (n <= 0) return;
		if (output.size() < static_cast<size_t>(n)) output.resize(n);

		// Для первых двух элементов границы не фильтруем. Границы - копируем
		for (int i = 0; i < 2 && i < n; i++)
		{
				output[i] = input[i];
		}

		// Основная часть
		for (int i = 2; i < n - 2; i++)
		{
				double w[5];
				w[0] = input[i - 2];
				w[1] = input[i - 1];
				w[2] = input[i];
				w[3] = input[i + 1];
				w[4] = input[i + 2];

				// Пузырьковая сортировка для 5 элементов
				for (int a = 0; a < 4; a++)
				{
						for (int b = 0; b < 4 - a; b++)
						{
								if (w[b] > w[b + 1])
								{
										double t = w[b];
										w[b] = w[b + 1];
										w[b + 1] = t;
								}
						}
				}

				output[i] = w[2]; // Медиана - средний из 5
		}

		// Границы
		for (int i = n - 2; i < n && i >= 0; i++)
		{
				output[i] = input[i];
		}
}

/**
 * @brief Расчёт коэффициента сглаживания alpha для фильтра экспоненциального сглаживания (EMA).
 * @param cutoff_freq Частота среза фильтра в Гц.
 * @param sample_rate Частота дискретизации измерений (Гц).
 * @return Коэффициент сглаживания alpha от 0 до 1.
 */
inline double calculate_alpha(double cutoff_freq, double sample_rate)
{
		double dt = 1.0 / sample_rate; // Период дискретизации
		double RC = 1.0 / (2.0 * PI * cutoff_freq); // Постоянная времени RC-фильтра

		return dt / (RC + dt);
}

/**
 * @brief Оценка фактической частоты дискретизации по меткам времени блока.
 * Поток imu.dat неравномерный, поэтому частоту берём из самих данных.
 * @param time_arr Массив меток времени.
 * @param first Индекс первого отсчёта блока.
 * @param count Число отсчётов в блоке.
 * @param fallback Значение, возвращаемое при непригодных метках времени.
 * @return Частота дискретизации в Гц.
 */
inline double estimate_sample_rate(const std::vector<double>& time_arr, size_t first, size_t count, double fallback)
{
		if (count < 2 || first + count > time_arr.size()) return fallback;

		double span = time_arr[first + count - 1] - time_arr[first];
		if (span <= 0.0) return fallback;

		return static_cast<double>(count - 1) / span;
}

/**
 * @brief Функция экспоненциального сглаживания (EMA) с поддержкой блочной обработки.
 * @param input Входной массив.
 * @param output Выходной отфильтрованный массив.
 * @param alpha Коэффициент сглаживания.
 * @param n Размер блока.
 * @param last_val Указатель на последнее сглаженное значение предыдущего блока (для непрерывности).
 * @param is_first_block Флаг: является ли текущий блок первым (1 — да, 0 — нет).
 */
inline void EMA_Filter(const std::vector<double>& input, std::vector<double>& output, double alpha, int n, double* last_val, int is_first_block)
{
		if (n <= 0) return;
		if (output.size() < static_cast<size_t>(n)) output.resize(n);

		int start_idx = 0;

		// Если это первый блок в файле - инициализируем первым значением
		if (is_first_block)
		{
				output[0] = input[0];
				start_idx = 1;
		} 
		
		// Если это 2-ой, 3-й... блок - стартуем с последнего значения предыдущего блока
		else
		{
				output[0] = alpha * input[0] + (1.0 - alpha) * (*last_val);
				start_idx = 1;
		}

		// Фильтрация оставшихся элементов блока
		for (int i = start_idx; i < n; i++)
		{
				output[i] = alpha * input[i] + (1.0 - alpha) * output[i - 1];
		}

		// Сохраняем последнее значение для следующего блока
		*last_val = output[n - 1];
}

/**
 * @brief Расчёт углов ориентации по осреднённым показаниям акселей и ДУС.
 * Тангаж и крен — по проекциям g, курс — гирокомпасированием по проекциям
 * угловой скорости Земли. Курс отсчитывается от севера по часовой стрелке.
 * @param g Ускорение свободного падения в точке стоянки.
 * @[out] yaw Курс в радианах.
 * @[out] pitch Тангаж в радианах.
 * @[out] roll Крен в радианах.
 */
inline void angles_from_means(double Ax_mean, double Ay_mean, double Az_mean,
                              double Wx_mean, double Wy_mean, double Wz_mean,
                              double g, double& yaw, double& pitch, double& roll)
{
		double ax_norm = Ax_mean / g;
		if (ax_norm > 1.0) ax_norm = 1.0;
		if (ax_norm < -1.0) ax_norm = -1.0;

		pitch = asin(ax_norm);
		roll = atan2(-Az_mean, Ay_mean);
		//yaw = atan2(-Wz_mean, Wx_mean);
		// Проекции угловой скорости Земли на горизонт
		double Wn = Wx_mean * cos(pitch) - Wy_mean * sin(pitch) * cos(roll) + Wz_mean * sin(pitch) * sin(roll);
		double We = Wy_mean * sin(roll) + Wz_mean * cos(roll);

		yaw = atan2(-We, Wn);
}

/**
 * @brief Рекурсивное обновление среднего значения.
 * @param current_mean Текущее среднее.
 * @param new_val Новое входящее значение.
 * @param n Общее количество учтенных элементов на данный момент.
 */
inline void update_recursive_mean(double& current_mean, double new_val, long n)
{
		if (n <= 1) 
		{
				current_mean = new_val;	
		} 

		else 
		{
				current_mean += (new_val - current_mean) / static_cast<double>(n);
		}
}

// Объявление главной функции начальной выставки инерциальной навигационной системы.
// ba_x/ba_y/ba_z — смещение акселерометра (можно nullptr, если не нужно).
void get_angle_start(double *Yaw, double *Pitch, double *Roll,
                     double *ba_x, double *ba_y, double *ba_z,
                     const char *IMU_path, double lat_deg, double h, double iter);

#endif