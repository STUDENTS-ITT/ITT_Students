#include "aligner.hpp"

using namespace std;

// Размер буфера для накопления и обработки данных поворотного блока
constexpr int BUF_rot = 3000;

/**
 * @brief Главная функция расчёта начальных углов ориентации (Курс, Тангаж, Крен).
 * @param Yaw Указатель на результирующий курс (рыскание).
 * @param Pitch Указатель на результирующий тангаж.
 * @param Roll Указатель на результирующий крен.
 * @param IMU_path Путь к файлу imu.dat.
 * @param StartupNav_path Путь к конфигурационному файлу StartupNav.ini.
 */
void get_angle_start(double* Yaw, double* Pitch, double* Roll, const char* IMU_path, const char* StartupNav_path)
{
		// Чтение параметров из StartupNav.ini
		ifstream file_startup(StartupNav_path);
		if (!file_startup.is_open())
		{
				cerr << "Error opening StartupNav file: " << StartupNav_path << "\n";
				return;
		}

		string buffer;

		// Строка 1: Долгота (град) — пропускаем
		getline(file_startup, buffer);

		// Строка 2: Широта (град)
		double lat_deg = 0.0;
		if (getline(file_startup, buffer))
		{
				sscanf(buffer.c_str(), "%lf", &lat_deg);
		}

		// Строка 3: Высота (м)
		double h = 0.0;
		if (getline(file_startup, buffer))
		{
				sscanf(buffer.c_str(), "%lf", &h);
		}

		// Строка 4: Курс выставки (град)
		double heading_deg = 0.0;
		if (getline(file_startup, buffer))
		{
				sscanf(buffer.c_str(), "%lf", &heading_deg);
		}

		// Строка 5: Запись выходных параметров — пропускаем
		getline(file_startup, buffer);

		// Строка 6: Время выставки (сек)
		double iter = 0.0;
		if (getline(file_startup, buffer))
		{
				sscanf(buffer.c_str(), "%lf", &iter);
		}
		file_startup.close();

		// Формула Клеро для расчёта g
		double g = calculate_g(lat_deg * DEG_TO_RAD, h);

		// Открытие файла IMU
		ifstream file_imu(IMU_path);
		if (!file_imu.is_open())
		{
				cerr << "Error opening IMU file!\n";
				return;
		}

		// Пропуск строки заголовка
		if (!getline(file_imu, buffer))
		{
				cerr << "Error reading IMU file!\n";
				return;
		}

		// Создание файла для записи промежуточных и отфильтрованных параметров
		ofstream graphfile("../data/processed/Aligner.dat");
		if (!graphfile.is_open())
		{
				cerr << "Error opening output graph file!\n";
				return;
		}

		graphfile << left 
							<< setw(10) << "Time" 
							<< setw(12) << "Ax" << setw(12) << "Ay" << setw(12) << "Az" 
							<< setw(12) << "Wx" << setw(12) << "Wy" << setw(12) << "Wz" 
							<< setw(12) << "Yaw_Smooth" << setw(12) << "Roll_Smooth" << setw(12) << "Pitch_Smooth" 
							<< setw(12) << "Ax_Smooth" << setw(12) << "Ay_Smooth" << setw(12) << "Az_Smooth" 
							<< setw(12) << "Wx_Smooth" << setw(12) << "Wy_Smooth" << setw(12) << "Wz_Smooth" 
							<< setw(12) << "Yaw" << setw(12) << "Roll" << setw(12) << "Pitch" << "\n"
							<< "------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";

		// Динамические массивы на базе vector
		vector<double> time_arr;
		vector<double> Ax_arr, Ay_arr, Az_arr;
		vector<double> Wx_arr, Wy_arr, Wz_arr;

		// Буферы фиксированного размера для блочной фильтрации
		vector<double> Ax_blc(BUF_rot), Ay_blc(BUF_rot), Az_blc(BUF_rot);
		vector<double> Wx_blc(BUF_rot), Wy_blc(BUF_rot), Wz_blc(BUF_rot);

		vector<double> Ax_aver_arr(BUF_rot), Ay_aver_arr(BUF_rot), Az_aver_arr(BUF_rot);
		vector<double> Wx_aver_arr(BUF_rot), Wy_aver_arr(BUF_rot), Wz_aver_arr(BUF_rot);

		vector<double> Ax_aver_arr_temp(BUF_rot), Ay_aver_arr_temp(BUF_rot), Az_aver_arr_temp(BUF_rot);
		vector<double> Wx_aver_arr_temp(BUF_rot), Wy_aver_arr_temp(BUF_rot), Wz_aver_arr_temp(BUF_rot);

		size_t count_ar = 0; // Счётчик элементов в текущем буфере

		// Состояния фильтра EMA между блоками (хранят последнее сглаженное значение)
		double last_Ax = 0, last_Ay = 0, last_Az = 0;
		double last_Wx = 0, last_Wy = 0, last_Wz = 0;

		// Суммы для расчёта общего среднего
		double Ax_sm = 0, Ay_sm = 0, Az_sm = 0;
		double Wx_sm = 0, Wy_sm = 0, Wz_sm = 0;
		long total_samples = 0;


		int is_first_block = 1; // Флаг первого блока для EMA фильтра
		double SAMPLE_RATE = 200.0; // Частота измерений IMU (Гц)
		double CUTOFF_FREQ = 1.0; // Частота среза фильтра (Гц)
		double alpha = calculate_alpha(CUTOFF_FREQ, SAMPLE_RATE);

		// Переменные для рекурсивного накопления среднего значения углов
		double final_yaw_mean = 0.0;
		double final_pitch_mean = 0.0;
		double final_roll_mean = 0.0;
		long final_angles_count = 0;

		double Time, Ax, Ay, Az, Wx, Wy, Wz;

		// Считывание данных
		while(getline(file_imu, buffer))
		{
				if (sscanf(buffer.c_str(), "%lf %*s %lf %lf %lf %lf %lf %lf", &Time, &Wx, &Wy, &Wz, &Ax, &Ay, &Az) == 7)
				{
						if (Time > iter) // Окончание выставки
						{
								if (count_ar > 0)
								{
										// Обработка хвоста данных (остатка буфера, не заполнившего полный размер BUF_rot)
										FastMedian(Ax_blc, Ax_aver_arr_temp, count_ar);
										EMA_Filter(Ax_aver_arr_temp, Ax_aver_arr, alpha, count_ar, &last_Ax, is_first_block);
										FastMedian(Ay_blc, Ay_aver_arr_temp, count_ar);
										EMA_Filter(Ay_aver_arr_temp, Ay_aver_arr, alpha, count_ar, &last_Ay, is_first_block);
										FastMedian(Az_blc, Az_aver_arr_temp, count_ar);
										EMA_Filter(Az_aver_arr_temp, Az_aver_arr, alpha, count_ar, &last_Az, is_first_block);

										FastMedian(Wx_blc, Wx_aver_arr_temp, count_ar);
										EMA_Filter(Wx_aver_arr_temp, Wx_aver_arr, alpha, count_ar, &last_Wx, is_first_block);
										FastMedian(Wy_blc, Wy_aver_arr_temp, count_ar);
										EMA_Filter(Wy_aver_arr_temp, Wy_aver_arr, alpha, count_ar, &last_Wy, is_first_block);
										FastMedian(Wz_blc, Wz_aver_arr_temp, count_ar);
										EMA_Filter(Wz_aver_arr_temp, Wz_aver_arr, alpha, count_ar, &last_Wz, is_first_block);

										size_t k = time_arr.size();
										for (size_t i = 0; i < count_ar; i++)
										{
												size_t global_index = k - count_ar + i;

												// Расчёт глобальных средних значений ускорений и угловых скоростей
												double Ax_mean = Ax_sm / total_samples;
												double Ay_mean = Ay_sm / total_samples;
												double Az_mean = Az_sm / total_samples;

												double Wx_mean = Wx_sm / total_samples;
												double Wy_mean = Wy_sm / total_samples;
												double Wz_mean = Wz_sm / total_samples;

												// Вычисление сглаженных углов (тангаж и крен по акселерометру)
												double ax_norm_sm = Ax_mean / g;
												if (ax_norm_sm > 1.0) ax_norm_sm = 1.0;
												if (ax_norm_sm < -1.0) ax_norm_sm = -1.0;

												double pitch_smooth = asin(ax_norm_sm);
												double roll_smooth = atan2(-Az_mean, Ay_mean);

												// Проекции угловой скорости Земли для расчёта курса (Yaw)
												double We_sm = Wx_mean * cos(pitch_smooth) + Wy_mean * sin(pitch_smooth) * sin(roll_smooth) + Wz_mean *  sin(pitch_smooth) * cos(roll_smooth);
												double Wn_sm = Wy_mean * cos(roll_smooth) - Wz_mean * sin(roll_smooth);
												double yaw_smooth = atan2(-We_sm , Wn_sm);

												// Сырые углы
												double raw_ax_norm = Ax_arr[global_index] / g;
												if (raw_ax_norm > 1.0) raw_ax_norm = 1.0;
												if (raw_ax_norm < -1.0) raw_ax_norm = -1.0;
												
												double raw_pitch = asin(raw_ax_norm);
												double raw_roll = atan2(-Az_arr[global_index], Ay_arr[global_index]);

												double raw_We = Wx_arr[global_index] * cos(pitch_smooth) + Wy_arr[global_index] * sin(pitch_smooth) * sin(roll_smooth) + Wz_arr[global_index] *  sin(pitch_smooth) * cos(roll_smooth);
												double raw_Wn = Wy_arr[global_index] * cos(roll_smooth) - Wz_arr[global_index] * sin(roll_smooth);
												double raw_yaw = atan2(-raw_We , raw_Wn);

												// Рекурсивное обновление среднего углов
												final_angles_count++;
												update_recursive_mean(final_yaw_mean, yaw_smooth, final_angles_count);
												update_recursive_mean(final_pitch_mean, pitch_smooth, final_angles_count);
												update_recursive_mean(final_roll_mean, roll_smooth, final_angles_count);

												char line_buf[512];
												snprintf(line_buf, sizeof(line_buf), "%-10.4f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f\n", time_arr[global_index], Ax_arr[global_index], Ay_arr[global_index], Az_arr[global_index], Wx_arr[global_index], Wy_arr[global_index], Wz_arr[global_index], yaw_smooth, roll_smooth, pitch_smooth, Ax_aver_arr[i], Ay_aver_arr[i], Az_aver_arr[i], Wx_aver_arr[i], Wy_aver_arr[i], Wz_aver_arr[i], raw_yaw, raw_roll, raw_pitch);
												graphfile << line_buf;
										}
								}
								break;
						}

						// Считываем данные в массив
						time_arr.push_back(Time);
						Ax_arr.push_back(Ax);
						Ay_arr.push_back(Ay);
						Az_arr.push_back(Az);
						Wx_arr.push_back(Wx * (PI / 180.0));
						Wy_arr.push_back(Wy * (PI / 180.0));
						Wz_arr.push_back(Wz * (PI / 180.0));

						// Накопление сумм для расчёта средних значений
						Ax_sm += Ax;
						Ay_sm += Ay;
						Az_sm += Az;

						Wx_sm += Wx * (PI / 180.0);
						Wy_sm += Wy * (PI / 180.0);
						Wz_sm += Wz * (PI / 180.0);

						total_samples++;

						// Заполнение блоков для медианной фильтрации
						if (count_ar < BUF_rot)
						{
								Ax_blc[count_ar] = Ax;
								Ay_blc[count_ar] = Ay;
								Az_blc[count_ar] = Az;
								Wx_blc[count_ar] = Wx * (PI / 180.0);
								Wy_blc[count_ar] = Wy * (PI / 180.0);
								Wz_blc[count_ar] = Wz * (PI / 180.0);
						}

						count_ar++;

						// Если буфер заполнен полностью, производим его фильтрацию и расчёт
						if (count_ar == BUF_rot)
						{
								FastMedian(Ax_blc, Ax_aver_arr_temp, count_ar);
								EMA_Filter(Ax_aver_arr_temp, Ax_aver_arr, alpha, count_ar, &last_Ax, is_first_block);
								FastMedian(Ay_blc, Ay_aver_arr_temp, count_ar);
								EMA_Filter(Ay_aver_arr_temp, Ay_aver_arr, alpha, count_ar, &last_Ay, is_first_block);
								FastMedian(Az_blc, Az_aver_arr_temp, count_ar);
								EMA_Filter(Az_aver_arr_temp, Az_aver_arr, alpha, count_ar, &last_Az, is_first_block);

								FastMedian(Wx_blc, Wx_aver_arr_temp, count_ar);
								EMA_Filter(Wx_aver_arr_temp, Wx_aver_arr, alpha, count_ar, &last_Wx, is_first_block);
								FastMedian(Wy_blc, Wy_aver_arr_temp, count_ar);
								EMA_Filter(Wy_aver_arr_temp, Wy_aver_arr, alpha, count_ar, &last_Wy, is_first_block);
								FastMedian(Wz_blc, Wz_aver_arr_temp, count_ar);
								EMA_Filter(Wz_aver_arr_temp, Wz_aver_arr, alpha, count_ar, &last_Wz, is_first_block);

								is_first_block = 0;

								size_t k = time_arr.size();
								for (size_t i = 0; i < BUF_rot; i++)
								{
										size_t global_index = k - BUF_rot + i;

										double Ax_mean = Ax_sm / total_samples;
										double Ay_mean = Ay_sm / total_samples;
										double Az_mean = Az_sm / total_samples;

										double Wx_mean = Wx_sm / total_samples;
										double Wy_mean = Wy_sm / total_samples;
										double Wz_mean = Wz_sm / total_samples;

										double ax_norm_sm = Ax_mean / g;
										if (ax_norm_sm > 1.0) ax_norm_sm = 1.0;
										if (ax_norm_sm < -1.0) ax_norm_sm = -1.0;

										double pitch_smooth = asin(ax_norm_sm);
										double roll_smooth = atan2(-Az_mean, Ay_mean);

										double We_sm = Wx_mean * cos(pitch_smooth) + Wy_mean * sin(pitch_smooth) * sin(roll_smooth) + Wz_mean *  sin(pitch_smooth) * cos(roll_smooth);
										double Wn_sm = Wy_mean * cos(roll_smooth) - Wz_mean * sin(roll_smooth);
										double yaw_smooth = atan2(-We_sm , Wn_sm);

										double raw_ax_norm = Ax_arr[global_index] / g;
										if (raw_ax_norm > 1.0) raw_ax_norm = 1.0;
										if (raw_ax_norm < -1.0) raw_ax_norm = -1.0;
										
										double raw_pitch = asin(raw_ax_norm);
										double raw_roll = atan2(-Az_arr[global_index], Ay_arr[global_index]);

										double raw_We = Wx_arr[global_index] * cos(pitch_smooth) + Wy_arr[global_index] * sin(pitch_smooth) * sin(roll_smooth) + Wz_arr[global_index] *  sin(pitch_smooth) * cos(roll_smooth);
										double raw_Wn = Wy_arr[global_index] * cos(roll_smooth) - Wz_arr[global_index] * sin(roll_smooth);
										double raw_yaw = atan2(-raw_We , raw_Wn);

										final_angles_count++;
										update_recursive_mean(final_yaw_mean, yaw_smooth, final_angles_count);
										update_recursive_mean(final_pitch_mean, pitch_smooth, final_angles_count);
										update_recursive_mean(final_roll_mean, roll_smooth, final_angles_count);

										char line_buf[512];
										snprintf(line_buf, sizeof(line_buf), "%-10.4f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f %-12.6f\n", time_arr[global_index], Ax_arr[global_index], Ay_arr[global_index], Az_arr[global_index], Wx_arr[global_index], Wy_arr[global_index], Wz_arr[global_index], yaw_smooth, roll_smooth, pitch_smooth, Ax_aver_arr[i], Ay_aver_arr[i], Az_aver_arr[i], Wx_aver_arr[i], Wy_aver_arr[i], Wz_aver_arr[i], raw_yaw, raw_roll, raw_pitch);
										graphfile << line_buf;
								}
								count_ar = 0; // Сброс счётчика буфера для следующей порции
						}
				}
		}

		file_imu.close();
		graphfile.close();

		// Записываем результаты: курс из StartupNav.ini, тангаж и крен — из фильтра
		*Yaw = heading_deg * DEG_TO_RAD;
		*Pitch = final_pitch_mean;
		*Roll = final_roll_mean;

}