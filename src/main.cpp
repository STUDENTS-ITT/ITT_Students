#include <stdio.h>
#include <stdlib.h>
#include "navigation/aligner.hpp"

int main () 
{
		double Yaw_0 = 0.0, Pitch_0 = 0.0, Roll_0 = 0.0;
		
		const char *base_dir = "../data/raw";
		const char *file_name_Nav = "Nav.DAT";
		const char *file_name_IMU = "IMU.DAT";

		char full_path_Nav[256];
		char full_path_IMU[256];

		snprintf(full_path_Nav, sizeof(full_path_Nav), "%s/%s", base_dir, file_name_Nav);
		snprintf(full_path_IMU, sizeof(full_path_IMU), "%s/%s", base_dir, file_name_IMU);

		printf("full_path_Nav: %s\n", full_path_Nav);
		printf("full_path_IMU: %s\n", full_path_IMU);

		get_angle_start(&Yaw_0, &Pitch_0, &Roll_0, full_path_IMU, full_path_Nav);
		printf("Yaw_0: %lf, Pitch_0: %lf, Roll_0: %lf", Yaw_0, Pitch_0, Roll_0);

		return 0;
}