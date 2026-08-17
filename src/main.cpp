#include <stdio.h>
#include <stdlib.h>
#include "navigation/aligner.hpp"

int main () 
{
		const char *base_dir = "../data/raw";
		const char *file_name_Nav = "Nav.DAT";
		const char *file_name_IMU = "IMU.DAT";
		const char *file_name_StartupNav = "StartupNav.ini";

		char full_path_Nav[256];
		char full_path_IMU[256];
		char full_path_StartupNav[256];

		snprintf(full_path_Nav, sizeof(full_path_Nav), "%s/%s", base_dir, file_name_Nav);
		snprintf(full_path_IMU, sizeof(full_path_IMU), "%s/%s", base_dir, file_name_IMU);
		snprintf(full_path_StartupNav, sizeof(full_path_StartupNav), "%s/%s", base_dir, file_name_StartupNav);

		printf("full_path_Nav: %s\n", full_path_Nav);
		printf("full_path_IMU: %s\n", full_path_IMU);
		printf("full_path_StartupNav: %s\n", full_path_StartupNav);
		
		double Yaw_0 = 0.0, Pitch_0 = 0.0, Roll_0 = 0.0;

		get_angle_start(&Yaw_0, &Pitch_0, &Roll_0, full_path_IMU, full_path_Nav, full_path_StartupNav);
		printf("Yaw_0: %lf, Pitch_0: %lf, Roll_0: %lf", Yaw_0, Pitch_0, Roll_0);

		return 0;
}