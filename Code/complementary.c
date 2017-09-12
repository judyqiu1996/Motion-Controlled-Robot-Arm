#include "complementary.h"

float x_filter=0, y_filter=0, z_filter=0, angle_x=0, angle_y=0, angle_z=0 ;
float pitch , yaw,  roll ;
float sign;
void complementary_filter(float * Buf,float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z,float dt)
{

	
		x_filter = 0.9*x_filter +  0.1*accel_x;
		z_filter = 0.9*z_filter + 0.1*accel_z;
		y_filter = 0.9*y_filter + 0.1*accel_y;
		//acc_angle_y = atan2(x_filter, -z_filter) * 180/3.14159265358979323 ;
		//angle_y = 0.98*(gyro_y*dt + angle_y) + 0.02*acc_angle_y;
	
		//acc_angle_x = atan2(y_filter, -z_filter) * 180/3.14159265358979323 ;
		//angle_x = 0.98*(gyro_x*dt + angle_x) + 0.02*acc_angle_x;
	
		//acc_angle_z = atan2(x_filter,y_filter) * 180/3.14159265358979323 ;
		//angle_z = 0.98*(gyro_z*dt + angle_z) + 0.02*acc_angle_z;
		angle_z = angle_z + gyro_z * dt;
		sign = z_filter > 0?-1:1;
		pitch = atan2(-x_filter,sqrt(y_filter*y_filter + z_filter*z_filter)) * 180/3.14159265358979323 ;
	  roll =  atan2(y_filter, sign* sqrt(z_filter*z_filter + 0.01*x_filter*x_filter))* 180/3.14159265358979323;
		angle_x = 0.98*(gyro_x*dt + angle_x) + 0.02*roll;
		angle_y = 0.98*(gyro_y*dt + angle_y) + 0.02*pitch;
		Buf[0] = angle_x;
		Buf[1] = angle_y;
		//Buf[2] = angle_z;
}
