#ifndef _Complementary_h
#define _Complementary_h

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <rt_misc.h>
#include <string.h>	
void complementary_filter(float * Buf,float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z, float dt);
#endif
