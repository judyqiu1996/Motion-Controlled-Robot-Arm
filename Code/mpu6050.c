
#include "mpu6050.h"


#include "i2c.h"

extern volatile uint8_t 	I2CMasterBuffer[BUFSIZE], I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t 	I2CReadLength, I2CWriteLength;

int16_t zero_acc_x = 0,zero_acc_z = 0 ,zero_acc_y=0;
int16_t zero_gyro_roll = 0, zero_X = 0, zero_Z = 0;

void clearBuffer()
{
	int i;
	for(i=0; i < BUFSIZE; i++) I2CMasterBuffer[i] = 0;
}

uint8_t MPU6050_init() {

	uint8_t state;
	
	clearBuffer();
	
	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_PWR_MGMT_1;
	I2CMasterBuffer[2] 	= 0x01; // reset device y-axis gyro as clk;

	state = I2CEngine();
	if(state != I2C_OK) return 1;


	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_CONFIG;
	I2CMasterBuffer[2] 	= 0x00; // Lowpass
	//I2CMasterBuffer[2] 	= 0b00000011; // Lowpass
	state = I2CEngine();
	if(state != I2C_OK) return 1;


	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_PWR_MGMT_1;
	I2CMasterBuffer[2] 	= 0x04; // wakeup

	state = I2CEngine();
	if(state != I2C_OK) return 1;


	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_GYRO_CONFIG;
	I2CMasterBuffer[2] 	= 0x08; // gyro range 600°/s

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_CONFIG;
	I2CMasterBuffer[2] 	= 0x18; // 16G range

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	return 0;
}

uint8_t MPU6050_whoami(){

	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 1;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;	  
	I2CMasterBuffer[1] 	= MPU6050_RA_WHO_AM_I;
	I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return state;

	if(I2CSlaveBuffer[0] != 0x68) return 1;

	return 0;
}

void MPU6050_setZero(){

	uint16_t i;
	uint16_t n = 1000;
	int32_t tmp_y= 0,tmp_x = 0, tmp_z = 0;
	int32_t ac_x=0, ac_y = 0, ac_z = 0;

	for(i=0;i<n;i++){
		tmp_y += MPU6050_getGyroRoll_raw();
	}
	for (i=0;i<n;i++){
		tmp_z += MPU6050_getGyroZ_raw();
	}
	for(i=0;i<n;i++){
		
		tmp_x += MPU6050_getGyroX_raw();
	}
	for (i=0;i<n;i++){
		ac_x += MPU6050_getAccel_x_raw();
	}
	for (i=0;i<n;i++){
		ac_y += MPU6050_getAccel_y_raw();
	}
	for (i=0;i<n;i++){
		ac_z += MPU6050_getAccel_z_raw();
	}
	tmp_y /= n;
	tmp_x /= n;
	tmp_z /= n;
	ac_y /= n;
	ac_x /= n;
	ac_z /= n;
	zero_gyro_roll = tmp_y;
	zero_X = tmp_x;
	zero_Z = tmp_z;
	zero_acc_x = ac_x;
	zero_acc_z = ac_z + 2048;
	zero_acc_y = ac_y;
	
}


int16_t MPU6050_getGyroRoll_raw(){

	int16_t tmp;
	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 2; //1;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_GYRO_YOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	tmp = (I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1];
	//tmp = (I2CSlaveBuffer[0] << 8) ;

//	I2CWriteLength 		= 2;
//	I2CReadLength		= 1;
//	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
//	I2CMasterBuffer[1] 	= MPU6050_RA_GYRO_YOUT_L;
//	I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;
//
//	state = I2CEngine();
//	if(state != I2C_OK) return 1;
//	tmp |= (I2CSlaveBuffer[0]) ;
	return tmp - zero_gyro_roll;
}

float MPU6050_getGyroRoll_degree(){

	int16_t tmp;
	float roll;

	tmp = MPU6050_getGyroRoll_raw();
	roll = (tmp / 65.536 );
	return roll;
}

float MPU6050_getGyroRoll_rad(){

	int16_t tmp;
	float roll;

	tmp = MPU6050_getGyroRoll_raw();
	roll = (float) ((tmp / 65.536) * 3.14159) / 180;
	return roll ;
}

int16_t MPU6050_getAccel_x_raw(){
	int16_t tmp;
	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 2;   //1;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_XOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

    tmp = (I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1];
//	tmp = (I2CSlaveBuffer[0] << 8) ;
//
//	I2CWriteLength 		= 2;
//	I2CReadLength		= 1;
//	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
//	I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_YOUT_L;
//	I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;
//
//	state = I2CEngine();
//	if(state != I2C_OK) return 1;
//
//	tmp |= (I2CSlaveBuffer[0]) ;

	return tmp - zero_acc_x ;
}

float MPU6050_getAccel_x(){

	int16_t tmp;
	float x;

	tmp = MPU6050_getAccel_x_raw();
	x = (float) tmp / 2048;
	return x;
}

int16_t MPU6050_getAccel_z_raw(){

		int16_t tmp;
		uint8_t state;

		I2CWriteLength 		= 2;
		I2CReadLength		= 2;   //1;
		I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
		I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_ZOUT_H;
		I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;

		state = I2CEngine();
		if(state != I2C_OK) return 1;

		tmp = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1];
//		tmp = (I2CSlaveBuffer[0] << 8) ;
//
//		I2CWriteLength 		= 2;
//		I2CReadLength		= 1;
//		I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
//		I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_ZOUT_L;
//		I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;
//
//		state = I2CEngine();
//		if(state != I2C_OK) return 1;
//
//		tmp |= (I2CSlaveBuffer[0]) ;
		return tmp - zero_acc_z; /* TBD */

}
float MPU6050_getAccel_z(){

	int16_t tmp;
	float z;

	tmp =  MPU6050_getAccel_z_raw() ;
	z = (float) tmp / 2048;
	return z;
}


int16_t MPU6050_getAccel_y_raw(){
	
	
		int16_t tmp;
		uint8_t state;

		I2CWriteLength 		= 2;
		I2CReadLength		= 2;   //1;
		I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
		I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_YOUT_H;
		I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;

		state = I2CEngine();
		if(state != I2C_OK) return 1;

		tmp = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1];
	
	return tmp - zero_acc_y;
}

float MPU6050_getAccel_y(){
	
	int16_t tmp;
	float y;
	tmp = MPU6050_getAccel_y_raw();
	y = (float) tmp/2048;
	return y;
	
}

int16_t MPU6050_getGyroX_raw(void)
{
	int16_t tmp;
	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 2; //1;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_GYRO_XOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	tmp = (I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1];

	return tmp - zero_X;
	
}
float MPU6050_getGyroX(void)
{
	int16_t tmp;
	float roll;

	tmp = MPU6050_getGyroX_raw();
	roll = (tmp  / 65.536 );
	return roll;
}

int16_t MPU6050_getGyroZ_raw(void)
{
	int16_t tmp;
	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 2; //1;
	I2CMasterBuffer[0] 	= MPU6050_ADDRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_GYRO_ZOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADDRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	tmp = (I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1];

	return tmp - zero_Z;
	
}

float MPU6050_getGyroZ(void)
{
	int16_t tmp;
	float roll;

	tmp = MPU6050_getGyroZ_raw();
	roll = (tmp  / 65.536 );
	return roll;
}