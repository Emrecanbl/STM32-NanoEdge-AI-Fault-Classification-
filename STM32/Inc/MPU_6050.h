/*
 * MPU_6050.h
 *
 *  Created on: Mar 28, 2024
 *      Author: EmrecanBL
 */

#ifndef INC_MPU_6050_H_
#define INC_MPU_6050_H_

#include <stdint.h>
#include "stm32G4xx_hal.h"

#define MPU6050_I2C_ADRESS_AD0 ( 0x68<<1 )
#define MPU6050_I2C_ADRESS_AD1 ( 0x69<<1 )

#define MPU_REG_SELF_TEST_X	( 13 )
#define MPU_REG_SELF_TEST_Y	( 14 )
#define MPU_REG_SELF_TEST_Z	( 15 )
#define MPU_REG_SELF_TEST_A	( 16 )
#define MPU_REG_SMPLRT_DIV	( 25 )
#define MPU_REG_CONFIG	( 26 )
#define MPU_REG_GYRO_CONFIG	( 27 )
#define MPU_REG_ACCEL_CONFIG	( 28 )
#define MPU_REG_ACCEL_XOUT_H	( 59 )
#define MPU_REG_ACCEL_XOUT_L	( 60 )
#define MPU_REG_ACCEL_YOUT_H	( 61 )
#define MPU_REG_ACCEL_YOUT_L	( 62 )
#define MPU_REG_ACCEL_ZOUT_H	( 63 )
#define MPU_REG_ACCEL_ZOUT_L	( 64 )
#define MPU_REG_ACCEL_TEMP_OUT_H ( 65 )
#define MPU_REG_ACCEL_TEMP_OUT_L	( 66 )
#define MPU_REG_GYRO_XOUT_H	( 67 )
#define MPU_REG_GYRO_XOUT_L	( 68 )
#define MPU_REG_GYRO_YOUT_H	( 69 )
#define MPU_REG_GYRO_YOUT_L	( 70 )
#define MPU_REG_GYRO_ZOUT_H	( 71 )
#define MPU_REG_GYRO_ZOUT_L	( 72 )
#define MPU_REG_USER_CTRL	( 106)
#define MPU_REG_PWR_MGMT_1	( 107 )
#define MPU_REG_PWR_MGMT_2	( 108 )
#define MPU_REG_FIFO_COUNTH ( 114 )
#define MPU_REG_FIFO_COUNTL ( 115 )
#define MPU_REG_FIFO_R_W	( 116 )
#define MPU_REG_WHO_AM_I	( 117 )



typedef struct {
	float GYRO_XOUT;
	float GYRO_YOUT;
	float GYRO_ZOUT;
	float Angle_X;
	float Angle_Y;
	float Angle_Z;
	float x_cali;
	float y_cali;
	float z_cali;
	float ACCEL_XOUT;
	float ACCEL_YOUT;
	float ACCEL_ZOUT;
}SensorData_t;

typedef struct {
	float K_Angle_X;//0
	float K_Angle_Y;//0
	float Uncertainty_Angle_X; // 4
	float Uncertainty_Angle_Y; // 4
	float State_OUT;
	float Uncertainty_OUT;
}Kalman_t;

void MPU_6050_Who_am_I();
uint8_t MPU_6050_Init();
int16_t MPU_6050_Temp_Read();
void MPU_6050_Accelerometer_Read(SensorData_t *SensorData);
void MPU_6050_Angle(SensorData_t *SensorData);
void MPU_6050_Gyroscope_Read(SensorData_t *SensorData);
void MPU_6050_Gyroscope_Cali(SensorData_t *SensorData);
void Kalman_1d(float State,float uncertainty,float Input,float Messured_Degree,float Time_s,Kalman_t *Kalman);
void Kalman_X_Angle(SensorData_t *SensorData,Kalman_t *Kalman);
#endif /* INC_MPU_6050_H_ */
