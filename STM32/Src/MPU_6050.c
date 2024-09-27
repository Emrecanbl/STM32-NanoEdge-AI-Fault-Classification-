/*
 * MPU_6050.c
 *
 *  Created on: Mar 28, 2024
 *      Author: EmrecanBL
 */


#include "stm32G4xx_hal.h"
#include "MPU_6050.h"


extern I2C_HandleTypeDef hi2c1;
void MPU_6050_Who_am_I(){
	uint8_t Data[1];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_WHO_AM_I, 1, &Data, 1, 100);
}
uint8_t MPU_6050_Init(){
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_I2C_ADRESS_AD0,MPU_REG_WHO_AM_I,1, &check, 1, 1000);

	if (check == 0x68)  // Check Sensor
	{
		Data = 0;// Power-Up
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_PWR_MGMT_1, 1, &Data, 1, 100);
		Data = 0x07;// 1 kHZ
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_SMPLRT_DIV, 1, &Data, 1, 100);
		Data=0;
		Data |= (3 << 3);// 2000 °/s
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_GYRO_CONFIG, 1, &Data, 1, 100);
		Data=0;
		Data |= (3 << 3);// 16g
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_ACCEL_CONFIG, 1, &Data, 1, 100);
		return 1;
	}
	else {
		return 0;
	}
}
int16_t MPU_6050_Temp_Read(){
	int8_t Data[2];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_ACCEL_TEMP_OUT_H, 1, Data, 2, 100);
	int16_t Temp =((Data[0] << 8)|Data[1]);
	Temp = (Temp/340) + 36.5;
	return Temp;
}

void MPU_6050_Accelerometer_Read(SensorData_t *SensorData){
	uint8_t Data[6];
	int16_t RAW_ACCEL_XOUT = 0 ;
	int16_t RAW_ACCEL_YOUT = 0 ;
	int16_t RAW_ACCEL_ZOUT = 0 ;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_ACCEL_XOUT_H, 1, Data, 6, 100);

	RAW_ACCEL_XOUT = (int16_t)(Data[0]<<8)|(Data[1]);
	RAW_ACCEL_YOUT = (int16_t)(Data[2]<<8)|(Data[3]);
	RAW_ACCEL_ZOUT = (int16_t)(Data[4]<<8)|(Data[5]);

	SensorData->ACCEL_XOUT = RAW_ACCEL_XOUT/2048.0; //LSB Sensitivity
	SensorData->ACCEL_YOUT = RAW_ACCEL_YOUT/2048.0;
	SensorData->ACCEL_ZOUT = RAW_ACCEL_ZOUT/2048.0;
}

void MPU_6050_Angle(SensorData_t *SensorData){
	float x,y,z;
	x = SensorData->ACCEL_XOUT;
	y = SensorData->ACCEL_YOUT;
	z = SensorData->ACCEL_ZOUT;
	float tanx,tany,tanz;
	tanx = x/(sqrt((z)*(z))+(y*y));
	tany = y/(sqrt((x)*(x))+(z*z));
	tanz = z/(sqrt((y)*(y))+(x*x));
	tanx = atan(tanx);
	tany = atan(tany);
	tanz = atan(tanz);
	SensorData->Angle_X = tanx*(1/(3.14/180.0));
	SensorData->Angle_Y = tany*(1/(3.14/180.0));
	SensorData->Angle_Z = tanz*(1/(3.14/180.0));
}

void MPU_6050_Gyroscope_Read(SensorData_t *SensorData){
	uint8_t Data[6];
	int16_t RAW_Gyro_XOUT = 0 ;
	int16_t RAW_Gyro_YOUT = 0 ;
	int16_t RAW_Gyro_ZOUT = 0 ;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADRESS_AD0, MPU_REG_GYRO_XOUT_H, 1, Data, 6, 100);

	RAW_Gyro_XOUT = (int16_t)(Data[0]<<8)|(Data[1]);
	RAW_Gyro_YOUT = (int16_t)(Data[2]<<8)|(Data[3]);
	RAW_Gyro_ZOUT = (int16_t)(Data[4]<<8)|(Data[5]);

	SensorData->GYRO_XOUT = RAW_Gyro_XOUT/16.4-(-2.6245); // LSB/°/s
	SensorData->GYRO_YOUT = RAW_Gyro_YOUT/16.4-(2.9287); //MPU_6050_Gyroscope_Cali outputs
	SensorData->GYRO_ZOUT = RAW_Gyro_ZOUT/16.4-(-2.502);

}

void MPU_6050_Gyroscope_Cali(SensorData_t *SensorData){
	for(int i =0 ;i<=1000;i++){
		SensorData->x_cali=SensorData->x_cali+SensorData->GYRO_XOUT;
		SensorData->y_cali=SensorData->y_cali+SensorData->GYRO_YOUT;
		SensorData->z_cali=SensorData->z_cali+SensorData->GYRO_ZOUT;
	}
	SensorData->x_cali =SensorData->x_cali/1000;	//-2.6245
	SensorData->y_cali =SensorData-> y_cali/1000;	//2.9287  Calibration value Gyro
	SensorData->z_cali =SensorData->z_cali/1000;	//-2.502
}

void Kalman_1d(float State,float uncertainty,float Input,float Messured_Degree,float Time_s,Kalman_t *Kalman){
	State = State+ Time_s*State;
	uncertainty = uncertainty + Time_s*Time_s*4*4;
	float Gain =uncertainty*1/(uncertainty*uncertainty+3*3);
	State = State+Gain*(Messured_Degree-State);
	uncertainty = (1-Gain)*uncertainty;

	Kalman->State_OUT = State ;
	Kalman->Uncertainty_OUT = uncertainty ;
}

void Kalman_X_Angle(SensorData_t *SensorData,Kalman_t *Kalman){
	float Time = 0;
	Time = 0.0001;
	Kalman_1d(Kalman->K_Angle_X,Kalman->Uncertainty_Angle_X,SensorData->GYRO_XOUT,SensorData->Angle_X,Time,&Kalman);
}

