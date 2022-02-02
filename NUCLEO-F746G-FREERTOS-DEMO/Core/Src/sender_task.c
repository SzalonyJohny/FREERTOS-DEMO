/*
 * sender_task.c
 *
 *  Created on: Jan 29, 2022
 *      Author: jan
 */



#include "sender_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "MPU6050.h"


extern I2C_HandleTypeDef hi2c1;



struct imu_data_struct{
	float ax, ay, az;
	float gx, gy, gz;
}imu_data;






void Start_Sender_task(void *argument){

	MPU6050_Init(&hi2c1);

	MPU6050_SetIntEnableRegister(0);

	MPU6050_SetDHPFMode(MPU6050_DHPF_5);

	MPU6050_SetIntMotionEnabled(0);
	MPU6050_SetIntZeroMotionEnabled(0);
	MPU6050_SetIntFreeFallEnabled(0);

	MPU6050_SetFreeFallDetectionDuration(2);
	MPU6050_SetFreeFallDetectionThreshold(5);

	MPU6050_SetMotionDetectionDuration(5);
	MPU6050_SetMotionDetectionThreshold(2);

	MPU6050_SetZeroMotionDetectionDuration(2);
	MPU6050_SetZeroMotionDetectionThreshold(4);



	for(;;){

		osDelay(100);
		MPU6050_GetAccelerometerScaled( &imu_data.ax, &imu_data.ay, &imu_data.az);
		MPU6050_GetGyroscopeScaled(&imu_data.gx, &imu_data.gy, &imu_data.gz);

	}
}
