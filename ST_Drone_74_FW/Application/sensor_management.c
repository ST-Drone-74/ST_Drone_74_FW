/*
 * sensor_management.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <sensor_management.h>
/*Golbal sensor init*/
sensorInitState_e sensorInitState = SENSOR_INIT_OK;

/*Global sensor varibales for BLE updating*/
baroData_st baroBleSentValue_st =
{
	.TEMP = 0,
	.PRESSURE = 0.0
};
/*Global sensor varibales for BLE updating*/
SensorAxes_t magBleSentValue_st =
{
	.AXIS_X = 0,
	.AXIS_Y = 0,
	.AXIS_Z = 0
};
/*Global sensor varibales for BLE updating*/
SensorAxes_t accelBleSentValue_st =
{
	.AXIS_X = 0,
	.AXIS_Y = 0,
	.AXIS_Z = 0
};
/*Global sensor varibales for BLE updating*/
SensorAxes_t gyroBleSentValue_st =
{
	.AXIS_X = 0,
	.AXIS_Y = 0,
	.AXIS_Z = 0
};

void all_Sensor_Init(void)
{
	uint8_t getSensorInitFail = 0x00;
	/*init compass lis2dmdl*/
	if(compass_Init_Device() != 1)
	{
		/*if init fail*/
		getSensorInitFail++;
	}
	/*init barometer lps22hd*/
	if(baro_Init_Device() != 1)
	{
		/*if init fail*/
		getSensorInitFail++;
	}
	/*init accel gyro lsm6dsl*/
	if(accelGyro_Init_Device() != 1)
	{
		/*if init fail*/
		getSensorInitFail++;
	}

	if(getSensorInitFail == 0)
	{	
		sensorInitState = SENSOR_INIT_OK;
	}
	else
	{
		sensorInitState = SENSOR_INIT_ERROR;
	}
}
