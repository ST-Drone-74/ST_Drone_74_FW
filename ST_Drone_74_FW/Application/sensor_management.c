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

void sensorUpdateValue(uint8_t checkFlag)
{
	static uint8_t counterUp_u8=0x00;
	float accelGetValue=0x00;
	if(checkFlag == TRUE)
	{
		counterUp_u8++;
		switch (counterUp_u8)
		{
			case 1://handle accel X value
			accel_X_Out(&accelGetValue);
			accelBleSentValue_st.AXIS_X = ((int32_t)accelGetValue)*1000;
			break;

			case 2://handle accel Y value
			accel_Y_Out(&accelGetValue);
			accelBleSentValue_st.AXIS_Y = ((int32_t)accelGetValue)*1000;
			break;

			case 3://handle accel Z value
			accel_Z_Out(&accelGetValue);
			accelBleSentValue_st.AXIS_Z = ((int32_t)accelGetValue)*1000;
			break;

			case 4://handle barometer
			break;

			default:
			counterUp_u8 = 0x00;
		}
	}
	else
	{

	}
}
