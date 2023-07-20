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

void sensorBleUpdateValue(uint8_t checkFlag)
{
	static uint8_t counterUp_u8=0x00;
	float sensorGetValue_f=0x00;

	if(checkFlag == TRUE)
	{
		counterUp_u8++;
		switch (counterUp_u8)
		{
			case 1://handle accel X value
			accel_X_Out(&sensorGetValue_f);
			accelBleSentValue_st.AXIS_X = ((int32_t)sensorGetValue_f);
			break;

			case 2://handle accel Y value
			accel_Y_Out(&sensorGetValue_f);
			accelBleSentValue_st.AXIS_Y = ((int32_t)sensorGetValue_f);
			break;

			case 3://handle accel Z value
			accel_Z_Out(&sensorGetValue_f);
			accelBleSentValue_st.AXIS_Z = ((int32_t)sensorGetValue_f);
			break;

			case 4://handle gyro X
			gyro_X_Out(&sensorGetValue_f);
			gyroBleSentValue_st.AXIS_X = ((int32_t)sensorGetValue_f);
			break;

			case 5://handle gyro y
			gyro_Y_Out(&sensorGetValue_f);
			gyroBleSentValue_st.AXIS_Y = ((int32_t)sensorGetValue_f);
			break;

			case 6://handle gyro z
			gyro_Z_Out(&sensorGetValue_f);
			gyroBleSentValue_st.AXIS_Z = ((int32_t)sensorGetValue_f);
			break;

			case 7://handle magnetic x
			compass_Read_X_Data(&sensorGetValue_f);
			magBleSentValue_st.AXIS_X = ((int32_t)sensorGetValue_f);
			break;

			case 8://handle magnetic y
			compass_Read_Y_Data(&sensorGetValue_f);
			magBleSentValue_st.AXIS_Y = ((int32_t)sensorGetValue_f);
			break;

			case 9://handle magnetic z
			compass_Read_Z_Data(&sensorGetValue_f);
			magBleSentValue_st.AXIS_Z = ((int32_t)sensorGetValue_f);
			break;

			case 10://handle barometer
			baro_HPA_Pressure(&sensorGetValue_f);
			baro_Read_Temperature(&baroBleSentValue_st.TEMP);
			baroBleSentValue_st.PRESSURE = ((int32_t)sensorGetValue_f);
			break;

			default:
			counterUp_u8 = 0x00;
		}
	}
	else
	{

	}
}
