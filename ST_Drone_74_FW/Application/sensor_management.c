/*
 * sensor_management.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <sensor_management.h>

/*Global snesor varibales for BLE updating*/
SensorAxes_t baroBleSentValue_st =
{
	.AXIS_X = 0,
	.AXIS_Y = 0,
	.AXIS_Z = 0
};
/*Global snesor varibales for BLE updating*/
SensorAxes_t magBleSentValue_st =
{
	.AXIS_X = 0,
	.AXIS_Y = 0,
	.AXIS_Z = 0
};
/*Global snesor varibales for BLE updating*/
SensorAxes_t accelBleSentValue_st =
{
	.AXIS_X = 0,
	.AXIS_Y = 0,
	.AXIS_Z = 0
};
/*Global snesor varibales for BLE updating*/
SensorAxes_t gyroBleSentValue_st =
{
	.AXIS_X = 0,
	.AXIS_Y = 0,
	.AXIS_Z = 0
};

void all_Sensor_Init(void)
{
	/*init compass lis2dmdl*/
	compass_Init_Device();
	/*init barometer lps22hd*/
	baro_Init_Device();
	/*init accel gyro lsm6dsl*/
}
