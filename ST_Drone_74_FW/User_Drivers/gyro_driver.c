/*
 * gyro_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <gyro_driver.h>

uint8_t gyro_X_Out(float *gyroReturnValue)
{
    uint8_t stateReturn = AG_OK;
	uint8_t highByte = 0x00;
	uint8_t lowByte = 0x00;
	int16_t gyroReturnValue_s16=0x00;
	if(gyroReturnValue != NULL)
	{
		accelGyro_Read_Single_Register(AG_OUTX_L_G,  &lowByte);
		accelGyro_Read_Single_Register(AG_OUTX_H_G,  &highByte);
		gyroReturnValue_s16 = ((int16_t)highByte)<<8 | (int16_t)lowByte;
		*gyroReturnValue = ((float)gyroReturnValue_s16 * GYRO_SENSITIVITY)/1000.0;
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t gyro_Y_Out(float *gyroReturnValue)
{
    uint8_t stateReturn = AG_OK;
	uint8_t highByte = 0x00;
	uint8_t lowByte = 0x00;
	int16_t gyroReturnValue_s16=0x00;
	if(gyroReturnValue != NULL)
	{
		accelGyro_Read_Single_Register(AG_OUTY_L_G,  &lowByte);
		accelGyro_Read_Single_Register(AG_OUTY_H_G,  &highByte);
		gyroReturnValue_s16 = ((int16_t)highByte)<<8 | (int16_t)lowByte;
		*gyroReturnValue = ((float)gyroReturnValue_s16 * GYRO_SENSITIVITY)/1000.0;
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t gyro_Z_Out(float *gyroReturnValue)
{
    uint8_t stateReturn = AG_OK;
	uint8_t highByte = 0x00;
	uint8_t lowByte = 0x00;
	int16_t gyroReturnValue_s16=0x00;
	if(gyroReturnValue != NULL)
	{
		accelGyro_Read_Single_Register(AG_OUTZ_L_G,  &lowByte);
		accelGyro_Read_Single_Register(AG_OUTZ_H_G,  &highByte);
		gyroReturnValue_s16 = ((int16_t)highByte)<<8 | (int16_t)lowByte;
		*gyroReturnValue = ((float)gyroReturnValue_s16 * GYRO_SENSITIVITY)/1000.0;
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}