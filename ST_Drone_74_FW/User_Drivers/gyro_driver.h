/*
 * gyro_driver.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#ifndef GYRO_DRIVER_H_
#define GYRO_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "mxconstants.h"
#include <accel_driver.h>

extern SPI_HandleTypeDef hspi2;

#define GYRO_SENSITIVITY               (float)4.375 //mdps/LSB
/*GYROSCOPE OUTPUT VALUE*/
extern uint8_t gyro_X_Out(float *gyroReturnValue);
extern uint8_t gyro_Y_Out(float *gyroReturnValue);
extern uint8_t gyro_Z_Out(float *gyroReturnValue);


#endif /* GYRO_DRIVER_H_ */
