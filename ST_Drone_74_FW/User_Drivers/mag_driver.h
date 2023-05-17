/*
 * mag_driver.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */

#ifndef MAG_DRIVER_H_
#define MAG_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "mxconstants.h"

extern SPI_HandleTypeDef hspi2;

/*TYPE OF MAGNETO DEFINE*/
#define MAGNETO_LIS2MDL

#ifdef MAGNETO_LIS2MDL

#define MAG_DEVICE_NAME         0x40
/*LIS2MDL REGISTER - READ ONLY*/
#define MAG_WHO_I_AM			0x4F
#define MAG_INT_SOURCE			0x64
#define MAG_STATUS 				0x67
#define MAG_OUTX_L				0x68
#define MAG_OUTX_H				0x69
#define MAG_OUTY_L				0x6A
#define MAG_OUTY_H				0x6B
#define MAG_OUTZ_L				0x6C
#define MAG_OUTZ_H				0x6D
#define MAG_TEMP_OUT_L			0x6E
#define MAG_TEMP_OUT_H			0X6F

/*LIS2MDL REGISTER - R/W*/
#define MAG_OFFSET_X_L			0x45
#define MAG_OFFSET_X_H			0x46
#define MAG_OFFSET_Y_L			0x47
#define MAG_OFFSET_Y_H			0x48
#define MAG_OFFSET_Z_L			0x49
#define MAG_OFFSET_Z_H			0x4A
#define MAG_CTRL_REG_A          0x10
#define MAG_CTRL_REG_B          0x11
#define MAG_CTRL_REG_C          0x13
#define MAG_INIT_CTRL			0x63
#define MAG_INT_THS_L			0x65
#define MAG_INT_THS_H			0x66

#endif /* MAGNETO_LIS2MDL */

/*COMPASS CONTROL FUNCTION*/
uint8_t compass_Read_Device_Name(uint8_t *ptr);
void Compass_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size );
void Compass_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size );

#endif /* MAG_DRIVER_H_ */
