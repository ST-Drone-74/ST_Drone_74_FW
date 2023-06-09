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
#define MAG_TEMP_SENSITIVITY    (float)8.0 // LSB/*C
#define MAG_DATA_SENSITIVITY    (float)1.5 // mgauss/LSB
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
#define MAG_CFG_REG_A           0x60
#define MAG_CFG_REG_B           0x61
#define MAG_CFG_REG_C           0x62
#define MAG_INIT_CTRL			0x63
#define MAG_INT_THS_L			0x65
#define MAG_INT_THS_H			0x66

typedef enum 
{
    COMP_ERROR,
    COMP_OK
}Compass_State_e;

#endif /* MAGNETO_LIS2MDL */

/*COMPASS CONTROL FUNCTION*/
uint8_t compass_Init_Device(void);
uint8_t compass_Read_Device_Name(uint8_t *ptr);
uint8_t compass_Read_Device_Status(void);

/*MAGNETIC AND TEMPERATURE DATA*/
uint8_t compass_Read_X_Data(float *ptr);
uint8_t compass_Read_Y_Data(float *ptr);
uint8_t compass_Read_Z_Data(float *ptr);
uint8_t compass_Read_Temp_Out_L(void);
uint8_t compass_Read_Temp_Out_H(void);
uint8_t compass_Read_Temperature(float *ptr);
uint8_t compass_Read_All_Offset(uint8_t *rxPtr);

/*RW SINGLE REGISTER*/
uint8_t compass_Write_Single_Register(uint8_t address, uint8_t *txData);
uint8_t compass_Read_Single_Register(uint8_t address, uint8_t *rxData);

/*BASIC CONTROL*/
void compassEnable(void);
void compassDisable(void);
void compass_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size );
void compass_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size );

#endif /* MAG_DRIVER_H_ */
