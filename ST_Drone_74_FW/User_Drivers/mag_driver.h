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

/*COMPASS CONTROL FUNCTION*/
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint8_t size);

void Compass_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size );
void Compass_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size );

#endif /* MAG_DRIVER_H_ */
