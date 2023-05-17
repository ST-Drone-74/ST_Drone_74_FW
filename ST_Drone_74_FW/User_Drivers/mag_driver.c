/*
 * mag_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <mag_driver.h>

uint8_t compass_Read_Device_Name(uint8_t *ptr)
{
    uint8_t val = 0;
    Compass_SPI_Read(&hspi2, MAG_WHO_I_AM, ptr, 1);
    if((*ptr) == MAG_DEVICE_NAME)
    {
        val = 1;
    }
    else
    {
        /*read device name fail*/
        val = 0;
    }
    return val;
}

void Compass_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size )
{
	/*chip select*/
	HAL_GPIO_WritePin(LIS2MDL_CS_Port, LIS2MDL_CS_Pin, GPIO_PIN_RESET);
	/*write data*/
	SPI_Write(xSpiHandle, ReadAddr | 0x80);

	__HAL_SPI_DISABLE(xSpiHandle);
	SPI_1LINE_RX(xSpiHandle);

	/*read data*/
	if(size > 1U)
	{
		SPI_Read_nBytes(xSpiHandle, pBuffer, size);
	}
	else
	{
		SPI_Read(xSpiHandle, pBuffer);
	}
	/*chip deselect*/
	HAL_GPIO_WritePin(LIS2MDL_CS_Port, LIS2MDL_CS_Pin, GPIO_PIN_SET);

	SPI_1LINE_TX(xSpiHandle);
	__HAL_SPI_ENABLE(xSpiHandle);
}

void Compass_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size )
{
	/*chip select*/
	HAL_GPIO_WritePin(LIS2MDL_CS_Port, LIS2MDL_CS_Pin, GPIO_PIN_RESET);

	SPI_Write(xSpiHandle, WriteAddr);

	for(uint8_t i=0;i<size;i++)
	{
		SPI_Write(xSpiHandle, pBuffer[i]);
	}
	/*chip deselect*/
	HAL_GPIO_WritePin(LIS2MDL_CS_Port, LIS2MDL_CS_Pin, GPIO_PIN_SET);
}
