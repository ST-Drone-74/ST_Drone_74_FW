/*
 * accel_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <accel_driver.h>

uint8_t accelGyro_Init_Device(void)
{

}

uint8_t accelGyro_Read_Device_Name(uint8_t *ptr)
{
	AccelGyro_State_e val = AG_OK;
    accelGyro_Read_Single_Register(AG_WHO_AM_I, ptr);
	if((*ptr) == ACCEL_GYRO_DEVICE_NAME)
	{
		val = AG_OK;
	}
	else
	{
		val = AG_ERROR;/*read device name fail*/
	}
	return val;
}

/**
 * @brief write to single register
 * @param address accessed register's address
 * @param txData data will be written
 * @return sequence state  
 */
uint8_t accelGyro_Write_Single_Register(uint8_t address, uint8_t *txData)
{
	AccelGyro_State_e val = AG_OK;
	if((txData != NULL) && (address != 0x00))
	{
		accelGyro_SPI_Write(&hspi2, address, txData, 1);
		val = AG_OK;
	}
	else
	{
		val = AG_ERROR;
	}
	return val;
}

/**
 * @brief read from single register
 * @param address accessed register's address
 * @param rxData data read from register
 * @return sequence state 
 */
uint8_t accelGyro_Read_Single_Register(uint8_t address, uint8_t *rxData)
{
	AccelGyro_State_e val = AG_OK;
	if((rxData != NULL) && (address != 0x00))
	{
		accelGyro_SPI_Read(&hspi2, address, rxData, 1);
		val = AG_OK;
	}
	else
	{
		val = AG_ERROR;
	}
	return val;
}

/**
 * @brief SPI read function
 * @param xSpiHandle point to SPI configuration
 * @param ReadAddr register will be accessed
 * @param pBuffer store read data
 * @param size how many bytes will be read
 */
void accelGyro_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size )
{
	/*chip select*/
	accelGyroEnable();
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
	accelGyroDisable();

	SPI_1LINE_TX(xSpiHandle);
	__HAL_SPI_ENABLE(xSpiHandle);
}

/**
 * @brief SPI write fucntion
 * @param xSpiHandle point to SPI configuration
 * @param WriteAddr register will be accessed
 * @param pBuffer data which will be written to register
 * @param size size of written data
 */
void accelGyro_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size )
{
	/*chip select*/
	accelGyroEnable();

	SPI_Write(xSpiHandle, WriteAddr);

	for(uint8_t i=0;i<size;i++)
	{
		SPI_Write(xSpiHandle, pBuffer[i]);
	}
	/*chip deselect*/
	accelGyroDisable();
}

/**
 * @brief enable device's SPI
 */
void accelGyroEnable(void)
{
	HAL_GPIO_WritePin(LSM6DS33_CS_Port, LSM6DS33_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief disable device's SPI
 */
void accelGyroDisable(void)
{
	HAL_GPIO_WritePin(LSM6DS33_CS_Port, LSM6DS33_CS_Pin, GPIO_PIN_SET);
}
