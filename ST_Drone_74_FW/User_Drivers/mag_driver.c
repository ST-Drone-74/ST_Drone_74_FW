/*
 * mag_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <mag_driver.h>

/**
 * @brief initalize the compass, implement soft-reset then reboot the memory
 * config register A, B and C to enable temperature compensation, ODR
 * @return sequence state 
 */
uint8_t compass_Init_Device(void)
{
	uint8_t device_name = 0x00;
	uint8_t set_Cfg_Reg_A = 0x00;
	uint8_t set_Cfg_Reg_B = 0x00;
	uint8_t set_Cfg_Reg_C = 0x00;
	uint8_t offset_Value[6] = {0x00};
	while((compass_Read_Device_Name(&device_name) != 1))
	{
		/*soft reset and memory reboot*/
		set_Cfg_Reg_A = 0x60;
		compass_Write_Single_Register(MAG_CFG_REG_A, &set_Cfg_Reg_A);
		HAL_Delay(1000);
		/*set config register A*/
		set_Cfg_Reg_A = 0x88; //0b1000 1000
		compass_Write_Single_Register(MAG_CFG_REG_A, &set_Cfg_Reg_A);
		HAL_Delay(1);
		/*set config register B*/
		set_Cfg_Reg_B = 0x03; //0b0000 0011
		compass_Write_Single_Register(MAG_CFG_REG_B, &set_Cfg_Reg_B);
		HAL_Delay(1);
		/*set config register C*/
		set_Cfg_Reg_C = 0x10;//0b0001 0000
		compass_Write_Single_Register(MAG_CFG_REG_C, &set_Cfg_Reg_C);
		HAL_Delay(1);
		/*write sensor offset*/
		for(uint8_t i = 0; i<6; i++)
		{
			compass_Write_Single_Register(MAG_OFFSET_X_L + i, &offset_Value[i]);
		}
		/*Offset register*/
	}
	return COMP_OK;
}

/**
 * @brief read default device name
 * @param ptr store device name value
 * @return device name matching 
 */
uint8_t compass_Read_Device_Name(uint8_t *ptr)
{
    uint8_t val = 0;
    compass_SPI_Read(&hspi2, MAG_WHO_I_AM, ptr, 1);
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

/**
 * @brief read device status
 * @return register's value 
 */
uint8_t compass_Read_Device_Status(void)
{
	uint8_t rxBuffer = 0x00;
	compass_Read_Single_Register(MAG_STATUS, &rxBuffer);
	return rxBuffer;
}

/**
 * @brief read X magnetic data
 * @param ptr store register raw value
 * @return sequence state 
 */
uint8_t compass_Read_X_Data(float *ptr)
{
	uint8_t val = COMP_OK;
	uint8_t rx_Data[2] = {};
	int16_t rawData = 0x00;
	if(ptr != NULL)
	{
		compass_SPI_Read(&hspi2, MAG_OUTX_L, rx_Data, sizeof(rx_Data)+1);
		rawData = ((int16_t)rx_Data[1])<<8 | (int16_t)rx_Data[0];
		(*ptr) = (float)((float)(rawData)*MAG_DATA_SENSITIVITY);
	}
	else
	{
		val = COMP_ERROR;
	}
	return val;
}

/**
 * @brief read Y magnetic data
 * @param ptr store register raw value
 * @return sequence state 
 */
uint8_t compass_Read_Y_Data(float *ptr)
{
	uint8_t val = COMP_OK;
	uint8_t rx_Data[2] = {};
	int16_t rawData = 0x00;
	if(ptr != NULL)
	{
		compass_SPI_Read(&hspi2, MAG_OUTY_L, rx_Data, sizeof(rx_Data)+1);
		rawData = ((int16_t)rx_Data[1])<<8 | (int16_t)rx_Data[0];
		(*ptr) = (float)((float)(rawData)*MAG_DATA_SENSITIVITY);
	}
	else
	{
		val = COMP_ERROR;
	}
	return val;
}

/**
 * @brief read Z magnetic data
 * @param ptr store register raw value
 * @return sequence state 
 */
uint8_t compass_Read_Z_Data(float *ptr)
{
	uint8_t val = COMP_OK;
	uint8_t rx_Data[2] = {};
	int16_t rawData = 0x00;
	if(ptr != NULL)
	{
		compass_SPI_Read(&hspi2, MAG_OUTZ_L, rx_Data, sizeof(rx_Data)+1);
		rawData = ((int16_t)rx_Data[1])<<8 | (int16_t)rx_Data[0];
		(*ptr) = (float)((float)(rawData)*MAG_DATA_SENSITIVITY);
	}
	else
	{
		val = COMP_ERROR;
	}
	return val;
}

/**
 * @brief temperature raw value from lower byte
 * @return raw data 
 */
uint8_t compass_Read_Temp_Out_L(void)
{
	uint8_t rxData = 0x00;
	compass_Read_Single_Register(MAG_TEMP_OUT_L, &rxData);
	return rxData;
}

/**
 * @brief temperature raw value from upper byte
 * @return raw data 
 */
uint8_t compass_Read_Temp_Out_H(void)
{
	uint8_t rxData = 0x00;
	compass_Read_Single_Register(MAG_TEMP_OUT_H, &rxData);
	return rxData;
}

/**
 * @brief read and combine raw temperature data
 * @param ptr store temperature value
 * @return sequence state 
 */
uint8_t compass_Read_Temperature(float *ptr)
{
	uint8_t val = COMP_OK;
	uint8_t rx_Data[2] = {};
	int16_t rawData = 0x00;
	if(ptr != NULL)
	{
		compass_SPI_Read(&hspi2, MAG_TEMP_OUT_L, rx_Data, sizeof(rx_Data)+1);
		rawData = ((int16_t)rx_Data[1])<<8 | (int16_t)rx_Data[0];
		(*ptr) = (float)((float)(rawData)/MAG_TEMP_SENSITIVITY);
		val = COMP_OK;
	}
	else
	{
		val = COMP_ERROR;
	}
	return val;
}

/**
 * @brief read offset registers
 * @param rxPtr store read value
 * @return sequence state  
 */
uint8_t compass_Read_All_Offset(uint8_t *rxPtr)
{
	uint8_t val = COMP_OK;
	if(rxPtr != NULL)
	{
		compass_SPI_Read(&hspi2, MAG_OFFSET_X_L, rxPtr, 7);
		val = COMP_OK;
	}
	else
	{
		val = COMP_ERROR;
	}
	return val;
}

/**
 * @brief write to single register
 * @param address accessed register's address
 * @param txData data will be written
 * @return sequence state  
 */
uint8_t compass_Write_Single_Register(uint8_t address, uint8_t *txData)
{
	Compass_State_e val = COMP_OK;
	if((txData != NULL) && (address != 0x00))
	{
		compass_SPI_Write(&hspi2, address, txData, 1);
		val = COMP_OK;
	}
	else
	{
		val = COMP_ERROR;
	}
	return val;
}

/**
 * @brief read from single register
 * @param address accessed register's address
 * @param rxData data read from register
 * @return sequence state 
 */
uint8_t compass_Read_Single_Register(uint8_t address, uint8_t *rxData)
{
	Compass_State_e val = COMP_OK;
	if((rxData != NULL) && (address != 0x00))
	{
		compass_SPI_Read(&hspi2, address, rxData, 1);
		val = COMP_OK;
	}
	else
	{
		val = COMP_ERROR;
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
void compass_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size )
{
	/*chip select*/
	compassEnable();
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
	compassDisable();

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
void compass_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size )
{
	/*chip select*/
	compassEnable();

	SPI_Write(xSpiHandle, WriteAddr);

	for(uint8_t i=0;i<size;i++)
	{
		SPI_Write(xSpiHandle, pBuffer[i]);
	}
	/*chip deselect*/
	compassDisable();
}

/**
 * @brief enable device's SPI
 */
void compassEnable(void)
{
	HAL_GPIO_WritePin(LIS2MDL_CS_Port, LIS2MDL_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief disable device's SPI
 */
void compassDisable(void)
{
	HAL_GPIO_WritePin(LIS2MDL_CS_Port, LIS2MDL_CS_Pin, GPIO_PIN_SET);
}
