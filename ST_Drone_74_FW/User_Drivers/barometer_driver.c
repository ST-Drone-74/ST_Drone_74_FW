/*
 * barometer_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <barometer_driver.h>

uint8_t baro_Init_Device(void)
{
    uint8_t ctrl_reg_1 = 0x31;
    uint8_t ctrl_reg_2 = 0x84;
    uint8_t device_name = 0x00;
	Fifo_Mode_e  set_Fifo_Mode = BYPASS;
	uint8_t set_Level = 0x20;
    /*read device name*/
    while((baro_Read_Device_Name(&device_name) != 1))
    {
    	/*enable 3wire SPI communication*/
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_1, &ctrl_reg_1, 1);
		HAL_Delay(1);
		/*software reset and memory reboot*/
		barometer_SPI_Read(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
		ctrl_reg_2 = 0x84;
		HAL_Delay(1);
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
		HAL_Delay(1000);

		/*enable 3wire SPI communication*/
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_1, &ctrl_reg_1, 1);
		HAL_Delay(1);
		/*disable i2c*/
		barometer_SPI_Read(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
		HAL_Delay(1);
		ctrl_reg_2 |= 0x08;
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
		HAL_Delay(1);
		/*set FIFO mode when init*/
		baro_Set_FIFO_Mode(set_Fifo_Mode, set_Level);
		HAL_Delay(1);
    }
    return 0;
}

uint8_t baro_Set_FIFO_Mode(Fifo_Mode_e set_Mode, uint8_t set_level)
{
	uint8_t reg_val = 0x00;
	/*check unvalid watermak*/
	if(set_level > 32)
	{
		set_level = 0;
	}
	else
	{
		/*write water mark level*/
		barometer_SPI_Write(&hspi2, BARO_FIFO_CTRL, &set_level, 1);
	}
	/*read back FIFO control register*/
	barometer_SPI_Read(&hspi2, BARO_FIFO_CTRL, &reg_val, 1);
	/*reset 3 BIT FIFO mode into 000*/
	reg_val &= 0x1F;
	/*reset FIFO mode by set to BYPASS*/
	barometer_SPI_Write(&hspi2, BARO_FIFO_CTRL, &reg_val, 1);
	switch(set_Mode)
	{
		case BYPASS:
		reg_val |= BYPASS;
    	break;
		
		case FIFO:
		reg_val |= FIFO;
		break;
    	
		case STREAM:
		reg_val |= STREAM;
		break;
    	
		case DYNAMIC_STREAM:
		reg_val |= DYNAMIC_STREAM;
		break;
    	
		case STREAM_2_FIFO:
		reg_val |= STREAM_2_FIFO;
    	break;
		
		case BYPASS_2_STREAM:
		reg_val |= BYPASS_2_STREAM;
    	break;
		
		case BYPASS_2_FIFO:
		break;
		
		default:
		break;
	}
	/*set fifo mode*/
	barometer_SPI_Write(&hspi2, BARO_FIFO_CTRL, &reg_val, 1);
	return SENSOR_OK;
}

void baro_Reset_FIFO(void)
{
	uint8_t reg_val = 0x00;
	/*read back FIFO control register*/
	barometer_SPI_Read(&hspi2, BARO_FIFO_CTRL, &reg_val, 1);
	/*reset 3 BIT FIFO mode into 000*/
	reg_val &= 0x1F;
	/*reset FIFO mode by set to BYPASS*/
	barometer_SPI_Write(&hspi2, BARO_FIFO_CTRL, &reg_val, 1);
}

uint8_t baro_Read_Pressure(uint32_t *rxData)
{
	uint8_t val = SENSOR_OK;
	uint8_t rx_Data[3] = {};
	if(rxData != NULL)
	{
		switch(baro_Read_Current_FIFO_Mode())
		{
			case BYPASS:
				barometer_SPI_Read(&hspi2, BARO_PRESS_OUT_XL, rx_Data, sizeof(rx_Data)+1);
				(*rxData) = ((uint32_t)rx_Data[2])<<16 |
							((uint32_t)rx_Data[1])<<8  |
							(uint32_t)rx_Data[0];
			break;
			
			case FIFO:
			break;
			
			case STREAM:
			break;
			
			case DYNAMIC_STREAM:
			break;
			
			default:
				barometer_SPI_Read(&hspi2, BARO_PRESS_OUT_XL, rx_Data, sizeof(rx_Data)+1);
				(*rxData) = ((uint32_t)rx_Data[2])<<16 |
							((uint32_t)rx_Data[1])<<8  |
							(uint32_t)rx_Data[0];
			break;
		}
	}
	else
	{
		val = SENSOR_ERROR;
	}
	return val;
}

uint8_t baro_HPA_Pressure(float *pressure)
{
	uint8_t val = SENSOR_OK;
	uint32_t rawPressure=0x00;
	if(pressure != NULL)
	{
		baro_Read_Pressure(&rawPressure);
		(*pressure) = (float)(((float)rawPressure)/(float)BARO_SCALING_FACTOR);
	}
	else
	{
		val = SENSOR_ERROR;
	}
	return val;
}

uint8_t baro_Read_Temperature(int16_t *rxData)
{
	uint8_t val = SENSOR_OK;
	uint8_t rx_Data[2] = {};
	if(rxData != NULL)
	{
		barometer_SPI_Read(&hspi2, BARO_TEMP_OUT_L, rx_Data, sizeof(rx_Data)+1);
		(*rxData) = ((int16_t)rx_Data[1])<<8 | (int16_t)rx_Data[0];
	}
	else
	{
		val = SENSOR_ERROR;
	}
	return val;
}

Fifo_Mode_e baro_Read_Current_FIFO_Mode(void)
{
	uint8_t fifo_Ctrl_Reg = 0x00;
	Fifo_Mode_e fifo_Mode = BYPASS;
	/*read FIFO_CTRL_REG*/
	baro_read_Single_Register(BARO_FIFO_CTRL, &fifo_Ctrl_Reg);
	fifo_Ctrl_Reg = fifo_Ctrl_Reg >> 5;
	/*Compare and return mode of FIFO buffer*/
	if(fifo_Ctrl_Reg == 0x00)
	{
		fifo_Mode = BYPASS;
	}
	else if (fifo_Ctrl_Reg == 0x01)
	{
		fifo_Mode = FIFO;
	}
	else if (fifo_Ctrl_Reg == 0x02)
	{
		fifo_Mode = STREAM;
	}
	else if (fifo_Ctrl_Reg == 0x03)
	{
		fifo_Mode = STREAM_2_FIFO;
	}
	else if (fifo_Ctrl_Reg == 0x04)
	{
		fifo_Mode = BYPASS_2_STREAM;
	}
	else if (fifo_Ctrl_Reg == 0x06)
	{
		fifo_Mode = DYNAMIC_STREAM;
	}
	else if (fifo_Ctrl_Reg == 0x07)
	{
		fifo_Mode = BYPASS_2_FIFO;
	}
	else
	{
		/*nothing mode*/
	}
	return fifo_Mode;
}

uint8_t baro_Read_Device_Name(uint8_t *ptr)
{
    uint8_t val = 0;
	if(ptr != NULL)
	{
		barometer_SPI_Read(&hspi2, BARO_WHO_I_AM, ptr, 1);
		if((*ptr) == BARO_DEVICE_NAME)
		{
			val = 1;
		}
		else
		{
			/*read device name fail*/
			val = 0;
		}
	}
	else
	{
		val = 0;
	}
    return val;
}
uint8_t baro_read_Init_Source(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_INT_SOURCE, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_FIFO_Status(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_FIFO_STATUS, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_Baro_Status(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_STATUS, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_Baro_Press_Out_XL(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_PRESS_OUT_XL, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_Baro_Press_Out_L(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_PRESS_OUT_L, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_Baro_Press_Out_H(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_PRESS_OUT_H, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_Baro_Temp_Out_L(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_TEMP_OUT_L, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_Baro_Temp_Out_H(void)
{
	uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_TEMP_OUT_H, &reg_return, 1);
	return reg_return;
}
uint8_t baro_read_Baro_LPFP_RES(void)
{
    uint8_t reg_return = 0x00;
	barometer_SPI_Read(&hspi2, BARO_LPFP_RES, &reg_return, 1);
	return reg_return;
}
/*PRIVATE FUCNTION*/
void _baroChipEnable(void)
{
    HAL_GPIO_WritePin(LPS22H_CS_Port, LPS22H_CS_Pin, GPIO_PIN_RESET);
}
void _baroChipDisable(void)
{
	HAL_GPIO_WritePin(LPS22H_CS_Port, LPS22H_CS_Pin, GPIO_PIN_SET);
}
void barometer_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size )
{
    /*chip select*/
	_baroChipEnable();
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
	_baroChipDisable();

	SPI_1LINE_TX(xSpiHandle);
	__HAL_SPI_ENABLE(xSpiHandle);
}
void barometer_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size )
{
    /*chip select*/
	_baroChipEnable();

	SPI_Write(xSpiHandle, WriteAddr);

	for(uint8_t i=0;i<size;i++)
	{
		SPI_Write(xSpiHandle, pBuffer[i]);
	}
	/*chip deselect*/
    _baroChipDisable();
}

uint8_t baro_write_Single_Register(uint8_t address, uint8_t *txData)
{
	Baro_State_e val = SENSOR_OK;
	if((txData != NULL) && (address != 0x00))
	{
		barometer_SPI_Write(&hspi2, address, txData, 1);
		val = SENSOR_OK;
	}
	else
	{
		val = SENSOR_ERROR;
	}
	return val;
}
uint8_t baro_read_Single_Register(uint8_t address, uint8_t *rxData)
{
	Baro_State_e val = SENSOR_OK;
	if((rxData != NULL) && (address != 0x00))
	{
		barometer_SPI_Read(&hspi2, address, rxData, 1);
		val = SENSOR_OK;
	}
	else
	{
		val = SENSOR_ERROR;
	}
	return val;
}
