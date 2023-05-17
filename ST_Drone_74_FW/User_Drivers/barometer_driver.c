/*
 * barometer_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <barometer_driver.h>

uint8_t barometer_Init_device(void)
{
    uint8_t ctrl_reg_1 = 0x31;
    uint8_t ctrl_reg_2 = 0x84;
    uint8_t device_name = 0x00;
    /*read device name*/
    while((barometer_Read_Device_Name(&device_name) != 1))
    {
    	/*enable 3wire SPI communication*/
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_1, &ctrl_reg_1, 1);

		/*software reset and memory reboot*/
		barometer_SPI_Read(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
		ctrl_reg_2 = 0x84;
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
		HAL_Delay(1000);

		/*enable 3wire SPI communication*/
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_1, &ctrl_reg_1, 1);

		/*disable i2c*/
		barometer_SPI_Read(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
		ctrl_reg_2 |= 0x08;
		barometer_SPI_Write(&hspi2, BARO_CTRL_REG_2, &ctrl_reg_2, 1);
    }
    return 0;
}

uint8_t barometer_Read_Device_Name(uint8_t *ptr)
{
    uint8_t val = 0;
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
    return val;
}
uint8_t read_Init_Source(void)
{

}
uint8_t read_FIFO_Status(void)
{

}
uint8_t read_Baro_Status(void)
{

}
uint8_t read_Baro_Press_Out_XL(void)
{

}
uint8_t read_Baro_Press_Out_L(void)
{

}
uint8_t read_Baro_Press_Out_H(void)
{

}
uint8_t read_Baro_Temp_Out_L(void)
{

}
uint8_t read_Baro_Temp_Out_H(void)
{

}
uint8_t read_Baro_LPFP_RES(void)
{
    
}
/*PRIVATE FUCNTION*/
uint8_t _SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, Size, 0xFF);
    return 0;
}
uint8_t _SPI_Transmit(uint8_t *pData, uint16_t Size)
{
    HAL_SPI_Transmit(&hspi2, pData, Size, 0xFF);
    return 0;
}
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
