/*
 * barometer_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <barometer_driver.h>

uint8_t read_Device_Name(uint8_t *ptr)
{

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
    HAL_GPIO_WritePin(LPS25H_CS_Port, LPS25H_CS_Pin, GPIO_PIN_RESET);
}
void _baroChipDisable(void)
{
	HAL_GPIO_WritePin(LPS25H_CS_Port, LPS25H_CS_Pin, GPIO_PIN_SET);
}
