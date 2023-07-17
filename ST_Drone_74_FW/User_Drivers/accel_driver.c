/*
 * accel_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <accel_driver.h>

uint8_t accelGyro_Init_Device(void)
{
	uint8_t device_name = 0x00;
	uint8_t ctrl1_Xl_Register = 0x00;
	uint8_t ctrl2_G_Register = 0x00;
	uint8_t ctrl3_C_Register = 0x00;
	uint8_t ctrl4_C_Register = 0x00;
	uint8_t ctrl5_C_Register = 0x00;
	uint8_t ctrl7_G_Register = 0x00;
	while(accelGyro_Read_Device_Name(&device_name) != 1)
	{
		/*enable 3wire SPI communication*/
		ctrl3_C_Register = 0x08;
		accelGyro_Write_Single_Register(AG_CTRL3_C, &ctrl3_C_Register);
		HAL_Delay(1);
		/*soft reset and memory reboot*/
		ctrl3_C_Register = 0x81;
		accelGyro_Write_Single_Register(AG_CTRL3_C, &ctrl3_C_Register);
		HAL_Delay(1000);
		/*enable 3wire SPI communication*/
		ctrl3_C_Register = 0x08;
		accelGyro_Write_Single_Register(AG_CTRL3_C, &ctrl3_C_Register);
		HAL_Delay(1);
		/*accelerometer ODR*/
		ctrl1_Xl_Register = 0x40; //104Hz normal mode
		accelGyro_Write_Single_Register(AG_CTRL1_XL, &ctrl1_Xl_Register);
		HAL_Delay(1);
		/*gyroscope ODR*/
		ctrl2_G_Register = 0x42; //104Hz normal mode and 125dps
		accelGyro_Write_Single_Register(AG_CTRL2_G, &ctrl2_G_Register);
		HAL_Delay(1);
		/*control register 4 config*/
		accelGyro_Write_Single_Register(AG_CTRL4_C, &ctrl4_C_Register);
		HAL_Delay(1);
		/*control register 5 config*/
		accelGyro_Write_Single_Register(AG_CTRL5_C, &ctrl5_C_Register);
		HAL_Delay(1);
		/*control register 7 config*/
		ctrl7_G_Register = 0x50; //enable HPF
		accelGyro_Write_Single_Register(AG_CTRL7_G, &ctrl7_G_Register);
		HAL_Delay(1);
	}
	return AG_OK;
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

uint8_t accel_Set_Power_Mode(Accel_Odr_Select_e accel_Odr)
{
	uint8_t ctrl1_Xl_Curr_Val=0x00;
	uint8_t stateReturn = AG_OK;
	uint8_t isParaValid = 0;
	//check valid input
	switch(accel_Odr)
	{
		case accel_odr_lv_0:
		isParaValid = 1;
		break;
		case accel_odr_lv_1:
		isParaValid = 1;
		break;
		case accel_odr_lv_2:
		isParaValid = 1;
		break;
		case accel_odr_lv_3:
		isParaValid = 1;
		break;
		case accel_odr_lv_4:
		isParaValid = 1;
		break;
		case accel_odr_lv_5:
		isParaValid = 1;
		break;
		case accel_odr_lv_6:
		isParaValid = 1;
		break;
		case accel_odr_lv_7:
		isParaValid = 1;
		break;
		case accel_odr_lv_8:
		isParaValid = 1;
		break;
		case accel_odr_lv_9:
		isParaValid = 1;
		break;
		case accel_odr_lv_10:
		isParaValid = 1;
		break;
		case accel_odr_lv_11:
		isParaValid = 1;
		break;
		default:
		isParaValid = 0;
	}

	if(isParaValid != 0)
	{
		//read back ctrl1_xl register
		accelGyro_Read_Single_Register(AG_CTRL1_XL, &ctrl1_Xl_Curr_Val);
		//reset previous ODR value
		ctrl1_Xl_Curr_Val &= 0x0F; //00001111
		//asign the fullscale selection
		ctrl1_Xl_Curr_Val |= (((uint8_t)accel_Odr)<<4);
		//wrire fullscale to register
		accelGyro_Write_Single_Register(AG_CTRL1_XL, &ctrl1_Xl_Curr_Val);
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t accel_Set_FullScale(uint8_t fullScale_Selection)
{
	uint8_t ctrl1_Xl_Curr_Val=0x00;
	uint8_t stateReturn = AG_OK;
	//check valid input
	if(fullScale_Selection < 4)
	{
		//read back ctrl1_xl register
		accelGyro_Read_Single_Register(AG_CTRL1_XL, &ctrl1_Xl_Curr_Val);
		//reset previous fullscale value
		ctrl1_Xl_Curr_Val &= 0xF3; //11110011
		//asign the fullscale selection
		ctrl1_Xl_Curr_Val |= (fullScale_Selection << 2);
		//wrire fullscale to register
		accelGyro_Write_Single_Register(AG_CTRL1_XL, &ctrl1_Xl_Curr_Val);
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t accel_X_Out(float *accelReturnValue)
{
	uint8_t stateReturn = AG_OK;
	uint8_t highByte = 0x00;
	uint8_t lowByte = 0x00;
	int16_t accelReturnValue_s16 = 0x00;
	if(accelReturnValue != NULL)
	{
		accelGyro_Read_Single_Register(AG_OUTX_L_XL,  &lowByte);
		accelGyro_Read_Single_Register(AG_OUTX_H_XL,  &highByte);
		accelReturnValue_s16 = ((int16_t)highByte)<<8 | (int16_t)lowByte;
		*accelReturnValue = ((float)accelReturnValue_s16 * ACCEL_SENSITIVITY)/1000.0;
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t accel_Y_Out(float *accelReturnValue)
{
	uint8_t stateReturn = AG_OK;
	uint8_t highByte = 0x00;
	uint8_t lowByte = 0x00;
	int16_t accelReturnValue_s16 = 0x00;
	if(accelReturnValue != NULL)
	{
		accelGyro_Read_Single_Register(AG_OUTY_L_XL,  &lowByte);
		accelGyro_Read_Single_Register(AG_OUTY_H_XL,  &highByte);
		accelReturnValue_s16 = ((int16_t)highByte)<<8 | (int16_t)lowByte;
		*accelReturnValue = ((float)accelReturnValue_s16 * ACCEL_SENSITIVITY)/1000.0;
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t accel_Z_Out(float *accelReturnValue)
{
	uint8_t stateReturn = AG_OK;
	uint8_t highByte = 0x00;
	uint8_t lowByte = 0x00;
	int16_t accelReturnValue_s16 = 0x00;
	if(accelReturnValue != NULL)
	{
		accelGyro_Read_Single_Register(AG_OUTZ_L_XL,  &lowByte);
		accelGyro_Read_Single_Register(AG_OUTZ_H_XL,  &highByte);
		accelReturnValue_s16 = ((int16_t)highByte)<<8 | (int16_t)lowByte;
		*accelReturnValue = ((float)accelReturnValue_s16 * ACCEL_SENSITIVITY)/1000.0;
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t accelGyro_Temp_Out(int16_t *accelTempValue)
{
	uint8_t stateReturn = AG_OK;
	uint8_t highByte = 0x00;
	uint8_t lowByte = 0x00;
	uint16_t rawData = 0x00;
	if(accelTempValue != NULL)
	{
		accelGyro_Read_Single_Register(AG_OUT_TEMP_L,  &lowByte);
		accelGyro_Read_Single_Register(AG_OUT_TEMP_H,  &highByte);
		rawData = ((uint16_t)highByte)<<8 | (uint16_t)lowByte;
		*accelTempValue = (int16_t)((int16_t)rawData / (int16_t)ACCEL_GYRO_TEMP_SENSITIVITY)+25;
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
}

uint8_t accelGyro_Set_FIFO_Mode_Odr(uint8_t *fifoOdr, uint8_t *fifoMode)
{
	uint8_t stateReturn = AG_OK;
	uint8_t txFifoCtrl5 = 0x00;
	if((*fifoOdr < 0x0B) && (*fifoMode < 0x08))
	{
		txFifoCtrl5 |= ((*fifoOdr)<<3) | (*fifoMode);
		accelGyro_Write_Single_Register(AG_FIFO_CTRL_5, &txFifoCtrl5);
	}
	else
	{
		stateReturn = AG_ERROR;
	}
	return stateReturn;
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
