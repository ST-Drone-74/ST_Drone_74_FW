/*
 * mag_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <mag_driver.h>

void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */

  __disable_irq();
  //GPIOA->BSRR = (uint32_t)GPIO_PIN_8 << 16U;
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);

  __enable_irq();

  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;

  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_SR_TXE) != SPI_SR_TXE);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint8_t size)
{
  /* Interrupts should be disabled during this operation */
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);

  /* Transfer loop */
  while (size > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      size--;
    }
  }
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
