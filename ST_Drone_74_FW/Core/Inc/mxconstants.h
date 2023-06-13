/**
  ******************************************************************************
  * File Name          :
  * Description        :
  ******************************************************************************
  *
  * COPYRIGHT(c) 2023 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANTS_H
#define __MXCONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint8_t size);
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLE_IRQ_Pin             GPIO_PIN_4
#define BLE_IRQ_Port            GPIOA
#define LSM6DS33_CS_Pin         GPIO_PIN_8
#define LSM6DS33_CS_Port        GPIOA
#define BLE_CS_Pin              GPIO_PIN_0
#define BLE_CS_Port             GPIOB
#define BLE_RSTN_Pin            GPIO_PIN_2
#define BLE_RSTN_Port           GPIOB
#define LED3_Pin                GPIO_PIN_5
#define LED3_Port               GPIOB
#define LED2_Pin                GPIO_PIN_4
#define LED2_Port               GPIOB
#define LPS22H_CS_Pin           GPIO_PIN_13
#define LPS22H_CS_Port          GPIOC
#define LIS2MDL_CS_Pin          GPIO_PIN_12
#define LIS2MDL_CS_Port         GPIOB

#define NAME_BLUEMS 'D','R','N','1','1','2','0'
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#define GAP_PERIPHERAL_ROLE_IDB05A1			(0x01)
#define GAP_BROADCASTER_ROLE_IDB05A1		        (0x02)
#define GAP_CENTRAL_ROLE_IDB05A1			(0x04)
#define GAP_OBSERVER_ROLE_IDB05A1			(0x08)
#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

/*BLE define*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}


/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TERM_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_STDERR_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Configuration Service */
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MXCONSTANTS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
