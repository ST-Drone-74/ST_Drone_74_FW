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
uint32_t vBat_ADC_Value(void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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

/*  SPBTLE-RF pin define*/
#define BNRG_SPI_IRQ_PIN            GPIO_PIN_4
#define BNRG_SPI_IRQ_MODE           GPIO_MODE_IT_RISING
#define BNRG_SPI_IRQ_PULL           GPIO_NOPULL
#define BNRG_SPI_IRQ_SPEED          GPIO_SPEED_HIGH
#define BNRG_SPI_IRQ_ALTERNATE      0
#define BNRG_SPI_IRQ_PORT           GPIOA
#define BNRG_SPI_IRQ_CLK_ENABLE()   __GPIOA_CLK_ENABLE()

#define BNRG_SPI_EXTI_IRQn          EXTI4_IRQn
#define BNRG_SPI_EXTI_PIN           BNRG_SPI_IRQ_PIN
#define BNRG_SPI_EXTI_PORT          BNRG_SPI_IRQ_PORT

/*BLE define*/
#define NAME_BLUEMS 'D','R','N','1','1','2','0'
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#define GAP_PERIPHERAL_ROLE_IDB05A1			(0x01)
#define GAP_BROADCASTER_ROLE_IDB05A1		        (0x02)
#define GAP_CENTRAL_ROLE_IDB05A1			(0x04)
#define GAP_OBSERVER_ROLE_IDB05A1			(0x08)
#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

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
#define CONFIG_DATA_PUBADDR_OFFSET          (0x00) /**< Bluetooth public address */
#define CONFIG_DATA_DIV_OFFSET              (0x06) /**< DIV used to derive CSRK */
#define CONFIG_DATA_ER_OFFSET               (0x08) /**< Encryption root key used to derive LTK and CSRK */
#define CONFIG_DATA_IR_OFFSET               (0x18) /**< Identity root key used to derive LTK and CSRK */
#define CONFIG_DATA_LL_WITHOUT_HOST         (0x2C) /**< Switch on/off Link Layer only mode. Set to 1 to disable Host.
 	 	 	 	 	 	 	 	 	 	 	 	 	 It can be written only if aci_hal_write_config_data() is the first command 	 	 	 	 	 	 	 	 	 	 	 	 after reset. */
#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */
/**
 * @name Length for configuration values.
 * See @ref aci_hal_write_config_data().
 * @{
 */
#define CONFIG_DATA_PUBADDR_LEN             (6)
#define CONFIG_DATA_DIV_LEN                 (2)
#define CONFIG_DATA_ER_LEN                  (16)
#define CONFIG_DATA_IR_LEN                  (16)
#define CONFIG_DATA_LL_WITHOUT_HOST_LEN     (1)
#define CONFIG_DATA_MODE_LEN                (1)
#define CONFIG_DATA_WATCHDOG_DISABLE_LEN    (1)

/**
 * Select the BlueNRG mode configurations.\n
 * @li Mode 1: slave or master, 1 connection, RAM1 only (small GATT DB)
 * @li Mode 2: slave or master, 1 connection, RAM1 and RAM2 (large GATT DB)
 * @li Mode 3: master/slave, 8 connections, RAM1 and RAM2.
 * @li Mode 4: master/slave, 4 connections, RAM1 and RAM2 simultaneous scanning and advertising.
 */
#define CONFIG_DATA_MODE_OFFSET 			(0x2D)

#define CONFIG_DATA_WATCHDOG_DISABLE 		(0x2F) /**< Set to 1 to disable watchdog. It is enabled by default. */

/**
 * @anchor Auth_req
 * @name Authentication requirements
 * @{
 */
#define BONDING				            (0x01)
#define NO_BONDING				        (0x00)
/**
 * @}
 */

/**
 * @anchor MITM_req
 * @name MITM protection requirements
 * @{
 */
#define MITM_PROTECTION_NOT_REQUIRED	(0x00)
#define MITM_PROTECTION_REQUIRED        (0x01)
/**
 * @}
 */

/**
 * @anchor OOB_Data
 * @name Out-Of-Band data
 * @{
 */
#define OOB_AUTH_DATA_ABSENT		    (0x00)
#define OOB_AUTH_DATA_PRESENT      		(0x01)
/**
 * @}
 */
/**
 * @anchor Use_fixed_pin
 * @name Use fixed pin
 * @{
 */
#define USE_FIXED_PIN_FOR_PAIRING		(0x00)
#define DONOT_USE_FIXED_PIN_FOR_PAIRING	(0x01)
/**
 * @}
 */


//static int HCI_ProcessEvent=0;
//static volatile uint32_t HCI_ProcessEvent=0;
extern volatile uint32_t HCI_ProcessEvent;
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};


/* USER CODE END Private defines */

/**
  * @}
  */

/**
  * @}
*/

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MXCONSTANTS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
