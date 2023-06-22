/*
 * SBTLE_RF.h
 *
 *  Created on: June 12, 2023
 *      Author: Mai Huynh Long Nhan
 */

#ifndef __STM32_BLUENRG_BLE_H
#define __STM32_BLUENRG_BLE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/ 
#include "stm32f4xx_hal.h"
#include <mxconstants.h>
#include "ble_status.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_utils.h"
#include "sm.h"
#include "bluenrg_l2cap_aci.h"

 extern volatile uint32_t HCI_ProcessEvent;

// SPI Instance
#define BNRG_SPI_INSTANCE           SPI1
#define BNRG_SPI_CLK_ENABLE()       __SPI1_CLK_ENABLE()
#define SYSCLK_FREQ 80000000

// SPI Configuration
#define BNRG_SPI_MODE               SPI_MODE_MASTER
#define BNRG_SPI_DIRECTION          SPI_DIRECTION_2LINES
#define BNRG_SPI_DATASIZE           SPI_DATASIZE_8BIT
#define BNRG_SPI_CLKPOLARITY        SPI_POLARITY_LOW
#define BNRG_SPI_CLKPHASE           SPI_PHASE_1EDGE
#define BNRG_SPI_NSS                SPI_NSS_SOFT
#define BNRG_SPI_FIRSTBIT           SPI_FIRSTBIT_MSB
#define BNRG_SPI_TIMODE             SPI_TIMODE_DISABLED
#define BNRG_SPI_CRCPOLYNOMIAL      7
#define BNRG_SPI_BAUDRATEPRESCALER  SPI_BAUDRATEPRESCALER_16
#define BNRG_SPI_CRCCALCULATION     SPI_CRCCALCULATION_DISABLED

typedef enum
 {
   COMPONENT_OK = 0,
   COMPONENT_ERROR,
   COMPONENT_TIMEOUT,
   COMPONENT_NOT_IMPLEMENTED
 } DrvStatusTypeDef;

extern void Enable_SPI_IRQ(void);
extern void Disable_SPI_IRQ(void);
extern void Clear_SPI_IRQ(void);
extern void Clear_SPI_EXTI_Flag(void);

extern void BNRG_SPI_Init(void);
extern void BlueNRG_RST(void);
extern uint8_t BlueNRG_DataPresent(void);
extern void    BlueNRG_HW_Bootloader(void);
extern int32_t BlueNRG_SPI_Read_All(SPI_HandleTypeDef *hspi,
                             uint8_t *buffer,
                             uint8_t buff_size);
extern int32_t BlueNRG_SPI_Write(SPI_HandleTypeDef *hspi,
                          uint8_t* data1,
                          uint8_t* data2,
                          uint8_t Nb_bytes1,
                          uint8_t Nb_bytes2);

extern void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,
                      int32_t n_bytes2);

extern void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi);
extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
extern void BlueNRG_Init(void);
extern void Init_BlueNRG_Custom_Services(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_BLUENRG_BLE_H */

