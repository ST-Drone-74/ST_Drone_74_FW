/*
 * barometer_driver.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include "stm32f4xx_hal.h"
#include "mxconstants.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

#ifndef BAROMETER_DRIVER_H_
#define BAROMETER_DRIVER_H_

/*TYPE OF BAROMETER DEFINE*/
#define BAROMETER_LPS22HD

#ifdef BAROMETER_LPS22HD
/*LPS22HD COMMUNICATION*/
#define BARO_I2C_ADDRESS_1          0x5D
#define BARO_I2C_ADDRESS_2          0x5C
/*LPS22HD REGISTER - READ ONLY*/
#define BARO_WHO_I_AM               0x0F
#define BARO_INT_SOURCE             0x25
#define BARO_FIFO_STATUS            0x26
#define BARO_STATUS                 0x27
#define BARO_PRESS_OUT_XL           0x28
#define BARO_PRESS_OUT_L            0x29
#define BARO_PRESS_OUT_H            0x2A
#define BARO_TEMP_OUT_L             0x2B
#define BARO_TEMP_OUT_H             0x2C
#define BARO_LPFP_RES               0x33
/*LPS22HD REGISTER - R/W*/
#define BARO_INTERRUPT_CFG          0x0B
#define BARO_THS_P_L                0x0C
#define BARO_THS_P_H                0x0D
#define BARO_CTRL_REG_1             0x10
#define BARO_CTRL_REG_2             0x11
#define BARO_CTRL_REG_3             0x13
#define BARO_FIFO_CTRL              0x14
#define BARO_REF_P_XL               0x15
#define BARO_REF_P_L                0x16
#define BARO_REF_P_H                0x17
#define BARO_RPDS_L                 0x18
#define BARO_RPDS_H                 0x19
#define BARO_RES_CONF               0x1A

/*FUNCITON FOR READ ONLY REGISTERS*/
uint8_t read_Device_Name(uint8_t *ptr);
uint8_t read_Init_Source(void);
uint8_t read_FIFO_Status(void);
uint8_t read_Baro_Status(void);
uint8_t read_Baro_Press_Out_XL(void);
uint8_t read_Baro_Press_Out_L(void);
uint8_t read_Baro_Press_Out_H(void);
uint8_t read_Baro_Temp_Out_L(void);
uint8_t read_Baro_Temp_Out_H(void);
uint8_t read_Baro_LPFP_RES(void);

/*FUNCITON FOR RW ONLY REGISTERS*/
uint8_t readDeviceId(void);
uint8_t readCtrlReg1(void);
uint8_t readCtrlReg2(void);
uint8_t readCtrlReg3(void);


#endif/*BAROMETER_LPS22HD*/

/*PRIVATE FUCNTION*/
uint8_t _SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
uint8_t _SPI_Transmit(uint8_t *pData, uint16_t Size);
void _baroChipEnable(void);
void _baroChipDisable(void);

#endif /* BAROMETER_DRIVER_H_ */
