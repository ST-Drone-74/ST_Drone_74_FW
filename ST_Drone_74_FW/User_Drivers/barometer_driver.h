/*
 * barometer_driver.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#ifndef BAROMETER_DRIVER_H_
#define BAROMETER_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "mxconstants.h"


extern SPI_HandleTypeDef hspi2;


/*TYPE OF BAROMETER DEFINE*/
#define BAROMETER_LPS22HD

#ifdef BAROMETER_LPS22HD
/*LPS22HD COMMUNICATION*/
#define BARO_I2C_ADDRESS_1          0x5D
#define BARO_I2C_ADDRESS_2          0x5C
#define BARO_DEVICE_NAME            0xB1
#define BARO_SCALING_FACTOR         4096
#define BARO_TEMP_SENSITIVE         (float)100.0
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

/*FIFO MODE ENUM*/
typedef enum 
{
    BYPASS = 0x00,
    FIFO = 0x20,
    STREAM = 0x40,
    DYNAMIC_STREAM = 0xC0,
    STREAM_2_FIFO = 0x60,
    BYPASS_2_STREAM = 0x80,
    BYPASS_2_FIFO = 0xE0
}Fifo_Mode_e;

/*SENSOR STATE ENUM*/
typedef enum 
{
    SENSOR_ERROR,
    SENSOR_OK
}Baro_State_e;

/*FUNCITON FOR READ ONLY REGISTERS*/
uint8_t baro_Read_Device_Name(uint8_t *ptr);
uint8_t baro_read_Init_Source(void);
uint8_t baro_read_FIFO_Status(void);
uint8_t baro_read_Baro_Status(void);
uint8_t baro_read_Baro_Press_Out_XL(void);
uint8_t baro_read_Baro_Press_Out_L(void);
uint8_t baro_read_Baro_Press_Out_H(void);
uint8_t baro_read_Baro_Temp_Out_L(void);
uint8_t baro_read_Baro_Temp_Out_H(void);
uint8_t baro_read_Baro_LPFP_RES(void);

/*FUNCITON FOR RW ONLY REGISTERS*/
uint8_t baro_write_Single_Register(uint8_t address, uint8_t *txData);
uint8_t baro_read_Single_Register(uint8_t address, uint8_t *rxData);

/*SENSOR CONTROL*/
uint8_t baro_Init_Device(void);
uint8_t baro_Set_FIFO_Mode(Fifo_Mode_e set_Mode, uint8_t set_level);
void baro_Reset_FIFO(void);
uint8_t baro_Read_Pressure(uint32_t *rxData);
uint8_t baro_HPA_Pressure(float *pressure);
uint8_t baro_Read_Temperature(uint16_t *rxData);
Fifo_Mode_e baro_Read_Current_FIFO_Mode(void);
#endif/*BAROMETER_LPS22HD*/

/*PRIVATE FUCNTION*/

void _baroChipEnable(void);
void _baroChipDisable(void);
void barometer_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size );
void barometer_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size );


#endif /* BAROMETER_DRIVER_H_ */
