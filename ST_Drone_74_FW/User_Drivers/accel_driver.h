/*
 * accel_driver.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#ifndef ACCEL_DRIVER_H_
#define ACCEL_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "mxconstants.h"

extern SPI_HandleTypeDef hspi2;

/*TYPE OF ACCEL_GYRO DEFINE*/
#define ACCEL_GYRO_LSM6DSL

#ifdef ACCEL_GYRO_LSM6DSL

#define ACCEL_GYRO_DEVICE_NAME           0x6A
/***********************READ-WRITE REGISTER***********************/
#define AG_FUNC_CFG_ACCESS               0x01 // AG: accel gyro

//sensor sync config register
#define AG_SYNC_TIME_FRAME               0x04
#define AG_SYNC_RES_RATIO                0x05

//FIFO config register
#define AG_FIFO_CTRL_1                   0x06
#define AG_FIFO_CTRL_2                   0x07
#define AG_FIFO_CTRL_3                   0x08
#define AG_FIFO_CTRL_4                   0x09
#define AG_FIFO_CTRL_5                   0x0A
#define AG_DRDY_PULSE_CFG_G              0x0B

//INT1 and 2 pin control
#define AG_INT_1_CTRL                    0x0D
#define AG_INT_2_CTRL                    0x0E

//accelerometer and gyroscope control registers
#define AG_CTRL1_XL                      0x10
#define AG_CTRL2_G                       0x11
#define AG_CTRL3_C                       0x12
#define AG_CTRL4_C                       0x13
#define AG_CTRL5_C                       0x14
#define AG_CTRL6_C                       0x15
#define AG_CTRL7_G                       0x16
#define AG_CTRL8_XL                      0x17
#define AG_CTRL9_XL                      0x18
#define AG_CTRL10_C                      0x19

//i2c master config register
#define AG_MASTER_CONFIG                 0x1A

//interrupt registers
#define AG_TAP_CFG                       0x58
#define AG_THS_6D                        0x59
#define AG_INT_DUR2                      0x5A
#define AG_WAKE_UP_THS                   0x5B
#define AG_WAKE_UP_DUR                   0x5C
#define AG_FREE_FALL                     0x5D
#define AG_MD1_CFG                       0x5E
#define AG_MD2_CFG                       0x5F

#define AG_MASTER_CMD_CODE               0x60

#define AG_SENS_SYNC_SPI_ERR_CODE        0x61

//accelerometer user offset correction
#define AG_X_OFS_USR                     0x73
#define AG_Y_OFS_USR                     0x74
#define AG_Z_OFS_USR                     0x75

/***********************READ ONLY REGISTER***********************/
#define AG_WHO_AM_I                      0x0F
#define AG_WAKEUP_SRC                    0x1B

//interrupt registers
#define AG_TAP_SRC                       0x1C
#define AG_D6D_SRC                       0x1D
#define AG_STATUS_REG                    0x1E

//temperature ouput data register
#define AG_OUT_TEMP_L                    0x20
#define AG_OUT_TEMP_H                    0x21

//gyroscope output registers for user interface
#define AG_OUTX_L_G                      0x22
#define AG_OUTX_H_G                      0x23
#define AG_OUTY_L_G                      0x24
#define AG_OUTY_H_G                      0x25
#define AG_OUTZ_L_G                      0x26
#define AG_OUTz_H_G                      0x27

//accelerometer output registers
#define AG_OUTX_L_XL                     0x28
#define AG_OUTX_H_XL                     0x29
#define AG_OUTY_L_XL                     0x2A
#define AG_OUTY_H_XL                     0x2B
#define AG_OUTZ_L_XL                     0x2C
#define AG_OUTz_H_XL                     0x2D

//sensor hub output registers
#define AG_SENSORHUB_1_REG               0x2E
#define AG_SENSORHUB_2_REG               0x2F
#define AG_SENSORHUB_3_REG               0x30
#define AG_SENSORHUB_4_REG               0x31
#define AG_SENSORHUB_5_REG               0x32
#define AG_SENSORHUB_6_REG               0x33
#define AG_SENSORHUB_7_REG               0x34
#define AG_SENSORHUB_8_REG               0x35
#define AG_SENSORHUB_9_REG               0x36
#define AG_SENSORHUB_10_REG              0x37
#define AG_SENSORHUB_11_REG              0x38
#define AG_SENSORHUB_12_REG              0x39

//FIFO status registers
#define AG_FIFO_STATUS_1                 0x3A
#define AG_FIFO_STATUS_2                 0x3B
#define AG_FIFO_STATUS_3                 0x3C
#define AG_FIFO_STATUS_4                 0x3D

//FIFO data output registers
#define AG_FIFO_DATA_OUT_L               0x3E
#define AG_FIFO_DATA_OUT_H               0x3F

//timestamp output registers
#define AG_TIME_STAMP0_REG               0x40
#define AG_TIME_STAMP1_REG               0x41
#define AG_TIME_STAMP2_REG               0x42 //rw register

//step counter timestamp registers
#define AG_STEP_TIMESTAMP_L              0x49
#define AG_STEP_TIMESTAMP_H              0X4A
//step counter output registers
#define AG_STEP_COUNTER_L                0x4B
#define AG_STEP_COUNTER_H                0x4C

//sensor hub output registers
#define AG_SENSORHUB_13_REG              0x4D
#define AG_SENSORHUB_14_REG              0x4E
#define AG_SENSORHUB_15_REG              0x4F
#define AG_SENSORHUB_16_REG              0x50
#define AG_SENSORHUB_17_REG              0x51
#define AG_SENSORHUB_18_REG              0x52

//interrupt registers
#define AG_FUNC_SRC1                     0x53
#define AG_FUNC_SRC2                     0x54
#define AG_WRIST_TILT_IA                 0x55

//external magnetometer raw data
#define AG_OUT_MAG_RAW_X_L               0x66
#define AG_OUT_MAG_RAW_X_H               0x67
#define AG_OUT_MAG_RAW_Y_L               0x68
#define AG_OUT_MAG_RAW_Y_H               0x69
#define AG_OUT_MAG_RAW_Z_L               0x6A
#define AG_OUT_MAG_RAW_Z_H               0x6B

#endif /*ACCEL_GYRO_LSM6DSL*/

typedef enum 
{
    AG_ERROR,
    AG_OK
}AccelGyro_State_e;

typedef enum
{
    accel_odr_lv_0 = 0b0000, //power down
    accel_odr_lv_1 = 0b1011, //1.6Hz
    accel_odr_lv_2 = 0b0001, //12.5Hz
    accel_odr_lv_3 = 0b0010, //26Hz
    accel_odr_lv_4 = 0b0011, //52Hz
    accel_odr_lv_5 = 0b0100, //104Hz
    accel_odr_lv_6 = 0b0101, //208hz
    accel_odr_lv_7 = 0b0110, //416Hz
    accel_odr_lv_8 = 0b0111, //833Hz
    accel_odr_lv_9 = 0b1000, // 1.66KHz
    accel_odr_lv_10 = 0b1001, //3.33KHz
    accel_odr_lv_11 = 0b1010 //6.66KHz
}Accel_Odr_Select_e;

/*ACCEL-GYRO CONTROL FUNCTION*/
extern uint8_t accelGyro_Init_Device(void);
extern uint8_t accelGyro_Read_Device_Name(uint8_t *ptr);

/*ACCEL CONTROL FUNCTION*/
extern uint8_t accel_Set_Power_Mode(Accel_Odr_Select_e accel_Odr);
extern uint8_t accel_Set_FullScale(uint8_t fullScale_Selection);

/*ACCELEROMETER OUTPUT VALUE*/
extern uint8_t accel_X_Out(int16_t *accelReturnValue);
extern uint8_t accel_Y_Out(int16_t *accelReturnValue);
extern uint8_t accel_Z_Out(int16_t *accelReturnValue);

/*RW SINGLE REGISTER*/
extern uint8_t accelGyro_Write_Single_Register(uint8_t address, uint8_t *txData);
extern uint8_t accelGyro_Read_Single_Register(uint8_t address, uint8_t *rxData);

/*BASIC CONTROL*/
extern void accelGyroEnable(void);
extern void accelGyroDisable(void);
extern void accelGyro_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t ReadAddr, uint8_t *pBuffer, uint8_t size );
extern void accelGyro_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t WriteAddr, uint8_t *pBuffer, uint8_t size );

#endif /* ACCEL_DRIVER_H_ */
