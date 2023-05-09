/*
 * barometer_driver.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */

#ifndef BAROMETER_DRIVER_H_
#define BAROMETER_DRIVER_H_

/*TYPE OF BAROMETER DEFINE*/
#define BAROMETER_LPS22HD

#ifdef BAROMETER_LPS22HD
/*LPS22HD COMMUNICATION*/
#define BARO_I2C_ADDRESS_1          0x5D
#define BARO_I2C_ADDRESS_1          0x5C
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

#endif/*BAROMETER_LPS22HD*/



#endif /* BAROMETER_DRIVER_H_ */
