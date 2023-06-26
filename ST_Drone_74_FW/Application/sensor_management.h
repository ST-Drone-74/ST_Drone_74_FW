/*
 * sensor_management.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */

#ifndef SENSOR_MANAGEMENT_H_
#define SENSOR_MANAGEMENT_H_

#include "accel_driver.h"
#include "barometer_driver.h"
#include "gyro_driver.h"
#include "mag_driver.h"
#include "motor_driver.h"
#include "sensor_service.h"

/*Variables*/
extern baroData_st baroBleSentValue_st;
extern SensorAxes_t magBleSentValue_st;
extern SensorAxes_t accelBleSentValue_st;
extern SensorAxes_t gyroBleSentValue_st;
/*Function*/
extern void all_Sensor_Init(void);

#endif /* SENSOR_MANAGEMENT_H_ */
