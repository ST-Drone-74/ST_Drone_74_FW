/*
 * motor_driver.h
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "mxconstants.h"

extern TIM_HandleTypeDef htim4;

/*Define current type of motor*/
#define DC_MOTOR

#ifdef DC_MOTOR
	#define MOTOR_MAX_PWM_VALUE     (float)1900.0
	#define MOTOR_MIN_PWM_VALUE     (float)0.0
	typedef struct
	{
		float motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm;
	}motor_st;
#endif /*DC_MOTOR*/

#ifdef BRSHLESS_MOTOR
	#define MOTOR_MAX_PWM_VALUE     (float)1700.0
	#define MOTOR_MIN_PWM_VALUE     (float)850.0
#endif/*BRSHLESS_MOTOR*/

void set_Motor_PWM(motor_st *motor_pwm);
void set_All_Motor_Stop(void);
void set_All_Motor_Run(void);

#endif /* MOTOR_DRIVER_H_ */
