/*
 * motor_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include <motor_driver.h>

void set_Motor_PWM(motor_st *motor_pwm)
{
	//motor 1
	if (motor_pwm->motor1_pwm >= MOTOR_MAX_PWM_VALUE)
	{
		htim4.Instance->CCR1 = MOTOR_MAX_PWM_VALUE;
	}
	else if (motor_pwm->motor1_pwm <= MOTOR_MIN_PWM_VALUE)
	{
	    htim4.Instance->CCR1 = MOTOR_MIN_PWM_VALUE;
	}
	else
	{
	    htim4.Instance->CCR1 = (uint32_t) motor_pwm->motor1_pwm;
	}
	//motor 2
	if (motor_pwm->motor2_pwm >= MOTOR_MAX_PWM_VALUE)
	{
	    htim4.Instance->CCR2 = MOTOR_MAX_PWM_VALUE;
	}
	else if (motor_pwm->motor2_pwm <= MOTOR_MIN_PWM_VALUE)
	{
	    htim4.Instance->CCR2 = MOTOR_MIN_PWM_VALUE;
	}
	else
	{
	    htim4.Instance->CCR2 = (uint32_t) motor_pwm->motor2_pwm;
	}
	//motor 3
	if (motor_pwm->motor3_pwm >= MOTOR_MAX_PWM_VALUE)
	{
	    htim4.Instance->CCR3 = MOTOR_MAX_PWM_VALUE;
	}
	else if (motor_pwm->motor3_pwm <= MOTOR_MIN_PWM_VALUE)
	{
	    htim4.Instance->CCR3 = MOTOR_MIN_PWM_VALUE;
	}
	else
	{
	    htim4.Instance->CCR3 = (uint32_t) motor_pwm->motor3_pwm;
	}
	//motor 4
	if (motor_pwm->motor4_pwm >= MOTOR_MAX_PWM_VALUE)
	{
	    htim4.Instance->CCR4 = MOTOR_MAX_PWM_VALUE;
	}
	else if (motor_pwm->motor4_pwm <= MOTOR_MIN_PWM_VALUE)
	{
	    htim4.Instance->CCR4 = MOTOR_MIN_PWM_VALUE;
	}
	else
	{
	    htim4.Instance->CCR4 = (uint32_t) motor_pwm->motor4_pwm;
	}
}

void set_All_Motor_Stop(void)
{
	htim4.Instance->CCR1 = htim4.Instance->CCR2 =
	htim4.Instance->CCR3 = htim4.Instance->CCR4 = MOTOR_MAX_PWM_VALUE;
}

void set_All_Motor_Run(void)
{
	htim4.Instance->CCR1 = htim4.Instance->CCR2 =
	htim4.Instance->CCR3 = htim4.Instance->CCR4 = MOTOR_MAX_PWM_VALUE;
}
