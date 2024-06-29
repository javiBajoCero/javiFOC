/*
 * hardware_pwm.h
 *
 *  Created on: Jun 29, 2024
 *      Author: javi
 */

#ifndef INC_HARDWARE_PWM_H_
#define INC_HARDWARE_PWM_H_

#include "tim.h"


typedef struct {
	float dutyA;
	float dutyB;
	float dutyC;
}struct_duty_cycles_pu;

void configureHardwarePWM(TIM_HandleTypeDef *pwm_htim,
		HAL_TIM_ActiveChannel channel1,
		HAL_TIM_ActiveChannel channel2,
		HAL_TIM_ActiveChannel channel3);

void runHardwarePWM(struct_duty_cycles_pu * d);

#endif /* INC_HARDWARE_PWM_H_ */
