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
}struct_duty_cycles_pu;//input to hardware_pwm block

typedef struct {
	uint32_t pwmregisterA;
	uint32_t pwmregisterB;
	uint32_t pwmregisterC;
}struct_raw_timer_register_values;//output from hardware_pwm block

extern struct_duty_cycles_pu pwm_duty_cycles;
extern struct_raw_timer_register_values  pwm_registers;

void configureHardwarePWM(TIM_HandleTypeDef *pwm_htim,
		HAL_TIM_ActiveChannel channel1,
		HAL_TIM_ActiveChannel channel2,
		HAL_TIM_ActiveChannel channel3);

void runHardwarePWM(struct_duty_cycles_pu * input_duties, struct_raw_timer_register_values * output_registerspwm);

#endif /* INC_HARDWARE_PWM_H_ */
