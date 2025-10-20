/*
 * hardware_pwm.c
 *
 *  Created on: Jun 29, 2024
 *      Author: javi
 */
#include "hardware_pwm.h"



TIM_HandleTypeDef *_pwm_htim;

void configureHardwarePWM(TIM_HandleTypeDef *pwm_htim,
		HAL_TIM_ActiveChannel channel1,
		HAL_TIM_ActiveChannel channel2,
		HAL_TIM_ActiveChannel channel3){
	_pwm_htim=pwm_htim;
	HAL_TIM_PWM_Start	(_pwm_htim, channel1);
	HAL_TIMEx_PWMN_Start(_pwm_htim, channel1);
	HAL_TIM_PWM_Start	(_pwm_htim, channel2);
	HAL_TIMEx_PWMN_Start(_pwm_htim, channel2);
	HAL_TIM_PWM_Start	(_pwm_htim, channel3);
	HAL_TIMEx_PWMN_Start(_pwm_htim, channel3);


}

void runHardwarePWM(struct_duty_cycles_pu * d){

	_pwm_htim->Instance->CCR1=(uint32_t)(d->dutyA*PWM_COUNTER +PWM_COUNTER)/2;
	_pwm_htim->Instance->CCR2=(uint32_t)(d->dutyB*PWM_COUNTER +PWM_COUNTER)/2;
	_pwm_htim->Instance->CCR3=(uint32_t)(d->dutyC*PWM_COUNTER +PWM_COUNTER)/2;

}

