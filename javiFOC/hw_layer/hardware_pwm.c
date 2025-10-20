/*
 * hardware_pwm.c
 *
 *  Created on: Jun 29, 2024
 *      Author: javi
 */
#include "hardware_pwm.h"



TIM_HandleTypeDef *_pwm_htim;
HAL_TIM_ActiveChannel _channel1;
HAL_TIM_ActiveChannel _channel2;
HAL_TIM_ActiveChannel _channel3;

struct_duty_cycles_pu pwm_duty_cycles;
struct_raw_timer_register_values  pwm_registers;

void configureHardwarePWM(TIM_HandleTypeDef *pwm_htim,
		HAL_TIM_ActiveChannel channel1,
		HAL_TIM_ActiveChannel channel2,
		HAL_TIM_ActiveChannel channel3){
	_pwm_htim=pwm_htim;
	_channel1=channel1;
	_channel2=channel2;
	_channel3=channel3;
	HAL_TIM_PWM_Start	(_pwm_htim, _channel1);
	HAL_TIMEx_PWMN_Start(_pwm_htim, _channel1);
	HAL_TIM_PWM_Start	(_pwm_htim, _channel2);
	HAL_TIMEx_PWMN_Start(_pwm_htim, _channel2);
	HAL_TIM_PWM_Start	(_pwm_htim, _channel3);
	HAL_TIMEx_PWMN_Start(_pwm_htim, _channel3);
}

void runHardwarePWM(struct_duty_cycles_pu *d, struct_raw_timer_register_values *reg)
{
    // Convert normalized duty cycle (-1.0 to +1.0) to raw timer register values
	reg->pwmregisterA = (uint32_t)(((d->dutyA + 1.0f) * PWM_COUNTER) / 2.0f);
	reg->pwmregisterB = (uint32_t)(((d->dutyB + 1.0f) * PWM_COUNTER) / 2.0f);
	reg->pwmregisterC = (uint32_t)(((d->dutyC + 1.0f) * PWM_COUNTER) / 2.0f);

    // Write the computed values to the hardware timer compare registers
    _pwm_htim->Instance->CCR1 = reg->pwmregisterA;
    _pwm_htim->Instance->CCR2 = reg->pwmregisterB;
    _pwm_htim->Instance->CCR3 = reg->pwmregisterC;
}


