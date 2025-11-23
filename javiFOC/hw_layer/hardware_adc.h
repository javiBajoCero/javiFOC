/*
 * hardware_adc.h
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */

#ifndef HARDWARE_ADC_H_
#define HARDWARE_ADC_H_

#include "adc.h"

typedef struct {
	uint32_t ia;
	uint32_t ib;
	uint32_t ic;
}struct_raw_adc_currents;

typedef struct {
	float vdc;//Volts
	float temperature_feedback;//Celsius Degrees
	float bluepotentiometer;//%per one
	float internal_stm32_temperature;//Celsius Degrees
	float internal_stm32_vref;//Volts
}struct_decoded_adc_variables;

extern struct_decoded_adc_variables adc_variables;

void configureSlowHardwareADC(ADC_HandleTypeDef *slow_hadc,TIM_HandleTypeDef *slow_adc_trigger_htim);
void configureFastHardwareADC(ADC_HandleTypeDef *fast_hadc);
void decodeSlowHardwareADC();

#endif /* HARDWARE_ADC_H_ */
