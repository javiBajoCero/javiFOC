/*
 * hardware_adc.c
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */

#define SLOW_ADC_BUFFER_LENGTH 5
#define VBUS_ADC_BUFFER_INDEX 0
#define TEMPERATUREFEEDBACK_ADC_BUFFER_INDEX 1
#define BLUEPOTENTIOMETER_ADC_BUFFER_INDEX 2
#define INTERNAL_STM32_TEMP_ADC_BUFFER_INDEX 3
#define INTERNAL_STM32_VREF_ADC_BUFFER_INDEX 4
//internal temperature sensor factory calibration
#define TS_CAL1     ((uint16_t*)0x1FFF75A8) // @30°C
#define TS_CAL2     ((uint16_t*)0x1FFF75CA) // @110°C
#define TEMP_CAL1   30.0f
#define TEMP_CAL2   110.0f

#include "hardware_adc.h"



ADC_HandleTypeDef *_slow_hadc;
TIM_HandleTypeDef *_slow_adc_trigger_htim;

uint32_t slow_adc_buffer[SLOW_ADC_BUFFER_LENGTH];

struct_decoded_adc_variables adc_variables;

void configureHardwareADC(ADC_HandleTypeDef *slow_hadc,TIM_HandleTypeDef *slow_adc_trigger_htim){
	_slow_hadc=slow_hadc;
	_slow_adc_trigger_htim=slow_adc_trigger_htim;
	HAL_ADCEx_Calibration_Start(_slow_hadc, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(_slow_hadc, slow_adc_buffer, SLOW_ADC_BUFFER_LENGTH);
	HAL_TIM_Base_Start(_slow_adc_trigger_htim);
}

//RawAdc = (Vsense / 3.3f) * 4095.0f; or RawAdc = Vsense  * 1240.9091f;
//Vsense = RawAdc * 0.00080586f;

void decodeHArdwareADC(){
//Vsensed = VBUS * (18 / (18 + 169)) = VBUS * 0.09625668449f
//VBUS =  RawAdc * 0.00080586f/0.09625668449f ; or RawAdc * 0.0083743f;
	adc_variables.vdc=
			slow_adc_buffer[VBUS_ADC_BUFFER_INDEX] * 0.0083743f;

	adc_variables.bluepotentiometer=
			slow_adc_buffer[BLUEPOTENTIOMETER_ADC_BUFFER_INDEX] * 0.00024420f;

	adc_variables.internal_stm32_vref=
			slow_adc_buffer[INTERNAL_STM32_VREF_ADC_BUFFER_INDEX] * 0.00080586f;

	//adc_variables.internal_stm32_temperature=
	//					((float)(slow_adc_buffer[INTERNAL_STM32_TEMP_ADC_BUFFER_INDEX] - *TS_CAL1) *
	 //                   (TEMP_CAL2 - TEMP_CAL1) /
	 //                   ((float)(*TS_CAL2 - *TS_CAL1))) + TEMP_CAL1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Full buffer is ready here (called repeatedly in circular mode)
    // Process or signal a task
	HAL_GPIO_WritePin(STATUS_REDLED_GPIO_Port, STATUS_REDLED_Pin,GPIO_PIN_SET );
	decodeHArdwareADC();
	HAL_GPIO_WritePin(STATUS_REDLED_GPIO_Port, STATUS_REDLED_Pin,GPIO_PIN_RESET );
}
