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

#define FAST_ADC_BUFFER_LENGTH 2

//internal temperature sensor factory calibration
#define TS_CAL1     ((uint16_t*)0x1FFF75A8) // @30°C
#define TS_CAL2     ((uint16_t*)0x1FFF75CA) // @110°C
#define TEMP_CAL1   30.0f
#define TEMP_CAL2   110.0f

#include "hardware_adc.h"



ADC_HandleTypeDef *_slow_hadc;
TIM_HandleTypeDef *_slow_adc_trigger_htim;
ADC_HandleTypeDef *_fast_hadc;
TIM_HandleTypeDef *_fast_adc_trigger_htim;

uint32_t slow_adc_buffer[SLOW_ADC_BUFFER_LENGTH];
uint32_t fastadc_buffer[FAST_ADC_BUFFER_LENGTH];

struct_decoded_adc_variables adc_variables;

void configureSlowHardwareADC(ADC_HandleTypeDef *slow_hadc,TIM_HandleTypeDef *slow_adc_trigger_htim){
	_slow_hadc=slow_hadc;
	_slow_adc_trigger_htim=slow_adc_trigger_htim;

	HAL_ADCEx_Calibration_Start(_slow_hadc, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(_slow_hadc, slow_adc_buffer, SLOW_ADC_BUFFER_LENGTH);
	HAL_TIM_Base_Start(_slow_adc_trigger_htim);
}

void configureFastHardwareADC(ADC_HandleTypeDef *fast_hadc){
	_fast_hadc=fast_hadc;

	HAL_ADCEx_Calibration_Start(_fast_hadc, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(_fast_hadc, fastadc_buffer, FAST_ADC_BUFFER_LENGTH);

}

/**
 * @brief  Convert ADC reading of NTC divider to temperature in °C.
 *
 * Wiring assumed:
 *   3.3V ----[ NTC ]---- ADC_IN ----[ 5.7k ]---- GND
 *
 * @param adc  Raw ADC conversion (0..4095 for 12-bit)
 * @param vdda   Measured VDDA in volts
 * @return float    Temperature in degrees Celsius
 */
float ntc_temp(uint16_t adc, float vdda) {
    // Precomputed for NTCG103JF103FT1, Rfixed = 4.7k, 12-bit ADC, 3.3V
    static const uint16_t adc_lut[] = {
        234, 377,  576,  833, 1141, 1484, 1840, 2187,
        2506, 2786, 3025, 3223, 3384, 3515, 3619, 3703
    }; // ADC at -20, -10, ..., 130 °C

    static const int8_t temp_lut[] = {
        -20, -10,   0,  10,  20,  30,  40,  50,
         60,  70,  80,  90, 100, 110, 120, 130
    };

    const int N = sizeof(adc_lut) / sizeof(adc_lut[0]);

    // Optional: normalize ADC to 3.3V reference if vdda != 3.3
    float x = adc * (3.3f / vdda);   // effective 3.3V-based ADC count

    if (x <= adc_lut[0])     return (float)temp_lut[0];
    if (x >= adc_lut[N - 1]) return (float)temp_lut[N - 1];

    for (int i = 0; i < N - 1; ++i) {
        if (x <= adc_lut[i + 1]) {
            float a0 = (float)adc_lut[i];
            float a1 = (float)adc_lut[i + 1];
            float t0 = (float)temp_lut[i];
            float t1 = (float)temp_lut[i + 1];
            return t0 + (t1 - t0) * (x - a0) / (a1 - a0);
        }
    }

    return (float)temp_lut[N - 1]; // fallback (should not hit)
}


void decodeSlowHardwareADC(){
	uint16_t vref_raw 					= slow_adc_buffer[INTERNAL_STM32_VREF_ADC_BUFFER_INDEX];
	uint16_t temp_raw 					= slow_adc_buffer[INTERNAL_STM32_TEMP_ADC_BUFFER_INDEX];
	uint16_t bluepote_raw 				= slow_adc_buffer[BLUEPOTENTIOMETER_ADC_BUFFER_INDEX];
	uint16_t temperaturefeedback_raw 	= slow_adc_buffer[TEMPERATUREFEEDBACK_ADC_BUFFER_INDEX];
	uint16_t vbus_raw 					= slow_adc_buffer[VBUS_ADC_BUFFER_INDEX];
	// Prevent crash if ADC hasn't filled buffer yet
	if (vref_raw == 0 || temp_raw == 0) {
	    return;
	}

	// 1. Compute VDDA in millivolts
	uint32_t vdda_mV =__HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_12B);
	adc_variables.internal_stm32_vref =vdda_mV/1000.0f;

	// 2. Compute internal temperature in °C
	adc_variables.internal_stm32_temperature =
	    __HAL_ADC_CALC_TEMPERATURE(vdda_mV, temp_raw, ADC_RESOLUTION_12B);

	//Blue potentiometer p.u.
	adc_variables.bluepotentiometer= (bluepote_raw * vdda_mV) / (4095.0f * 1000.0f)/adc_variables.internal_stm32_vref;

	//external temperature
	adc_variables.temperature_feedback= ntc_temp(temperaturefeedback_raw, adc_variables.internal_stm32_vref);

	//vbus
	adc_variables.vdc= ((vbus_raw * vdda_mV) / (4095.0f * 1000.0f))*10.387096774f;//(18 / (18 + 169)) resistor divider

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Full buffer is ready here (called repeatedly in circular mode)
    // Process or signal a task
	HAL_GPIO_WritePin(STATUS_REDLED_GPIO_Port, STATUS_REDLED_Pin,GPIO_PIN_SET );
	decodeSlowHardwareADC();
	HAL_GPIO_WritePin(STATUS_REDLED_GPIO_Port, STATUS_REDLED_Pin,GPIO_PIN_RESET );
}
