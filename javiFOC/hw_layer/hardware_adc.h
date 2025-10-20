/*
 * hardware_adc.h
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */

#ifndef HARDWARE_ADC_H_
#define HARDWARE_ADC_H_

typedef struct {
	uint32_t ia;
	uint32_t ib;
	uint32_t ic;
}struct_raw_adc_currents;

typedef struct {
	uint32_t vdc;
}struct_raw_adc_voltages;

void configureHardwareADC();

void decodeHArdwareADC();

#endif /* HARDWARE_ADC_H_ */
