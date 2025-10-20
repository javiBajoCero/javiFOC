/*
 * voltagetoduty.h
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */

#ifndef VABCTODUTY_H_
#define VABCTODUTY_H_

#include "hardware_pwm.h"

typedef struct {
	float va;
	float vb;
	float vc;
}struct_vabc_pu;

extern struct_vabc_pu vabc;

void configure_vabc_to_duty_modulator();

void run_vabc_to_duty_modulator(float input_vdc, struct_vabc_pu * input_vabc, struct_duty_cycles_pu * output_duty);

#endif /* VABCTODUTY_H_ */
