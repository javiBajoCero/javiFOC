/*
 * voltagetoduty.h
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */

#ifndef VABCTODUTY_H_
#define VABCTODUTY_H_

#include "hardware_pwm.h"
#include "clarke_and_park.h"

void configure_vabc_to_duty_modulator();

void run_vabc_to_duty_modulator(float input_vdc, struct_abc_pu * input_vabc, struct_duty_cycles_pu * output_duty);

#endif /* VABCTODUTY_H_ */
