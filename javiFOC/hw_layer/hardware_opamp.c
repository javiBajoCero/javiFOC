/*
 * hardware_opamp.c
 *
 *  Created on: Nov 23, 2025
 *      Author: JavierMunozSaez
 */

#include "hardware_opamp.h"

void configureHardwareOpAmp(OPAMP_HandleTypeDef * opamp1, OPAMP_HandleTypeDef * opamp2, OPAMP_HandleTypeDef * opamp3){
	  HAL_OPAMP_SelfCalibrate(opamp1);
	  HAL_OPAMP_SelfCalibrate(opamp2);
	  HAL_OPAMP_SelfCalibrate(opamp3);

	  HAL_OPAMP_Start(opamp1);
	  HAL_OPAMP_Start(opamp2);
	  HAL_OPAMP_Start(opamp3);
}
