/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <vabctoduty.h>
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
#include"hardware_pwm.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim8;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/
float sineWave[256] = {
    0.000000, 0.024541, 0.049068, 0.073565, 0.098017, 0.122411, 0.146730, 0.170962, 0.195090, 0.219101,
    0.242980, 0.266713, 0.290285, 0.313682, 0.336890, 0.359895, 0.382683, 0.405241, 0.427555, 0.449611,
    0.471397, 0.492898, 0.514103, 0.534998, 0.555570, 0.575808, 0.595699, 0.615232, 0.634393, 0.653173,
    0.671559, 0.689541, 0.707107, 0.724247, 0.740951, 0.757209, 0.773010, 0.788346, 0.803208, 0.817585,
    0.831470, 0.844854, 0.857729, 0.870087, 0.881921, 0.893224, 0.903989, 0.914210, 0.923880, 0.932993,
    0.941544, 0.949528, 0.956940, 0.963776, 0.970031, 0.975702, 0.980785, 0.985278, 0.989177, 0.992480,
    0.995185, 0.997290, 0.998795, 0.999699, 1.000000, 0.999699, 0.998795, 0.997290, 0.995185, 0.992480,
    0.989177, 0.985278, 0.980785, 0.975702, 0.970031, 0.963776, 0.956940, 0.949528, 0.941544, 0.932993,
    0.923880, 0.914210, 0.903989, 0.893224, 0.881921, 0.870087, 0.857729, 0.844854, 0.831470, 0.817585,
    0.803208, 0.788346, 0.773010, 0.757209, 0.740951, 0.724247, 0.707107, 0.689541, 0.671559, 0.653173,
    0.634393, 0.615232, 0.595699, 0.575808, 0.555570, 0.534998, 0.514103, 0.492898, 0.471397, 0.449611,
    0.427555, 0.405241, 0.382683, 0.359895, 0.336890, 0.313682, 0.290285, 0.266713, 0.242980, 0.219101,
    0.195090, 0.170962, 0.146730, 0.122411, 0.098017, 0.073565, 0.049068, 0.024541, 0.000000, -0.024541,
    -0.049068, -0.073565, -0.098017, -0.122411, -0.146730, -0.170962, -0.195090, -0.219101, -0.242980, -0.266713,
    -0.290285, -0.313682, -0.336890, -0.359895, -0.382683, -0.405241, -0.427555, -0.449611, -0.471397, -0.492898,
    -0.514103, -0.534998, -0.555570, -0.575808, -0.595699, -0.615232, -0.634393, -0.653173, -0.671559, -0.689541,
    -0.707107, -0.724247, -0.740951, -0.757209, -0.773010, -0.788346, -0.803208, -0.817585, -0.831470, -0.844854,
    -0.857729, -0.870087, -0.881921, -0.893224, -0.903989, -0.914210, -0.923880, -0.932993, -0.941544, -0.949528,
    -0.956940, -0.963776, -0.970031, -0.975702, -0.980785, -0.985278, -0.989177, -0.992480, -0.995185, -0.997290,
    -0.998795, -0.999699, -1.000000, -0.999699, -0.998795, -0.997290, -0.995185, -0.992480, -0.989177, -0.985278,
    -0.980785, -0.975702, -0.970031, -0.963776, -0.956940, -0.949528, -0.941544, -0.932993, -0.923880, -0.914210,
    -0.903989, -0.893224, -0.881921, -0.870087, -0.857729, -0.844854, -0.831470, -0.817585, -0.803208, -0.788346,
    -0.773010, -0.757209, -0.740951, -0.724247, -0.707107, -0.689541, -0.671559, -0.653173, -0.634393, -0.615232,
    -0.595699, -0.575808, -0.555570, -0.534998, -0.514103, -0.492898, -0.471397, -0.449611, -0.427555, -0.405241,
    -0.382683, -0.359895, -0.336890, -0.313682, -0.290285, -0.266713, -0.242980, -0.219101, -0.195090, -0.170962,
    -0.146730, -0.122411, -0.098017, -0.073565, -0.049068, -0.024541
};
float rampuplimit=0.2;
float rampup=0;
uint8_t a=0;
uint8_t b=256/3;
uint8_t c=256*2/3;
/**
  * @brief This function handles TIM8 update interrupt.
  */
void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */

  /* USER CODE END TIM8_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_IRQn 1 */
  vabc.va=rampup*sineWave[a];
  vabc.vb=rampup*sineWave[b];
  vabc.vc=rampup*sineWave[c];

  if(rampup<rampuplimit){
	  rampup+=rampuplimit/1000;
  }else{rampup=rampuplimit;}

  a++;
  b++;
  c++;
  run_vabc_to_duty_modulator(1.0, &vabc, &pwm_duty_cycles);
  runHardwarePWM(&pwm_duty_cycles,&pwm_registers);
  /* USER CODE END TIM8_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
