/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l496xx.h"
#include "config.h"


#ifndef STM32L4xx_HAL_DEF_H
	#include "stm32l4xx_hal_def.h"
#endif

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* USER CODE BEGIN Private defines */
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
USART_HandleTypeDef husart3;
DMA_HandleTypeDef hdma2;

extern volatile DDI_state soilSensor_state;
extern volatile DAQ_state soil_daq_state;
extern uint8_t curr_soil_data[RX_SIZE];

extern TIM_HandleTypeDef htim1;

extern uint32_t startTime;

extern void rosWriteStr(const char* topic, uint8_t* str_msg_ptr, size_t num_bytes);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
