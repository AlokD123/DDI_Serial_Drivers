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
#include "DDI.h"


#ifndef STM32L4xx_HAL_DEF_H
	#include "stm32l4xx_hal_def.h"
#endif

/* Exported Functions */

//extern uint32_t startTime;

/**
  * @brief Publish ROS message
  * @todo Complete prototype
  * @param topic: ROS topic
  * @param str_msg_ptr: message string
  * @param num_bytes: length of message
  * @retval None
  */
extern void rosWriteStr(const char* topic, volatile uint8_t* str_msg_ptr, size_t num_bytes);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
