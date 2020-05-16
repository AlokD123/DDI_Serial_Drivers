/*
 * DDI.h
 *
 *  Created on: Apr 10, 2020
 *      Author: Alok Deshpande
 */

#ifndef DDI_H_
#define DDI_H_

#ifndef __STM32L496xx_H
	#include "stm32l496xx.h"
#endif
#ifndef STM32L4xx_HAL_TIM_H
	#include "stm32l4xx_HAL_TIM.h"
#endif

#include "config.h"

#define PRESCALER 7220 //722				    //For 65kHz freq, with 47MHz clk
#define DDI_TIMEOUT_PERIOD_TICKS 655360 //22936 //For 350ms period, with prescaler=722

#define SENSOR_GPIO_PORT GPIOB
#define SENSOR_GPIO_PIN GPIO_PIN_14

#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB



volatile DDI_state soilSensor_state = INIT;
volatile DAQ_state soil_daq_state = START;

TIM_HandleTypeDef htim1 = {0};
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim1);
UART_HandleTypeDef huart4 = {0};

volatile uint8_t soil_chars_read = 0;
volatile uint8_t soil_data[RX_SIZE];
uint8_t curr_soil_data[RX_SIZE];

uint32_t startTime;

void getDDI_Val(void);

#endif /* DDI_H_ */
