/*
 * DDI.c
 *
 *  Created on: Apr 10, 2020
 *      Author: Alok Deshpande
 */
#ifndef DDI_H_
	#include "../include/DDI.h"
#endif

void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim1)
{

  /* IMPORTANT INITIALIZATION */
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 15, 15);
	__TIM1_CLK_ENABLE();
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP; //enable timer 1 stop

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* BASE INITIALIZATION */
  htim1->Instance = TIM1;
  htim1->Init.Prescaler = PRESCALER-1; //(uint32_t) (( HAL_RCC_GetPCLK2Freq() / 1<<16) - 1U); //1<<16 Hz clock
  htim1->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1->Init.Period = DDI_TIMEOUT_PERIOD_TICKS-1;
  htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1->Init.RepetitionCounter = 0;
  htim1->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(htim1) != HAL_OK)
  {
    Error_Handler();
  }

  //IF DISABLING/ENABLING EXTERNAL CLOCK SOURCE
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  //IF ENABLING OTHER INTERRUPTS (duplicate of HAL_TIM_Base_Init, with TIMx_CCR1 changed
  if (HAL_TIM_OnePulse_Init(htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  //IF SETTING MASTER MODE FOR CLOCK
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* IMPORTANT INITIALIZATION */
  htim1->Instance->SR = 0; //Reset status register to initialize

}

void getDDI_Val(void){

	//State machine...
	switch(soilSensor_state){
		case INIT:
			soilSensor_state = WAIT;
			//Start sensor
			HAL_GPIO_TogglePin(SENSOR_GPIO_PORT,SENSOR_GPIO_PIN); //TO DO: possibly connect to MOSFET
			//Start timer
			HAL_TIM_Base_Start(&htim1);
			HAL_TIM_Base_Start_IT(&htim1);
			HAL_UART_Receive_DMA(&huart4, soil_data, SOIL_DMA_READ_BYTES); //Start receiving data
			//HAL_TIM_OnePulse_Start_IT(&htim1, TIM_CHANNEL_1);
			startTime = HAL_GetTick();
			break;
		case WAIT:
			break;
		case ACQUIRED:
			HAL_UART_AbortReceive_IT(&huart4); 					  //Abort last DMA read, since reached end (need to restart)
			HAL_GPIO_TogglePin(SENSOR_GPIO_PORT,SENSOR_GPIO_PIN); //Turn off sensor
			soilSensor_state = INIT; 							  //Reset to read for next time
			soil_daq_state = GOT_DATA; 							  //Enable data transmission and disable next DAQ till complete
			#ifndef NO_DEBUG
				if(curr_soil_data[2]!=soil_data[2]) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			#endif
			memcpy(curr_soil_data,soil_data,++soil_chars_read);	  //Copy to non-volatile
			break;
		default:
			break;
	}
}
