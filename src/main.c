/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "../include/main.h"
#include <assert.h>

//Macro to set register. Only use with OR-MASK and OR-VALUE. Only use if more than 1 bit to set
//e.g. RCC_AHB2ENR_GPIOAEN only has 1 bit to set b/c 1UL is bit-shifted
#define SET_REGISTER(REG,MASK,VALUE) ( ((REG) & ~(MASK)) | VALUE)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_Init(void);
static void DMA2_Init(void);

/* USER CODE BEGIN PFP */
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1);
/* USER CODE END PFP */

void HAL_MspInit(void)
{
__HAL_RCC_SYSCFG_CLK_ENABLE();
/* System interrupt init*/
 /* SVC_IRQn interrupt configuration */
 HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0); //Group Priority, Subgroup Priority. GRPPrioirty determines preemption
 /* PendSV_IRQn interrupt configuration */
 HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
 /* SysTick_IRQn interrupt configuration */
 //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(SysTick_IRQn);
}
void HardFault_Handler(void){
  Error_Handler();
}


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void rosWriteStr(const char* topic, uint8_t* str_msg_ptr, size_t num_bytes){
	//HAL_USART_Transmit(&husart3, str_msg_ptr, num_bytes, 100); //100ms timeout
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

   HAL_DBGMCU_EnableDBGStandbyMode();
   HAL_DBGMCU_EnableDBGStopMode();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART3_Init();
  DMA2_Init();

  HAL_TIM_OnePulse_MspInit(&htim1);

  while (1)
  {
	  if(soil_daq_state == START) getDDI_Val();

	  if(soil_daq_state == GOT_DATA){
		  soil_daq_state = SENT_DATA;
		  rosWriteStr("",curr_soil_data,soil_chars_read);   //Publish data
		  soil_chars_read = 0; 								//Reset number of chars read for next time
	  }
	  if(soil_daq_state == SENT_DATA){
		  soil_daq_state = START;
	  }
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_USB;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}



/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

    __HAL_RCC_UART4_CLK_ENABLE();

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 1200;					//Speed of soil sensor
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;           //Nonzero
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  //huart4.RxISR = &HAL_UART_RxCpltCallback;
  huart4.TxISR = &HAL_UART_TxCpltCallback;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  UART4->ICR = 0x121bdf;
  UART4->CR1 = SET_REGISTER(UART4->CR1,USART_CR1_RXNEIE_Msk,USART_CR1_RXNEIE);
  //UART4->CR1 = SET_REGISTER(UART4->CR1,USART_CR1_TCIE_Msk,USART_CR1_TCIE);
  HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(UART4_IRQn);
}

/**
  * @brief UART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void) //Currently UART4 in Amalthea_Firmware... need to replace with USART3 pins
{

    __HAL_RCC_USART3_CLK_ENABLE();

  husart3.Instance = USART3;
  husart3.Init.BaudRate = 57600;
  husart3.Init.WordLength = UART_WORDLENGTH_8B;
  husart3.Init.StopBits = UART_STOPBITS_1;
  husart3.Init.Parity = UART_PARITY_NONE;
  husart3.Init.Mode = UART_MODE_TX_RX;
  if (HAL_UART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}


/**
  * @brief DMA2 Initialization Function for UART4
  * @param None
  * @retval None
  */
static void DMA2_Init(void)
{
  #define UART4_RX_CH_IDX 16 //LSb # of channel to select in DMA
  #define UART4_RX_REQUEST 0b10 //Set channel selection (as UART4_RX)
  #define UART4_TX_CH_IDX 8
  #define UART4_TX_REQUEST 0b10 //Same sel for TX, coincidentally
    //__HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

  __HAL_LINKDMA(&huart4,hdmarx,hdma2);
  __HAL_LINKDMA(&huart4,hdmatx,hdma2);

  hdma2.DmaBaseAddress = DMA2;
  hdma2.Instance = DMA2_Channel5;
  hdma2.Init.Request = UART4_RX_REQUEST;
  hdma2.ChannelIndex = UART4_RX_CH_IDX;
  hdma2.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma2.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma2.Init.MemInc = DMA_MINC_DISABLE;						//MUST DISABLE so reset DMA read location to start of arr each measurement cycle
  hdma2.Init.Mode = DMA_NORMAL;
  hdma2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma2.Init.Priority = DMA_PRIORITY_MEDIUM;
  if (HAL_DMA_Init(&hdma2) != HAL_OK)
  {
    Error_Handler();
  }
  NVIC_EnableIRQ(DMA2_Channel5_IRQn);
  NVIC_EnableIRQ(DMA2_Channel3_IRQn);
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOC_CLK_ENABLE();
  //__HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE(); //Enable UART4 GPIO pins //<-----
  __HAL_RCC_GPIOB_CLK_ENABLE(); //Enable USART3 GPIO + LED GPIO pins //<-----
  //__HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  //__HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB14, LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|LD2_Pin; //<------- LED3 and LED2
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10, PB11 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //USART3 TX
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //USART3 RX
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Configure GPIO pin : PA0
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //UART4 TX
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Configure GPIO pin : PA1
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;//UART4 RX
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}



/* USER CODE BEGIN 4 */
/**
  * @brief  This function is executed at the end of the wait period
  * @param *htim1: pointer to an initialized timer struct
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1){
	if(htim1->Instance == TIM1)
	{
		///*
		#ifndef NO_DEBUG
			uint32_t time = HAL_GetTick();
			if((time - startTime) < 349) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		#endif
		HAL_TIM_Base_Stop(htim1);	 	//Stop timer till next time
		htim1->Instance->SR = 0; 		//Reset status register for future use
		soilSensor_state = ACQUIRED;
		//*/
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while(1){
		#ifdef NO_DEBUG
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //Error LED
			HAL_Delay(1000);
		#endif
	}

  /* USER CODE END Error_Handler_Debug */
}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

void DMA2_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma2);
}

void DMA2_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma2);
}

void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}

void TIM1_UP_TIM16_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4){
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4){
		if(soil_chars_read < (sizeof(soil_data)/sizeof(soil_data[0]) -1)){
			soil_chars_read++; HAL_UART_Receive_DMA(huart, soil_data+soil_chars_read, SOIL_DMA_READ_BYTES); //When received SOIL_DMA_READ_BYTES, restart DMA read to receive the next batch
		}
	}
}
