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

//Soil sensor definition
#define PRESCALER 7220 //722				    //For 65kHz freq, with 47MHz clk
#define SOIL_TIMEOUT_PERIOD_TICKS 655360 //22936 //For 350ms period, with prescaler=722
#define SOIL_PWR_GPIO_PORT GPIOB
#define SOIL_PWR_GPIO_PIN GPIO_PIN_14
#define RX_SIZE 30

#define DMA_GRP_PRIORITY 15
#define DMA_SUB_PRIORITY 15
#define TIM_GRP_PRIORITY 15
#define TIM_SUB_PRIORITY 15
#define UART_GRP_PRIORITY 15
#define UART_SUB_PRIORITY 15



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1);
/* USER CODE END PFP */

void HAL_MspInit(void)
{
__HAL_RCC_SYSCFG_CLK_ENABLE();
/* System interrupt init*/
 /* SVC_IRQn interrupt configuration */
 HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0); //Group Priority, Subgroup Priority. GRPPriority determines preemption
 /* PendSV_IRQn interrupt configuration */
 HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
 /* SysTick_IRQn interrupt configuration */
 //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(SysTick_IRQn);
}

void rosWriteStr(const char* topic, volatile uint8_t* str_msg_ptr, size_t num_bytes){
	//HAL_USART_Transmit(&husart3, str_msg_ptr, num_bytes, 100); //100ms timeout
}

/**
  * @brief  The application entry point.
  * @retval int
  */

 static DDI_TypeDef soilDDI = {0}; //Sensor
 static DMA_HandleTypeDef hdma2;   //Using DMA2
 //Storage variables
 static uint8_t soil_chars_read;
 volatile static uint8_t soil_data[RX_SIZE];
 static uint8_t old_soil_data[RX_SIZE];
 //Program state variables
 DAQ_state soil_daq_state = START;

int main(void)
{
  /* ----- MCU Configuration ------- */
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

   HAL_DBGMCU_EnableDBGStandbyMode();
   HAL_DBGMCU_EnableDBGStopMode();

  /* Initialize sensors */

  //Soil sensor
    soilDDI.pwr_gpio_port = SOIL_PWR_GPIO_PORT;
    soilDDI.pwr_gpio_pin = SOIL_PWR_GPIO_PIN;

    DDI_Init_DMA(&hdma2, &soilDDI,soil_data,DMA_PRIORITY_MEDIUM,DMA_GRP_PRIORITY,DMA_SUB_PRIORITY,
                PRESCALER,SOIL_TIMEOUT_PERIOD_TICKS,TIM_GRP_PRIORITY,TIM_SUB_PRIORITY,
                UART_MODE_TX_RX, UART_HWCONTROL_NONE,UART_OVERSAMPLING_16,UART_ONE_BIT_SAMPLE_DISABLE,
                HAL_UART_TxCpltCallback,UART_GRP_PRIORITY,UART_SUB_PRIORITY);

  while (1)
  {
	  switch(soil_daq_state){
            case START:
                soil_chars_read = DDI_getVal(&soilDDI,old_soil_data,&soil_daq_state,soilDDI.pwr_gpio_port,&(soilDDI.pwr_gpio_pin));
                break;
            case GOT_DATA:
                soil_daq_state = SENT_DATA;
                rosWriteStr("",soilDDI.data,soil_chars_read);   //Publish data
                break;
            case SENT_DATA:
                soil_daq_state = START;
      };
  }

}

/**
  * @brief System Clock Configuration
  * @todo Make periph clock independent of selected peripherals
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
  * @brief Generic UART Initialization Function
  * Most important parameters exposed. Can add more if desired.
  * @retval None
  */
void UARTx_Init(UART_HandleTypeDef* huart_x, USART_TypeDef* UART_x, int IRQ_num, uint8_t boolEnIRQ,
                uint32_t baud, uint32_t word_len, uint32_t stop_bits, uint32_t parity, uint32_t mode,
                uint32_t hwflwctrl, uint32_t ov_flg, uint32_t obs_flg, uint32_t adv_feat_flg,
                void (*TxISR)(UART_HandleTypeDef* huart),void (*RxISR)(UART_HandleTypeDef* huart),
                uint32_t grpPriority, uint32_t subPriority)
{
  huart_x->Instance = UART_x;
  huart_x->Init.BaudRate = baud;					
  huart_x->Init.WordLength = word_len;
  huart_x->Init.StopBits = stop_bits;
  huart_x->Init.Parity = parity;
  huart_x->Init.Mode = mode;
  huart_x->Init.HwFlowCtl = hwflwctrl;
  huart_x->Init.OverSampling = ov_flg;
  huart_x->Init.OneBitSampling = obs_flg;
  huart_x->AdvancedInit.AdvFeatureInit = adv_feat_flg;
  huart_x->RxISR = RxISR;
  huart_x->TxISR = TxISR;
  if (HAL_UART_Init(huart_x) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(IRQ_num, grpPriority, subPriority);
  if(boolEnIRQ) HAL_NVIC_EnableIRQ(IRQ_num);
}

/**
  * @brief Generic Timer Initialization Function
  * Most important parameters exposed. Can add more if desired.
  * @retval None
  */
void TIMx_Init(TIM_HandleTypeDef* htim_x, TIM_TypeDef* tim_x, int IRQ_num, uint8_t boolEnIRQ,
                uint32_t prescaler, uint32_t mode, uint32_t clk_div, uint32_t period, uint32_t rep_cntr_val, uint32_t auto_rp, uint32_t clk_src,
                uint32_t master_out_trig1, uint32_t master_out_trig2, uint32_t master_slave_mode, uint32_t op_mode, uint32_t grpPriority, uint32_t subPriority)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* BASE INITIALIZATION */
    htim_x->Instance = tim_x;
    htim_x->Init.Prescaler = prescaler-1; //(uint32_t) (( HAL_RCC_GetPCLK2Freq() / 1<<16) - 1U); //1<<16 Hz clock
    htim_x->Init.CounterMode = mode;
    htim_x->Init.Period = period-1;
    htim_x->Init.ClockDivision = clk_div;
    htim_x->Init.RepetitionCounter = rep_cntr_val;
    htim_x->Init.AutoReloadPreload = auto_rp;
    if (HAL_TIM_Base_Init(htim_x) != HAL_OK)
    {
        Error_Handler();
    }

    //IF DISABLING/ENABLING EXTERNAL CLOCK SOURCE
    sClockSourceConfig.ClockSource = clk_src;
    if (HAL_TIM_ConfigClockSource(htim_x, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    //IF ENABLING OTHER INTERRUPTS (duplicate of HAL_TIM_Base_Init, with TIMx_CCR1 changed)
    if (HAL_TIM_OnePulse_Init(htim_x, op_mode) != HAL_OK)
    {
        Error_Handler();
    }
    //IF SETTING MASTER MODE FOR CLOCK
    sMasterConfig.MasterOutputTrigger = master_out_trig1;
    sMasterConfig.MasterOutputTrigger2 = master_out_trig2;
    sMasterConfig.MasterSlaveMode = master_slave_mode;
    if (HAL_TIMEx_MasterConfigSynchronization(htim_x, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* VERY IMPORTANT: ENABLE INTERRUPTS */
    HAL_NVIC_SetPriority(IRQ_num, grpPriority, subPriority);
    if(boolEnIRQ) HAL_NVIC_EnableIRQ(IRQ_num);

    /* IMPORTANT INITIALIZATION */
    htim_x->Instance->SR = 0; //Reset status register to initialize
}
/**
  * @brief Generic DMA Initialization Function
  * Most important parameters exposed. Can add more if desired.
  * @retval None
  */
void DMAx_Init(DMA_HandleTypeDef* hdma_x, DMA_TypeDef* base_addr, int IRQ_num, DMA_Channel_TypeDef* inst, uint8_t boolEnIRQ,
                uint32_t request, uint32_t ch_idx, uint32_t dir, uint32_t mem_align, uint32_t m_inc, uint32_t mode,
                uint32_t periph_align, uint32_t p_inc, uint32_t priority, uint32_t grpPriority, uint32_t subPriority){
    
    hdma_x->DmaBaseAddress = base_addr;
    hdma_x->Instance = inst;
    hdma_x->Init.Request = request;
    hdma_x->ChannelIndex = ch_idx;
    hdma_x->Init.Direction = dir;
    hdma_x->Init.MemDataAlignment = mem_align;
    hdma_x->Init.MemInc = m_inc;				
    hdma_x->Init.Mode = mode;
    hdma_x->Init.PeriphDataAlignment = periph_align;
    hdma_x->Init.PeriphInc = p_inc;
    hdma_x->Init.Priority = priority;
    if (HAL_DMA_Init(hdma_x) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(IRQ_num, grpPriority, subPriority);
    if(boolEnIRQ) NVIC_EnableIRQ(IRQ_num);
}

/**
  * @brief Generic GPIO Initialization Function
  * Most important parameters exposed. Can add more if desired.
  * @retval None
  */
void GPIOx_Init(GPIO_TypeDef* gpio_x, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t alt, uint32_t speed){
    GPIO_InitTypeDef GPIO_InitStruct;
    HAL_PWREx_EnableVddIO2();

    /*Configure GPIO pins */
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Alternate = alt;
    GPIO_InitStruct.Speed = speed;
    HAL_GPIO_WritePin(gpio_x, pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(gpio_x, &GPIO_InitStruct);
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

void DMA2_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma2);
}

void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&(soilDDI.UART));
}

void TIM1_UP_TIM16_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&(soilDDI.TIM));
}

/**
  * @brief  This function is executed after all data is received (for specified read length)
  * @param *huart: pointer to an initialized UART struct
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &soilDDI.UART){
    //For DDI soil sensor, call the appropriate update after received a BATCH of total expected measurement data
    soilDDI.UART_RXCplt_CB(&soilDDI,sizeof(soilDDI.data)/sizeof(soilDDI.data[0]) );
	}
}

/**
  * @brief  This function is executed at the end of the wait period
  * @param *htim: pointer to an initialized timer struct
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &soilDDI.TIM)
	{
		//#ifndef NO_DEBUG
		//	uint32_t time = HAL_GetTick();
		//	if((time - startTime) < 349) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		//#endif
		
    //For DDI soil sensor, call the appropriate update to after reading timeout  
        soilDDI.TIM_Elapsed_CB(htim);
	}
}


void HardFault_Handler(void){
  Error_Handler();
}