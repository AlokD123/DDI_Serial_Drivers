/*
 * DDI.c
 *
 *  Created on: May 17, 2020
 *      Author: Alok Deshpande
 */
#ifndef DDI_H_
	#include "../include/DDI.h"
#endif

static volatile DDI_state state = INIT;
static volatile uint8_t chars_read = 0;

/* Exported Functions */


void DDI_UARTx_Init(UART_HandleTypeDef* huart_x, USART_TypeDef* UART_x, int IRQ_num,
                    uint32_t mode, uint32_t hwflwctrl, uint32_t ov_flg, uint32_t obs_flg,
                    void (*TxISR)(UART_HandleTypeDef* huart),uint32_t grpPriority, uint32_t subPriority)
{
    UARTx_Init(huart_x,UART_x,IRQ_num,UART_EN_IRQ,DDI_SERIAL_BAUD,UART_WORDLENGTH_8B,UART_STOPBITS_1,UART_PARITY_NONE,
                mode,hwflwctrl,ov_flg,obs_flg,                                                       
                UART_ADVFEATURE_NO_INIT,TxISR,NULL,grpPriority,subPriority);
    UART_x->ICR = UART_CLEAR_FLAGS;
    UART_x->CR1 = SET_REGISTER(UART_x->CR1,USART_CR1_RXNEIE_Msk,USART_CR1_RXNEIE);
}

void DDI_TIMx_Init(TIM_HandleTypeDef* htim_x, TIM_TypeDef* tim_x, int IRQ_num,
                    uint32_t prescaler, uint32_t period, uint32_t grpPriority, uint32_t subPriority, uint32_t dbg_reg, uint32_t dbg_mode)
{
    TIMx_Init(htim_x, tim_x, IRQ_num, TIM_EN_IRQ, prescaler, TIM_COUNTERMODE_UP, TIM_CLOCKDIVISION_DIV1, period, REP_CNTR_INIT_VAL, TIM_AUTORELOAD_PRELOAD_ENABLE, TIM_CLOCKSOURCE_INTERNAL,
                TIM_TRGO_RESET, TIM_TRGO2_RESET, TIM_MASTERSLAVEMODE_DISABLE, TIM_OPMODE_SINGLE, grpPriority, subPriority);
    dbg_reg |= dbg_mode;
}


void DDI_DMAx_Init(DMA_HandleTypeDef* hdma_x, DMA_TypeDef* base_addr, int IRQ_num, DMA_Channel_TypeDef* inst,
                    uint32_t request, uint32_t ch_idx, uint32_t priority, uint32_t grpPriority, uint32_t subPriority)
{
    DMAx_Init(hdma_x, base_addr, IRQ_num, inst, DMA_EN_IRQ, request, ch_idx, DMA_PERIPH_TO_MEMORY, DMA_MDATAALIGN_BYTE, DMA_MINC_DISABLE, //MUST DISABLE so reset DMA read location to start of arr each measurement cycle
              DMA_NORMAL,DMA_PDATAALIGN_WORD,DMA_PINC_DISABLE,priority,grpPriority,subPriority);
}

void DDI_DMA2_UART4_TIM1_Init(DMA_HandleTypeDef* hdma2,uint32_t dma_priority, uint32_t dma_grpPriority, uint32_t dma_subPriority,
                              TIM_HandleTypeDef* htim1, uint32_t prescaler, uint32_t period, uint32_t tim_grpPriority, uint32_t tim_subPriority,
                              UART_HandleTypeDef* huart4, uint32_t mode, uint32_t hwflwctrl, uint32_t ov_flg, uint32_t obs_flg, void (*TxISR)(UART_HandleTypeDef* huart),uint32_t uart_grpPriority, uint32_t uart_subPriority){
    #define UART4_RX_CH_IDX 16 //LSb # of channel to select in DMA
    #define UART4_RX_REQUEST 0b10 //Set channel selection (as UART4_RX)
    #define UART4_TX_CH_IDX 8
    #define UART4_TX_REQUEST 0b10 //Same sel for TX, coincidentally

    DDI_DMAx_Init(hdma2,DMA2,DMA2_Channel5_IRQn,DMA2_Channel5,UART4_RX_REQUEST,UART4_RX_CH_IDX,dma_priority,dma_grpPriority,dma_subPriority);
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_LINKDMA(huart4,hdmarx,*hdma2); __HAL_LINKDMA(huart4,hdmatx,*hdma2);

    DDI_TIMx_Init(htim1, TIM1, TIM1_UP_TIM16_IRQn,prescaler, period, tim_grpPriority, tim_subPriority, DBGMCU->APB2FZ,DBGMCU_APB2FZ_DBG_TIM1_STOP);
    __TIM1_CLK_ENABLE();

    //UART_MODE_TX_RX (non-zero), UART_HWCONTROL_NONE, UART_OVERSAMPLING_16, UART_ONE_BIT_SAMPLE_DISABLE, 0,0
    DDI_UARTx_Init(huart4, UART4, UART4_IRQn, mode, hwflwctrl, ov_flg, obs_flg, TxISR,uart_grpPriority, uart_subPriority);
    __HAL_RCC_UART4_CLK_ENABLE();

    GPIOx_Init(GPIOA,GPIO_PIN_0,GPIO_MODE_AF_PP,GPIO_PULLUP,GPIO_AF8_UART4,GPIO_SPEED_FREQ_VERY_HIGH);
    GPIOx_Init(GPIOA,GPIO_PIN_1,GPIO_MODE_AF_PP,GPIO_PULLUP,GPIO_AF8_UART4,GPIO_SPEED_FREQ_VERY_HIGH);
    __HAL_RCC_GPIOA_CLK_ENABLE();

}

static void DDI_TIM_CB(TIM_HandleTypeDef* htim){
    HAL_TIM_Base_Stop(htim);	 	//Stop timer till next time
    htim->Instance->SR = 0; 		//Reset status register for future use
    state = ACQUIRED;
}

static void DDI_RXCplt_CB(DDI_TypeDef* DDI_device, int DDI_rxSz){
    if(chars_read < (DDI_rxSz-1) ){
        //When received DDI_DMA_READ_LEN, restart DMA read to receive the next batch
        chars_read+=DDI_DMA_READ_LEN;
        HAL_UART_Receive_DMA(&(DDI_device->UART), (DDI_device->data)+chars_read, DDI_DMA_READ_LEN);
    }
}

void DDI_Init_DMA(DMA_HandleTypeDef* hdma, DDI_TypeDef* DDI_device, volatile uint8_t* data_buf, uint32_t dma_priority, uint32_t dma_grpPriority, uint32_t dma_subPriority,
                uint32_t prescaler, uint32_t period, uint32_t tim_grpPriority, uint32_t tim_subPriority,
                uint32_t mode, uint32_t hwflwctrl, uint32_t ov_flg, uint32_t obs_flg, void (*TxISR)(UART_HandleTypeDef* huart),uint32_t uart_grpPriority, uint32_t uart_subPriority)
{
    DDI_device->data = data_buf;
    DDI_DMA2_UART4_TIM1_Init(hdma, dma_priority, dma_grpPriority, dma_subPriority,
                            &(DDI_device->TIM),prescaler,period,tim_grpPriority,tim_subPriority,
                            &(DDI_device->UART),mode,hwflwctrl,ov_flg,obs_flg,TxISR,uart_grpPriority,uart_subPriority);
    
    ///*
    DMA2_Channel5->CCR = 0x1200; DMA2_CSELR->CSELR = 0x20000;
    TIM1->CR1 = 0x88; TIM1->PSC = 0x2D1; TIM1->ARR = 0x5997; TIM1->DMAR = 0x88; TIM1->OR2 = 0x1; TIM1->OR3 = 0x1;
    GPIOA->MODER = 0xABFFFFFA; GPIOA->OSPEEDR = 0x0C00000F; GPIOA->PUPDR = 0x64000005; GPIOA->IDR = 0x8003; GPIOA->AFR[0] = 0x88;
    UART4->CR1 = 0x2D; UART4->BRR = 0x4D0A; UART4->ISR = 0x600090;
    //*/

    DDI_device->TIM_Elapsed_CB = &DDI_TIM_CB;
    DDI_device->UART_RXCplt_CB = &DDI_RXCplt_CB;
}

uint8_t DDI_getVal(DDI_TypeDef* DDI_device, uint8_t* old_data, DAQ_state* DDI_daq_state, GPIO_TypeDef* pwr_gpio_port, uint16_t* pwr_gpio_pin){
	//State machine...
	switch(state){
		case INIT:
			//Start sensor, initialize
            chars_read = 0;
			HAL_GPIO_TogglePin(pwr_gpio_port,*pwr_gpio_pin); //TO DO: possibly connect to MOSFET
			//Start timer
			HAL_TIM_Base_Start(&(DDI_device->TIM));
			HAL_TIM_Base_Start_IT(&(DDI_device->TIM));
            HAL_UART_Receive_DMA(&(DDI_device->UART), DDI_device->data, DDI_DMA_READ_LEN); //Start receiving data
            //Start to wait for reading
            //startTime = HAL_GetTick();
            state = WAIT;
			break;
		case WAIT:
			break;
		case ACQUIRED:
			HAL_UART_AbortReceive_IT(&(DDI_device->UART)); 		  //Abort last DMA read, since reached end (need to restart)
			HAL_GPIO_TogglePin(pwr_gpio_port,*pwr_gpio_pin);         //Turn off sensor
			state = INIT; 							               //Reset to read for next time
			*DDI_daq_state = GOT_DATA; 							  //Enable data transmission and disable next DAQ till complete

            chars_read++;
			#ifndef NO_DEBUG
				if(DDI_device->data[2]!=old_data[2]) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                memcpy(old_data,DDI_device->data,chars_read);	    //Copy to non-volatile
			#endif
			break;
		default:
            Error_Handler();
			break;
	}
    return chars_read;
}
