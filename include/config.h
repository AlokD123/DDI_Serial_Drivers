/**
  * @file           : config.h
  * @brief          : Common exported functions and structs in application exposed here
  * @author         : Alok Deshpande
  * @date           : May 17, 2020
  */

#ifndef CONFIG_H
#define CONFIG_H

#ifndef STM32L4xx_HAL_H
    #include "stm32l4xx_hal.h"
#endif

/* MACROS */
#define CONCAT2(x,y) x ## y
#define CONCAT3(x,y,z) x ## y ## z
#define XSTR(x) STR(x)
#define STR(x) #x

//Macro to set register. Only use with OR-MASK and OR-VALUE. Only use if more than 1 bit to set
//e.g. RCC_AHB2ENR_GPIOAEN only has 1 bit to set b/c 1UL is bit-shifted
#define SET_REGISTER(REG,MASK,VALUE) ( ((REG) & ~(MASK)) | VALUE)

/* DEFINES */
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* TYPEDEFS */
typedef enum DDI_state{INIT,WAIT,ACQUIRED} DDI_state;
typedef enum DAQ_state{START,GOT_DATA,SENT_DATA} DAQ_state;

/* Exported Functions */
void Error_Handler(void);

void UARTx_Init(UART_HandleTypeDef* huart_x, USART_TypeDef* UART_x, int IRQ_num, uint8_t boolEnIRQ,
                uint32_t baud, uint32_t word_len, uint32_t stop_bits, uint32_t parity, uint32_t mode,
                uint32_t hwflwctrl, uint32_t ov_flg, uint32_t obs_flg, uint32_t adv_feat_flg,
                void (*TxISR)(UART_HandleTypeDef* huart),void (*RxISR)(UART_HandleTypeDef* huart),
                uint32_t grpPriority, uint32_t subPriority);

void TIMx_Init(TIM_HandleTypeDef* htim_x, TIM_TypeDef* tim_x, int IRQ_num, uint8_t boolEnIRQ,
                uint32_t prescaler, uint32_t mode, uint32_t clk_div, uint32_t period, uint32_t rep_cntr_val, uint32_t auto_rp, uint32_t clk_src,
                uint32_t master_out_trig1, uint32_t master_out_trig2, uint32_t master_slave_mode, uint32_t op_mode, uint32_t grpPriority, uint32_t subPriority);

void DMAx_Init(DMA_HandleTypeDef* hdma_x, DMA_TypeDef* base_addr, int IRQ_num, DMA_Channel_TypeDef* inst, uint8_t boolEnIRQ,
                uint32_t request, uint32_t ch_idx, uint32_t dir, uint32_t mem_align, uint32_t m_inc, uint32_t mode,
                uint32_t periph_align, uint32_t p_inc, uint32_t priority, uint32_t grpPriority, uint32_t subPriority);

void GPIOx_Init(GPIO_TypeDef* gpio_x, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t alt, uint32_t speed);

#endif
