/*
 * DDI Serial Library
 *
 *  Created on: May 17, 2020
 *      Author: Alok Deshpande
 */

#ifndef DDI_H_
#define DDI_H_

#ifndef __STM32L496xx_H
	#include "stm32l496xx.h"
#endif

#ifndef CONFIG_H
	#include "config.h"
#endif

#include <string.h>

#define DDI_DMA_READ_LEN  1u
#define DDI_SERIAL_BAUD   1200
#define UART_EN_IRQ       1
#define UART_CLEAR_FLAGS  0x121bdf
#define TIM_EN_IRQ        1
#define REP_CNTR_INIT_VAL 0
#define DMA_EN_IRQ        1


typedef struct DDI_TypeDef{
	//DDI power connection
	GPIO_TypeDef* pwr_gpio_port; uint16_t pwr_gpio_pin;
	//Peripherals
	UART_HandleTypeDef UART;
	TIM_HandleTypeDef TIM;
	//Data collected
	volatile uint8_t* data;
	//Callbacks for timeout and data read
	void (*TIM_Elapsed_CB)(TIM_HandleTypeDef*);
	void (*UART_RXCplt_CB)(struct DDI_TypeDef* DDI_device, int DDI_rxSz);
}DDI_TypeDef;

/**
  * @brief Function to initialize a DDI device using DMA reads
  * @param hdma: DMA for DDI
  * @param DDI_device: DDI device instance
  * @param data_buf: buffer to store read data
  * Other important parameters for DDI device (DMA, UART and TIM parameters) exposed.
  * @retval None
  */
void DDI_Init_DMA(DMA_HandleTypeDef* hdma, DDI_TypeDef* DDI_device, volatile uint8_t* data_buf, uint32_t dma_priority, uint32_t dma_grpPriority, uint32_t dma_subPriority,
                uint32_t prescaler, uint32_t period, uint32_t tim_grpPriority, uint32_t tim_subPriority,
                uint32_t mode, uint32_t hwflwctrl, uint32_t ov_flg, uint32_t obs_flg, void (*TxISR)(UART_HandleTypeDef* huart),uint32_t uart_grpPriority, uint32_t uart_subPriority);


/**
  * @brief Function to do non-blocking read from a DDI device using DMA
  * @param DDI_device: DDI device instance
  * @param old_buf: last read data, for reference
  * @param DDI_daq_state: state of data acquisition (reading vs output to application) for DDI device
  * @param pwr_gpio_port: GPIO port for powering DDI device
  * @param pwr_gpio_pin: GPIO pin for powering DDI device
  * @retval number of bytes read
  */
uint8_t DDI_getVal(DDI_TypeDef* DDI_device, uint8_t* old_data, DAQ_state* DDI_daq_state, GPIO_TypeDef* pwr_gpio_port, uint16_t* pwr_gpio_pin);


#endif /* DDI_H_ */
