#ifndef CONFIG_H
#define CONFIG_H

#ifndef STM32L4xx_HAL_H
    #include "stm32l4xx_hal.h"
#endif

/* Private defines -----------------------------------------------------------*/
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define SOIL_DMA_READ_BYTES 1u
#define RX_SIZE 30

/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef huart4;
extern volatile uint8_t soil_data[RX_SIZE];
extern volatile uint8_t soil_chars_read;

typedef enum DDI_state{INIT,WAIT,ACQUIRED} DDI_state;
typedef enum DAQ_state{START,GOT_DATA,SENT_DATA} DAQ_state;

#endif
