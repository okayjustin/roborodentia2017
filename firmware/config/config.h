#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "main.h"

#define FIRMWARE_MAJOR_VERSION   1
#define FIRMWARE_MINOR_VERSION   1
#define FIRMWARE_REVISION  0

#define AHB_CLK_FREQ      180000000UL
#define APB1_CLK_FREQ      45000000UL
#define APB1_TMR_CLK_FREQ  90000000UL
#define APB2_CLK_FREQ      90000000UL
#define APB2_TMR_CLK_FREQ 180000000UL

//Callback Timer
#define CALLBACK_TIMER  TIM9
#define CALLBACK_FREQUENCY 1000
#define TIMER_FREQUENCY 100000
#define TIMER_MAX_VALUE    0xFFFF
#define MAX_NUM_CALLBACK_TIMERS  32
#define CALLBACK_TIMER_IRQ TIM1_BRK_TIM9_IRQn
#define CALLBACK_TIMER_ISR TIM1_BRK_TIM9_IRQHandler

//I2C Peripheral
#define I2C_MAX_NUM_TRANSACTIONS 8
#define I2C_TRANSACTION_TX_BUF_SIZE 64
#define I2C_TRANSACTION_RX_BUF_SIZE 64

#define GPIO_I2C_SCL  GPIO_PIN_10
#define GPIO_I2C_SDA  GPIO_PIN_11
#define GPIO_I2C_PORT GPIOB
#define GPIO_I2C_AF   GPIO_AF4_I2C2

#define I2C1_TX_DMA_CHANNEL DMA_CHANNEL_1
#define I2C1_TX_DMA_STREAM DMA1_Stream6
#define I2C1_RX_DMA_CHANNEL DMA_CHANNEL_1
#define I2C1_RX_DMA_STREAM DMA1_Stream0
#define I2C1_DMA_TX_IRQn DMA1_Stream6_IRQn
#define I2C1_DMA_RX_IRQn DMA1_Stream0_IRQn
#define I2C1_DMA_TX_IRQHandler DMA1_Stream6_IRQHandler
#define I2C1_DMA_RX_IRQHandler DMA1_Stream0_IRQHandler 

#define I2C2_TX_DMA_CHANNEL DMA_CHANNEL_7
#define I2C2_TX_DMA_STREAM DMA1_Stream7
#define I2C2_RX_DMA_CHANNEL DMA_CHANNEL_7
#define I2C2_RX_DMA_STREAM DMA1_Stream2
#define I2C2_DMA_TX_IRQn DMA1_Stream7_IRQn
#define I2C2_DMA_RX_IRQn DMA1_Stream2_IRQn
#define I2C2_DMA_TX_IRQHandler DMA1_Stream7_IRQHandler
#define I2C2_DMA_RX_IRQHandler DMA1_Stream2_IRQHandler 

//#define ONBOARD_LED     GPIOA, GPIO_PIN_5
//#define ONBOARD_LED_INIT    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW

//Iris GPIO
#define GPIO_USART_TX   GPIO_PIN_6
#define GPIO_USART_RX   GPIO_PIN_7
#define GPIO_USART_PORT GPIOB
#define GPIO_USART_AF   GPIO_AF7_USART1;

#define COMMS_USART        USART1
#define COMMS_BAUDRATE     115200
#define COMMS_TX_BUFFER_SIZE (2048)
#define COMMS_RX_BUFFER_SIZE (1024)
#define COMMS_TX_NUM_BUFFERS (4)

#define COMMS_TX_DMA       DMA2
#define COMMS_RX_DMA       DMA2
#define COMMS_TX_DMA_STREAM DMA2_Stream7
#define COMMS_RX_DMA_STREAM DMA2_Stream5
#define COMMS_TX_DMA_CHANNEL DMA_CHANNEL_4
#define COMMS_RX_DMA_CHANNEL DMA_CHANNEL_4
#define COMMS_TX_DMA_IRQ   DMA2_Stream7_IRQn
#define COMMS_TX_DMA_ISR   DMA2_Stream7_IRQHandler

#endif
