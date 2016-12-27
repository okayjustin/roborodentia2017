/**
 ******************************************************************************
 * File Name          : gpio.c
 * Description        : This file provides code for the configuration
 *                      of all used GPIO pins.
 ******************************************************************************
 */

#include "gpio.h"
#include "config.h"

void Setup_GPIO(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t speed) 
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = speed;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void Board_GPIO_Init(void)
{
    #ifdef ONBOARD_LED
    Setup_GPIO(ONBOARD_LED, ONBOARD_LED_INIT);
    #endif
}
