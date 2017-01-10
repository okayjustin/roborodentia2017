#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Setup_GPIO(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t speed); 
void Board_GPIO_Init(void);
void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

