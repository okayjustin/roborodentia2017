/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define RF1_G_Pin GPIO_PIN_0
#define RF1_G_GPIO_Port GPIOC
#define RF2_G_Pin GPIO_PIN_1
#define RF2_G_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_2
#define SW1_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_3
#define SW3_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define M3_1_Pin GPIO_PIN_6
#define M3_1_GPIO_Port GPIOA
#define M3_3_Pin GPIO_PIN_7
#define M3_3_GPIO_Port GPIOA
#define M3_2_Pin GPIO_PIN_4
#define M3_2_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOC
#define RF3_G_Pin GPIO_PIN_0
#define RF3_G_GPIO_Port GPIOB
#define LED_DBG1_Pin GPIO_PIN_1
#define LED_DBG1_GPIO_Port GPIOB
#define M1_2_Pin GPIO_PIN_2
#define M1_2_GPIO_Port GPIOB
#define M3_4_Pin GPIO_PIN_13
#define M3_4_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_14
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_15
#define SERVO2_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_6
#define SW4_GPIO_Port GPIOC
#define M1_4_Pin GPIO_PIN_7
#define M1_4_GPIO_Port GPIOC
#define M2_2_Pin GPIO_PIN_10
#define M2_2_GPIO_Port GPIOA
#define RF4_G_Pin GPIO_PIN_11
#define RF4_G_GPIO_Port GPIOA
#define RF5_G_Pin GPIO_PIN_12
#define RF5_G_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LED_DBG2_Pin GPIO_PIN_10
#define LED_DBG2_GPIO_Port GPIOC
#define RF6_G_Pin GPIO_PIN_11
#define RF6_G_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define M2_4_Pin GPIO_PIN_5
#define M2_4_GPIO_Port GPIOB
#define M1_1_Pin GPIO_PIN_6
#define M1_1_GPIO_Port GPIOB
#define M1_3_Pin GPIO_PIN_7
#define M1_3_GPIO_Port GPIOB
#define M2_1_Pin GPIO_PIN_8
#define M2_1_GPIO_Port GPIOB
#define M2_3_Pin GPIO_PIN_9
#define M2_3_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */
#define MOTOR_FL_PWM_GPIO_Port M1_3_GPIO_Port    
#define MOTOR_FL_PWM_Pin M1_3_Pin 
#define MOTOR_FL_DIR_GPIO_Port M1_4_GPIO_Port   
#define MOTOR_FL_DIR_Pin M1_4_Pin 

#define MOTOR_FR_PWM_GPIO_Port M2_3_GPIO_Port
#define MOTOR_FR_PWM_Pin M2_3_Pin  
#define MOTOR_FR_DIR_GPIO_Port M2_4_GPIO_Port
#define MOTOR_FR_DIR_Pin M2_4_Pin  

#define MOTOR_BR_PWM_GPIO_Port M1_1_GPIO_Port
#define MOTOR_BR_PWM_Pin M1_1_Pin   
#define MOTOR_BR_DIR_GPIO_Port M1_2_GPIO_Port
#define MOTOR_BR_DIR_Pin M1_2_Pin   

#define MOTOR_BL_PWM_GPIO_Port M2_1_GPIO_Port
#define MOTOR_BL_PWM_Pin M2_1_Pin   
#define MOTOR_BL_DIR_GPIO_Port M2_2_GPIO_Port
#define MOTOR_BL_DIR_Pin M2_2_Pin   

#define MOTOR_LAUNCH_TOP_PWM_GPIO_Port M3_1_GPIO_Port
#define MOTOR_LAUNCH_TOP_PWM_Pin M3_1_Pin   
#define MOTOR_LAUNCH_TOP_DIR_GPIO_Port M3_2_GPIO_Port
#define MOTOR_LAUNCH_TOP_DIR_Pin M3_2_Pin   

#define MOTOR_LAUNCH_BOT_PWM_GPIO_Port M3_3_GPIO_Port
#define MOTOR_LAUNCH_BOT_PWM_Pin M3_3_Pin   
#define MOTOR_LAUNCH_BOT_DIR_GPIO_Port M3_4_GPIO_Port
#define MOTOR_LAUNCH_BOT_DIR_Pin M3_4_Pin   
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
