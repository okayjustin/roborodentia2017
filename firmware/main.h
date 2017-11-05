
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define RANGE_I2C_ADDR_INITIAL 0x52

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
// Reserved GPIO
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

//#define I2C1_SDA_Pin GPIO_PIN_7 used for motor
//#define I2C1_SDA_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB

#define I2C2_SDA_Pin GPIO_PIN_12
#define I2C2_SDA_GPIO_Port GPIOC
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB

#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB


// Used GPIO
#define BOARD_BUTTON_Pin GPIO_PIN_13
#define BOARD_BUTTON_GPIO_Port GPIOC
#define BOARD_LED_Pin GPIO_PIN_5
#define BOARD_LED_GPIO_Port GPIOA

#define RANGEX_XSHUT_Pin GPIO_PIN_0
#define RANGEX_XSHUT_GPIO_Port GPIOA
#define RANGEX_INT_Pin GPIO_PIN_1
#define RANGEX_INT_GPIO_Port GPIOA

#define RANGEY_XSHUT_Pin GPIO_PIN_2
#define RANGEY_XSHUT_GPIO_Port GPIOC
#define RANGEY_INT_Pin GPIO_PIN_1
#define RANGEY_INT_GPIO_Port GPIOC

#define MOTOR_L_PWM_GPIO_Port GPIOB
#define MOTOR_L_PWM_Pin GPIO_PIN_7 
#define MOTOR_L_DIR_GPIO_Port GPIOA
#define MOTOR_L_DIR_Pin GPIO_PIN_8 

#define MOTOR_R_PWM_GPIO_Port GPIOB
#define MOTOR_R_PWM_Pin GPIO_PIN_8 
#define MOTOR_R_DIR_GPIO_Port GPIOA
#define MOTOR_R_DIR_Pin GPIO_PIN_9 

// Planned GPIO
#define SERVO_ARM_Pin GPIO_PIN_6
#define SERVO_ARM_GPIO_Port GPIOA
#define SERVO_DELIVER_Pin GPIO_PIN_7
#define SERVO_DELIVER_GPIO_Port GPIOA
#define SERVO_EXTRA_1_Pin GPIO_PIN_0
#define SERVO_EXTRA_1_GPIO_Port GPIOB
#define SERVO_EXTRA_2_Pin GPIO_PIN_1
#define SERVO_EXTRA_2_GPIO_Port GPIOB
#define MOTOR_N_PWM_Pin GPIO_PIN_8
#define MOTOR_N_PWM_GPIO_Port GPIOA
#define MOTOR_S_PWM_Pin GPIO_PIN_9
#define MOTOR_S_PWM_GPIO_Port GPIOA
#define MOTOR_W_PWM_Pin GPIO_PIN_10
#define MOTOR_W_PWM_GPIO_Port GPIOA
#define MOTOR_E_PWM_Pin GPIO_PIN_11
#define MOTOR_E_PWM_GPIO_Port GPIOA
#define MOTOR_VERT_PWM_Pin GPIO_PIN_8
#define MOTOR_VERT_PWM_GPIO_Port GPIOB
#define MOTOR_EXTRA_PWM_Pin GPIO_PIN_9
#define MOTOR_EXTRA_PWM_GPIO_Port GPIOB


int _write (int fd, char *ptr, int len); 

#endif
