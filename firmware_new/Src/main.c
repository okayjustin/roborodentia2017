/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "imu.h"
#include "vl53l0x_top.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DATA_PRINT_EN
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void TimeStamp_Reset();
uint32_t TimeStamp_Get();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

    // Initialize all clocks so we don't run into weird clock problems...
    RCC->AHB1ENR |= 0xFFFFFFFF;
    RCC->AHB2ENR |= 0xFFFFFFFF;
    RCC->AHB3ENR |= 0xFFFFFFFF;
    RCC->APB1ENR |= 0xFFFFFFFF;
    RCC->APB2ENR |= 0xFFFFFFFF;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
    // Turn off all the rangefinders
    HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, 0);
    HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, 0);
    HAL_GPIO_WritePin(SW3_GPIO_Port, SW3_Pin, 0);
    HAL_GPIO_WritePin(SW4_GPIO_Port, SW4_Pin, 0);

    TimeStamp_Reset();
    serviceUART();
    printf("Enabling IMU...\r\n");
    IMU_begin();
    printf("Initializing rangefinders...\r\n");
    VL53L0X_begin();
    VL53L0X_SetupSingleShot();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint32_t print_time = 0;
    uint32_t cur_time = 0;
    uint32_t dt = 0;  // Units of 0.1 ms based on Timer5

    while (1)
    {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

        dt = TimeStamp_Get() - print_time;
        if (dt < 88){
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

            gyro_read();
            accelerometer_read();
            magnetometer_read();
            rangefinderRead(0);
            rangefinderRead(1);
            rangefinderRead(2);
            rangefinderRead(3);
        }
        cur_time = TimeStamp_Get();
        dt = cur_time - print_time;
        if (dt > 99){   // Data rate = 100 Hz
            print_time = cur_time;
#ifdef DATA_PRINT_EN
            printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", 
                    dt, magData.orientation, 
                    rangeData[0], rangeData[1], 
                    rangeData[2], rangeData[3], 
                    accelData.x, accelData.y, accelData.z, 
                    gyroData.x, gyroData.y, gyroData.z);
#endif
        }
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void TimeStamp_Reset(){
    HAL_TIM_Base_Start(&htim5);
    htim5.Instance->CNT=0;
}

uint32_t TimeStamp_Get(){
    return htim5.Instance->CNT;
}

// Redirect printf to UART
int _write (int fd, char *ptr, int len) 
{ 
    transmitUART(ptr, len);
    return len; 
}

// Execute a command from the console
void consoleCommand(uint8_t *ptr, int len)
{
    // V for version
    if (ptr[0] == 'V' || ptr[0] == 'v'){
        printf("Robojustin v0.2\r\n");
    }

    // I for I2C bus scan
    else if (ptr[0] == 'I' || ptr[0] == 'i'){
        // I2C2
        if (ptr[1] == '2'){
            I2C_Scan(&hi2c2);
        }
        // I2C3
        if (ptr[1] == '3'){
            I2C_Scan(&hi2c3);
        }
    }

    // B for sending IMU data
    else if (ptr[0] == 'B' || ptr[0] == 'b') {
        gyro_read();
        accelerometer_read();
        magnetometer_read();
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", 
                accelData.x, accelData.y, accelData.z, 
                gyroData.x, gyroData.y, gyroData.z,
                magData.x, magData.y, magData.z);
    }

    // M for motor control commands
    else if (ptr[0] == 'M' || ptr[0] == 'm'){  
        GPIO_TypeDef *GPIOx;
        uint16_t GPIO_Pin;
        GPIO_PinState PinState;
        uint32_t TIM_Channel;

        // Set variables based on which motor needs to be controlled. Motor 1 is front left, increments clockwise
        if (ptr[1] == '0'){  // 0 for front left
            GPIOx = MOTOR_FL_DIR_GPIO_Port;
            GPIO_Pin = MOTOR_FL_DIR_Pin;
            TIM_Channel = TIM_CHANNEL_1;
        }
        else if (ptr[1] == '1'){  // 1 for front right
            GPIOx = MOTOR_FR_DIR_GPIO_Port;
            GPIO_Pin = MOTOR_FR_DIR_Pin;
            TIM_Channel = TIM_CHANNEL_2;
        }
        else if (ptr[1] == '2'){  // 2 for back right
            GPIOx = MOTOR_BR_DIR_GPIO_Port;
            GPIO_Pin = MOTOR_BR_DIR_Pin;
            TIM_Channel = TIM_CHANNEL_3;
        }
        else if (ptr[1] == '3'){  // 3 for back right
            GPIOx = MOTOR_BL_DIR_GPIO_Port;
            GPIO_Pin = MOTOR_BL_DIR_Pin;
            TIM_Channel = TIM_CHANNEL_4;
        }
        else {
            return;
        }

        // Direction control commands
        if (ptr[2] == 'D' || ptr[2] == 'd'){
            if (ptr[3] == 'F' || ptr[3] == 'f'){
                PinState = GPIO_PIN_RESET;
            }
            else if (ptr[3] == 'R' || ptr[3] == 'r'){
                PinState = GPIO_PIN_SET;
            }
            else {
                return;
            }
            HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
        }

        // S for speed command
        if (ptr[2] == 'S' || ptr[2] == 's'){  
            TIM_OC_InitTypeDef sConfigOC;
            sConfigOC.OCMode = TIM_OCMODE_PWM1;
            sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
            sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
            sConfigOC.Pulse = atoi((char *)ptr + 3); // Accepts 0-2047

            // Alter the PWM duty cycle and start PWM again
            if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_Channel) != HAL_OK) { Error_Handler(); }
            if (HAL_TIM_PWM_Start(&htim4, TIM_Channel) != HAL_OK){ Error_Handler(); }
        }
    }
}            
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    printf("Entered error handler.\r\n");
    while(1) 
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        HAL_Delay(100);
    }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    printf("Assert failed.\r\n");
    while(1) 
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        HAL_Delay(100);
    }
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
