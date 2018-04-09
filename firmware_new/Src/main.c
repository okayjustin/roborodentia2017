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
#include <stdio.h>
#include "imu.h"
#include "vl53l0x_top.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct motor_t {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIO_PinState PinState;

    TIM_HandleTypeDef *TIM_Handle;
    TIM_OC_InitTypeDef sConfigOC;
    uint32_t TIM_Channel;
};

struct motor_t motorConfigs[4] = { 
    {MOTOR_FL_DIR_GPIO_Port, MOTOR_FL_DIR_Pin, GPIO_PIN_RESET, 0, {0}, TIM_CHANNEL_2}, 
    {MOTOR_FR_DIR_GPIO_Port, MOTOR_FR_DIR_Pin, GPIO_PIN_RESET, 0, {0}, TIM_CHANNEL_4}, 
    {MOTOR_BR_DIR_GPIO_Port, MOTOR_BR_DIR_Pin, GPIO_PIN_RESET, 0, {0}, TIM_CHANNEL_1}, 
    {MOTOR_BL_DIR_GPIO_Port, MOTOR_BL_DIR_Pin, GPIO_PIN_RESET, 0, {0}, TIM_CHANNEL_3}}; 
struct motor_t* motorConfigsPtr = motorConfigs;

// WARNING!! Do not spam the UART or else Raspberry PI doesn't communicate well
//#define DATA_PRINT_EN
//
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

  /* USER CODE BEGIN 2 */
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

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = 0;

    int i;
    for (i=0; i<4; i++) {
        motorConfigs[i].TIM_Handle = &htim4; 
        motorConfigs[i].sConfigOC = sConfigOC; 
    }

    TimeStamp_Reset();
    serviceUART();
    printf("Enabling IMU...\r\n");
    IMU_begin();
    printf("Initializing rangefinders...\r\n");
    VL53L0X_begin();

    // Move servo to center position
    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }

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

//        dt = TimeStamp_Get() - print_time;
//        if (dt < 98) gyro_read();
//        dt = TimeStamp_Get() - print_time;
//        if (dt < 98) accelerometer_read();
//        dt = TimeStamp_Get() - print_time;
//        if (dt < 98) magnetometer_read();

//        cur_time = TimeStamp_Get();
//        dt = cur_time - print_time;
//        if (dt > 499){   // Data rate = 20 Hz
//            rangefinderRead(0);
//            rangefinderRead(1);
//            rangefinderRead(2);
//            rangefinderRead(3);
//            magnetometer_read();
//            gyro_read();
//            accelerometer_read();
//            print_time = cur_time;
//#ifdef DATA_PRINT_EN
//            printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", 
//                    dt, magData.orientation, 
//                    rangeData[0], rangeData[1], 
//                    rangeData[2], rangeData[3], 
//                    accelData.x, accelData.y, accelData.z, 
//                    gyroData.x, gyroData.y, gyroData.z);
//#endif
//        }

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

    // A for start sensor read
    else if (ptr[0] == 'A' || ptr[0] == 'a') {
        rangefinderRead(0);
        rangefinderRead(1);
        rangefinderRead(2);
        rangefinderRead(3);
        magnetometer_read();
        accelerometer_read();
//        gyro_read();
    }

    // B for send data in binary
    else if (ptr[0] == 'B' || ptr[0] == 'b') {
        unsigned char bytes[21];
        bytes[ 0] = (rangeData[0] >> 8) & 0xFF;
        bytes[ 1] =  rangeData[0]       & 0xFF;

        bytes[ 2] = (rangeData[1] >> 8) & 0xFF;
        bytes[ 3] =  rangeData[1]       & 0xFF;

        bytes[ 4] = (rangeData[2] >> 8) & 0xFF;
        bytes[ 5] =  rangeData[2]       & 0xFF;

        bytes[ 6] = (rangeData[3] >> 8) & 0xFF;
        bytes[ 7] =  rangeData[3]       & 0xFF;

        bytes[ 8] = (magData.x >> 8) & 0xFF;
        bytes[ 9] =  magData.x       & 0xFF;
        bytes[10] = (magData.y >> 8) & 0xFF;
        bytes[11] =  magData.y       & 0xFF;
        bytes[12] = (magData.z >> 8) & 0xFF;
        bytes[13] =  magData.z       & 0xFF;

        bytes[14] = (accelData.x >> 8) & 0xFF;
        bytes[15] =  accelData.x       & 0xFF;
        bytes[16] = (accelData.y >> 8) & 0xFF;
        bytes[17] =  accelData.y       & 0xFF;
        bytes[18] = (accelData.z >> 8) & 0xFF;
        bytes[19] =  accelData.z       & 0xFF;

//        bytes[20] = (gyroData.x >> 8) & 0xFF;
//        bytes[21] =  gyroData.x       & 0xFF;
//        bytes[22] = (gyroData.y >> 8) & 0xFF;
//        bytes[23] =  gyroData.y       & 0xFF;
//        bytes[24] = (gyroData.z >> 8) & 0xFF;
//        bytes[25] =  gyroData.z       & 0xFF;
//        bytes[20] = (magData.orientation >> 8) & 0xFF;
//        bytes[21] =  magData.orientation       & 0xFF;
        bytes[20] = '\n';

        transmitUART(bytes, 21);
    }
     
    // D for send data in human readable format
    else if (ptr[0] == 'D' || ptr[0] == 'd') {
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", 
                rangeData[0], rangeData[1], 
                rangeData[2], rangeData[3], 
                magData.x, magData.y, magData.z,
                accelData.x, accelData.y, accelData.z, 
                gyroData.x, gyroData.y, gyroData.z);
    }
    
    // M for motor control commands
    else if (ptr[0] == 'M' || ptr[0] == 'm'){  
        int motorU[4];
        sscanf (ptr,"%*s %d %d %d %d", &motorU[0], &motorU[1], &motorU[2], &motorU[3]);
        printf ("%d %d %d %d\n", motorU[0], motorU[1], motorU[2], motorU[3]);

        int i;
        for (i=0; i<4; i++) {
            if (motorU[i] < 0){
                motorConfigs[i].PinState = GPIO_PIN_SET;
                motorConfigs[i].sConfigOC.Pulse = 2047 + motorU[i]; 
            } else {
                motorConfigs[i].PinState = GPIO_PIN_RESET;
                motorConfigs[i].sConfigOC.Pulse = motorU[i]; 
            }
        }

        for (i=0; i<4; i++) {
            // Set the direction pin
            HAL_GPIO_WritePin(motorConfigs[i].GPIOx, motorConfigs[i].GPIO_Pin, motorConfigs[i].PinState);

            // Alter the PWM duty cycle and start PWM again
            if (HAL_TIM_PWM_ConfigChannel(motorConfigs[i].TIM_Handle, &motorConfigs[i].sConfigOC,
                       motorConfigs[i].TIM_Channel) != HAL_OK) { Error_Handler(); }
            if (HAL_TIM_PWM_Start(motorConfigs[i].TIM_Handle, motorConfigs[i].TIM_Channel)
                    != HAL_OK){ Error_Handler(); }
        }
    }
    // S for servo commands
    else if (ptr[0] == 'S' || ptr[0] == 's'){  
        TIM_OC_InitTypeDef sConfigOC;
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        if (ptr[1] == '0'){         // Set servo to 0 degrees
            sConfigOC.Pulse = SERVO1_PULSE_1;
        }
        else if (ptr[1] == '1'){    // Set servo to slot 1 release
            sConfigOC.Pulse = SERVO1_PULSE_1;
        }
        else if (ptr[1] == '2'){    // Set servo to slot 2 release
            sConfigOC.Pulse = SERVO1_PULSE_2;
        }
        else if (ptr[1] == '3'){    // Set servo to slot 3 release
            sConfigOC.Pulse = SERVO1_PULSE_3;
        }
        else if (ptr[1] == '4'){    // Set servo to slot 4 release
            sConfigOC.Pulse = SERVO1_PULSE_4;
        }

        if (ptr[2] == 'S' || ptr[2] == 's'){  
            sConfigOC.Pulse = atoi((char *)ptr + 3);
        }

        // Alter the PWM duty cycle and start PWM again
        if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
        if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }
    }
    // L for launcher macro commands
    else if (ptr[0] == 'L' || ptr[0] == 'l'){  
        uint32_t slot_pulse1 = SERVO1_PULSE_90;
        uint32_t slot_pulse2 = SERVO1_PULSE_90;

        // Fire left side slots
        if (ptr[1] == 'L' || ptr[1] == 'l'){    
            slot_pulse1 = SERVO1_PULSE_2;
            slot_pulse2 = SERVO1_PULSE_1;
        }
        // Fire right side slots 
        else if (ptr[1] == 'R' || ptr[1] == 'r'){    
            slot_pulse1 = SERVO1_PULSE_3;
            slot_pulse2 = SERVO1_PULSE_4;
        }
        // Stop    
        else if (ptr[1] == 'S' || ptr[1] == 's'){    
            // Disable blower and launcher motors
            HAL_GPIO_WritePin(BLOWER_DIR_GPIO_Port, BLOWER_DIR_Pin, 0);
            HAL_GPIO_WritePin(MOTOR_LAUNCH_DIR_GPIO_Port, MOTOR_LAUNCH_DIR_Pin, 0);
        }

        if (ptr[1] == 'L' || ptr[1] == 'l' || ptr[1] == 'R' || ptr[1] == 'r'){
            uint32_t cur_time = 0;
            uint32_t cur_time2 = 0;
            uint32_t wiggle_amt = 100;
            uint32_t wiggle_period = 1000;


            TIM_OC_InitTypeDef sConfigOC;
            sConfigOC.OCMode = TIM_OCMODE_PWM1;
            sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
            sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

            // Enable blower and launcher motors
            HAL_GPIO_WritePin(BLOWER_DIR_GPIO_Port, BLOWER_DIR_Pin, 1);
            HAL_GPIO_WritePin(MOTOR_LAUNCH_DIR_GPIO_Port, MOTOR_LAUNCH_DIR_Pin, 1);

            // Wait for balls to finish firing first slot
            cur_time = TimeStamp_Get();
            while (TimeStamp_Get() - cur_time < 13000){
                // Move servo to release first slot and jiggle it
                sConfigOC.Pulse = slot_pulse1 + wiggle_amt;
                if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
                if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }

                cur_time2 = TimeStamp_Get();
                while (TimeStamp_Get() - cur_time2 < wiggle_period){}

                sConfigOC.Pulse = slot_pulse1 - wiggle_amt;
                if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
                if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }

                cur_time2 = TimeStamp_Get();
                while (TimeStamp_Get() - cur_time2 < wiggle_period){}
            }

            // Wait for balls to finish firing second slot
            cur_time = TimeStamp_Get();
            while (TimeStamp_Get() - cur_time < 27000){
                // Move servo to release first slot and jiggle it
                sConfigOC.Pulse = slot_pulse2 + wiggle_amt;
                if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
                if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }

                cur_time2 = TimeStamp_Get();
                while (TimeStamp_Get() - cur_time2 < wiggle_period){}

                sConfigOC.Pulse = slot_pulse2 - wiggle_amt;
                if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
                if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }

                cur_time2 = TimeStamp_Get();
                while (TimeStamp_Get() - cur_time2 < wiggle_period){}
            }

            // Disable blower and launcher motors
            HAL_GPIO_WritePin(BLOWER_DIR_GPIO_Port, BLOWER_DIR_Pin, 0);
            HAL_GPIO_WritePin(MOTOR_LAUNCH_DIR_GPIO_Port, MOTOR_LAUNCH_DIR_Pin, 0);

            // Move servo to center position
            sConfigOC.Pulse = SERVO1_PULSE_90;
            if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
            if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }
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
