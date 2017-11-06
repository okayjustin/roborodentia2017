
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "config.h"
#include "lsm303.h"
#include "vl53l0x_top.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Error_Handler(void);
void TimeStamp_Reset();
uint32_t TimeStamp_Get();
void Initialization();

void _init(void) {;}


int main(void)
{
    Initialization();
    //HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_SET);

    // Main loop
    uint32_t print_time = 0;
    uint32_t cur_time = 0;
    uint32_t dt = 0;  // Units of 0.1 ms based on Timer5
    
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK){ Error_Handler(); }

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        LSM303_read();
        rangefinderRead(0);
        rangefinderRead(1);
        cur_time = TimeStamp_Get();
        dt = cur_time - print_time;
        if (dt > 98){   // Ends up w/ data rate ~100 Hz
            print_time = cur_time;
            printf("%lu,%d,%d,%d\r\n", dt, magData.orientation, rangeMillimeterX, rangeMillimeterY);
        }
    }
}


/* Private functions ---------------------------------------------------------*/
void Initialization(){
    // Initialize all clocks so we don't run into weird clock problems...
    RCC->AHB1ENR |= 0xFFFFFFFF;
    RCC->AHB2ENR |= 0xFFFFFFFF;
    RCC->AHB3ENR |= 0xFFFFFFFF;
    RCC->APB1ENR |= 0xFFFFFFFF;
    RCC->APB2ENR |= 0xFFFFFFFF;

    // Reset of all peripherals, Initializes the Flash interface and the Systick
    HAL_Init();
    // Configure the system clock to 180 MHz
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    //MX_TIM1_Init();
    //MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init(); // Used for UART timestamping
    //MX_TIM10_Init();
    //MX_TIM11_Init();
    serviceUART();
    //MX_I2C1_Init();
    MX_I2C2_Init();

    printf("Enabling magnetometer...\r\n");
    LSM303_begin();
    printf("Done enabling magnetometer...\r\n");
    HAL_Delay(100);

    TimeStamp_Reset();

    printf("Initializing rangefinders...\r\n");
    VL53L0X_begin();
    VL53L0X_SetupSingleShot();
    printf("Done initializing rangefinders.\r\n");
}

// Execute a command from the console
void consoleCommand(uint8_t *ptr, int len)
{
    // M for motor control commands
    if (ptr[0] == 'M'){  
        GPIO_TypeDef *GPIOx;
        uint16_t GPIO_Pin;
        GPIO_PinState PinState;
//        uint32_t TIM_Channel;

        // Set variables based on left or right side control
        if (ptr[1] == 'L'){  // L for left side
            GPIOx = MOTOR_L_DIR_GPIO_Port;
            GPIO_Pin = MOTOR_L_DIR_Pin;
//            TIM_Channel = TIM_CHANNEL_2;
        }
        else if (ptr[1] == 'R'){  // L for left side
            GPIOx = MOTOR_R_DIR_GPIO_Port;
            GPIO_Pin = MOTOR_R_DIR_Pin;
//            TIM_Channel = TIM_CHANNEL_3;
        }
        else {
            return;
        }

        // Direction control commands
        if (ptr[2] == 'D'){
            if (ptr[3] == 'F'){
                PinState = GPIO_PIN_RESET;
            }
            else if (ptr[3] == 'R'){
                PinState = GPIO_PIN_SET;
            }
            else {
                return;
            }
            HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
        }

        // S for speed command
        if (ptr[2] == 'S'){  
            TIM_OC_InitTypeDef sConfigOC;
            sConfigOC.OCMode = TIM_OCMODE_PWM1;
            sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
            sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
            sConfigOC.Pulse = atoi((char *)ptr + 3);

            printf("%lu\r\n", sConfigOC.Pulse);
            // Alter the PWM duty cycle and start PWM again
 //           if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_Channel) != HAL_OK) { Error_Handler(); }
 //           if (HAL_TIM_PWM_Start(&htim4, TIM_Channel) != HAL_OK){ Error_Handler(); }
        }
    }
}

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

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 2
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is 
       clocked below the maximum system frequency, to update the voltage scaling value 
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Activate the OverDrive to reach the 180 MHz Frequency */  
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
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


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    printf("Entered error handler.\r\n");
    while(1) 
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }
    /* USER CODE END Error_Handler */ 
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
