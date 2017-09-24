
#include <string.h>
#include <limits.h>
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "config.h"
//#include "lsm303.h"
#include "vl53l0x_api.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * Global ranging struct
 */
VL53L0X_RangingMeasurementData_t RangingMeasurementData;

enum XNUCLEO53L0A1_dev_e{
    XNUCLEO53L0A1_DEV_LEFT =  0,    //!< left satellite device P21 header : 'l'
    XNUCLEO53L0A1_DEV_CENTER  =  1, //!< center (built-in) vl053 device : 'c"
    XNUCLEO53L0A1_DEV_RIGHT=  2     //!< Right satellite device P22 header : 'r'
};

/** leaky factor for filtered range
 *
 * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
 *
 * */
int LeakyFactorFix8 = (int)( 0.0 *256); //(int)( 0.6 *256);
/** How many device detect set by @a DetectSensors()*/
int nDevPresent=0;
/** bit is index in VL53L0XDevs that is not necessary the dev id of the BSP */
int nDevMask;

I2C_HandleTypeDef  XNUCLEO53L0A1_hi2c;

VL53L0X_Dev_t VL53L0XDevs[]={
        {.Id=XNUCLEO53L0A1_DEV_LEFT, .DevLetter='l', .I2cHandle=&hi2c2, .I2cDevAddr=0x52},
        {.Id=XNUCLEO53L0A1_DEV_CENTER, .DevLetter='c', .I2cHandle=&hi2c2, .I2cDevAddr=0x52},
        {.Id=XNUCLEO53L0A1_DEV_RIGHT, .DevLetter='r', .I2cHandle=&hi2c2, .I2cDevAddr=0x52},
};

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
    if( pRange->RangeStatus == 0 ){
    	pDev->RangeStatus = 0;
        if( pDev->LeakyFirst ){
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }
        else{
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }
    else{
    	pDev->RangeStatus = pRange->RangeStatus;
    	pDev->LeakyFirst = 1;
    }
}

/** Timer
 *
 * Used get timestamp for UART logging
 */
TIM_HandleTypeDef htim5;

/* TIM5 init function */
void MX_TIM5_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&htim5);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);

}

void TimeStamp_Init(){
    MX_TIM5_Init();
}

void TimeStamp_Reset(){
    HAL_TIM_Base_Start(&htim5);
    htim5.Instance->CNT=0;
}

uint32_t TimeStamp_Get(){
    return htim5.Instance->CNT;
}

/**
 *  Setup all sensors for single shot mode
 */
void SetupSingleShot(){
    int i;
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    for( i=0; i<3; i++){
        if( VL53L0XDevs[i].Present){
            status=VL53L0X_StaticInit(&VL53L0XDevs[i]);
            if( status ){
                printf("VL53L0X_StaticInit %d fail",i);
            }

            status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   printf("VL53L0X_PerformRefCalibration");
			}

			status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i], &refSpadCount, &isApertureSpads);
			if( status ){
			   printf("VL53L0X_PerformRefSpadManagement");
			}

            status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
            if( status ){
               printf("VL53L0X_SetDeviceMode");
            }

            status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i],  20*1000);
            if( status ){
               printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
            }
            VL53L0XDevs[i].LeakyFirst=1;
        }
    }
}



void _init(void) {return;}

int main(void)
{
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
    // Initialize all the board IO
    //Board_GPIO_Init();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();

    serviceUART();
    printf("Hello. Rev 3\r\n");

    MX_I2C1_Init();
    MX_I2C2_Init();
    /* Initialize and start timestamping for UART logging */
    TimeStamp_Init();
    TimeStamp_Reset();

    int i = 0;
    uint8_t NewDataReady=0;
    uint16_t Id;
    int status;
    int FinalAddress;
    VL53L0X_Dev_t *pDev;
    pDev = &VL53L0XDevs[i];
    pDev->I2cDevAddr = 0x52;
    pDev->Present = 0;
//    status = XNUCLEO53L0A1_ResetId( pDev->Id, 1);
    FinalAddress=0x52; //+(i+1)*2;
    do {
        status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
        if (status == VL53L0X_ERROR_CONTROL_INTERFACE){
            printf("Control interface error.\r\n");
        }
        if (status) {
            printf("#%d Read id fail\r\n", i);
            break;
        }
        if (Id == 0xEEAA) {
            /* Sensor is found => Change its I2C address to final one */
            status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
            if (status != 0) {
                printf("VL53L0X_SetDeviceAddress fail\r\n");
                break;
            }
            pDev->I2cDevAddr = FinalAddress;
            /* Check all is OK with the new I2C address and initialize the sensor */
            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
            status = VL53L0X_DataInit(pDev);
            if( status == 0 ){
                pDev->Present = 1;
            }
            else{
                printf("VL53L0X_DataInit %d fail\r\n", i);
                break;
            }
            printf("VL53L0X %d Present and initiated to final 0x%x\r\n", i, pDev->I2cDevAddr);
            nDevPresent++;
            nDevMask |= 1 << i;
            pDev->Present = 1;
        }
        else {
            printf("#%d unknown ID %x\r\n", i, Id);
            status = 1;
        }
    } while (0);

    SetupSingleShot();

//    printf("Enabling magnetometer.\r\n");
//    LSM303_begin();
//    printf("Reading magnetometer.\r\n");

    // Main loop
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//        HAL_Delay(1000);

//        LSM303_read();
//        printf("Orient:%d\r\n", magData.orientation);
        status = VL53L0X_StartMeasurement(&VL53L0XDevs[i]);
        if( status ){
            printf("VL53L0X_StartMeasurement failed on device %d",i);
        }

        /* Is new sample ready ? */
        status = VL53L0X_GetMeasurementDataReady(&VL53L0XDevs[i], &NewDataReady);
        if( status ){
          printf("VL53L0X_GetMeasurementDataReady failed on device %d",i);
        }
        /* Skip if new sample not ready */
        if (NewDataReady == 0)
          continue;

        /* Clear Interrupt */
        status = VL53L0X_ClearInterruptMask(&VL53L0XDevs[i], 0);

        /* Otherwise, get new sample data and store */
        status = VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[i], &RangingMeasurementData);
        if( status ){
          printf("VL53L0X_GetRangingMeasurementData failed on device %d",i);
        }
        /* Data logging */
 //       printf("%d,%lu,%d,%d,%f\r\n", VL53L0XDevs[i].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps / 1.0);
        if (RangingMeasurementData.RangeStatus == 0){ 
            printf("%d\r\n", RangingMeasurementData.RangeMilliMeter);
        }
        Sensor_SetNewRange(&VL53L0XDevs[i],&RangingMeasurementData);

    }
}

// Execute a command from the console
void consoleCommand(uint8_t *ptr, int len)
{
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
