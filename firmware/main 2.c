/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32xxx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "X-NUCLEO-53L0A1.h"
#include "vl53l0x_api.h"
#include "tof_gestures.h"
#include "tof_gestures_TAP_1.h"
#include "tof_gestures_SWIPE_1.h"
#include "tof_gestures_DIRSWIPE_1.h"
#include <limits.h>

/**
 * @defgroup Configuration Static configuration
 * @{
 */

/** Time the initial 53L0 message is shown at power up */
#define WelcomeTime 660

/** Time the initial 53L0 message is shown at power up */
#define ModeChangeDispTime  500

/**
 * Time considered as  a "long push" on push button
 */
#define PressBPSwicthTime   1000

/** @}  */ /* cofnig group */

#ifndef MIN
#   define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

#ifndef ARRAY_SIZE
#   define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))
#endif

/**
 * @defgroup ErrCode Errors code shown on display
 * @{
 */
#define ERR_DETECT             -1
#define ERR_DEMO_GESTURE_MODE   1

/** }@} */ /* defgroup ErrCode */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/**
 * Global ranging struct
 */
VL53L0X_RangingMeasurementData_t RangingMeasurementData;


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


VL53L0X_Dev_t VL53L0XDevs[]={
        {.Id=XNUCLEO53L0A1_DEV_LEFT, .DevLetter='l', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
        {.Id=XNUCLEO53L0A1_DEV_CENTER, .DevLetter='c', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
        {.Id=XNUCLEO53L0A1_DEV_RIGHT, .DevLetter='r', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
};

/********************************************************************************/
/* GESTURES : Various demo states that can be selected by a short or long press */
/*            on blue button                                                    */
/********************************************************************************/
enum State_t {
    RANGE=0,               		/* Display ranging from center device (no gesture recognition more for debug) */
	TAP_CENTER,                	/* Tap gesture detection using center device */
	SWIPE_CENTER,              	/* Swipe gesture detection using center device */
    DIRECTIONAL_SWIPE,     		/* Directional (left & right) swipes detection using Left and Right devices */
};
uint32_t tof_gestures_enableDebugModuleMask;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void debug_stuff(void);
int ResetAndDetectSensor(int SetDisplay);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define debug_printf    trace_printf
char WelcomeMsg[]="Hi I am Ranging VL53L0X mcu " MCU_NAME "\n";

volatile int IntrCount;
volatile int LastIntrPin;
volatile int LastIntrId;

volatile int IntrCounts[3];

#define BSP_BP_PORT GPIOC
#define BSP_BP_PIN  GPIO_PIN_13

int BSP_GetPushButton(void){
    GPIO_PinState state ;
    state = HAL_GPIO_ReadPin(BSP_BP_PORT, BSP_BP_PIN);
    return state;
}

/**
 * When button is already pressed it Wait for user to release it
 * if button remain pressed for given time it return true
 * These is used to detect mode switch by long press on blue Push Button
 *
 * As soon as time is elapsed -rb- is displayed  to let user know order
 * the  request to switch mode is taken into account
 *
 * @return True if button remain pressed more than specified time
 */
int PusbButton_WaitUnPress(void){
    uint32_t TimeStarted;
    TimeStarted = HAL_GetTick();
    while( !BSP_GetPushButton() ){ ; /* debounce */
        if(HAL_GetTick()- TimeStarted> PressBPSwicthTime){
            XNUCLEO53L0A1_SetDisplayString (" rb ");
        }
    }
    return  HAL_GetTick() - TimeStarted>PressBPSwicthTime;

}

void VL53L0A1_EXTI_Callback(int DevNo, int GPIO_Pin){
    IntrCount++;
    LastIntrPin=GPIO_Pin;
    LastIntrId=DevNo;

    if( DevNo< ARRAY_SIZE(IntrCounts)  ){
        IntrCounts[DevNo]++;
    }
}



/**
 * Handle Error
 *
 * Set err on display and loop forever
 * @param err Error case code
 */
void HandleError(int err){
    char msg[16];
    sprintf(msg,"Er%d", err);
    XNUCLEO53L0A1_SetDisplayString(msg);
    while(1){};
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
 * Reset all sensor then goes true presence detection
 *
 * All present devices are data initiated and assigned with their final address
 * @return
 */
int DetectSensors(int SetDisplay) {
    int i;
    uint16_t Id;
    int status;
    int FinalAddress;

    char PresentMsg[5]="    ";
    /* Reset all */
    for (i = 0; i < 3; i++)
        status = XNUCLEO53L0A1_ResetId(i, 0);

    /* detect all sensors (even on-board)*/
    for (i = 0; i < 3; i++) {
        VL53L0X_Dev_t *pDev;
        pDev = &VL53L0XDevs[i];
        pDev->I2cDevAddr = 0x52;
        pDev->Present = 0;
        status = XNUCLEO53L0A1_ResetId( pDev->Id, 1);
        HAL_Delay(2);
        FinalAddress=0x52+(i+1)*2;

        do {
        	/* Try to read one register using default 0x52 address */
            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
            if (status) {
                debug_printf("#%d Read id fail\n", i);
                break;
            }
            if (Id == 0xEEAA) {
            	/* Sensor is found => Change its I2C address to final one */
                status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
                if (status != 0) {
                    debug_printf("#i VL53L0X_SetDeviceAddress fail\n", i);
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
                    debug_printf("VL53L0X_DataInit %d fail\n", i);
                    break;
                }
                debug_printf("VL53L0X %d Present and initiated to final 0x%x\n", i, pDev->I2cDevAddr);
                nDevPresent++;
                nDevMask |= 1 << i;
                pDev->Present = 1;
            }
            else {
                debug_printf("#%d unknown ID %x\n", i, Id);
                status = 1;
            }
        } while (0);
        /* if fail r can't use for any reason then put the  device back to reset */
        if (status) {
            XNUCLEO53L0A1_ResetId(i, 0);
        }
    }
    /* Display detected sensor(s) */
    if( SetDisplay ){
        for(i=0; i<3; i++){
            if( VL53L0XDevs[i].Present ){
                PresentMsg[i+1]=VL53L0XDevs[i].DevLetter;
            }
        }
        PresentMsg[0]=' ';
        XNUCLEO53L0A1_SetDisplayString(PresentMsg);
        HAL_Delay(1000);
    }

    return nDevPresent;
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
                debug_printf("VL53L0X_StaticInit %d fail",i);
            }

            status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   debug_printf("VL53L0X_PerformRefCalibration");
			}

			status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i], &refSpadCount, &isApertureSpads);
			if( status ){
			   debug_printf("VL53L0X_PerformRefSpadManagement");
			}

            status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
            if( status ){
               debug_printf("VL53L0X_SetDeviceMode");
            }

            status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i],  20*1000);
            if( status ){
               debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
            }
            VL53L0XDevs[i].LeakyFirst=1;
        }
    }
}

/* Translate a right or left swipe into a pattern on the display showing a kind of page position and direction */
void FormatDirSwipe(bool right_notleft, int position, char * DisplayStr){
    char * _digitPtr;
    int i;

    _digitPtr = DisplayStr;
    for(i=0;i<4;i++){
        *_digitPtr++ = (i==position) ? (right_notleft ? ']' : '[') : '-';
    }
}

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


int ResetAndDetectSensor(int SetDisplay){
    int nSensor;
    nSensor = DetectSensors(SetDisplay);
    /* at least one sensor and if one it must be the built-in one  */
    if( (nSensor <=0) ||  (nSensor ==1 && VL53L0XDevs[1].Present==0) ){
        HandleError(ERR_DETECT);
    }
    return nSensor;
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  int i;
  char StrDisplay[5];
  int nSensorPresent=0;
  int nSensorEnabled=0;
  int nReady;
  int status;
  uint8_t NewDataReady=0;
  int state=RANGE; // Start demo with ranging (with center device)
  Gesture_TAP_1_Data_t gestureTapData1;
  Gesture_SWIPE_1_Data_t gestureSwipeData1;
  Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;
  int gesture_code;
  int tap=0,swipe=0;
  int32_t leftRange, rightRange;
  int pagePosition=1;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Initialize and start timestamping for UART logging */
  TimeStamp_Init();
  TimeStamp_Reset();

  /* USER CODE BEGIN 2 */
  XNUCLEO53L0A1_Init();
  uart_printf(WelcomeMsg);
  XNUCLEO53L0A1_SetDisplayString("53L0");
  HAL_Delay(WelcomeTime);
  nSensorPresent = ResetAndDetectSensor(1);

  /* Set VL53L0X API trace level */
  VL53L0X_trace_config(NULL, TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_NONE); // No Trace
  //VL53L0X_trace_config(NULL,TRACE_MODULE_ALL, TRACE_LEVEL_ALL, TRACE_FUNCTION_ALL); // Full trace

  /* First enable only the center device */
  VL53L0XDevs[XNUCLEO53L0A1_DEV_LEFT].Enabled = 0;
  VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].Enabled = 1;
  VL53L0XDevs[XNUCLEO53L0A1_DEV_RIGHT].Enabled = 0;

  /* Initialize Tap gesture recognition */
  tof_gestures_initTAP_1(300, &gestureTapData1);
  /* Initialize Swipe gesture recognition */
  tof_gestures_initSWIPE_1(&gestureSwipeData1);
  /* Initialize directional swipes recognition : swipe detected below 400 mm, no max speed, min duration is 1 sec for a swipe and hand must not cover both devices */
  tof_gestures_initDIRSWIPE_1(400, 0, 1000, false, &gestureDirSwipeData);
  /* Select which module to debug (code must be compiled with TRACE defined in compiler command line) */
  TOF_GESTURES_DEBUG_SET_MODULES(NONE); // Could be NONE or TAP_1|TAP_SWIPE_2|DIRSWIPE_1 for instance (or any other combinations);

  /* Setup sensors in single mode */
  SetupSingleShot();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /********************************************************************************/
	  /* GESTURES : Enable/Disable devices as per selected state                      */
	  /********************************************************************************/
	  switch(state){
		  case RANGE:
		  case TAP_CENTER:
		  case SWIPE_CENTER:
			  /* Enable TOP device only*/
			  nSensorEnabled = 1;
			  VL53L0XDevs[XNUCLEO53L0A1_DEV_LEFT].Enabled = 0;
			  VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].Enabled = 1;
			  VL53L0XDevs[XNUCLEO53L0A1_DEV_RIGHT].Enabled = 0;
			  break;
		  case DIRECTIONAL_SWIPE:
			  nSensorEnabled = 2;
			  /* Enable LEFT and RIGHT devices only */
			  VL53L0XDevs[XNUCLEO53L0A1_DEV_LEFT].Enabled = 1;
			  VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].Enabled = 0;
			  VL53L0XDevs[XNUCLEO53L0A1_DEV_RIGHT].Enabled = 1;
			  break;
		  default:
			  HandleError(ERR_DEMO_GESTURE_MODE);
	  }

	  /* kick off measure on enabled devices */
	  for( i=0; i<3; i++){
		  if( ! VL53L0XDevs[i].Present  || ! VL53L0XDevs[i].Enabled )
			  continue;
		  status = VL53L0X_StartMeasurement(&VL53L0XDevs[i]);
		  if( status ){
			  debug_printf("VL53L0X_StartMeasurement failed on device %d",i);
		  }
		  VL53L0XDevs[i].Ready=0;
	  }

	  /* wait for all enabled devices to have a measure */
	  nReady=0;
	  do{
		  HAL_Delay(1);
		  for( i=0; i<3; i++){
			  /* Skip devices not present or not enabled */
			  if( ! VL53L0XDevs[i].Present  || ! VL53L0XDevs[i].Enabled )
				  continue;
			  /* Is new sample ready ? */
			  status = VL53L0X_GetMeasurementDataReady(&VL53L0XDevs[i], &NewDataReady);
			  if( status ){
				  debug_printf("VL53L0X_GetMeasurementDataReady failed on device %d",i);
			  }
			  /* Skip if new sample not ready */
			  if (NewDataReady == 0)
				  continue;

			  /* Clear Interrupt */
			  status = VL53L0X_ClearInterruptMask(&VL53L0XDevs[i], 0);

			  /* Otherwise, get new sample data and store */
			  status = VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[i], &RangingMeasurementData);
			  if( status ){
				  debug_printf("VL53L0X_GetRangingMeasurementData failed on device %d",i);
			  }
			  /* Data logging */
			  trace_printf("%d,%u,%d,%d,%d\n", VL53L0XDevs[i].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
			  Sensor_SetNewRange(&VL53L0XDevs[i],&RangingMeasurementData);
			  VL53L0XDevs[i].Ready=1;
			  nReady++;
		  }
	  }
	  while(nReady<nSensorEnabled);

	  /********************************************************************************/
	  /* GESTURES : Apply gestures detection functions on the measures                */
	  /********************************************************************************/
	  switch(state){
		  case RANGE:
			  if( VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].RangeStatus == 0 ){
				  sprintf(StrDisplay, "%3dc",(int)VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].LeakyRange/10);
			  }else{
				  sprintf(StrDisplay, "r---");
				  StrDisplay[0]=VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].DevLetter;
			  }
			  /* Check blue button */
			  if( !BSP_GetPushButton() ){
				  /* when button get pressed, wait it get released (keep doing display) */
				  status = PusbButton_WaitUnPress();
				  if(status && nSensorPresent==3){
					  /* BP stay pressed very long time and 3 devices are present */
					  state = DIRECTIONAL_SWIPE;
					  sprintf(StrDisplay, "----");
				  }else{
					  /* BP short pressed or only one device is there */
					  state = TAP_CENTER;
					  sprintf(StrDisplay, "[@@]");
				  }
				  /* Reset Timestamping */
				  TimeStamp_Reset();
			  }
			  break;

		  case TAP_CENTER:
			  /* Call ToF Gesture module all the time (even if no object is detected) */
			  if ( VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].RangeStatus == 0 ){
				  /* Object (hand) detected */
				  gesture_code = tof_gestures_detectTAP_1(VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].LeakyRange, &gestureTapData1);
			  }else{
				  /* No object (hand) detected : call the ToF Gesture module with 1200 mm as distance */
				  gesture_code = tof_gestures_detectTAP_1(1200, &gestureTapData1);
			  }
			  /* Format data to display */
			  if(gesture_code == GESTURES_SINGLE_TAP){
				  tap = !tap;
				  sprintf(StrDisplay, (tap ? " [] " : "[@@]"));
			  }
			  /* Check blue button */
			  if (!BSP_GetPushButton()) {
				  /* when button get pressed, wait it get released (keep doing display) */
				  status = PusbButton_WaitUnPress();
				  if(status && nSensorPresent==3){
					  /* BP stay pressed very long time and 3 devices are present */
					  state = DIRECTIONAL_SWIPE;
					  sprintf(StrDisplay, "----");
				  }else{
					  /* BP short pressed or only one device is there */
					  state = SWIPE_CENTER;
					  sprintf(StrDisplay, "----");
				  }
				  /* Reset Timestamping */
				  TimeStamp_Reset();
			  }
			  break;

		  case SWIPE_CENTER:
			  /* Call ToF Gesture module all the time (even if no object is detected) */
			  if ( VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].RangeStatus == 0 && VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].LeakyRange !=0 ){
				  /* Object (hand) detected */
				  gesture_code = tof_gestures_detectSWIPE_1(VL53L0XDevs[XNUCLEO53L0A1_DEV_CENTER].LeakyRange, &gestureSwipeData1);
			  }else{
				  /* No object (hand) detected : call the ToF Gesture module with 1200 mm as distance */
				  gesture_code = tof_gestures_detectSWIPE_1(1200, &gestureSwipeData1);
			  }
			  /* Format data to display */
			  if(gesture_code == GESTURES_SINGLE_SWIPE){
				  swipe = !swipe;
				  sprintf(StrDisplay, (swipe ? "^^^^" : "____"));
			  }
			  /* Check blue button */
			  if (!BSP_GetPushButton()) {
				  /* when button get pressed, wait it get released (keep doing display) */
				  status = PusbButton_WaitUnPress();
				  if(status && nSensorPresent==3){
					  /* BP stay pressed very long time and 3 devices are present */
					  state = DIRECTIONAL_SWIPE;
					  sprintf(StrDisplay, "----");
				  }else{
					  /* BP short pressed or only one device is there */
					  state = RANGE;
					  sprintf(StrDisplay, "    ");
				  }
				  /* Reset Timestamping */
				  TimeStamp_Reset();
			  }
			  break;

		  case DIRECTIONAL_SWIPE:
			  /* Clip ranging values as tof_gestures_detectDIRSWIPE_1 function must be called all the times to detect the gesture */
			  leftRange = (VL53L0XDevs[XNUCLEO53L0A1_DEV_LEFT].RangeStatus == 0) ? VL53L0XDevs[XNUCLEO53L0A1_DEV_LEFT].LeakyRange : 1200;
			  rightRange = (VL53L0XDevs[XNUCLEO53L0A1_DEV_RIGHT].RangeStatus == 0) ? VL53L0XDevs[XNUCLEO53L0A1_DEV_RIGHT].LeakyRange : 1200;
			  gesture_code = tof_gestures_detectDIRSWIPE_1(leftRange, rightRange, &gestureDirSwipeData);

			  /* Format data to display */
			  if(gesture_code == GESTURES_SWIPE_LEFT_RIGHT){
				  pagePosition++;
				  pagePosition = (pagePosition>3) ? 0 : pagePosition;
				  FormatDirSwipe(true, pagePosition, StrDisplay);
			  }else if(gesture_code == GESTURES_SWIPE_RIGHT_LEFT){
				  pagePosition--;
				  pagePosition = (pagePosition<0) ? 3 : pagePosition;
				  FormatDirSwipe(false, pagePosition, StrDisplay);
			  }else{
				  //ShowGestureHelpMsg(gesture_code);
			  }
			  /* Check blue button */
			  if (!BSP_GetPushButton()) {
				  //when button get pressed, wait it get released (keep doing display)
				  status = PusbButton_WaitUnPress();
				  if(status){
					  //BP stay pressed very long time and 3 devices are present
					  state = RANGE;
					  sprintf(StrDisplay, "    ");
				  }else{
					  //BP short pressed or only one device is there
					  state = DIRECTIONAL_SWIPE;
				  }
				  /* Reset Timestamping */
				  TimeStamp_Reset();
			  }
			  break;
      }
	  XNUCLEO53L0A1_SetDisplayString(StrDisplay);
  }
}

/** System Clock Configuration
*/
#ifdef STM32F401xE
void SystemClock_Config(void) {

    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    __PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

}
#endif
#ifdef STM32L053xx
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


  __SYSCFG_CLK_ENABLE();

}
#endif

#ifdef STM32L476xx
void SystemClock_Config(void)
{
     RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


  __PWR_CLK_ENABLE();

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
#endif

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * call for debug to do basic reg access

 * After  breaking at entrance of this function
 * change  the the local index/data/DevNo and cmd variable to do what needed
 * reg_cmd -1 wr byte -2wr word -3 wr dword
 * reg_cmd 1 rd byte 2 rd word 3 rd dword
 * step to last statement before return and read variable to get rd result exit
 */
void debug_stuff(void) {
    int reg_cmd = 0;
    static uint32_t reg_data;
    static uint16_t reg_index;
    static int DevNo=1;

    if (reg_cmd) {
        VL53L0X_Dev_t *pDev;
        pDev = &VL53L0XDevs[DevNo];
        if( !pDev->Present ){
            debug_printf("dev %d not present", DevNo);
        }
        switch (reg_cmd) {
            case -1:
                VL53L0X_WrByte(pDev, reg_index, reg_data);
                debug_printf("Wr B 0x%X = %d", reg_index, (int )reg_data);
                break;
            case -2:
                VL53L0X_WrWord(pDev, reg_index, reg_data);
                debug_printf("Wr W 0x%X = %d", reg_index, (int ) reg_data);
                break;

            case -3:
                VL53L0X_WrDWord(pDev, reg_index, reg_data);
                debug_printf("WrDW 0x%X = %d", reg_index, (int )reg_data);
                break;

            case 1:
                reg_data = 0;
                VL53L0X_RdByte(pDev, reg_index, (uint8_t*) &reg_data);
                debug_printf("RD B 0x%X = %d", reg_index, (int )reg_data);
                break;
            case 2:
                reg_data = 0;
                VL53L0X_RdWord(pDev, reg_index, (uint16_t*) &reg_data);
                debug_printf("RD W 0x%X = %d", reg_index, (int )reg_data);
                break;

            case 3:
                VL53L0X_RdDWord(pDev, reg_index, &reg_data);
                debug_printf("RD DW 0x%X = %d", reg_index, (int )reg_data);
                break;
            default:
                debug_printf("Invalid command %d", reg_cmd);
                /* nothing to do*/
                ;
        }
    }
}
/* USER CODE END 4 */

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
