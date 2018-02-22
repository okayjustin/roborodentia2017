#include "vl53l0x_top.h"

void VL53L0X_begin(){
    rangeMillimeterX = 0;
    rangeMillimeterY = 0;

    /** leaky factor for filtered range
     * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
     * */
    LeakyFactorFix8 = (int)( 0.6 *256); //(int)( 0.6 *256);

    VL53L0XDevs[0].Id = 0;
    VL53L0XDevs[0].DevLetter = 'l';
    VL53L0XDevs[0].I2cHandle = &hi2c2;
    VL53L0XDevs[0].I2cDevAddr = 0x52;
    VL53L0XDevs[1].Id = 1;
    VL53L0XDevs[1].DevLetter = 'c';
    VL53L0XDevs[1].I2cHandle = &hi2c2;
    VL53L0XDevs[1].I2cDevAddr = 0x52;

    int status;
    VL53L0X_Dev_t *pDev;
    int i;
    for (i = 0; i < 2; i++) {
        printf("Attempting to find rangefinder #%d...\r\n", i);
        uint16_t Id;
        pDev = &VL53L0XDevs[i];
        pDev->I2cDevAddr = RANGE_I2C_ADDR_INITIAL;
        pDev->Present = 0;
        int FinalAddress = RANGE_I2C_ADDR_INITIAL + (2 * (i + 1));

        // Enable the device by setting XSHUT pin high
        switch (pDev->Id){
        case 0:
            // FIX THIS FUNCTION
            //HAL_GPIO_WritePin(RANGEX_XSHUT_GPIO_Port, RANGEX_XSHUT_Pin, GPIO_PIN_SET);
            break;
        case 1:
            // FIX THIS FUNCTION
            //HAL_GPIO_WritePin(RANGEY_XSHUT_GPIO_Port, RANGEY_XSHUT_Pin, GPIO_PIN_SET);
            break;
        default:
            printf("Error: unknown device ID\r\n");
        }
        HAL_Delay(3); // MUST HAVE THIS OR ELSE 2ND SENSOR WONT WORK

        /* Set I2C standard mode (400 KHz) before doing the first register access */
        status = VL53L0X_WrByte(pDev, 0x88, 0x00);

        do{
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
                pDev->Present = 1;
            }
            else {
                printf("#%d unknown ID %x\r\n", i, Id);
                status = 1;
            }
        } while(0);
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


/**
 *  Setup all sensors for single shot mode
 */
void VL53L0X_SetupSingleShot(){
    int i;
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    for( i=0; i<2; i++){
        if( VL53L0XDevs[i].Present){
            //printf("Initializing device #%d\r\n", i);
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

            //status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
            status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in continuous ranging mode
            if( status ){
               printf("VL53L0X_SetDeviceMode");
            }

            status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i],  33*1000);
            if( status ){
               printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
            }
            VL53L0XDevs[i].LeakyFirst=1;

            status = VL53L0X_StartMeasurement(&VL53L0XDevs[i]);
            if( status ){
                printf("VL53L0X_StartMeasurement failed on device %d\r\n",i);
                Error_Handler();
            }
            VL53L0XDevs[i].Ready=0;
        }
        else {
            printf("Missing range finder #%d\r\n.", i);
            Error_Handler();
        }
    }
}

// Updates the range measurement of a particular rangefinder
void rangefinderRead(int id){
    if (id == 0){
        if (HAL_GPIO_ReadPin(RF1_G_GPIO_Port, RF1_G_Pin)){
            return; // New data not ready
        }
        VL53L0X_ClearInterruptMask(&VL53L0XDevs[0], 0);
        if (VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[0], &RangingMeasurementDataX)){
            printf("VL53L0X_GetRangingMeasurementData failed on device %d\r\n", id);
        }
        if (RangingMeasurementDataX.RangeStatus == 0){ 
            Sensor_SetNewRange(&VL53L0XDevs[0],&RangingMeasurementDataX);
            rangeMillimeterX = RangingMeasurementDataX.RangeMilliMeter;
        }
    } else {
        if (HAL_GPIO_ReadPin(RF2_G_GPIO_Port, RF2_G_Pin)){
            return; // New data not ready
        }
        VL53L0X_ClearInterruptMask(&VL53L0XDevs[1], 0);
        if (VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[1], &RangingMeasurementDataY)){
            printf("VL53L0X_GetRangingMeasurementData failed on device %d\r\n", id);
        }
        if (RangingMeasurementDataY.RangeStatus == 0){ 
            Sensor_SetNewRange(&VL53L0XDevs[1],&RangingMeasurementDataY);
            rangeMillimeterY = RangingMeasurementDataY.RangeMilliMeter;
        }
    }
//    printf("%d,%d\r\n", VL53L0XDevs[i].Id, RangingMeasurementData.RangeMilliMeter);
//   printf("%d,%lu,%d,%d,%f\r\n", VL53L0XDevs[i].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps / 1.0);
}

