#include "vl53l0x_top.h"

void VL53L0X_begin(){
    VL53L0X_Dev_t *pDev;

    // Setup rangefinder device structs
    VL53L0XDevs[0].Id = 0;
    VL53L0XDevs[0].DevLetter = 'a';
    VL53L0XDevs[0].I2cHandle = &hi2c3;
    VL53L0XDevs[0].I2cDevAddr = 0x29;
    VL53L0XDevs[0].XSHUT_Port = SW1_GPIO_Port;
    VL53L0XDevs[0].XSHUT_Pin = SW1_Pin;
    VL53L0XDevs[0].GPIO_Port = RF1_G_GPIO_Port;
    VL53L0XDevs[0].GPIO_Pin = RF1_G_Pin;
    VL53L0XDevs[0].TimingBudgetUsecs = 49000;

    VL53L0XDevs[1].Id = 1;
    VL53L0XDevs[1].DevLetter = 'b';
    VL53L0XDevs[1].I2cHandle = &hi2c3;
    VL53L0XDevs[1].I2cDevAddr = 0x29;
    VL53L0XDevs[1].XSHUT_Port = SW2_GPIO_Port;
    VL53L0XDevs[1].XSHUT_Pin = SW2_Pin;
    VL53L0XDevs[1].GPIO_Port = RF2_G_GPIO_Port;
    VL53L0XDevs[1].GPIO_Pin = RF2_G_Pin;
    VL53L0XDevs[1].TimingBudgetUsecs = 49000;

    VL53L0XDevs[2].Id = 2;
    VL53L0XDevs[2].DevLetter = 'c';
    VL53L0XDevs[2].I2cHandle = &hi2c2;
    VL53L0XDevs[2].I2cDevAddr = 0x29;
    VL53L0XDevs[2].XSHUT_Port = SW3_GPIO_Port;
    VL53L0XDevs[2].XSHUT_Pin = SW3_Pin;
    VL53L0XDevs[2].GPIO_Port = RF4_G_GPIO_Port;
    VL53L0XDevs[2].GPIO_Pin = RF4_G_Pin;
    VL53L0XDevs[2].TimingBudgetUsecs = 49000;

    VL53L0XDevs[3].Id = 3;
    VL53L0XDevs[3].DevLetter = 'd';
    VL53L0XDevs[3].I2cHandle = &hi2c2;
    VL53L0XDevs[3].I2cDevAddr = 0x29;
    VL53L0XDevs[3].XSHUT_Port = SW4_GPIO_Port;
    VL53L0XDevs[3].XSHUT_Pin = SW4_Pin;
    VL53L0XDevs[3].GPIO_Port = RF5_G_GPIO_Port;
    VL53L0XDevs[3].GPIO_Pin = RF5_G_Pin;
    VL53L0XDevs[3].TimingBudgetUsecs = 49000;

    // Turn off all the rangefinders
    int i;
    for (i = 0; i < NUM_RANGEFINDERS; i++) {
        pDev = &VL53L0XDevs[i];
        HAL_GPIO_WritePin(pDev->XSHUT_Port, pDev->XSHUT_Pin, 0);
    }

    // Initialize range data to 0mm
    for (i = 0; i < NUM_RANGEFINDERS; i++){
        rangeData[i] = 0;
    }

    /** leaky factor for filtered range
     * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
     * */
    LeakyFactorFix8 = (int)( 0.6 *256); //(int)( 0.6 *256);

    // Setup each sensor
    for (i = 0; i < NUM_RANGEFINDERS; i++) {
        VL53L0X_SetupSingleShot(i);
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
void VL53L0X_SetupSingleShot(int i){
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
#ifdef PERFORM_OFFSET_CAL
    uint8_t OffsetMicrometer;
#endif
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    printf("Attempting to find rangefinder #%d...\r\n", i);
    uint16_t Id;
    VL53L0X_Dev_t *pDev;
    pDev = &VL53L0XDevs[i];
    pDev->I2cDevAddr = RANGE_I2C_ADDR_INITIAL;
    pDev->Present = 0;
    int FinalAddress = RANGE_I2C_ADDR_INITIAL + (2 * (i + 1));

    // Enable the device by setting XSHUT pin high
    HAL_GPIO_WritePin(pDev->XSHUT_Port, pDev->XSHUT_Pin, 1);
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

    if( VL53L0XDevs[i].Present){
        //printf("Initializing device #%d\r\n", i);
        status=VL53L0X_StaticInit(&VL53L0XDevs[i]);
        if( status ){
            printf("VL53L0X_StaticInit %d fail",i);
        }

        status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i], &refSpadCount, &isApertureSpads);
        if( status ){
           printf("VL53L0X_PerformRefSpadManagement");
        }

        status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
        if( status ){
           printf("VL53L0X_PerformRefCalibration");
        }

#ifdef PERFORM_OFFSET_CAL
        status = VL53L0X_PerformOffsetCalibration(&VL53L0XDevs[i], 100.0, &OffsetMicrometer);
        if( status ){
           printf("VL53L0X_PerformOffsetCalibration");
        }
#endif

        status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        //status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in continuous ranging mode
        if( status ){
           printf("VL53L0X_SetDeviceMode");
        }

        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i], VL53L0XDevs[i].TimingBudgetUsecs);
        if( status ){
           printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
        }
        VL53L0XDevs[i].LeakyFirst=1;

        status = VL53L0X_StartMeasurement(&VL53L0XDevs[i]);
        if( status ){
            printf("VL53L0X_StartMeasurement failed on device %d\r\n",i);
        }
        VL53L0XDevs[i].Ready=0;
    }
    else {
        printf("Missing range finder #%d\r\n.", i);
    }
}

// Updates the range measurement of a particular rangefinder
void rangefinderRead(int id){
    VL53L0X_Dev_t *pDev;
    pDev = &VL53L0XDevs[id];

    if (HAL_GPIO_ReadPin(pDev->GPIO_Port, pDev->GPIO_Pin)){
        printf("New data not ready on device %d.\r\n", id);
        return; // New data not ready
    }

    VL53L0X_ClearInterruptMask(pDev, 0);
    if (VL53L0X_GetRangingMeasurementData(pDev, &(pDev->RangeData))){
        printf("VL53L0X_GetRangingMeasurementData failed on device %d\r\n", id);

//        // Reset sensor
//        printf("Turning off device %d\r\n",id);
//        HAL_GPIO_WritePin(VL53L0XDevs[id].XSHUT_Port, VL53L0XDevs[id].XSHUT_Pin, 0);
//        HAL_Delay(1000);
//        printf("Enabling device %d\r\n",id);
//        HAL_GPIO_WritePin(VL53L0XDevs[id].XSHUT_Port, VL53L0XDevs[id].XSHUT_Pin, 1);
//        HAL_Delay(5);
//        printf("Setting up single shot %d\r\n",id);
//        VL53L0X_SetupSingleShot(id);
    }
    if (pDev->RangeData.RangeStatus == 0){ 
        Sensor_SetNewRange(pDev,&(pDev->RangeData));
        rangeData[id] = pDev->LeakyRange;
    }

    // Start next measurement
    int status;
    status = VL53L0X_StartMeasurement(pDev);
    if( status ){
        printf("VL53L0X_StartMeasurement failed on device %d\r\n", id);
    }
}

