#ifndef _VL53L0X_TOP_H_
#define _VL53L0X_TOP_H_

#define RANGE_I2C_ADDR_INITIAL 0x52

#include "vl53l0x_api.h"

void VL53L0X_begin(void);
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange);
void VL53L0X_SetupSingleShot();
void rangefinderRead(int id);

VL53L0X_RangingMeasurementData_t RangingMeasurementDataX;
VL53L0X_RangingMeasurementData_t RangingMeasurementDataY;
uint16_t rangeMillimeterX;
uint16_t rangeMillimeterY;

/** leaky factor for filtered range
 * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
 * */
int LeakyFactorFix8; 

VL53L0X_Dev_t VL53L0XDevs[2];

#endif  /* _VL53L0X_TOP_H_ */
