// Library for controling Adafruit 9-DOF IMU
// LSM303DLHC magnetometer/accelerometer combo
// L3G4200D gyroscope

#include "main.h"
#include "imu.h"
#include "i2c.h"
#include "math.h"

void IMU_begin() {
//    // BDU block data update enabled
//    uint8_t write_data_gyro[2] = {L3G4200D_REG_CTRL_REG4, 0x80}; 
//    I2C_Write(IMU_I2C_INTERFACE, L3G4200D_ADDRESS, write_data_gyro, 2); 
//     
//    // Enable the gyroscope
//    // DR = 00 (100 Hz ODR); BW = 01 (25 Hz bandwidth); PD = 1 (normal mode); 
//    // Zen = Yen = Xen = 1 (all axes enabled)
//    uint8_t write_data_gyro2[2] = {L3G4200D_REG_CTRL_REG1, 0x1F}; 
//    I2C_Write(IMU_I2C_INTERFACE, L3G4200D_ADDRESS, write_data_gyro2, 2); 


    // Enable the accelerometer
    // 100 Hz mode
    // HPF disabled
    // 12-bit high resolution mode
    // BDU enabled
    uint8_t write_data_accel[5] = {LSM303_REG_ACCEL_CTRL_REG1_A | 0x80, 0x57, 0x00, 0x00, 0x88}; 
    I2C_Write(IMU_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data_accel, 5); 

    // Set magnetometer to 220 Hz mode
    uint8_t write_data_mag[2] = {LSM303_REG_MAG_CRA_REG_M, 0x1C};
//    // Set magnetometer to 30 Hz mode
//    uint8_t write_data_mag[2] = {LSM303_REG_MAG_CRA_REG_M, 0x14};
    I2C_Write(IMU_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data_mag, 2); 

    // Enable the magnetometer
    // LSM303_REG_MAG_MR_REG_M tends to revert back to 0x01 if any other config regs change
    // This delay is required or else the register won't change value. Must wait about 5 ms
    // after any other config register is altered.
    HAL_Delay(5); 
    uint8_t write_data_mag2[2] = {LSM303_REG_MAG_MR_REG_M, 0x00};
    I2C_Write(IMU_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data_mag2, 2); 
}

void gyro_read() {
    uint8_t write_data[1] = {L3G4200D_REG_OUT_X_L | 0x80};  // 0x80 addr autoincrement
    uint8_t read_data[6];
    I2C_WriteRead(IMU_I2C_INTERFACE, L3G4200D_ADDRESS, write_data, 1, read_data, 6); 

    // Shift values to create properly formed integer 
    gyroData.x = (int16_t)((read_data[1] << 8) | read_data[0]);
    gyroData.y = (int16_t)((read_data[3] << 8) | read_data[2]);
    gyroData.z = (int16_t)((read_data[5] << 8) | read_data[4]);  
}

void accelerometer_read() {
    // Read the accelerometer
    uint8_t write_data[1] = {LSM303_REG_ACCEL_OUT_X_L_A | 0x80}; // 0x80 addr autoincrement
    uint8_t read_data[6];
    I2C_WriteRead(IMU_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data, 1, read_data, 6); 

    // Shift values to create properly formed integer 
    accelData.x = -1 * (int16_t)((read_data[1] << 8) | read_data[0]);
    accelData.y = (int16_t)((read_data[3] << 8) | read_data[2]);
    accelData.z = (int16_t)((read_data[5] << 8) | read_data[4]);
}

void magnetometer_read() {
//    do{
        // Read the magnetometer
        uint8_t write_data[1] = {LSM303_REG_MAG_OUT_X_H_M};
        uint8_t read_data[6];
        I2C_WriteRead(IMU_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data, 1, read_data, 6); 

        // Shift values to create properly formed integer 
        magData.x = (int16_t)((read_data[1]) | read_data[0] << 8);
        magData.y = -1 * (int16_t)((read_data[5]) | read_data[4] << 8);
        magData.z = -1 * (int16_t)((read_data[3]) | read_data[2] << 8);  
//    } while (magData.x == 10 && magData.y == 0 && magData.z == 0);

//    float x_uT = (float)(magData.x);
//    float y_uT = (float)(magData.y);
//    float z_uT = (float)(magData.z);

    // Calculate orientation
//    magData.orientation_prev = magData.orientation;
//    magData.orientation = (int16_t)((atan2(y_uT, x_uT) * 1800.0 / M_PI + 1800.0));
}


