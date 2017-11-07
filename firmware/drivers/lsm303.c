// Use EXACTLY LSM303DLHC datasheet for registers.

#include "main.h"
#include "LSM303.h"
#include "i2c.h"
#include "math.h"

void LSM303_begin() {
    // Enable the accelerometer
    // 100 Hz mode
    // HPF enabled
    // 12-bit high resolution mode
    // BDU enabled
    uint8_t write_data_accel[5] = {LSM303_REG_ACCEL_CTRL_REG1_A | 0x80, 0x57, 0x08, 0x00, 0x88}; 
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data_accel, 5); 

    // Set magnetometer to 220 Hz mode
    uint8_t write_data_mag[2] = {LSM303_REG_MAG_CRA_REG_M, 0x1C};
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data_mag, 2); 

    // Enable the magnetometer
    // LSM303_REG_MAG_MR_REG_M tends to revert back to 0x01 if any other config regs change
    // This delay is required or else the register won't change value. Must wait about 5 ms
    // after any other config register is altered.
    HAL_Delay(5); 
    uint8_t write_data_mag2[2] = {LSM303_REG_MAG_MR_REG_M, 0x00};
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data_mag2, 2); 
}

void LSM303_read() {
    // Read the magnetometer
    uint8_t write_data_mag[1] = {LSM303_REG_MAG_OUT_X_H_M};
    uint8_t read_data_mag[6];
    I2C_WriteRead(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data_mag, 1, read_data_mag, 6); 

    // Shift values to create properly formed integer 
    magData.x_raw = (int16_t)((read_data_mag[1]) | read_data_mag[0] << 8);
    magData.y_raw = (int16_t)((read_data_mag[5]) | read_data_mag[4] << 8);
    magData.z_raw = (int16_t)((read_data_mag[3]) | read_data_mag[2] << 8);  

    float x_uT = (float)(magData.x_raw);
    float y_uT = (float)(magData.y_raw);
//    float z_uT = (float)(magData.z_raw);

    // Calculate orientation
    magData.orientation_prev = magData.orientation;
    magData.orientation = (int16_t)((atan2(y_uT, x_uT) * 1800.0 / M_PI + 1800.0));

    // Read the accelerometer
    uint8_t write_data_accel[1] = {LSM303_REG_ACCEL_OUT_X_L_A | 0x80}; // 0x80 addr autoincrement
    uint8_t read_data_accel[6];
    I2C_WriteRead(LSM303_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data_accel, 1, read_data_accel, 6); 

    // Shift values to create properly formed integer 
    accelData.x = (int16_t)((read_data_accel[1] << 8) | read_data_accel[0]);
    accelData.y = (int16_t)((read_data_accel[3] << 8) | read_data_accel[2]);
    accelData.z = (int16_t)((read_data_accel[5] << 8) | read_data_accel[4]);

}

