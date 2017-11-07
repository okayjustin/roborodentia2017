
#include "main.h"
#include "LSM303.h"
#include "i2c.h"
#include "math.h"

void LSM303_begin() {
    // Enable the accelerometer
    uint8_t write_data_accel[2] = {LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57};
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data_accel, 2); 

    // Enable the magnetometer
    uint8_t write_data_mag[2] = {LSM303_REGISTER_MAG_MR_REG_M, 0x00};
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data_mag, 2); 
}

void LSM303_read() {
    // Read the accelerometer
    uint8_t write_data_accel[1] = {LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80}; // 0x80 addr autoincrement
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data_accel, 1); 
    uint8_t read_data_accel[6];
    I2C_Read(LSM303_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, read_data_accel, 6); 

    // Shift values to create properly formed integer (low uint8_t first)
    accelData.x = (int16_t)((read_data_accel[1] << 8) | read_data_accel[0]);
    accelData.y = (int16_t)((read_data_accel[3] << 8) | read_data_accel[2]);
    accelData.z = (int16_t)((read_data_accel[5] << 8) | read_data_accel[4]);

    // Read the magnetometer
    uint8_t write_data_mag[1] = {LSM303_REGISTER_MAG_OUT_X_H_M};
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data_mag, 1); 
    uint8_t read_data_mag[6];
    I2C_Read(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, read_data_mag, 6); 

    // Shift values to create properly formed integer (low uint8_t first)
    magData.x_raw = (int16_t)(read_data_mag[1] | ((int16_t)read_data_mag[0] << 8));
    magData.y_raw = (int16_t)(read_data_mag[5] | ((int16_t)read_data_mag[4] << 8));
    magData.z_raw = (int16_t)(read_data_mag[3] | ((int16_t)read_data_mag[2] << 8));  

    float x_uT = (float)(magData.x_raw);
    float y_uT = (float)(magData.y_raw);
//    float z_uT = (float)(magData.z_raw);

    // Calculate orientation
    magData.orientation_prev = magData.orientation;
    magData.orientation = (int16_t)((atan2f(y_uT, x_uT) * 1800.0 / M_PI + 1800.0));
}

