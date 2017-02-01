
#include "main.h"
#include "LSM303.h"
#include "i2c.h"
#include "math.h"

void LSM303_begin() {
    // Enable the accelerometer
    //    uint8_t write_data[2] = {LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27};
    //    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data, 2); 

    // Enable the magnetometer
    uint8_t write_data[2] = {LSM303_REGISTER_MAG_MR_REG_M, 0x00};
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data, 2); 
}

void LSM303_read() {
    // Read the accelerometer
    //    uint8_t write_data[2] = {reg, value};
    //    I2C_Write(LSM303_I2C_INTERFACE, address, write_data, 2); 
    //    Wire.beginTransmission((uint8_t)LSM303_ADDRESS_ACCEL);
    //    Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
    //    Wire.endTransmission();
    //    Wire.requestFrom((uint8_t)LSM303_ADDRESS_ACCEL, (uint8_t)6);
    //
    //    // Wait around until enough data is available
    //    while (Wire.available() < 6);
    //
    //    uint8_t xlo = Wire.read();
    //    uint8_t xhi = Wire.read();
    //    uint8_t ylo = Wire.read();
    //    uint8_t yhi = Wire.read();
    //    uint8_t zlo = Wire.read();
    //    uint8_t zhi = Wire.read();
    //
    //    // Shift values to create properly formed integer (low uint8_t first)
    //    // KTOWN: 12-bit values are left-aligned, no shift needed
    //    // accelData.x = (xlo | (xhi << 8)) >> 4;
    //    // accelData.y = (ylo | (yhi << 8)) >> 4;
    //    // accelData.z = (zlo | (zhi << 8)) >> 4;
    //    accelData.x = (int16_t)((xhi << 8) | xlo);
    //    accelData.y = (int16_t)((yhi << 8) | ylo);
    //    accelData.z = (int16_t)((zhi << 8) | zlo);

    // Read the magnetometer
    uint8_t write_data[1] = {LSM303_REGISTER_MAG_OUT_X_H_M};
    I2C_Write(LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data, 1); 
    HAL_Delay(1);

    uint8_t read_data[6];
    I2C_Read (LSM303_I2C_INTERFACE, LSM303_ADDRESS_MAG, read_data, 6); 
    HAL_Delay(1);

    // Shift values to create properly formed integer (low uint8_t first)
    magData.x_raw = (int16_t)(read_data[1] | ((int16_t)read_data[0] << 8));
    magData.y_raw = (int16_t)(read_data[5] | ((int16_t)read_data[4] << 8));
    magData.z_raw = (int16_t)(read_data[3] | ((int16_t)read_data[2] << 8));  

    float x_uT = (float)(magData.x_raw);
    float y_uT = (float)(magData.y_raw);
//    float z_uT = (float)(magData.z_raw);

    // Calculate orientation
    if (y_uT > 0){
        magData.orientation = (int16_t)(900.0 - atanf(x_uT / y_uT) * 1800.0 / M_PI);
    }
    else if (y_uT < 0) {
        magData.orientation = (int16_t)(2700.0 - atanf(x_uT / y_uT) * 1800.0 / M_PI);
    }
    else {
        if (x_uT < 0) {
            magData.orientation = 1800;
        }
        else {
            magData.orientation = 0;
        }
    }
}

