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
//    // 100 Hz mode
//    // HPF disabled
//    // 12-bit high resolution mode
//    // BDU enabled
//    uint8_t write_data_accel[5] = {LSM303_REG_ACCEL_CTRL_REG1_A | 0x80, 0x57, 0x00, 0x00, 0x88}; 
//
    // 1344 Hz mode
    // HPF disabled
    // 12-bit high resolution mode
    // BDU enabled
    uint8_t write_data_accel[5] = {LSM303_REG_ACCEL_CTRL_REG1_A | 0x80, 0x97, 0x00, 0x00, 0x88}; 
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
    gyroData.x_filt = gyroData.x_filt - (gyroData.x_filt >> FILTER_SHIFT_GYRO) + 
        ((read_data[1] << 8) | read_data[0]);
    gyroData.y_filt = gyroData.y_filt - (gyroData.y_filt >> FILTER_SHIFT_GYRO) + 
        ((read_data[3] << 8) | read_data[2]);
    gyroData.z_filt = gyroData.z_filt - (gyroData.z_filt >> FILTER_SHIFT_GYRO) + 
        ((read_data[5] << 8) | read_data[4]);  

    gyroData.x = gyroData.x_filt >> FILTER_SHIFT_GYRO;
    gyroData.y = gyroData.y_filt >> FILTER_SHIFT_GYRO;
    gyroData.z = gyroData.z_filt >> FILTER_SHIFT_GYRO;
}

void accelerometer_read() {
    // Read the accelerometer
    uint8_t write_data[1] = {LSM303_REG_ACCEL_OUT_X_L_A | 0x80}; // 0x80 addr autoincrement
    uint8_t read_data[6];
    I2C_WriteRead(IMU_I2C_INTERFACE, LSM303_ADDRESS_ACCEL, write_data, 1, read_data, 6); 

    // Shift values to create properly formed integer 
    accelData.x_filt = accelData.x_filt - (accelData.x_filt >> FILTER_SHIFT_ACCEL) + 
        ((int16_t)((read_data[1] << 8) | read_data[0]) >> 4);
    accelData.y_filt = accelData.y_filt - (accelData.y_filt >> FILTER_SHIFT_ACCEL) + 
        ((int16_t)((read_data[3] << 8) | read_data[2]) >> 4);
    accelData.z_filt = accelData.z_filt - (accelData.z_filt >> FILTER_SHIFT_ACCEL) + 
        ((int16_t)((read_data[5] << 8) | read_data[4]) >> 4);  

    accelData.x_raw = (accelData.x_filt >> FILTER_SHIFT_ACCEL);
    accelData.y_raw = (accelData.y_filt >> FILTER_SHIFT_ACCEL);
    accelData.z_raw = (accelData.z_filt >> FILTER_SHIFT_ACCEL);

    accelData.x = -1 * ((double)accelData.x_raw * ACC_S_X + ACC_O_X); 
    accelData.y = ((double)accelData.y_raw * ACC_S_Y + ACC_O_Y); 
    accelData.z = ((double)accelData.z_raw * ACC_S_Z + ACC_O_Z); 
}

void magnetometer_read() {
    // Read the magnetometer
    uint8_t write_data[1] = {LSM303_REG_MAG_OUT_X_H_M};
    uint8_t read_data[6];
    I2C_WriteRead(IMU_I2C_INTERFACE, LSM303_ADDRESS_MAG, write_data, 1, read_data, 6); 

    // Shift values to create properly formed integer 
    magData.x_filt = magData.x_filt - (magData.x_filt >> FILTER_SHIFT_MAG) + 
        (int16_t)((read_data[0] << 8) | read_data[1]);
    magData.y_filt = magData.y_filt - (magData.y_filt >> FILTER_SHIFT_MAG) + 
        (int16_t)((read_data[4] << 8) | read_data[5]);
    magData.z_filt = magData.z_filt - (magData.z_filt >> FILTER_SHIFT_MAG) + 
        (int16_t)((read_data[2] << 8) | read_data[3]);  

    magData.x_raw = magData.x_filt >> FILTER_SHIFT_MAG;
    magData.y_raw = -1 * (magData.y_filt >> FILTER_SHIFT_MAG);
    magData.z_raw = -1 * (magData.z_filt >> FILTER_SHIFT_MAG);

    magData.x = (double)magData.x_raw;// * MAG_S_X + MAG_O_X; 
    magData.y = (double)magData.y_raw;// * MAG_S_Y + MAG_O_Y; 
    magData.z = (double)magData.z_raw;// * MAG_S_Z + MAG_O_Z; 
}

void calc_compass(void){
    double pitch;
    double cos_pitch;
    double roll;
    double xh;
    double yh;

    magnetometer_read();
    accelerometer_read();

    // Returns a heading from +pi to -pi
    // Tilt compensated heading calculation
    pitch = asin(-1.0 * accelData.x);
    cos_pitch = cos(pitch);
    if (cos_pitch == 0.){
        cos_pitch = 0.000000001;
    }
    roll = asin(accelData.y / cos_pitch);

    pitch_int = (int16_t)(pitch*1800/3.14159);
    roll_int = (int16_t)(roll*1800/3.14159);

    xh = magData.x * cos(pitch) + magData.z * sin(pitch);
    yh = magData.x * sin(roll) * sin(pitch) + magData.y * cos(roll) - magData.z * sin(roll) * cos(pitch);
    heading_f = atan2(yh, xh);
    heading = (int16_t)heading_f;
    //heading = (int16_t)(atan2(yh, xh) * 10000);
}

