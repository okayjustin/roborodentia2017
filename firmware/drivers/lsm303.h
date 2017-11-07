
#ifndef __LSM303_H__
#define __LSM303_H__

#define LSM303_I2C_INTERFACE          &hi2c2
#define LSM303_ADDRESS_ACCEL          (0x19)         // 0011 001x
#define LSM303_ADDRESS_MAG            (0x1E)         // 0011 110x
#define SENSORS_GAUSS_TO_MICROTESLA   100

typedef enum
{                                                
    LSM303_REG_ACCEL_CTRL_REG1_A         = 0x20, 
    LSM303_REG_ACCEL_CTRL_REG2_A         = 0x21, 
    LSM303_REG_ACCEL_CTRL_REG3_A         = 0x22, 
    LSM303_REG_ACCEL_CTRL_REG4_A         = 0x23, 
    LSM303_REG_ACCEL_CTRL_REG5_A         = 0x24, 
    LSM303_REG_ACCEL_CTRL_REG6_A         = 0x25, 
    LSM303_REG_ACCEL_REFERENCE_A         = 0x26, 
    LSM303_REG_ACCEL_STATUS_REG_A        = 0x27, 
    LSM303_REG_ACCEL_OUT_X_L_A           = 0x28,
    LSM303_REG_ACCEL_OUT_X_H_A           = 0x29,
    LSM303_REG_ACCEL_OUT_Y_L_A           = 0x2A,
    LSM303_REG_ACCEL_OUT_Y_H_A           = 0x2B,
    LSM303_REG_ACCEL_OUT_Z_L_A           = 0x2C,
    LSM303_REG_ACCEL_OUT_Z_H_A           = 0x2D,
    LSM303_REG_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
    LSM303_REG_ACCEL_FIFO_SRC_REG_A      = 0x2F,
    LSM303_REG_ACCEL_INT1_CFG_A          = 0x30,
    LSM303_REG_ACCEL_INT1_SRC_A          = 0x31,
    LSM303_REG_ACCEL_INT1_THS_A          = 0x32,
    LSM303_REG_ACCEL_INT1_DURATION_A     = 0x33,
    LSM303_REG_ACCEL_INT2_CFG_A          = 0x34,
    LSM303_REG_ACCEL_INT2_SRC_A          = 0x35,
    LSM303_REG_ACCEL_INT2_THS_A          = 0x36,
    LSM303_REG_ACCEL_INT2_DURATION_A     = 0x37,
    LSM303_REG_ACCEL_CLICK_CFG_A         = 0x38,
    LSM303_REG_ACCEL_CLICK_SRC_A         = 0x39,
    LSM303_REG_ACCEL_CLICK_THS_A         = 0x3A,
    LSM303_REG_ACCEL_TIME_LIMIT_A        = 0x3B,
    LSM303_REG_ACCEL_TIME_LATENCY_A      = 0x3C,
    LSM303_REG_ACCEL_TIME_WINDOW_A       = 0x3D
} lsm303AccelRegisters_t;

typedef enum
{
    LSM303_REG_MAG_CRA_REG_M             = 0x00,
    LSM303_REG_MAG_CRB_REG_M             = 0x01,
    LSM303_REG_MAG_MR_REG_M              = 0x02,
    LSM303_REG_MAG_OUT_X_H_M             = 0x03,
    LSM303_REG_MAG_OUT_X_L_M             = 0x04,
    LSM303_REG_MAG_OUT_Z_H_M             = 0x05,
    LSM303_REG_MAG_OUT_Z_L_M             = 0x06,
    LSM303_REG_MAG_OUT_Y_H_M             = 0x07,
    LSM303_REG_MAG_OUT_Y_L_M             = 0x08,
    LSM303_REG_MAG_SR_REG_M              = 0x09,
    LSM303_REG_MAG_IRA_REG_M             = 0x0A,
    LSM303_REG_MAG_IRB_REG_M             = 0x0B,
    LSM303_REG_MAG_IRC_REG_M             = 0x0C
} lsm303MagRegisters_t;

typedef enum
{
    LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
    LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
    LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
    LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
    LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
    LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
    LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} lsm303MagGain;    

typedef struct lsm303AccelData_s
{
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303AccelData;

typedef struct lsm303MagData_s
{
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t orientation;   // In units of degrees
    int16_t orientation_prev;   // In units of degrees
} lsm303MagData;

void LSM303_begin (void);
void LSM303_read (void);

lsm303AccelData accelData;    // Last read accelerometer data will be available here
lsm303MagData magData;        // Last read magnetometer data will be available here

#endif
