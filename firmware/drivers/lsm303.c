#include <Adafruit_LSM303.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
bool Adafruit_LSM303::begin()
{
  Wire.begin();

  // Enable the accelerometer
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27);
  
  // Enable the magnetometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303::read()
{
  // Read the accelerometer
  Wire.beginTransmission((uint8_t)LSM303_ADDRESS_ACCEL);
  Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)LSM303_ADDRESS_ACCEL, (uint8_t)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();

  // Shift values to create properly formed integer (low uint8_t first)
  // KTOWN: 12-bit values are left-aligned, no shift needed
  // accelData.x = (xlo | (xhi << 8)) >> 4;
  // accelData.y = (ylo | (yhi << 8)) >> 4;
  // accelData.z = (zlo | (zhi << 8)) >> 4;
  accelData.x = (int16_t)((xhi << 8) | xlo);
  accelData.y = (int16_t)((yhi << 8) | ylo);
  accelData.z = (int16_t)((zhi << 8) | zlo);
  
  // Read the magnetometer
  Wire.beginTransmission((uint8_t)LSM303_ADDRESS_MAG);
  Wire.write(LSM303_REGISTER_MAG_OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)LSM303_ADDRESS_MAG, (uint8_t)6);
  
  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Note high before low (different than accel)  
  xhi = Wire.read();
  xlo = Wire.read();
  zhi = Wire.read();
  zlo = Wire.read();
  yhi = Wire.read();
  ylo = Wire.read();
  
  // Shift values to create properly formed integer (low uint8_t first)
  magData.x = (xlo | (xhi << 8));
  magData.y = (ylo | (yhi << 8));
  magData.z = (zlo | (zhi << 8));  
  
  // ToDo: Calculate orientation
  magData.orientation = 0.0;
}

void Adafruit_LSM303::setMagGain(lsm303MagGain gain)
{
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void write8 (uint8_t address, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t read8 (uint8_t address, uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}
