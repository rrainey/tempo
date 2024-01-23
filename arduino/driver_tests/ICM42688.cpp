/* 01/14/2022 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The ICM42688 is a combo sensor with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "ICM42688.h"
#include "I2Cdev.h"

ICM42688::ICM42688( I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t ICM42688::getChipID()
{
  uint8_t temp =_i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_WHO_AM_I);
  return temp;
}


float ICM42688::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
  }
}

float ICM42688::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
     case GFS_15_625DPS:
          _gRes = 15.625f/32768.0f;
          return _gRes;
          break;
    case GFS_31_25DPS:
          _gRes = 31.25f/32768.0f;
          return _gRes;
          break;
    case GFS_62_50DPS:
          _gRes = 62.5f/32768.0f;
          return _gRes;
          break;
    case GFS_125DPS:
          _gRes = 125.0f/32768.0f;
          return _gRes;
          break;
    case GFS_250DPS:
          _gRes = 250.0f/32768.0f;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0f/32768.0f;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0f/32768.0f;
         return _gRes;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0f/32768.0f;
         return _gRes;
         break;
  }
}


void ICM42688::reset()
{
  // reset device
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_DEVICE_CONFIG,  0x01); // Set bit 0 to 1 to reset ICM42688
  delay(1); // Wait 1 ms for all registers to reset 
}


uint8_t ICM42688::DRStatus()
{
  uint8_t temp = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_INT_STATUS);  
  return temp;
}


void ICM42688::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t aMode, uint8_t gMode, bool CLKIN)
{
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_PWR_MGMT0,  gMode << 2 | aMode); // set accel and gyro modes
  delay(1);

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
  
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_GYRO_CONFIG0,  Gscale << 5 | GODR); // set gyro ODR and FS
  
  //RBR: used defaults since we're using LN data rates
  //_i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10
 
   // interrupt handling
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_INT_CONFIG, 0x18 | 0x03 );      // push-pull, pulsed, active HIGH interrupts

  uint8_t temp = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_INT_CONFIG1); 
  temp &= ~(0x10);              // clear bit 4 to allow async interrupt reset (required for proper interrupt operation)
  if (AODR <= AODR_4kHz || GODR <= GODR_4kHz) {
    temp |= (1<<5) | (1<<6);    // shorten interrupt pulse/deassertion duration for higher data rates (required; see 14.5)
  }
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_INT_CONFIG1, temp);
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_INT_SOURCE0, 0x08);   // data ready interrupt routed to INT1
  
  // Use external clock source
  if(CLKIN) {
    _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0

    _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_INTF_CONFIG1, 0x95); // enable RTC

    _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x01); // select register bank 1

    _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_INTF_CONFIG5, 0x04); // use CLKIN as clock source
  }

    _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


void ICM42688::selfTest(int16_t * accelDiff, int16_t * gyroDiff, float * ratio)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelSTest[3] = {0, 0, 0}, gyroSTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
 
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_PWR_MGMT0,  0x0F); // turn on accel and gyro in low noise mode
  delay(1);

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_ACCEL_CONFIG0, AFS_4G << 5 | AODR_1kHz);  // FS = 2
  
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_GYRO_CONFIG0,  GFS_250DPS << 5 | GODR_1kHz);  // FS = 3
 
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10

  readData(temp);
  accelNom[0] = temp[1];
  accelNom[1] = temp[2];
  accelNom[2] = temp[3];
  gyroNom[0]  = temp[4];
  gyroNom[1]  = temp[5];
  gyroNom[2]  = temp[6];
  
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_SELF_TEST_CONFIG, 0x78); // accel self test
  delay(100); // let accel respond
  readData(temp);
  accelSTest[0] = temp[1];
  accelSTest[1] = temp[2];
  accelSTest[2] = temp[3];

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_SELF_TEST_CONFIG, 0x07); // gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroSTest[0] = temp[4];
  gyroSTest[1] = temp[5];
  gyroSTest[2] = temp[6];

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_SELF_TEST_CONFIG, 0x00); // normal mode

  accelDiff[0] = accelSTest[0] - accelNom[0];
  if(accelDiff[0] < 0) accelDiff[0] *= -1;        // make sure difference values are positive
  accelDiff[1] = accelSTest[1] - accelNom[1];
  if(accelDiff[1] < 0)accelDiff[1] *= -1;
  accelDiff[2] = accelSTest[2] - accelNom[2];
  if(accelDiff[2] < 0) accelDiff[2] *= -1;
  gyroDiff[0] = gyroSTest[0] - gyroNom[0];
  if(gyroDiff[0] < 0) gyroDiff[0] *= -1;
  gyroDiff[1] = gyroSTest[1] - gyroNom[1];
  if(gyroDiff[1] < 0) gyroDiff[1] *= -1;
  gyroDiff[2] = gyroSTest[2] - gyroNom[2];
  if(gyroDiff[2] < 0) gyroDiff[2] *= -1;
  
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x01); // select register bank 1

  temp[4] = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_XG_ST_DATA); // gyro self-test output generated during manufacturing tests
  temp[5] = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_YG_ST_DATA);
  temp[6] = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_ZG_ST_DATA);

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x02); // select register bank 2

  temp[1] = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_XA_ST_DATA);  // accel self-test output generated during manufacturing tests
  temp[2] = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_YA_ST_DATA);
  temp[3] = _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_ZA_ST_DATA);

  ratio[1] = accelDiff[0] / (1310.0f * powf(1.01f, temp[1] - 1) + 0.5f);
  ratio[2] = accelDiff[1] / (1310.0f * powf(1.01f, temp[2] - 1) + 0.5f);
  ratio[3] = accelDiff[2] / (1310.0f * powf(1.01f, temp[3] - 1) + 0.5f);
  ratio[4] = gyroDiff[0]  / (2620.0f * powf(1.01f, temp[4] - 1) + 0.5f);
  ratio[5] = gyroDiff[1] /  (2620.0f * powf(1.01f, temp[5] - 1) + 0.5f);
  ratio[6] = gyroDiff[2] /  (2620.0f * powf(1.01f, temp[6] - 1) + 0.5f);
  
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


void ICM42688::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};
    
  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1]*_aRes/128.0f;
  dest1[1] = sum[2]*_aRes/128.0f;
  dest1[2] = sum[3]*_aRes/128.0f;
  dest2[0] = sum[4]*_gRes/128.0f;
  dest2[1] = sum[5]*_gRes/128.0f;
  dest2[2] = sum[6]*_gRes/128.0f;

  if(dest1[0] > 0.8f)  {dest1[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[0] < -0.8f) {dest1[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[1] > 0.8f)  {dest1[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[1] < -0.8f) {dest1[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[2] > 0.8f)  {dest1[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
  if(dest1[2] < -0.8f) {dest1[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
/*
  // load offset biases into offset registers (optional, comment out if not desired)
  temp[0] = (int16_t) (-dest1[0] / 0.00048828125f); // Ax 0.5 mg resolution
  temp[1] = (int16_t) (-dest1[1] / 0.00048828125f); // Ay
  temp[2] = (int16_t) (-dest1[2] / 0.00048828125f); // Az
  temp[3] = (int16_t) (-dest2[0] / 0.03125f);       // Gx 1/32 dps resolution
  temp[4] = (int16_t) (-dest2[1] / 0.03125f);       // Gy
  temp[5] = (int16_t) (-dest2[2] / 0.03125f);       // Gz

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER5,  temp[0] & 0x00FF); // lower Ax byte
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER6,  temp[1] & 0x00FF); // lower Ay byte
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER8,  temp[2] & 0x00FF); // lower Az byte
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER2,  temp[4] & 0x00FF); // lower Gy byte
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER3,  temp[5] & 0x00FF); // lower Gz byte
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER0,  temp[3] & 0x00FF); // lower Gx byte
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER4,  (temp[0] & 0x0F00) >> 4 | (temp[5] & 0x0F00) >> 8); // upper Ax and Gz bytes
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER7,  (temp[2] & 0x0F00) >> 4 | (temp[1] & 0x0F00) >> 8); // upper Az and Ay bytes
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_OFFSET_USER1,  (temp[4] & 0x0F00) >> 4 | (temp[3] & 0x0F00) >> 8); // upper Gy and Gx bytes
  
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  */
}


void ICM42688::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(ICM42688_ADDRESS, ICM42688_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}


void ICM42688::setTiltDetect()
{
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_APEX_CONFIG4, 0x00); // immediately interrupt on tilt

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_APEX_CONFIG0, 0x02); // DMP power save off, DMP ODR at 50 Hz
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_SIGNAL_PATH_RESET, 0x20); // DMP memory reset
  delay(1);
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_SIGNAL_PATH_RESET, 0x40); // DMP enable
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_APEX_CONFIG0, 0x12); // Tilt detect enable
  delay(2000); // wait for tilt to stabilize

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_INT_SOURCE7, 0x08); // route tilt interrupt to INT2

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


void ICM42688::setWakeonMotion()
{
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_ACCEL_WOM_X_THR, 0x50); // 80 x 3.9 mg is ~312 mg
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_ACCEL_WOM_Y_THR, 0x50); // 80 x 3.9 mg is ~312 mg
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_ACCEL_WOM_Z_THR, 0x50); // 80 x 3.9 mg is ~312 mg

  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_INT_SOURCE4, 0x07);  // route wake on motion for any axis to INT2
  _i2c_bus->writeByte(ICM42688_ADDRESS, ICM42688_SMD_CONFIG, 0x05);   // differential mode, WOM interrupt ORed with other APEX interrupts, enable WOM function (bit 0)
}


uint16_t ICM42688::APEXStatus()
{
  uint8_t status2 =  _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_INT_STATUS2);
  uint8_t status3 =  _i2c_bus->readByte(ICM42688_ADDRESS, ICM42688_INT_STATUS3); 
  uint16_t temp = ((uint16_t) status2 << 8) | status3; 
  return temp;
}
