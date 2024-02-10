/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "MMC5983MA.h"
#include <Wire.h>

MMC5983MA::MMC5983MA(uint8_t devAddress, TwoWire *i2c)
{
  _i2c = i2c;
  _deviceAddress = devAddress;

  for( int i=0; i<4; ++i) {
    shadowControlRegisters[i] = 0;
  }

}

uint8_t MMC5983MA::getChipID()
{
  uint8_t c = readByte(_deviceAddress, MMC5983MA_PRODUCT_ID);
  return c;
}


void MMC5983MA::reset()
{
  // reset device
  writeByte(_deviceAddress, MMC5983MA_CONTROL_1, (1<<7)); // Set bit 7 to 1 to reset MMC5983MA
  delay(11); // Wait 10 ms for all registers to reset 

  for( int i=0; i<4; ++i) {
    shadowControlRegisters[i] = 0;
  }
}


void MMC5983MA::init(uint8_t MODR, uint8_t MBW, uint8_t MSET)
{
 // enable data ready interrupt (bit2 == 1), enable auto set/reset (bit 5 == 1)
 writeByte(_deviceAddress, MMC5983MA_CONTROL_0, CONTROL0_INT_DONE_EN | CONTROL0_AUTO_SR_EN );

  shadowControlRegisters[0] = CONTROL0_INT_DONE_EN | CONTROL0_AUTO_SR_EN;

 // set magnetometer bandwidth
 writeByte(_deviceAddress, MMC5983MA_CONTROL_1, MBW);

 shadowControlRegisters[1] = MBW;

 // enable continuous measurement mode (bit 3 == 1), set sample rate
 // enable automatic Set/Reset (bit 7 == 1), set set/reset rate
 writeByte(_deviceAddress, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | (1<<3) | MODR);  

 shadowControlRegisters[2] = 0x80 | (MSET << 4) | (1<<3) | MODR;
}


void MMC5983MA::selfTest()
{
    uint8_t rawData[6] = {0};  // x/y/z mag register data stored here
    uint16_t data_set[3] ={0}, data_reset[3] = {0};
    uint32_t delta_data[3] = {0};
    
   // clear control registers
   writeByte(_deviceAddress, MMC5983MA_CONTROL_0, 0x00);  
   writeByte(_deviceAddress, MMC5983MA_CONTROL_1, 0x00);  
   writeByte(_deviceAddress, MMC5983MA_CONTROL_2, 0x00);

   for( int i=0; i<3; ++i) {
    shadowControlRegisters[i] = 0;
  }

   SET(); // enable set current
   writeByte(_deviceAddress, MMC5983MA_CONTROL_0, CONTROL0_TM_M);  //enable one-time mag measurement
   delay(10);
   
   readBytes(_deviceAddress, MMC5983MA_XOUT_0, 6, &rawData[0]);  // Read the 6 raw data registers into data array
   data_set[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
   data_set[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
   data_set[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis

   RESET(); // enable reset current
   writeByte(_deviceAddress, MMC5983MA_CONTROL_0, CONTROL0_TM_M);  //enable one-time mag measurement
   delay(10);
   
   readBytes(_deviceAddress, MMC5983MA_XOUT_0, 6, &rawData[0]);  // Read the 6 raw data registers into data array
   data_reset[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
   data_reset[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
   data_reset[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis
 
   for (uint8_t ii = 0; ii < 3; ii++)
   {
    if(data_set[ii] > data_reset[ii]) 
    { 
      delta_data[ii] = data_set[ii] - data_reset[ii];
    }
    else
    {
       delta_data[ii] = data_reset[ii] - data_set[ii];
    }
  }
  
  Serial.print("x-axis self test = "); Serial.print(delta_data[0]); Serial.println(", should be >100");
  Serial.print("y-axis self test = "); Serial.print(delta_data[1]); Serial.println(", should be >100");
  Serial.print("z-axis self test = "); Serial.print(delta_data[2]); Serial.println(", should be >100");
  }


  void MMC5983MA::getOffset(float * destination)
{
   uint8_t rawData[6] = {0};  // x/y/z mag register data stored here
   uint16_t data_set[3] ={0}, data_reset[3] = {0};
    
   powerDown();
 
   SET(); // enable set current
   writeByte(_deviceAddress, MMC5983MA_CONTROL_0, CONTROL0_TM_M);  //enable one-time mag measurement
   delay(11);
   
   readBytes(_deviceAddress, MMC5983MA_XOUT_0, 6, &rawData[0]);  // Read the 6 raw data registers into data array
   data_set[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
   data_set[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
   data_set[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis

   RESET(); // enable reset current
   writeByte(_deviceAddress, MMC5983MA_CONTROL_0, CONTROL0_TM_M);  //enable one-time mag measurement
   delay(11);
   
   readBytes(_deviceAddress, MMC5983MA_XOUT_0, 6, &rawData[0]);  // Read the 6 raw data registers into data array
   data_reset[0] = (uint16_t) (((uint16_t) rawData[0] << 8) | rawData[1]); // x-axis
   data_reset[1] = (uint16_t) (((uint16_t) rawData[2] << 8) | rawData[3]); // y-axis
   data_reset[2] = (uint16_t) (((uint16_t) rawData[4] << 8) | rawData[5]); // z-axis
 
   for (uint8_t ii = 0; ii < 3; ii++)
   {
      destination[ii] = ((float)data_set[ii] + (float)data_reset[ii])/2.0f;
   }
}


void MMC5983MA::SET()
{
   writeByte(_deviceAddress, MMC5983MA_CONTROL_0, 0x08);  
   delay(1); // self clearing after 500 us
}


void MMC5983MA::RESET()
{
   writeByte(_deviceAddress, MMC5983MA_CONTROL_0, 0x10);  
   delay(1); // self clearing after 500 us
}


uint8_t MMC5983MA::status()
{
  // Read status register
  uint8_t temp = readByte(_deviceAddress, MMC5983MA_STATUS);
  return temp;
}


void MMC5983MA::clearInt()
{
  // Clear data ready interrupts
  writeByte(_deviceAddress, MMC5983MA_STATUS, STATUS_MEAS_M_DONE | STATUS_MEAS_T_DONE);
}


void MMC5983MA::readData(uint32_t * destination)  
{
  uint8_t rawData[7];  // x/y/z mag register data stored here
  readBytes(_deviceAddress, MMC5983MA_XOUT_0, 7, &rawData[0]);  // Read the 7 raw data registers into data array
  destination[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
  destination[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
  destination[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
}


uint8_t MMC5983MA::readTemperature()
{
  uint8_t temp;

  writeByte(_deviceAddress, MMC5983MA_CONTROL_0, shadowControlRegisters[0] | CONTROL0_TM_T);   //enable one-time temp measurement

  delay (2);
  while (readByte(_deviceAddress, MMC5983MA_STATUS) & STATUS_MEAS_T_DONE == 0) {
    delay (1);
  }

  temp = readByte(_deviceAddress, MMC5983MA_TOUT);  // Read the raw temperature register 
  return temp;
}


void MMC5983MA::offsetBias(float * dest1, float * dest2)
{
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int32_t mag_max[3] = {-262143, -262143, -262143}, mag_min[3] = {262143, 262143, 262143};
  uint32_t mag_temp[3] = {0, 0, 0}, magOffset = 131072;
  float _mRes = 1.0f/16384.0f;        // mag sensitivity if using 18 bit data
  
  Serial.println("Calculate mag offset bias: rotate device");
  delay(2000);

  for (int ii = 0; ii < 1000; ii++)
  {
    readData(mag_temp);
    for (int jj = 0; jj < 3; jj++) {
      if((int32_t)(mag_temp[jj] - magOffset) > mag_max[jj]) mag_max[jj] = (int32_t)(mag_temp[jj] - magOffset);
      if((int32_t)(mag_temp[jj] - magOffset) < mag_min[jj]) mag_min[jj] = (int32_t)(mag_temp[jj] - magOffset);
    }
    delay(12);
  }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) (mag_bias[0]) * _mRes;  // save mag biases in G for main program
    dest1[1] = (float) (mag_bias[1]) * _mRes;   
    dest1[2] = (float) (mag_bias[2]) * _mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration complete");
}


void MMC5983MA::powerDown()
{
  shadowControlRegisters[2] = shadowControlRegisters[2] & 0xf0; 
  writeByte(_deviceAddress, MMC5983MA_CONTROL_2,  shadowControlRegisters[2]);
  delay(20); // make sure to finish the lest measurement

}


void MMC5983MA::powerUp(uint8_t MODR)
{
  shadowControlRegisters[2] = (shadowControlRegisters[2] & 0xf0) | (1<<3) | MODR;
  writeByte(_deviceAddress, MMC5983MA_CONTROL_2, shadowControlRegisters[2]);
}

uint8_t MMC5983MA::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data = 0;                         
  _i2c->beginTransmission(address);
  _i2c->write(subAddress);
  _i2c->endTransmission(false);
  _i2c->requestFrom(address, (uint8_t) 1);
  data = _i2c->read();
  return data;
  
}

void MMC5983MA::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  _i2c->beginTransmission(address);   // Initialize the Tx buffer
  _i2c->write(subAddress);            // Put slave register address in Tx buffer
  _i2c->endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  _i2c->requestFrom(address, count);  // Read bytes from slave register address 
  while (_i2c->available()) {
        dest[i++] = _i2c->read(); 
  }   // Put read results in the Rx buffer
}

// Legacy calling signature
void MMC5983MA::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  _i2c->beginTransmission(devAddr);  // Initialize the Tx buffer
  _i2c->write(regAddr);           // Put slave register address in Tx buffer
  _i2c->write(data);                 // Put data in Tx buffer
  _i2c->endTransmission();           // Send the Tx buffer
}