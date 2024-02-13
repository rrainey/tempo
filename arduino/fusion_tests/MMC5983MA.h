/* 06/14/2020 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef MMC5983MA_h
#define MMC5983MA_h

#include "Arduino.h"
#include <Wire.h>

//Register map for MMC5983MA'
//http://www.memsic.com/userfiles/files/DataSheets/Magnetic-Sensors-Datasheets/MMC5983MA_Datasheet.pdf
#define MMC5983MA_XOUT_0        0x00
#define MMC5983MA_XOUT_1        0x01
#define MMC5983MA_YOUT_0        0x02
#define MMC5983MA_YOUT_1        0x03
#define MMC5983MA_ZOUT_0        0x04
#define MMC5983MA_ZOUT_1        0x05
#define MMC5983MA_XYZOUT_2      0x06
#define MMC5983MA_TOUT          0x07
#define MMC5983MA_STATUS        0x08
#define MMC5983MA_CONTROL_0     0x09
#define MMC5983MA_CONTROL_1     0x0A
#define MMC5983MA_CONTROL_2     0x0B
#define MMC5983MA_CONTROL_3     0x0C
#define MMC5983MA_PRODUCT_ID    0x2F

#define MMC5983MA_ADDRESS       0x30

// Sample rates
#define MODR_ONESHOT   0x00
#define MODR_1Hz       0x01
#define MODR_10Hz      0x02
#define MODR_20Hz      0x03
#define MODR_50Hz      0x04
#define MODR_100Hz     0x05
#define MODR_200Hz     0x06 // BW = 0x01 only
#define MODR_1000Hz    0x07 // BW = 0x11 only

//Bandwidths
#define MBW_100Hz 0x00  // 8 ms measurement time
#define MBW_200Hz 0x01  // 4 ms
#define MBW_400Hz 0x02  // 2 ms
#define MBW_800Hz 0x03  // 0.5 ms


// Set/Reset as a function of measurements
#define MSET_1     0x00 // Set/Reset each data measurement
#define MSET_25    0x01 // each 25 data measurements
#define MSET_75    0x02
#define MSET_100   0x03
#define MSET_250   0x04
#define MSET_500   0x05
#define MSET_1000  0x06
#define MSET_2000  0x07

/*
Control Register 0
All bits Write-only
*/

#define CONTROL0_TM_M        (1<<0)
#define CONTROL0_TM_T        (1<<1)
#define CONTROL0_INT_DONE_EN (1<<2)
#define CONTROL0_SET         (1<<3)
#define CONTROL0_RESET       (1<<4)
#define CONTROL0_AUTO_SR_EN  (1<<5)
#define CONTROL0_OTP_READ    (1<<6)

/*
Status Register
All bits R/W
*/

#define STATUS_MEAS_M_DONE       (1<<0)
#define STATUS_MEAS_T_DONE       (1<<1)
#define STATUS_DONE_EN           (1<<2)
#define STATUS_OTP_READ_DONE     (1<<4)

class MMC5983MA
{
  public:
  MMC5983MA(uint8_t devAddress = MMC5983MA_ADDRESS, TwoWire *i2c = &Wire);

  // Reads the device ID from the chip
  uint8_t getChipID();

  void init(uint8_t MODR, uint8_t MBW, uint8_t MSET);
  void offsetBias(float * dest1, float * dest2);
  void reset();
  uint8_t status();
  void clearInt();
  void selfTest();
  void readData(uint32_t * destination);
  uint8_t readTemperature();
  void SET();
  void RESET();
  void getOffset(float * destination);
  void powerDown();
  void powerUp(uint8_t MODR);

  // Deprecated call
  void writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

  // Deprecated call
  uint8_t readByte(uint8_t devAddr, uint8_t subAddress);

  // Deprecated call
  void readBytes(uint8_t devAddr, uint8_t subAddress, uint8_t count, uint8_t * dest);
  
  private:
  float _mRes;
  TwoWire*  _i2c;
  uint8_t   _deviceAddress;

  // these registers are write-only on the device
  // so we save what was written to them here
  uint8_t shadowControlRegisters[4];

};

#endif
