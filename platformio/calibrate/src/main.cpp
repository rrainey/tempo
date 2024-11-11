/* 
 * Test peripherals and drivers that we use with the Tempo board.
 *
 * This application is an amalgam of several open source Arduino projects.
 *
 * Code credits:
 *
 * ICM42688 driver
 * Overall structure based on code by Kris Winer
 * "01/14/2022 Copyright Tlera Corporation"
 * "Library may be used freely and without limit with proper attribution."
 * see https://github.com/kriswiner/ICM42688
 *
 * Arduino SAMD51 Timer module
 * An interrupt timer based on the SAMD51 clock subsystem
 * from https://github.com/Dennis-van-Gils/SAMD51_InterruptTime
 * Dennis van Gils
 * MIT License
 */

#include <Arduino.h>
#include <stdio.h>
#include <avr/dtostrf.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <ICM42688.h>
#include <MMC5983MA.h>
#include "bmp3.h"
#include "TC_Timer.h"

/**
 * Configure compilation for the hardware runtime environment
 * 
 * One of four possibilities:
 * 
 * 1) Tempo board (#define TEMPO_V1)
 * 2) peakick board along with SAMD51 Thing Plus C board (#define SAMD51_THING_PLUS)
 * 3) peakick board along with ESP32 Thing Plus board (set Arduino board to ESP32 Dev)
 * 4) peakick board along with STM32 Thing Plus board (set Arduino board to STM32 Thing Plus)
 */

#if defined(__SAMD51J20A__)

// uncomment when compiling for tempo V1 board
#define TEMPO_V1
// define when using the SAM51 Thing Plus with a Peakick sensor board
//#define SAMD51_THING_PLUS

#endif

SFE_UBLOX_GNSS myGNSS;

#define GPS_I2C_ADDR      0x42  // SAM-M10Q

// "true" for extended Serial debugging messages
#define SerialDebug false

// flags RTC interrupt
volatile bool alarmFlag = false;

#if defined(ESP32)

#include <ESP32Time.h>
#include "ESP32TimerInterrupt.h"

// Sparkfun ESP32 Thing Plus C
#define RED_LED           13
#define GREEN_LED         12
#define CLKOUT            14        // ESP32 GPIO14 on the Sparkfun ESP32 Thing Plus C

// Unused in the current code, but I experimented with supplying the IMU clock from the ESP32
// using the ESP32 ledc API.
#define CPU_SUPPLIES_IMU_CLKIN  false
#define LEDC_CLKIN_CHANNEL    7     // channel (chosen randomly)
#define LEDC_CLKIN_RESOLUTION 8     // 8-BIT COUNTER (0..255)
#define LEDC_CLKIN_BASE_FREQ 32768  // Hz
#define LEDC_DUTY_CYCLE      128    // 50% duty cycle on a 8-bit counter

// ESP32 Thing Plus C/peackick-specific interrupt line connections
#define ICM42688_intPin1 32  // INT/INT1
#define ICM42688_intPin2 14  // INT2/CLKIN
#define MMC5983MA_intPin 15
#define BMP390_intPin    33
#endif

#if defined(STM32F4xx)
// Sparkfun STM-32 (DEV-17712)
#define RED_LED           13
#define GREEN_LED         12
#define CLKOUT            5        

#define CPU_SUPPLIES_IMU_CLKIN  false

// ESP32 Thing Plus C/peackick-specific interrupt line connections
#define ICM42688_intPin1 6  // INT/INT1
#define ICM42688_intPin2 5  // INT2/CLKIN
#define MMC5983MA_intPin 9
#define BMP390_intPin    10

#define IRAM_ATTR

#include <STM32RTC.h>

static STM32RTC::Hour_Format hourFormat = STM32RTC::HOUR_24;
static STM32RTC::AM_PM period = STM32RTC::AM;

STM32RTC& rtc = STM32RTC::getInstance();

void rtc_SecondsCB(void *data)
{
  UNUSED(data);
  alarmFlag = true;
}

#endif

#if defined(SAMD51_THING_PLUS) || defined(TEMPO_V1)

/**
 * Sparkfun Thing Plus SAMD51 
 * 
 * see Sparkfun SAMD51 Thing Plus Hookup Guide
 * 
 * BMP390    INT       D10 
 * MMC5983MA INT       D9  
 * ICM42688  INT/INT1  D6  
 * ICM42688  INT2      D5  
 * 
 */

/**
 * Tempo board connections
 * 
 * BMP390    INT       D11
 * MMC5983MA INT       D9  
 * ICM42688  INT/INT1  D6  
 * ICM42688  INT2      D5  
 * 
 */
#define RED_LED           13
#define GREEN_LED         12

#define CPU_SUPPLIES_IMU_CLKIN  false

#define ICM42688_intPin1 digitalPinToInterrupt(6)   // INT/INT1
#define MMC5983MA_intPin digitalPinToInterrupt(9)
#if defined(TEMPO_V1)
#define BMP390_intPin    digitalPinToInterrupt(11)  // 11 for tempo, 10 for Peakick
#define ICM42688_intPin2 digitalPinToInterrupt(10)   // INT2/CLKIN as an interrupt line
#define CLKOUT            10         // as a SAMD51 output line

ICM42688 imu(SPI, 5);

#else
// SAMD51 Thing Plus
#define BMP390_intPin    digitalPinToInterrupt(10)
#define ICM42688_intPin2 digitalPinToInterrupt(5)   // INT2/CLKIN as an interrupt line
#define CLKOUT            5         // as a SAMD51 output line

ICM42688 imu(Wire);
#endif

#define IRAM_ATTR


/*
 * end of Arduino SAMD51 Timer module
 */

void myISR() {
    alarmFlag = true;
}

#endif

#define bmp3_check_rslt(func, r)                     \
    if (r != BMP3_OK) {                              \
        Serial.println("Error returned from " func); \
    }

// global constants for 9 DoF fusion and AHRS
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError =
    pi * (40.0f /
          180.0f);  // gyroscope measurement error in rads/s (start at 40 deg/s)

// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float GyroMeasDrift = pi * (0.0f / 180.0f);
// compute beta
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;
// compute zeta, the other free parameter in the Madgwick
// scheme usually set to a small or zero value
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;
// components
// float deltat = 0.0f,
//       sum = 0.0f;  // integration interval for both filter schemes
float lin_ax, lin_ay,
    lin_az;  // linear acceleration (acceleration with gravity component
             // subtracted)
float eInt[3] = {0.0f, 0.0f,
                 0.0f};  // vector to hold integral error for Mahony method

extern void IRAM_ATTR MadgwickQuaternionUpdate(float ax, float ay, float az,
                                               float gx, float gy, float gz,
                                               float mx, float my, float mz);


/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G
      GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS,
 GFS_500DPS, GFS_1000DPS, GFS_2000DPS AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz,
 AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz,
      AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz,
 GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/
uint8_t Ascale = AFS_4G;
uint8_t Gscale = GFS_250DPS;
uint8_t AODR = AODR_200Hz;
uint8_t GODR = GODR_200Hz;

icm42688AccelPowerMode aMode = aMode_LN;
icm42688GyroPowerMode gMode = gMode_LN;

// used in Fusion code
#define SAMPLE_RATE (200)   // IMU samples per second
// must be set to match accel/gyro samples
float fSampleInterval_sec = 1.0f / SAMPLE_RATE;

float aRes, gRes;  // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f, 0.0f},
      gyroBias[3] = {0.0f, 0.0f, 0.0f};  // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0},
        gyroDiff[3] = {0, 0, 0};  // difference betwee ST and normal values
float STratio[7] = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f};    // self-test results for the accel and gyro
int16_t ICM42688Data[7];  // Stores the 16-bit signed sensor output
float imuTemp_C;  // Stores the real internal gyro temperature in deg C

// variables to hold latest accel/gyro data values
float ax, ay, az, gx, gy, gz;

/**
 * A counter to track the number of IMU interrupts since the last invocation of loop().
 * Ideally between 0 and 1, but the samples are stored in the FIFO, so nothing gets
 * lost. Reset to zero in each loop() pass.
 */
volatile uint16_t imuUnservicedISRCount = 0;

/* Specify magnetic sensor parameters (continuous mode sample rate is dependent
 * on bandwidth) choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz,
 * MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250,
 * MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th
 * measurement, etc.
 */
mmc5983ma_modr_t MODR = MODR_100Hz;
mmc5983ma_bandwidth_t MBW = MBW_100Hz;
mmc5983ma_sr_Interval_t MSET = MSET_2000;

float mRes = 1.0f / 16384.0f;  // mag sensitivity if using 18 bit data
float magBias[3] = {0, 0, 0}, magScale[3] = {1, 1, 1},
      magOffset[3] = {0};  // Bias corrections for magnetometer
uint32_t
    MMC5983MAData[3];  // Stores the 18-bit unsigned magnetometer sensor output
uint8_t
    MMC5983MAtemperature;  // Stores the magnetometer temperature register data
float Mtemperature;  // Stores the real internal chip temperature in degrees
                     // Celsius
float mx, my, mz;    // variables to hold latest mag data values
uint8_t MMC5983MAstatus;
float MMC5983MA_offset = 131072.0f;

volatile bool newMMC5983MAData = false;

MMC5983MA mmc(MMC5983MA_ADDRESS, &Wire); 

#define SEALEVELPRESSURE_HPA (1013.25)

volatile bool pressureSampleAvailable = false;

void IRAM_ATTR BMPSampleReadyHandler() { pressureSampleAvailable = true; }

/**
 * BMP390 globals
 */
int8_t rslt;
uint16_t settings_sel;
struct bmp3_dev dev;
struct bmp3_data data = {0};
struct bmp3_settings settings = {0};
struct bmp3_status status = {{0}};

// ESP32 RTC API
uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;

#if defined(ESP32)
// UTC timezone
ESP32Time RTC(0);
ESP32Timer ITimer0(0);

// one second status reports to Serial
#define ESP32_TIMER_INTERVAL_USEC (1000 * 1000)
#endif

bool IRAM_ATTR TimerHandler(void* timerNo) {
    alarmFlag = true;
    return true;
}

/*
 * Save a higher resolution timestamp for each IMU interrupt
 */
typedef unsigned long long longTime_t;

longTime_t lastLongTime_us = 0;
longTime_t longTime_us = 0;

#define IMU_TIME_RING_SIZE 16
uint8_t imuTimeRingFront = 0;
uint8_t imuTimeRingBack = 0;
longTime_t imuTimes_us[IMU_TIME_RING_SIZE];

int imuRingCount() {
    int res;
    if (imuTimeRingFront >= imuTimeRingBack) {
        res = imuTimeRingFront - imuTimeRingBack;
    }
    else {
        res = IMU_TIME_RING_SIZE + imuTimeRingFront - imuTimeRingBack;
    }
    return res;
}

int pushImuTimestamp(longTime_t t_us) {
    int ret = 0;
    if (imuRingCount() == IMU_TIME_RING_SIZE-1) {
        return -1;
    }
    imuTimes_us[imuTimeRingFront++] = t_us;
    if (imuTimeRingFront == IMU_TIME_RING_SIZE) {
        imuTimeRingFront = 0;
    }
    return ret;
}

int popImuTimestamp(longTime_t *pt_us) {
    int ret = 0;
    if (imuTimeRingFront != imuTimeRingBack) {
        *pt_us = imuTimes_us[imuTimeRingBack++];
        if (imuTimeRingBack == IMU_TIME_RING_SIZE) {
            imuTimeRingBack = 0;
        }
        ret = 1;
    }
    return ret;
}

longTime_t getLongMicros() {
    uint32_t m_us = micros();
    if ((longTime_us & 0xffffffff) > m_us) {
        longTime_us = ((longTime_us & 0xffffffff00000000) + 0x0000000100000000) | m_us;
    }
    else {
        longTime_us = (longTime_us & 0xffffffff00000000) | m_us;
    }
    return longTime_us;
}

void IRAM_ATTR myinthandler1() { 
    noInterrupts();
    pushImuTimestamp( getLongMicros() );
    interrupts();
    ++imuUnservicedISRCount; 
}

void IRAM_ATTR myinthandler2() { newMMC5983MAData = true; }

void IRAM_ATTR alarmMatch() { alarmFlag = true; }

// number of IMU data read interrupts processed / interval
unsigned long imuIntCount = 0;
// track total number of times we missed processing after an IMU interrupt 
unsigned long imuISROverflow = 0;
// track number of loop() calls / interval
unsigned long loopCount = 0;
// track number of invalid samples received in FIFO
unsigned long imuInvalidSamples = 0;
// FIFO header does not match expected bit pattern
unsigned long imuFifoEmpty = 0;
// total number of packets processed from IMU FIFO
unsigned long imuFIFOProcessed = 0;
// total number of passes through loop()
unsigned long loopTotal = 0;

bool ledState = false;
bool greenLedState = false;

float pressure_hPa = 1000.0f;
float bmpTemperature_degC = 15.0f;
float bmpAltitude_m = 0.0f;

/**
 * Fusion constants and globals
 */
#define CLOCKS_PER_SEC 1

void mmcCalibrationCallback(int percent) {
    Serial.print(percent);
    Serial.println(" complete");
}

void setup() {

    delay(2000);

    Serial.begin(115200);
    while (!Serial) {
    }

    // Configure led
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);

#if !defined(ESP32)
    pinMode(ICM42688_intPin1, INPUT);
    pinMode(ICM42688_intPin2, INPUT);
    pinMode(MMC5983MA_intPin, INPUT);
    pinMode(BMP390_intPin, INPUT);
#endif

    digitalWrite(RED_LED, HIGH);

    Wire.begin();           // set master mode
    Wire.setClock(100000);  // I2C frequency at 100 kHz

    SPI.begin();
    imu.begin();

    myGNSS.begin();

    myGNSS.setUART1Output(0);
    myGNSS.setUART2Output(0);

    myGNSS.setI2COutput(COM_TYPE_NMEA);
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    // Idle reporting will be at 0.5 Hz
    myGNSS.setMeasurementRate(2000);
    myGNSS.setNavigationRate(1);

    if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g) == false) 
    {
        Serial.println(F("Warning: GNSS setDynamicModel() failed"));
    }

    //This will pipe all NMEA sentences to the serial port so we can see them
    //myGNSS.setNMEAOutputPort(Serial);

    myGNSS.setNavigationFrequency(1);

    Serial.println("ICM42688 ");
    byte ICM42688ID = imu.getChipID(); 

    Serial.println("MMC5983MA");
    byte MMC5983ID = mmc.getChipID();  // Read CHIP_ID register for MMC5983MA

    Serial.println("BMP390");
    bmp3_begin(&dev, 0x76, &Wire);
    uint8_t BMP390ID = dev.chip_id;

    bool allSensorsAcknowledged =
        (ICM42688ID == 0x47 || ICM42688ID == 0xDB) && MMC5983ID == 0x30 && BMP390ID == 0x60;

    if (allSensorsAcknowledged) {
        Serial.println("All sensor ICs are connecting");
        Serial.println(" ");

        digitalWrite(RED_LED, LOW);

        imu.reset();

        // set sensor resolutions for self test
        aRes = 4.0f / 32768.0f;
        gRes = 250.0f / 32768.0f;

        imu.selfTest(accelDiff, gyroDiff, STratio);
        Serial.println("Accel Self Test:");
        Serial.print("Ax diff: ");
        Serial.print(accelDiff[0] * aRes * 1000.0f);
        Serial.println(" mg");
        Serial.print("Ay diff: ");
        Serial.print(accelDiff[1] * aRes * 1000.0f);
        Serial.println(" mg");
        Serial.print("Az diff: ");
        Serial.print(accelDiff[2] * aRes * 1000.0f);
        Serial.println(" mg");
        Serial.println("Should be between 50 and 1200 mg");

        Serial.println("Gyro Self Test:");
        Serial.print("Gx diff: ");
        Serial.print(gyroDiff[0] * gRes);
        Serial.println(" dps");
        Serial.print("Gy diff: ");
        Serial.print(gyroDiff[1] * gRes);
        Serial.println(" dps");
        Serial.print("Gz diff: ");
        Serial.print(gyroDiff[2] * gRes);
        Serial.println(" dps");
        Serial.println("Should be > 60 dps");

        Serial.println("Comparing measurement with factory measurements");
        Serial.print("Ax ratio: ");
        Serial.print(STratio[1] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Ay ratio: ");
        Serial.print(STratio[2] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Az ratio: ");
        Serial.print(STratio[3] * 100.0f, 0);
        Serial.println(" %");
        Serial.println("Excursions outside of 50 and 150% indicate sensor damage");

        Serial.println("Gyro Self Test:");
        Serial.print("Gx ratio: ");
        Serial.print(STratio[4] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Gy ratio: ");
        Serial.print(STratio[5] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Gz ratio: ");
        Serial.print(STratio[6] * 100.0f, 0);
        Serial.println(" %");
        Serial.println("Excursions outside of 50 and 150% indicate sensor damage");

        // get sensor resolutions for user settings, only need to do this once
        aRes = imu.getAres(Ascale);
        gRes = imu.getGres(Gscale);

        // Configure IMU

        imu.init(Ascale, Gscale, AODR, GODR, aMode, gMode, CPU_SUPPLIES_IMU_CLKIN);

        Serial.println(
            "Sampling accel and gyro offset biases: keep device flat and "
            "motionless");
        delay(1000);

        imu.calibrateOffsetBias(accelBias, gyroBias);

        Serial.println("accel biases (mg)");
        Serial.println(1000.0f * accelBias[0]);
        Serial.println(1000.0f * accelBias[1]);
        Serial.println(1000.0f * accelBias[2]);
        Serial.println("gyro biases (dps)");
        Serial.println(gyroBias[0]);
        Serial.println(gyroBias[1]);
        Serial.println(gyroBias[2]);
        delay(1000);

        Serial.println("");
        Serial.println("Magnetometer Calibration");

        digitalWrite(GREEN_LED, HIGH);

        mmc.selfTest();

        mmc.calibrateBridgeOffset();

        float dest1[3];
        float dest2[3];

        mmc.calibrateSoftIronSettings( dest1, dest2, mmcCalibrationCallback);

        uint32_t softIronOffset[3];
        float softIronScale[3];

        mmc.getSoftIronCalibration(softIronOffset, softIronScale);

        //mmc.getOffset(magOffset);
        Serial.println("Soft Iron Offsets:");
        Serial.println(softIronOffset[0]);
        Serial.println(softIronOffset[1]);
        Serial.println(softIronOffset[2]);

        Serial.println("Soft Iron Scaling factors:");
        Serial.println(softIronScale[0],8);
        Serial.println(softIronScale[1],8);
        Serial.println(softIronScale[2],8);

        mmc.reset();

        mmc.performSetOperation();

        mmc.performResetOperation();
        
        attachInterrupt(MMC5983MA_intPin, myinthandler2, RISING);

        mmc.startSampleCollection(MODR, MBW, MSET);
        //mmc.offsetBias(magBias, magScale);
        /*
        Serial.println("mag biases (mG)");
        Serial.println(1000.0f * magBias[0]);
        Serial.println(1000.0f * magBias[1]);
        Serial.println(1000.0f * magBias[2]);
        Serial.println("mag scale ");
        Serial.println(magScale[0]);
        Serial.println(magScale[1]);
        Serial.println(magScale[2]);
        */

        Serial.println ("--");
        Serial.println ("Complete -- stopping");

        digitalWrite(RED_LED, HIGH);

        while (true) ;

    } else {
        if (MMC5983ID != 0x30) Serial.println(" MMC5983MA not functioning!");
        
        if (ICM42688ID != 0x47 && ICM42688ID != 0xDB) {
                Serial.print(" ICM42688 ID not correct (wanting 0x47 or 0xDB); got: ");
                Serial.println(ICM42688ID, 16);
        }
        
        if (BMP390ID != 0x60) Serial.println(" BMP390 not functioning!");
    }

    /* Set up the RTC alarm interrupt */

#if defined(ESP32)
    if (ITimer0.attachInterruptInterval(ESP32_TIMER_INTERVAL_USEC,
                                        TimerHandler)) {
        Serial.print(F("Starting RTC report interval clock; millis() = "));
        Serial.println(millis());
    } else {
        Serial.println(F("Can't set RTC."));
    }

    RTC.setTime(0, 0, 0, 1, 1, 2024);  // 1st Jan 2024 00:00:00

#endif

#if defined(SAMD51_THING_PLUS) || defined (TEMPO_V1)
    TC.startTimer(1000000/50, myISR); // 50 Hz
#endif

    mmc.clearInterrupt();  
    imu.readIntStatus();

    // ICM42688 INT1/INT interrupt 
    attachInterrupt(ICM42688_intPin1, myinthandler1, RISING); 

    // Receive IMU samples via FIFO
    if (ICM42688_RETURN_OK != imu.startFifoSampling( 3 )) {
        Serial.println("enableFifoMode() failed");
    }
    delay (10);

    digitalWrite(GREEN_LED, LOW);
}

void loop() {
}