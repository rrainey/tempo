/* 
 * Test Seb Madwick's Fusion library using Tempo sensors and drivers.
 *
 * This application is an amalgam of several open source Arduino projects.
 * It was used to test several different processor boards using the Peakick sensor array.
 * The current version of this code has only been tested with the Tempo board.  It still
 * includes passags of code that might make it usable with a Peakick configuration, but I haven't
 * tested those in quite some time.
 *
 * Some passages of code in this example are from other open source projects:
 *
 *      Overall structure based on code by Kris Winer
 *      "01/14/2022 Copyright Tlera Corporation"
 *      "Library may be used freely and without limit with proper attribution."
 *      see https://github.com/kriswiner/ICM42688
 *
 *      Arduino SAMD51 Timer module
 *      An interrupt timer based on the SAMD51 clock subsystem
 *      from https://github.com/Dennis-van-Gils/SAMD51_InterruptTime
 *      Dennis van Gils
 *      MIT License
 */

#include <Arduino.h>
#include <stdio.h>
#include <avr/dtostrf.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <ICM42688.h>
#include <MMC5983MA.h>
#include "bmp3.h"
#include <Fusion.h>
#include "TC_Timer.h"

/**
 * Configure compilation for the hardware runtime environment
 * 
 * One of four possibilities (Again, only the first configuration has been tested recently)
 * 
 * 1) tempo board (#define TEMPO_V1)
 * 2) peakick board along with SAMD51 Thing Plus C board (#define SAMD51_THING_PLUS)
 * 3) peakick board along with ESP32 Thing Plus board (set Arduino board to ESP32 Dev)
 * 4) peakick board along with STM32 Thing Plus board (set Arduino board to STM32 Thing Plus)
 */

#define USE_MAGNETIC_SAMPLING false

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
 * tempo board connections
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

void samdTimerInterrupt() {
    alarmFlag = true;
}

#endif

#define bmp3_check_rslt(func, r)                     \
    if (r != BMP3_OK) {                              \
        Serial.println("Error returned from " func); \
    }

// global constants for 9 DoF fusion and AHRS
float pi = 3.141592653589793238462643383279502884f;
// gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasError = pi * (40.0f / 180.0f);
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

//extern void IRAM_ATTR MadgwickQuaternionUpdate(float ax, float ay, float az,
//                                               float gx, float gy, float gz,
//                                               float mx, float my, float mz);


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
uint8_t
    MMC5983MAtemperature;  // Stores the magnetometer temperature register data
float Mtemperature;  // Stores the real internal chip temperature in degrees
                     // Celsius
float mx, my, mz;    // variables to hold latest mag data values (Gauss)
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

uint32_t softIronOffset[3] = {139525, 132218, 133466};
float softIronScale[3] = {0.00052652f, 0.00049533f, 0.00044927f};

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
 * Save a higher resolution timestamp corresponding to each IMU interrupt
 */
typedef unsigned long long longTime_t;

longTime_t lastLongTime_us = 0;
longTime_t longTime_us = 0;

#define IMU_TIME_RING_SIZE 16
uint8_t imuTimeRingFront = 0;
uint8_t imuTimeRingBack = 0;
longTime_t imuTimes_us[IMU_TIME_RING_SIZE];

/**
 * Calculates the number of elements in the IMU time ring buffer.
 * 
 * @return The number of elements in the IMU time ring buffer.
 */
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

/**
 * Pushes an IMU timestamp into the ring buffer.
 *
 * @param t_us The IMU timestamp to be pushed, in microseconds.
 * @return 0 if successful, -1 if the ring buffer is full.
 */
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

/**
 * @brief Pops the next IMU timestamp from the ring buffer.
 * 
 * This function retrieves the next IMU timestamp from the ring buffer and updates the pointer
 * to the timestamp. If there are no more timestamps in the buffer, the function returns 0.
 * 
 * @param pt_us Pointer to store the retrieved timestamp in microseconds.
 * @return int Returns 1 if a timestamp was retrieved successfully, 0 otherwise.
 */
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

/// @brief get current microsecond-resolution time
/// @return returns a long long version of the time so as not to wrap so frequently (wraps every 1^25 years)
longTime_t getLongMicros() {
    uint32_t m_us = micros();
    // catch a wrapped long microsecond counter
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

volatile uint32_t mmcIntCount = 0;

void IRAM_ATTR myinthandler2() { 
    newMMC5983MAData = true; 
    ++ mmcIntCount;
}

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

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

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
    else
    {
        Serial.println(F("GNSS Dynamic Platform Model set to AIRBORNE2g"));
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
        Serial.println("All peripherals are operating");
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

#ifdef notdef
        imu.offsetBias(accelBias, gyroBias);
        Serial.println("accel biases (mg)");
        Serial.println(1000.0f * accelBias[0]);
        Serial.println(1000.0f * accelBias[1]);
        Serial.println(1000.0f * accelBias[2]);
        Serial.println("gyro biases (dps)");
        Serial.println(gyroBias[0]);
        Serial.println(gyroBias[1]);
        Serial.println(gyroBias[2]);
        delay(1000);
#endif

        //Serial.println("");
        //Serial.println("Magnetometer Calibration");

        digitalWrite(GREEN_LED, HIGH);

        //mmc.selfTest();

#if 0
        mmc.getOffset(magOffset);
        Serial.println("mag offsets:");
        Serial.println(magOffset[0]);
        Serial.println(magOffset[1]);
        Serial.println(magOffset[2]);
#endif

        mmc.reset();

        mmc.performSetOperation();
        attachInterrupt(MMC5983MA_intPin, myinthandler2, RISING);

        mmc.setSoftIronCalibration(softIronOffset, softIronScale);


#if 0
        mmc.offsetBias(magBias, magScale);
        Serial.println("mag biases (mG)");
        Serial.println(1000.0f * magBias[0]);
        Serial.println(1000.0f * magBias[1]);
        Serial.println(1000.0f * magBias[2]);
        Serial.println("mag scale ");
        Serial.println(magScale[0]);
        Serial.println(magScale[1]);
        Serial.println(magScale[2]);
#endif

        // initialize BMP390 per datasheet, section 3.4.1 for "Drone"
        rslt = bmp3_init(&dev);
        bmp3_check_rslt("bmp3_init", rslt);

        settings.int_settings.drdy_en = BMP3_ENABLE;
        settings.press_en = BMP3_ENABLE;
        settings.temp_en = BMP3_ENABLE;

        settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
        settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
        settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
        settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
      
        settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                       BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                       BMP3_SEL_DRDY_EN | BMP3_SEL_IIR_FILTER;

        rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
        bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

        attachInterrupt(BMP390_intPin, BMPSampleReadyHandler, RISING);

        settings.op_mode = BMP3_MODE_NORMAL;
        rslt = bmp3_set_op_mode(&settings, &dev);
        bmp3_check_rslt("bmp3_set_op_mode", rslt);

        digitalWrite(RED_LED, HIGH);
    } else {
        if (ICM42688ID != 0x47 && ICM42688ID != 0xDB) Serial.println(" ICM42688 not functioning!");
        if (MMC5983ID != 0x30) Serial.println(" MMC5983MA not functioning!");
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
    // The SAMD timer is used to trigger updates on the USB serial port
    // There is currently no alternate mechicm implemented for STM32 or ESP32 
    // peakick configureations
    TC.startTimer(1000000/50, samdTimerInterrupt);
#endif

#if defined(STM32F405xx)
        rtc.begin(hourFormat);

        rtc.setHours(0, period);
        rtc.setMinutes(0);
        rtc.setSeconds(0);

        rtc.setDay(1);
        rtc.setMonth(1);
        rtc.setYear(2024);

        rtc.attachSecondsInterrupt(rtc_SecondsCB);
#endif

    // activate CLKIN line as clock for IMU
    // Generate a 32.768K pps square wave on the IMU CLKIN pin
#if (CPU_SUPPLIES_IMU_CLKIN == true)
        int result = ledcSetup(LEDC_CLKIN_CHANNEL, 
                               LEDC_CLKIN_BASE_FREQ,
                               LEDC_CLKIN_RESOLUTION);
        ledcAttachPin(CLKOUT, LEDC_CLKIN_CHANNEL);
        Serial.print(F("ledcSetup() returned "));
        Serial.println(result);
        ledcWrite(LEDC_CLKIN_CHANNEL, LEDC_DUTY_CYCLE);
#endif

    imu.readIntStatus();  // clear ICM42688 data ready interrupt

    // ICM42688 INT1/INT interrupt 
    attachInterrupt(ICM42688_intPin1, myinthandler1, RISING); 

    // Receive IMU samples via FIFO
    if (ICM42688_RETURN_OK != imu.startFifoSampling( 3 )) {
        Serial.println("enableFifoMode() failed");
    }
    delay (10);

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNed,
            .gain = 0.5f,
            .gyroscopeRange = 250.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 4.0f, // We're operating on a 4g sampling scale, so this should likely be lower that full scale value
            .magneticRejection = 10.0f,     // Gauss units
            .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    mmc.startSampleCollection(MODR, MBW, MSET);

    digitalWrite(GREEN_LED, LOW);
}

FusionVector magnetometerZeroes = {0, 0, 0};

void loop() {

    ++loopCount;
    ++loopTotal;

    // MMC5983MA magnetometer has has sample available?

    //if (newMMC5983MAData == true) {  
        //newMMC5983MAData = false;

        //if (mmcIntCount % 1000 == 0) {
        //    Serial.println("interrupts");
        //}

        MMC5983MAstatus = mmc.getStatus();
        if (MMC5983MAstatus & STATUS_MEAS_M_DONE) {

            uint32_t sample[3];
            if( mmc.readData(sample) != -1 ) {
                // samples as nanoTeslas
                mx = (((int32_t) sample[0] - (int32_t) softIronOffset[0])) * softIronScale[0] * 100000.0f;
                my = (((int32_t) sample[1] - (int32_t) softIronOffset[1])) * softIronScale[1] * 100000.0f;
                mz = (((int32_t) sample[2] - (int32_t) softIronOffset[2])) * softIronScale[2] * 100000.0f;
            }
            mmc.clearInterrupt();

        }
        //else {
        //    Serial.println("assertion: mag measurement not ready");
        //}
    //}

    // IMU Sample available?

    if (imuUnservicedISRCount > 0) {

        longTime_t itime_us;

        noInterrupts();
        int res = popImuTimestamp(&itime_us);
        interrupts();

        if ( res == 0 ) {
            Serial.println("assertion error: no timestamp in queue");
        }

        if (imuUnservicedISRCount > 1) {
            ++imuISROverflow;
        }
        imuUnservicedISRCount = 0;
        ++imuIntCount;

        // Read status bits to clear interrupt (see definition of INT_CONFIG0)
        uint8_t intStatus = imu.readIntStatus();
        (void) intStatus;

        uint8_t status;
        icm42688::fifo_packet3 sampleBuf[140], *pBuf;
        uint16_t packetCount;

        status = imu.readFiFo(sampleBuf, &packetCount);
        if (status != ICM42688_RETURN_OK) {
            Serial.println("readFifo encountered an error");
        }

        imuFIFOProcessed += packetCount;

        for (int i = 0; i < packetCount; ++i) {
            pBuf = &sampleBuf[i];

            // valid sample?

            if ((pBuf->header & ICM42688_FIFO_HEADER_MSG) == 0) {
              if ((pBuf->header & (ICM42688_FIFO_HEADER_ACCEL | ICM42688_FIFO_HEADER_GYRO)) == 
                (ICM42688_FIFO_HEADER_ACCEL | ICM42688_FIFO_HEADER_GYRO)) {
                if (!(PACKET3_SAMPLE_MARKED_INVALID(pBuf->gx) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->gy) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->gz) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->ax) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->ay) ||
                      PACKET3_SAMPLE_MARKED_INVALID(pBuf->az))) {

                    ++ imuFIFOProcessed;

                    ax = (float)pBuf->ax * aRes - accelBias[0];
                    ay = (float)pBuf->ay * aRes - accelBias[1];
                    az = (float)pBuf->az * aRes - accelBias[2];

                    // Convert the gyro value into degrees per second
                    gx = (float)pBuf->gx * gRes - gyroBias[0];
                    gy = (float)pBuf->gy * gRes - gyroBias[1];
                    gz = (float)pBuf->gz * gRes - gyroBias[2];

                    imuTemp_C = ((float)pBuf->temp / 2.07f) + 25.0f;

                    // const clock_t timestamp_sec = clock();
                    //  deg/sec
                    FusionVector gyroscope = {gx, gy, gz};
                    // g's
                    FusionVector accelerometer = {ax, ay, az};
                    // TODO: we use Gauss here, but might need to switch to uT
                    FusionVector magnetometer = {mx, my, -mz};

                    // Apply calibration
                    gyroscope = FusionCalibrationInertial(
                        gyroscope, gyroscopeMisalignment, gyroscopeSensitivity,
                        gyroscopeOffset);

                    accelerometer = FusionCalibrationInertial(
                        accelerometer, accelerometerMisalignment,
                        accelerometerSensitivity, accelerometerOffset);

                    if ( USE_MAGNETIC_SAMPLING ) {
                        magnetometer = FusionCalibrationMagnetic(
                            magnetometer, softIronMatrix, hardIronOffset);
                    }
                    else {
                        magnetometer = FusionCalibrationMagnetic(
                            magnetometerZeroes, softIronMatrix, hardIronOffset);
                    }

                    // Update gyroscope offset correction algorithm
                    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

                    // Calculate delta time (in seconds) to account for
                    // gyroscope sample clock error
                    // static clock_t previousTimestamp_sec;
                    // const float deltaTime =
                    //    (float)(timestamp_sec - previousTimestamp_sec) /
                    //    (float)CLOCKS_PER_SEC;
                    // previousTimestamp_sec = timestamp_sec;

                    // Update gyroscope AHRS algorithm
                    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer,
                                     magnetometer, fSampleInterval_sec);

                } else {
                    imuInvalidSamples++;
                }
                } else {
                  imuInvalidSamples++;
                }
            } else {
                imuFifoEmpty++;
            }
        }
    }

    // Pressure sample available?

    if ( pressureSampleAvailable ) {
        pressureSampleAvailable = false;

        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        /* Read temperature and pressure data iteratively based on data ready
         * interrupt */
        if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
            /*
             * First parameter indicates the type of data to be read
             * BMP3_PRESS_TEMP : To read pressure and temperature data
             * BMP3_TEMP       : To read only temperature data
             * BMP3_PRESS      : To read only pressure data
             */
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);

            /* NOTE : Read status register again to clear data ready interrupt
             * status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

            pressure_hPa = data.pressure / 100.0f;
            bmpTemperature_degC = data.temperature;
            // altitude = 44330.0f * (1.0 - pow(Pressure / SEALEVELPRESSURE_HPA,
            // 0.1903));
        }
    }

    if (alarmFlag) {  // update RTC output (serial display) whenever the RTC
                      // alarm condition is detected

        alarmFlag = false;

#if defined(ESP32)
        if (SerialDebug) {
            Serial.println("RTC:");
            Day = RTC.getDay();
            Month = RTC.getMonth();
            Year = RTC.getYear();
            Seconds = RTC.getSecond();
            Minutes = RTC.getMinute();
            Hours = RTC.getHour();
            if (Hours < 10) {
                Serial.print("0");
                Serial.print(Hours);
            } else
                Serial.print(Hours);
            Serial.print(":");
            if (Minutes < 10) {
                Serial.print("0");
                Serial.print(Minutes);
            } else
                Serial.print(Minutes);
            Serial.print(":");
            if (Seconds < 10) {
                Serial.print("0");
                Serial.println(Seconds);
            } else
                Serial.println(Seconds);

            Serial.print(Month);
            Serial.print("/");
            Serial.print(Day);
            Serial.print("/");
            Serial.println(Year);
            Serial.println(" ");
        }
#endif

        if (SerialDebug) {
            char buf[128];

            sprintf(buf, "a = {%5.1f, %5.1f, %5.1f} mg", 1000.0f * ax,
                    1000.0f * ay, 1000.0f * az);
            Serial.println(buf);

            sprintf(buf, "g = {%5.1f, %5.1f, %5.1f} deg/s", gx, gy, gz);
            Serial.println(buf);

            sprintf(buf, "M = {%5.1f, %5.1f, %5.1f} nT",  mx,
                    my, mz);
            Serial.println(buf);
        }

        if (SerialDebug) {
            // Serial.print("Altimeter temperature = "); Serial.print(
            // Temperature, 2); Serial.println(" C"); // temperature in degrees
            // Celsius
            Serial.print("Altimeter temperature = ");
            Serial.print(9.0f * bmpTemperature_degC / 5.0f + 32.0f, 2);
            Serial.println(" F");
            Serial.print("Altimeter pressure = ");
            Serial.print(pressure_hPa, 2);
            Serial.println(" mbar");
            Serial.print("Altitude = ");
            Serial.print(bmpAltitude_m, 2);
            Serial.println(" feet");
            Serial.print("imuISRCount = ");
            Serial.print(imuUnservicedISRCount);
            Serial.println("");
            Serial.print("  loopCount = ");
            Serial.print(loopCount);
            Serial.println("");
        }

        // Print temperature in degrees Centigrade
        if (SerialDebug) {
            Serial.print("Gyro temperature is ");
            Serial.print(imuTemp_C, 1);
            Serial.println(" degC");
        }

#ifdef notdef
        MMC5983MAtemperature =
            mmc.readTemperature();  // this is not working....
        Mtemperature = (((float)MMC5983MAtemperature) * 0.80f) -
                       75.0f;  // Mag chip temperature in degrees Centigrade
        // Print temperature in degrees Centigrade
        if (SerialDebug) {
            Serial.print("Mag temperature is ");
            Serial.print(Mtemperature, 1);
            Serial.println(
                " degrees C");  // Print T values to tenths of s degree C
        }
#endif

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

#if true

#ifdef SPRINTF_HAS_FLOAT_SUPPORT
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%6.2f   %6.2f   %6.2f     %7.1f",
            euler.angle.yaw, euler.angle.pitch, euler.angle.roll, pressure_hPa);
        Serial.println(pbuf);
#else
        char pbuf[128];
        char xp[16], yp[16], zp[16];
        // Euler angles in degrees
        sprintf(
            pbuf,
            "EA,%s,%s,%s,N",
            dtostrf(euler.angle.yaw,7,3,xp), dtostrf(euler.angle.pitch,7,3,yp), dtostrf(euler.angle.roll,7,3,zp));
        Serial.println(pbuf);
        // nT units
        sprintf(pbuf, 
            "MA,%s,%s,%s",  
            dtostrf(mx,8,2,xp), dtostrf(my,8,2,yp), dtostrf(-mz,8,2,zp));
        Serial.println(pbuf);
#endif

#else
#ifdef SPRINTF_HAS_FLOAT_SUPPORT
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%6.2f   %6.2f   %6.2f     %7.1f",
            euler.angle.yaw, euler.angle.pitch, euler.angle.roll, pressure_hPa);
        Serial.println(pbuf);
#else
        char pbuf[128];
        char xp[16], yp[16], zp[16], ap[16];
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%s   %s   %s     %s",
            dtostrf(euler.angle.yaw,6,2,xp), dtostrf(euler.angle.pitch,6,2,yp), dtostrf(euler.angle.roll,6,2,zp), 
            dtostrf(pressure_hPa,7,1,ap));
        //sprintf(pbuf, "M = {%s, %s, %s} G",  dtostrf(mx,6,2,xp), dtostrf(my,6,2,yp), dtostrf(mz,6,2,zp));
#endif
#endif

#ifdef notdef
        sprintf(pbuf,
                "loopCount  IntCount ISROverflow  AvgFIFO  invalidFIFO\n %8d   %7d    %8d  %7d   %6d\n---",
                loopCount, imuIntCount, imuISROverflow, fifoTotal/imuIntCount, imuInvalidSamples);
        Serial.println(pbuf);
#endif


        imuIntCount = 0;
        loopCount = 0;
        imuFIFOProcessed = loopTotal = 0;

        ledState = !ledState;
        digitalWrite(RED_LED, (ledState ? HIGH : LOW));
    }

    greenLedState = !greenLedState;
    digitalWrite(GREEN_LED, (greenLedState ? HIGH : LOW));
}