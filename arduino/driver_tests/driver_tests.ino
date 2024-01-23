/* 

  Test peripheral interfaces that's we'll use with the Tempo hardware

  Based on code by Kris Winer
  01/14/2022 Copyright Tlera Corporation
  Library may be used freely and without limit with proper attribution.

*/

#if !defined(ESP32)
#error This code is intended to run on ESP32 Thing Plus C boards
#endif

#include <Arduino.h>

#include "I2Cdev.h"
#include "ICM42688.h"
#include "MMC5983MA.h"
#include <ESP32Time.h>

#include "ESP32TimerInterrupt.h"
#include "bmp3.h"

#define SerialDebug false           // set to true to get extended debugging
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

#define bmp3_check_rslt(func, r)                     \
    if (r != BMP3_OK) {                              \
        Serial.println("Error returned from " func); \
    }

// global constants for 9 DoF fusion and AHRS
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError =
    pi * (40.0f /
          180.0f);  // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift =
    pi *
    (0.0f /
     180.0f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;  // compute beta
float zeta =
    sqrtf(3.0f / 4.0f) *
    GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick
                    // scheme usually set to a small or zero value
float pitch, yaw, roll;  // absolute orientation
float a12, a22, a31, a32,
    a33;  // rotation matrix coefficients for Euler angles and gravity
          // components
float deltat = 0.0f,
      sum = 0.0f;  // integration interval for both filter schemes
uint32_t lastUpdate = 0,
         firstUpdate = 0;  // used to calculate integration interval
uint32_t Now = 0;          // used to calculate integration interval
float lin_ax, lin_ay,
    lin_az;  // linear acceleration (acceleration with gravity component
             // subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f,
                 0.0f};  // vector to hold integral error for Mahony method

extern void IRAM_ATTR MadgwickQuaternionUpdate(float ax, float ay, float az,
                                               float gx, float gy, float gz,
                                               float mx, float my, float mz);

#define I2C_BUS Wire  // Define the I2C bus (Wire instance) you wish to use

I2Cdev i2c_0(&I2C_BUS);  // Instantiate the I2Cdev object and point to the
                         // desired I2C bus

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
uint8_t Ascale = AFS_4G, Gscale = GFS_250DPS, AODR = AODR_1kHz,
        GODR = GODR_1kHz, aMode = aMode_LN, gMode = gMode_LN;

// must be set to match accel/gyro samples
float fSampleInterval_sec = 1 / 2000.0f;

float aRes, gRes;  // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f, 0.0f},
      gyroBias[3] = {0.0f, 0.0f, 0.0f};  // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0},
        gyroDiff[3] = {0, 0, 0};  // difference betwee ST and normal values
float STratio[7] = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f};    // self-test results for the accel and gyro
int16_t ICM42688Data[7];  // Stores the 16-bit signed sensor output
float Gtemperature;  // Stores the real internal gyro temperature in degrees
                     // Celsius
float ax, ay, az, gx, gy,
    gz;  // variables to hold latest accel/gyro data values

volatile bool newICM42688Data = false;
volatile unsigned long imuISRCount = 0;

ICM42688 imu(&i2c_0);  // instantiate ICM42688 class

/* Specify magnetic sensor parameters (continuous mode sample rate is dependent
 * on bandwidth) choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz,
 * MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250,
 * MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th
 * measurement, etc.
 */
uint8_t MODR = MODR_100Hz, MBW = MBW_100Hz, MSET = MSET_2000;

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

MMC5983MA mmc(&i2c_0);  // instantiate MMC5983MA class

#define SEALEVELPRESSURE_HPA (1013.25)

volatile bool pressureSampleAvailable = false;

void IRAM_ATTR BMPSampleReadyHandler() { pressureSampleAvailable = true; }

/*
 * BMP390 globals
 */
int8_t rslt;
uint16_t settings_sel;
struct bmp3_dev dev;
struct bmp3_data data = {0};
struct bmp3_settings settings = {0};
struct bmp3_status status = {{0}};

// RTC parameter for STM32L4 native RTC class
uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false;  // for RTC alarm interrupt

// UTC timezone
ESP32Time RTC(0);
ESP32Timer ITimer0(0);

// five second status reports to Serial
#define ESP32_TIMER_INTERVAL_USEC (5000 * 1000)

bool IRAM_ATTR TimerHandler(void* timerNo) {
    alarmFlag = true;
    return true;
}

void IRAM_ATTR myinthandler1() {
    newICM42688Data = true;
    ++imuISRCount;
}

void IRAM_ATTR myinthandler2() { newMMC5983MAData = true; }

void IRAM_ATTR alarmMatch() { alarmFlag = true; }

// number of IMU data read interrupts processed / second
unsigned long imuIntCount = 0;
// counts total number of times we missed one or more IMU samples
unsigned long imuISROverflow = 0;
// count number of loop() calls / second
unsigned long loopCount = 0;
bool ledState = false;

float pressure_hPa = 1000.0f;
float bmpTemperature_degC = 15.0f;
float bmpAltitude_m = 0.0f;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
    }  // wait for serial monitor to begin

    // Configure led
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);

    digitalWrite(RED_LED, HIGH);

    Wire.begin();           // set master mode
    Wire.setClock(400000);  // I2C frequency at 400 kHz
    delay(1000);

    //i2c_0.I2Cscan();

    Serial.println("ICM42688 ");
    byte ICM42688ID = imu.getChipID(); 

    Serial.println("MMC5983MA");
    byte MMC5983ID = mmc.getChipID();  // Read CHIP_ID register for MMC5983MA

    Serial.println("BMP390");
    bmp3_begin(&dev, 0x76, &Wire);
    uint8_t BMP390ID = dev.chip_id;

    bool allSensorsAckknowledged =
        ICM42688ID == 0x47 && MMC5983ID == 0x30 && BMP390ID == 0x60;

    if (allSensorsAckknowledged) {
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

        Serial.print("Ax ratio: ");
        Serial.print(STratio[1] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Ay ratio: ");
        Serial.print(STratio[2] * 100.0f, 0);
        Serial.println(" %");
        Serial.print("Az ratio: ");
        Serial.print(STratio[3] * 100.0f, 0);
        Serial.println(" %");
        Serial.println("Should be between 50 and 150%");

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
        Serial.println("Should be between 50 and 150%");
        delay(2000);

        // get sensor resolutions for user settings, only need to do this once
        aRes = imu.getAres(Ascale);
        gRes = imu.getGres(Gscale);

        // Configure IMU

        imu.init(Ascale, Gscale, AODR, GODR, aMode, gMode, CPU_SUPPLIES_IMU_CLKIN);

        Serial.println(
            "Calculate accel and gyro offset biases: keep sensor flat and "
            "motionless!");
        delay(1000);

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

        mmc.selfTest();
        mmc.getOffset(magOffset);
        Serial.println("mag offsets:");
        Serial.println(magOffset[0]);
        Serial.println(magOffset[1]);
        Serial.println(magOffset[2]);

        mmc.reset();

        mmc.SET();  // "deGauss"
        attachInterrupt(MMC5983MA_intPin, myinthandler2, RISING);

        mmc.init(MODR, MBW, MSET);
        mmc.offsetBias(magBias, magScale);
        Serial.println("mag biases (mG)");
        Serial.println(1000.0f * magBias[0]);
        Serial.println(1000.0f * magBias[1]);
        Serial.println(1000.0f * magBias[2]);
        Serial.println("mag scale ");
        Serial.println(magScale[0]);
        Serial.println(magScale[1]);
        Serial.println(magScale[2]);


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

        settings.op_mode = BMP3_MODE_NORMAL;
        rslt = bmp3_set_op_mode(&settings, &dev);
        bmp3_check_rslt("bmp3_set_op_mode", rslt);

        attachInterrupt(BMP390_intPin, BMPSampleReadyHandler, RISING);

        digitalWrite(RED_LED, HIGH);
    } else {
        if (ICM42688ID != 0x6A) Serial.println(" ICM42688 not functioning!");
        if (MMC5983ID != 0x30) Serial.println(" MMC5983MA not functioning!");
        if (BMP390ID != 0x60) Serial.println(" BMP390 not functioning!");
    }

    /* Set up the RTC alarm interrupt */

    if (ITimer0.attachInterruptInterval(ESP32_TIMER_INTERVAL_USEC,
                                        TimerHandler)) {
        Serial.print(F("Starting RTC report interval clock; millis() = "));
        Serial.println(millis());
    } else {
        Serial.println(F("Can't set RTC. Select another frequancy or timer"));
    }

    RTC.setTime(0, 0, 0, 1, 1, 2024);  // 1st Jan 2024 00:00:00

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

    mmc.clearInt();  //  clear MMC5983MA interrupts before main loop
    imu.DRStatus();  // clear ICM42688 data ready interrupt

    // ICM42688 INT1/INT interrupt 
    attachInterrupt(32, myinthandler1, RISING); 

    Serial.print("CPU frequency = ");
    Serial.println(getCpuFrequencyMhz());
}

void loop() {
    ++loopCount;

    // MMC5983MA magnetometer has new data?
    if (newMMC5983MAData == true) {  
        newMMC5983MAData = false;    

        MMC5983MAstatus = mmc.status();
        if (MMC5983MAstatus & STATUS_MEAS_M_DONE) {
          mmc.clearInt();  //  clear mmc interrupts
          mmc.readData(MMC5983MAData);

          // Now we'll calculate the magnetometer value into actual Gauss
          mx = ((float)MMC5983MAData[0] - MMC5983MA_offset) * mRes -
                magBias[0];  // get actual mag value
          my = ((float)MMC5983MAData[1] - MMC5983MA_offset) * mRes -
                magBias[1];
          mz = ((float)MMC5983MAData[2] - MMC5983MA_offset) * mRes -
                magBias[2];
          mx *= magScale[0];
          my *= magScale[1];
          mz *= magScale[2];
        }
    }

    // ICM42688 has new sample?
    if (newICM42688Data == true) {
        newICM42688Data = false;
        ++imuIntCount;

        deltat = fSampleInterval_sec * imuISRCount;

        if (imuISRCount > 1) {
            ++imuISROverflow;
        }

        imuISRCount = 0;

        imu.readData(ICM42688Data);  // INT1 cleared on any read

        // Now we'll calculate the acceleration value into actual g's
        ax = (float)ICM42688Data[1] * aRes - accelBias[0];  
        ay = (float)ICM42688Data[2] * aRes - accelBias[1];
        az = (float)ICM42688Data[3] * aRes - accelBias[2];

        // Calculate the gyro value into actual degrees per second
        gx = (float)ICM42688Data[4] * gRes - gyroBias[0];
        gy = (float)ICM42688Data[5] * gRes - gyroBias[1];
        gz = (float)ICM42688Data[6] * gRes - gyroBias[2];

        // signs and axes if mag values corrected for IC placement on the
        // peakick V1 board
        MadgwickQuaternionUpdate(ay, ax, az, gy * pi / 180.0f, gx * pi / 180.0f,
                                 gz * pi / 180.0f, -mx, my, -mz);
    }

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
                      // alarm condition is achieved

        alarmFlag = false;

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

        if (SerialDebug) {
            Serial.print("ax = ");
            Serial.print((int)1000 * ax);
            Serial.print(" ay = ");
            Serial.print((int)1000 * ay);
            Serial.print(" az = ");
            Serial.print((int)1000 * az);
            Serial.println(" mg");
            Serial.print("gx = ");
            Serial.print(gx, 2);
            Serial.print(" gy = ");
            Serial.print(gy, 2);
            Serial.print(" gz = ");
            Serial.print(gz, 2);
            Serial.println(" deg/s");

            Serial.print("q0 = ");
            Serial.print(q[0]);
            Serial.print(" qx = ");
            Serial.print(q[1]);
            Serial.print(" qy = ");
            Serial.print(q[2]);
            Serial.print(" qz = ");
            Serial.println(q[3]);
            Serial.print("mx = ");
            Serial.print((int)1000 * mx);
            Serial.print(" my = ");
            Serial.print((int)1000 * my);
            Serial.print(" mz = ");
            Serial.print((int)1000 * mz);
            Serial.println(" mG");

            Serial.print("q0 = ");
            Serial.print(q[0]);
            Serial.print(" qx = ");
            Serial.print(q[1]);
            Serial.print(" qy = ");
            Serial.print(q[2]);
            Serial.print(" qz = ");
            Serial.println(q[3]);
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
            Serial.print(imuISRCount);
            Serial.println("");
            Serial.print("  loopCount = ");
            Serial.print(loopCount);
            Serial.println("");
        }
    }

    Gtemperature = ((float)ICM42688Data[0]) / 132.48f +
                   25.0f;  // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade
    if (SerialDebug) {
        Serial.print("Gyro temperature is ");
        Serial.print(Gtemperature, 1);
        Serial.println(" degrees C");  // Print T values to tenths of s degree C
    }

#ifdef notdef
    MMC5983MAtemperature = mmc.readTemperature();  // this is not working....
    Mtemperature = (((float)MMC5983MAtemperature) * 0.80f) -
                   75.0f;  // Mag chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade
    if (SerialDebug) {
        Serial.print("Mag temperature is ");
        Serial.print(Mtemperature, 1);
        Serial.println(" degrees C");  // Print T values to tenths of s degree C
    }
#endif

#ifdef notdef

    a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll = atan2f(a31, a33);
    yaw = atan2f(a12, a22);
    pitch *= 180.0f / pi;
    yaw *= 180.0f / pi;
    yaw += 0.0f;                 // no magnetic declination correction for now
    if (yaw < 0) yaw += 360.0f;  // Ensure yaw stays between 0 and 360
    roll *= 180.0f / pi;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;

    if (SerialDebug) {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);

        Serial.print("Grav_x, Grav_y, Grav_z: ");
        Serial.print(-a31 * 1000.0f, 2);
        Serial.print(", ");
        Serial.print(-a32 * 1000.0f, 2);
        Serial.print(", ");
        Serial.print(a33 * 1000.0f, 2);
        Serial.println(" mg");
        Serial.print("Lin_ax, Lin_ay, Lin_az: ");
        Serial.print(lin_ax * 1000.0f, 2);
        Serial.print(", ");
        Serial.print(lin_ay * 1000.0f, 2);
        Serial.print(", ");
        Serial.print(lin_az * 1000.0f, 2);
        Serial.println(" mg");

        // Serial.print("rate = "); Serial.print((float)sumCount/sum, 2);
        // Serial.println(" Hz");
    }

    // For plotting comma-delimited Euler angles in a spreadsheet
    // Serial.print(millis()/1000);Serial.print(",");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.print(roll, 2);
    Serial.print(", ");
    Serial.println(Pressure, 2);
#endif

    Serial.print("q0 = ");
    Serial.print(q[0]);
    Serial.print(" qx = ");
    Serial.print(q[1]);
    Serial.print(" qy = ");
    Serial.print(q[2]);
    Serial.print(" qz = ");
    Serial.print(q[3]);
    Serial.print(", p = ");
    Serial.println(pressure_hPa, 2);
    // Serial.print("rate = "); Serial.print((float)sumCount/sum, 2);
    // Serial.println(" Hz");
    Serial.print("imuISROverflow = ");
    Serial.print(imuISROverflow);
    Serial.println("");

    imuISRCount = 0;
    loopCount = 0;

    sum = 0;

    // digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);  // toggle
    // led

    ledState = !ledState;
    digitalWrite(RED_LED, (ledState ? HIGH : LOW));
}