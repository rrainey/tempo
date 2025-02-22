/* 
 * This file is part of the Kick distribution (https://github.com/rrainey/tempo
 * Copyright (c) 2024 Riley Rainey
 * 
 * =========================================================================
 * 
 * COMPATIBILITY
 * 
 * This sketch is designed for use with the V1 peakick board and the Sparkfun ESP32 WROOM (USB-C)
 * 
 * =========================================================================
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.

 */

#include <Adafruit_BMP3XX.h>
#include <Adafruit_MAX1704X.h>
#include <ICM42688.h>
#include <SPI.h>
#include <SD.h>

/*
 * Last built with Arduino version 2.2.1
 * 
 * Change History affecting sensor output and log file contents
 * 
 * Version 155 (corresponding to first tempo configuration)
 *    Switch to ICM42688, BMP390, and SAM-M10Q peripherals. ICM42688 is sampled at 200Hz, reported at 40Hz.
 *    Use GPS time/date to set log file creation metadata for log files
 *    Include NMEA checksums for all proprietary sentences
 *    Use MAX 17048 lipo monitor IC for battery status monitoring
 * 
 * Version 55:
 *    Disable NMEA satellite status sentences when in higher GNSS position polling rates
 *    
 * Version 54:
 *    5 Hz GNSS message rate when in freefall till landing.
 * 
 * Version 53:
 *    u-blox Dynamic Platform Mode now set to Airborne 2g 
 *    (see pg. 21 of https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
 *    Logging GNSS fix rate raised from 2 to 4 fixes per second.
 *    millis() times appearing in a log file are now relative to the start of logging for the file
 *    Boot initialization now only waits for 20 seconds for a USB connection (was 30 seconds)
 */

/*
 * I2C peripheral addresses valid for the V1 peackick PCB
 */
#define GPS_I2C_ADDR      0x42  // SAM-M10Q
#define BMP390_I2C_ADDR   0x76
#define ICM42688_I2C_ADDR 0x68
#define MAX17048_ADDR     0x36
#define MMC5983MA_ADDR    0x30

/*
 * Peakick V1 peripheral IC interrupt line connections
 * Indicates the Sparkfun Thing Plus/ESP32 GPIO connection
 * for each IC's interrupt outputs.
 */
#define GPIO_ICM42688_INT1 6
#define GPIO_ICM42688_INT2 5
#define GPIO_MMC5983MA_INT 9
#define GPIO_BMP390_INT   10

/*
 * Sparkfun ESP32 WROOM GPIO definitions
 */
#define RED_LED       13
#define GREEN_SD_LED  12
#define SD_CHIP_SELECT 5 // see https://github.com/sparkfunX/SparkFun_ESP32_Thing_Plus_C/blob/main/Firmware/Test%20Sketches/SD_Test/SD_Test.ino

/*
 * I2C connection to the ICM-42688 IMU
 */
ICM42688 imu(Wire, ICM42688_I2C_ADDR);

volatile bool imuSampleReady = false;

void IRAM_ATTR imuInt1ISR() {
  imuSampleReady = true;
}

bool imuPresent = false;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS

SFE_UBLOX_GNSS myGNSS;

Adafruit_MAX17048 maxlipo;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

#define APP_STRING  "Tempo, version 0.155"
#define LOG_VERSION 2
#define NMEA_APP_STRING "$PVER,\"Tempo, version 0.155\",155"

/**
 * Fatal error codes (to be implemented)
 * 
 * 1 - no SD card
 * 2 - altitude sensor fail
 * 3 - IMU sensor fail
 */

/*
 * Operating mode
 */
#define OPS_FLIGHT        0  // normal mode; altimeter used to detect motion
#define OPS_STATIC_TEST   1  // for testing; time based simulation of vertical motion (preferred test mode)
#define OPS_GROUND_TEST   2  // for testing; uses GPS horizontal movement as an analogue to altitude changes

/*
 * Set operating mode for this build of the firmware here
 */
#define OPS_MODE OPS_STATIC_TEST

#if (OPS_MODE == OPS_STATIC_TEST) 
//#include "1976AtmosphericModel.h"

//sim1976AtmosphericModel g_atm;

#endif

#define TEST_SPEED_THRESHOLD_KTS     6.0
#define OPS_HDOT_THRESHOLD_FPM       300
#define OPS_HDOT_LAND_THRESHOLD_FPM  100
#define OPS_HDOT_JUMPING_FPM         -800

/*
 * Minutes to milliseconds
 */
#define MINtoMS(x) ((x) * 60 * 1000)

/*
 * Automatic jump logging
 * Generate a unique log file for each jump.
 * Log file contains GPS NMEA CSV records along with extra sensor data records in quasi-NMEA format
 * 
 * State 0: WAIT - gather baseline surface elevation information; compute HDOT_fps
 *                 GNSS update rate @ 1Hz
 * 
 * State 1: IN_FLIGHT (enter this state when HDOT_fpm indicates >= 300 fpm climb), 
 *                   GNSS update rate @ 1Hz, compute HDOT_fps, start logging if not already)
 *
 * State 2: JUMPING (enter this state whenever HDOT_FPM < -800 fpm)
 *                  GNSS update rate increases to 2Hz, compute HDOT_fps, start logging if not already)
 *                   
 *                   
 * State 3: LANDED1  like state 2 - if any conditions are vioated, return to state 2(JUMPING), 
 *                   go to state 0 when timer 1 reaches 60 seconds, set GNSS update rate set to 1Hz, close logfile
 *                   log data otherwise
 */

#define STATE_WAIT       0
#define STATE_IN_FLIGHT  1
#define STATE_JUMPING    2
#define STATE_LANDED_1   3
#define STATE_LANDED_2   4

int nAppState;

#define BLINK_STATE_OFF     0
#define BLINK_STATE_LOGGING 1
#define BLINK_STATE_BATTERY 2

int blinkState = BLINK_STATE_OFF;

bool bBatteryAlarm = false;
float measuredBattery_volts;

/*
 * Fully charged LiPoly battery is rated at 3.7V
 */
#define LOWBATT_THRESHOLD 3.50

/*
 * Estimated MSL altitude, based on standard day pressure @ sea level
 */
int nH_feet = 0;

/*
 * Estimated rate of climb (fpm)
 */
int nHDot_fpm = 0;
/*
 * Estimated ground elevation, ft
 * 
 * Computed while in WAIT state.
 */
int nHGround_feet = 0;

/*
 * Last Valid GPS date and time in DOS format
 * taken from last valid $GNRMC sentence
 */
uint16_t usGPSDate;
uint16_t usGPSTime;


#define NUM_H_SAMPLES 5
int nHSample[NUM_H_SAMPLES];
int nHDotSample[NUM_H_SAMPLES];
int nNextHSample = 0;
uint32_t ulLastHSampleMillis;

uint32_t ulLogfileOriginMillis;

boolean bFirstPressureSample = true;

/**
 * Currently unused.
 */
int computAvgHDot() {
  int i;
  int sum;
  for(i=0; i<NUM_H_SAMPLES; ++i) {
    sum += nHDotSample[i];
  }
  return sum / 5;
}

/**
 * Use pressure altitude samples to estimate rate of climb.
 * 
 * Rate of climb is re-estimated every 10 seconds.
 */
void updateHDot(float H_feet) {

  uint32_t ulMillis = millis();
  int nLastHSample_feet;
  int nInterval_ms =  ulMillis - ulLastHSampleMillis;

  /* update HDot every ten seconds */
  if (nInterval_ms > 10000) {
    if (!bFirstPressureSample) {
      if (nNextHSample == 0) {
        nLastHSample_feet = nHSample[NUM_H_SAMPLES-1];
      }
      else {
        nLastHSample_feet = nHSample[nNextHSample-1];
      }
      nHSample[nNextHSample] = H_feet;
      nHDotSample[nNextHSample] = (((long) H_feet - nLastHSample_feet) * 60000L) / nInterval_ms;
      nHDot_fpm = nHDotSample[nNextHSample];
    }
    else {
      bFirstPressureSample = false;
      nHSample[nNextHSample] = H_feet;
      nHDotSample[nNextHSample] = 0;
      nHDot_fpm = 0;
    }

    ulLastHSampleMillis = ulMillis;
    if (++nNextHSample >= NUM_H_SAMPLES) {
      nNextHSample = 0;
    }
  }
}

File logFile;

char logpath[32];

/*
 * Records last millis() time when timers were updated in
 * the main loop.
 */
uint32_t lastTime_ms = 0;

/*
 * Records last millis() time for start of NMEA sentence arrival. 
 * Useful to sync millis() time with GPS clock.
 */
uint32_t lastNMEATime_ms = 0;

/*
 * Set up timers
 * 
 * Timer 1: used in landing state machine (OFF initially)
 * 
 * Timer 2: periodic check of battery state
 * 
 * Timer 3: blink controller for RED LED  (OFF initially)
 * 
 * Timer 4: IMU sensor logging interval timer  (OFF initially)
 * 
 * Timer 5: Periodic SD-card log file flushing  (OFF initially)
 */

#define TIMER1_INTERVAL_MS 30000

bool bTimer1Active = false;
int32_t timer1_ms = 0;

#define TIMER2_INTERVAL_MS 30000

bool bTimer2Active = true;
int32_t timer2_ms = TIMER2_INTERVAL_MS;

#define TIMER3_ON_INTERVAL_MS     750
#define TIMER3_OFF_INTERVAL_1_MS  750 // off interval when signaling battery low
#define TIMER3_OFF_INTERVAL_2_MS  (3000 - TIMER3_ON_INTERVAL_MS) // off interval for flight mode

bool bTimer3Active = false;
int32_t timer3_ms = 0;

#define TIMER4_INTERVAL_MS  10   // 100 Hz -- twice the IMU sample interval

bool bTimer4Active = false;
int32_t timer4_ms = 0;

#define TIMER5_INTERVAL_MS 10000

bool bTimer5Active = false;
int32_t timer5_ms = 0;

int redLEDState = LOW;

// For debugging
bool printNMEA = false;

/**
 * Control RED (blinking) LED
 * This LED is locted to the left of the USB connector on
 * the Adalogger
 */
void setBlinkState( int newState ) {

  switch ( blinkState ) {

  case BLINK_STATE_OFF:
    // Was off, now on?
    if (newState != BLINK_STATE_OFF ) {
      bTimer3Active = true;
      timer3_ms = TIMER3_ON_INTERVAL_MS;
      redLEDState = HIGH;
    }
    break;
    
  case BLINK_STATE_LOGGING:
    if (newState == BLINK_STATE_BATTERY) {
      bTimer3Active = true;
      timer3_ms = TIMER3_ON_INTERVAL_MS;
      redLEDState = HIGH;
    }
    else if (newState == BLINK_STATE_OFF) {
        bTimer3Active = false;
        redLEDState = LOW;
     }
     break;
     
  case BLINK_STATE_BATTERY:
    if (newState == BLINK_STATE_OFF) {
      bTimer3Active = false;
      redLEDState = LOW;
    }
    else {
      // update state, but let blinking logic handle the transition
    }
    break;
  }

  // Update state and LED
  blinkState = newState;
  digitalWrite( RED_LED, redLEDState );
}

char * generateLogname(char *gname) 
{
    char * result = NULL;
    int i;
    for (i=0; true; i++) {
      sprintf (gname, "/log%05d.txt", i);
   
      if (!SD.exists(gname)) {
          result = gname;
          break;
      }
    }

    return result;
}

static inline uint16_t FAT_DATE(uint16_t year, uint8_t month, uint8_t day) {
  return (year - 1980) << 9 | month << 5 | day;
}

/** time field for FAT directory entry */
static inline uint16_t FAT_TIME(uint8_t hour, uint8_t minute, uint8_t second) {
  return hour << 11 | minute << 5 | second >> 1;
}

// callback for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
  *date = usGPSDate;
  *time = usGPSTime;
}

// This only callad for GNRMC sentences
// if the RMC status is marked as "Valid" update last GPS time
// This allows us to use GPS time to set a close-to-correct file creation time
// for each log file.
void updateGPSDateTime( char *pIncomingNMEA ) {
  // sentence has already been processed prior to this call
  if(true || nmea.isValid()) {
      usGPSDate = FAT_DATE(nmea.getYear(), nmea.getMonth(), nmea.getDay());
      usGPSTime = FAT_TIME(nmea.getHour(), nmea.getMinute(), nmea.getSecond());
  }
}

void updateTestStateMachine() {

  /**
   * State machine appropriate for ground testing
   */
  switch (nAppState) {

  case STATE_WAIT:
    if (nmea.isValid() && nmea.getSpeed() >= TEST_SPEED_THRESHOLD_KTS*1000) {

      Serial.println("Switching to STATE_IN_FLIGHT");
      
      // open log file
      generateLogname( logpath );
      logFile = SD.open( logpath, FILE_WRITE );

      logFile.println( NMEA_APP_STRING );

      // Activate altitude / battery sensor logging
      bTimer4Active = true;
      timer4_ms = TIMER4_INTERVAL_MS;

      // Activate periodic log file flushing
      startLogFileFlushing();

      // set nav update rate to 4Hz
      myGNSS.setNavigationFrequency(4);

      // Activate "in flight" LED blinking
      setBlinkState ( BLINK_STATE_LOGGING );
      
      nAppState = STATE_IN_FLIGHT;
    }
    break;

  case STATE_IN_FLIGHT:
    {

      if (nmea.isValid() && nmea.getSpeed() < TEST_SPEED_THRESHOLD_KTS*1000) {
        Serial.println("Switching to STATE_LANDED_1");
        nAppState = STATE_LANDED_1;
        timer1_ms = TIMER1_INTERVAL_MS;
        bTimer1Active = true;
      }
    }
    break;

  case STATE_LANDED_1:
    {

      if (nmea.isValid() && nmea.getSpeed() >= TEST_SPEED_THRESHOLD_KTS*1000) {
        Serial.println("Switching to STATE_IN_FLIGHT");
        nAppState = STATE_IN_FLIGHT;
        bTimer1Active = false;
      }
      else if (bTimer1Active && timer1_ms <= 0) {
        //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
        //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        bTimer4Active = false;
        Serial.println("Switching to STATE_WAIT");
        setBlinkState ( BLINK_STATE_OFF );
        nAppState = STATE_WAIT;
        bTimer1Active = false;

        // set nav update rate to 1Hz
        myGNSS.setNavigationFrequency(1);
        
        stopLogFileFlushing();
        logFile.close();
        
      }
      
    }
    break;

  case STATE_LANDED_2:
    {
      
      if (nmea.isValid() && nmea.getSpeed() >= TEST_SPEED_THRESHOLD_KTS*1000) {
        nAppState = STATE_IN_FLIGHT;
        bTimer1Active = false;
      }
      else if (bTimer1Active && timer1_ms <= 0) {
        //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
        //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        bTimer4Active = false;
        setBlinkState ( BLINK_STATE_OFF );
        nAppState = STATE_WAIT;
        Serial.println("Switching to STATE_WAIT");
        bTimer1Active = false;
        bTimer5Active = false;

        stopLogFileFlushing();
        logFile.close();
      }
    }
    break;
  }
}

void updateFlightStateMachine() {

  /**
   * State machine appropriate for flight
   */
  switch (nAppState) {

  case STATE_WAIT:
    if (nHDot_fpm > OPS_HDOT_THRESHOLD_FPM) {

      Serial.println("Switching to STATE_IN_FLIGHT");
      
      // open log file
      generateLogname( logpath );
      logFile = SD.open( logpath, FILE_WRITE );

      logFile.println( NMEA_APP_STRING );

      // Activate altitude / battery sensor logging
      bTimer4Active = true;
      timer4_ms = TIMER4_INTERVAL_MS;

      // Activate periodic log file flushing
      startLogFileFlushing();

      // Activate "in flight" LED blinking
      setBlinkState ( BLINK_STATE_LOGGING );

      // Set "time 0" for log file.
      ulLogfileOriginMillis = millis();
      
      nAppState = STATE_IN_FLIGHT;
    }
    break;

  case STATE_IN_FLIGHT:
    {
      if (nHDot_fpm <= OPS_HDOT_JUMPING_FPM) {
        Serial.println("Switching to STATE_JUMPING");
        nAppState = STATE_JUMPING;

        // set nav update rate to 4Hz
        myGNSS.setMeasurementRate(250);
        myGNSS.setNavigationRate(1);

        myGNSS.disableNMEAMessage( UBX_NMEA_GSA, COM_PORT_I2C );
        myGNSS.disableNMEAMessage (UBX_NMEA_GSV, COM_PORT_I2C );
      }
    }
    break;

  case STATE_JUMPING:
    {
      if (labs(nHDot_fpm) <= OPS_HDOT_LAND_THRESHOLD_FPM) {
        Serial.println("Switching to STATE_LANDED_1");
        nAppState = STATE_LANDED_1;
        timer1_ms = TIMER1_INTERVAL_MS;
        bTimer1Active = true;
      }
    }
    break;

  case STATE_LANDED_1:
    {
      if (nHDot_fpm <= OPS_HDOT_JUMPING_FPM) {
        Serial.println("Switching to STATE_JUMPING");
        nAppState = STATE_JUMPING;
        bTimer1Active = false;
      }
      else if (labs(nHDot_fpm) >= OPS_HDOT_THRESHOLD_FPM) {
        Serial.println("Switching to STATE_IN_FLIGHT");
        nAppState = STATE_IN_FLIGHT;
        bTimer1Active = false;
      }
      else if (bTimer1Active && timer1_ms <= 0) {

        // Back to 0.5Hz update rate
        myGNSS.setMeasurementRate(2000);
        myGNSS.setNavigationRate(1);

        myGNSS.enableNMEAMessage( UBX_NMEA_GSA, COM_PORT_I2C );
        myGNSS.enableNMEAMessage (UBX_NMEA_GSV, COM_PORT_I2C );
        
        bTimer4Active = false;
        Serial.println("Switching to STATE_WAIT");
        setBlinkState ( BLINK_STATE_OFF );
        nAppState = STATE_WAIT;
        bTimer1Active = false;

        stopLogFileFlushing();
        logFile.close();
      }
    }
    break;
  }
}

void sampleAndLogAltitude()
{

  double dAlt_ft;
  float dPressure_hPa;


  if (true) {

    bmp.performReading();

    dPressure_hPa = bmp.pressure / 100.0;

    if (OPS_MODE != OPS_STATIC_TEST) {

      dAlt_ft = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084;

    }
    else {
      
      /*
       * Simulate interpolated altitude based on this schedule:
       * 
       * Time (min)     Alt(ft)
       *     0             600
       *     2             600
       *     12           6500
       *     13           6500
       *     14           3500
       *     17            600
       *     19            600 
       *     
       *     Values clamped at finish to last value.
       */

    struct _vals {
      float time_ms;
      int alt_ft;
    };

    struct _vals *p, *prev;
    
    /*
     * time and altitude readings for a idealized hop-n-pop
     */
    static struct _vals table[7] = {
      { MINtoMS(0),   600 },
      { MINtoMS(2),   600 },
      { MINtoMS(12), 6500 },
      { MINtoMS(13), 6500 },
      { MINtoMS(13.5), 3500 },
      { MINtoMS(16.5),  600 },
      { MINtoMS(19),    600 }
    };

    static int tableSize = sizeof(table)/sizeof(struct _vals);

     int t = millis();

     if (t >= table[tableSize-1].time_ms || t <= table[0].time_ms ) {
      dAlt_ft = 600.0;
     }
     else {
        int i;
        p = &table[0];
        for (i=1; i<tableSize-1; ++i) {
          prev = p;
          p = &table[i];
  
          if (t < p->time_ms) {
            if (p->time_ms - prev->time_ms == 0) {
              Serial.println("divide by zero");
            }
            dAlt_ft =  prev->alt_ft + (t - prev->time_ms) * (p->alt_ft - prev->alt_ft) / (p->time_ms - prev->time_ms);
            break;
          }
        }
     }

      //g_atm.SetConditions( dAlt_ft, 0.0 );

      dPressure_hPa = 1000.0; //TODO hPA pressure
    }

    /*
     * Update based on estimated altitude
     */

    updateHDot(dAlt_ft);

    /*
     * Output a record
     */
    if (nAppState != STATE_WAIT) {
        
      logFile.print("$PENV,");
      logFile.print(millis() - ulLogfileOriginMillis);
      logFile.print(",");
      logFile.print(dPressure_hPa);
      logFile.print(",");
      logFile.print( dAlt_ft );
      logFile.print(",");
      logFile.println(measuredBattery_volts);
    
    }
    else {
      // When we're in WAIT mode, we can use the altitude
      // to set ground altitude.
      nHGround_feet = dAlt_ft;
    }
  }
}

/*
 * This method is called by the Sparkfun u-blox class to process an incoming character
 */

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.

char incomingNMEA[100];
char *pNMEA = incomingNMEA;
bool bStartOfNMEA = true;

void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  /*
   * New sentence arriving? record the time
   */
  if (bStartOfNMEA) {
    lastNMEATime_ms = millis() - ulLogfileOriginMillis;
    bStartOfNMEA = false;
  }
  *pNMEA++ = incoming;

  nmea.process(incoming);

  if (incoming == '\n') {

    *pNMEA++ = '\0';
    
    if (logFile) {
      
      logFile.print( incomingNMEA );

      /*
       * Include time hack for important GNSS messages.  This is
       * designed to allow us to correlate GPS time and millis() time in the output stream.
       */
      if (strncmp( incomingNMEA+3, "GGA", 3) == 0 || strncmp( incomingNMEA+3, "GLL", 3) == 0) {
        logFile.print( "$PTH," );
        logFile.println( lastNMEATime_ms );
      }

      if (strncmp( incomingNMEA, "$GNRMC", 6) == 0) {
        updateGPSDateTime( incomingNMEA );
      }
      
      flushLog();
    }
  
    if ( printNMEA ) {
      Serial.print( incomingNMEA );
    }

    pNMEA = incomingNMEA;
    bStartOfNMEA = true;
  }
}


void setup() {

  blinkState = BLINK_STATE_OFF;


  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

  lastTime_ms = millis();

  // Wait (maximum of 30 seconds) for hardware serial to appear
  while (!Serial) {
    if (millis() - lastTime_ms > 20000) {
      break;
    }
  }
  
  Serial.begin(115200);

  Serial.println( "" );

  Serial.println( APP_STRING );

  if (!SD.begin( SD_CHIP_SELECT )) {

    Serial.println("SD card initialization failed!");

    while (1);

  }

  delay(200);

  Wire.setClock( 400000 );
  Wire.begin();

  Serial.println("Initialize peripheral ICs");

  if (!maxlipo.begin()) {
    Serial.println(F("Couldnt initialize MAX17048 IC; this is part of the Sparkfun ESP32 WROOM board"));
    while (1) delay(100);
  }

  pNMEA = incomingNMEA;

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1) delay(100);
  }

  // Disable unused output channels
  
  myGNSS.setUART1Output(0);
  myGNSS.setUART2Output(0);

  //myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.setI2COutput(COM_TYPE_NMEA);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  // Idle reporting will be at 0.5 Hz
  myGNSS.setMeasurementRate(2000);
  myGNSS.setNavigationRate(1);

  if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g) == false) 
  {
    Serial.println(F("Warning: setDynamicModel failed"));
  }
  else
  {
    Serial.println(F("GNSS Dynamic Platform Model set to AIRBORNE2g"));
  }

  //This will pipe all NMEA sentences to the serial port so we can see them
  //myGNSS.setNMEAOutputPort(Serial);

  myGNSS.setNavigationFrequency(1);

  Serial.println("u-blox GNSS present");

  if (imu.begin()) {

    Serial.println("ICM42688 IMU present");

    imuPresent = true;

    imu.setAccelFS(ICM42688::gpm4);
    imu.setGyroFS(ICM42688::dps250);
    imu.setAccelODR(ICM42688::odr50);
    imu.setGyroODR(ICM42688::odr50);

    delay(10);

    attachInterrupt(GPIO_ICM42688_INT1, imuInt1ISR, RISING);
    imu.enableDataReadyInterrupt(); 

  }
  else {
    Serial.println(F("Failed to find ICM-42688-P chip"));
    imuPresent = false;
  }

  delay(100);

  if (! bmp.begin_I2C( BMP390_I2C_ADDR )) {
    Serial.println(F("Failed to find BMP390 chip; stopping"));
    while (1) yield();
  }
  Serial.println("BMP390 present");

  delay(100);

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (OPS_MODE == OPS_STATIC_TEST) {
    Serial.println(F("Welcome. The device has booted in OPS_STATIC_TEST mode."));
    Serial.println("");
    Serial.println(F("This test will take about 20 minutes to complete. You will see"));
    Serial.println("state change messages for STATE_WAIT, STATE_IN_FLIGHT, and STATE_LANDED_1. ");
    Serial.println("The test is complete when the device returns to STATE_WAIT.");
    Serial.println("You may then inspect the information in the last log file generated.");
    Serial.println("---");
  }

  Serial.println(F("Switching to STATE_WAIT"));
  nAppState = STATE_WAIT;

  /*
   * Set to 'true' do do some quick debugging of landing flow.
   */
  if (false) {
    Serial.println("Executing dummy flight run");
    Serial.println("Switching to STATE_IN_FLIGHT");
        
    // open log file
    generateLogname( logpath );
    logFile = SD.open( logpath, FILE_WRITE );

    logFile.println( NMEA_APP_STRING );

    logFile.flush();

    // Activate altitude / battery sensor logging
    bTimer4Active = true;
    timer4_ms = TIMER4_INTERVAL_MS;

    // Activate periodic log file flushing
    startLogFileFlushing();

    // Activate "in flight" LED blinking
    setBlinkState ( BLINK_STATE_LOGGING );

    // force battery test timer to fire
    timer2_ms = 0;

    ulLogfileOriginMillis = millis();
    
    nAppState = STATE_IN_FLIGHT;
  }
  
  lastTime_ms = millis();

  /*
   * Start USB disk operations
   */
  //uint32_t block_count = SD.getVolume().blocksPerCluster()*SD.getVolume().clusterCount();

  //Serial.print("Volume size (MB):  ");
  //Serial.println((block_count/2) / 1024);
  //usb_msc.setCapacity( block_count, 512 );
  //usb_msc.setUnitReady( true );


  /*
   * setup complete
   */
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(RED_LED, LOW);
  delay(500);
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(RED_LED, LOW);
}

void loop() {

  uint32_t curTime_ms = millis();

  uint32_t deltaTime_ms = curTime_ms - lastTime_ms;

  if ( deltaTime_ms > 0 ) {

    /*
     * Update active timer countdowns
     */
    if (bTimer1Active) {
      timer1_ms -= deltaTime_ms;
    }
  
    if (bTimer2Active) {
      timer2_ms -= deltaTime_ms;
    }
  
    if (bTimer3Active) {
      timer3_ms -= deltaTime_ms;
    }
  
    if (bTimer4Active) {
      timer4_ms -= deltaTime_ms;
    }

    if (bTimer5Active) {
      timer5_ms -= deltaTime_ms;
    }
    
    lastTime_ms = curTime_ms;

  }

  myGNSS.checkUblox();

  /*
   * Processing tasks below are outside of the
   * GPS NMEA processing loop.
   */

  if (OPS_MODE == OPS_GROUND_TEST) {
    updateTestStateMachine();
  }
  else {
    updateFlightStateMachine();
  }

  sampleAndLogAltitude();

  /*
   * RED LED Blink Logic
   * 
   * The RED LED will blink using different patterns to indicate
   * one of three states: Constant off, ON/OFF at 1.5Hz to indicates a low battery.
   * A 3-second blink is used to indicate flight mode.
   */
  if (bTimer3Active && timer3_ms <= 0) {
    
    redLEDState = (redLEDState == HIGH) ? LOW : HIGH;
    digitalWrite(RED_LED, redLEDState);

    if ( redLEDState == HIGH ) {
      timer3_ms = TIMER3_ON_INTERVAL_MS;
    }
    else {
      timer3_ms = bBatteryAlarm ? TIMER3_OFF_INTERVAL_1_MS : TIMER3_OFF_INTERVAL_2_MS;
    }
    
  }

  /*
   * Every 30 seconds, measure the battery state.
   * Blink red LED if low.
   */
  if (bTimer2Active && timer2_ms <= 0) {
    
    timer2_ms = TIMER2_INTERVAL_MS;
    
    measuredBattery_volts = maxlipo.cellVoltage();

    if ( measuredBattery_volts <= LOWBATT_THRESHOLD ) {
      bBatteryAlarm = true;
      setBlinkState ( BLINK_STATE_BATTERY );
    }
    else {
      if ( bBatteryAlarm ) {
        setBlinkState( (nAppState != STATE_WAIT) ? BLINK_STATE_LOGGING : BLINK_STATE_OFF );
      }
      bBatteryAlarm = false;
    }
  }

  /*
   * Log IMU samples
   */
  if (bTimer4Active && timer4_ms <= 0) {

    IMU();

    timer4_ms = TIMER4_INTERVAL_MS;
  }

  /*
   * Yield - primarily as insurance the SD card driver gets control when
   *         required.
   */

  yield();
  
}

void startLogFileFlushing()
{
  if (! bTimer5Active) {
    timer5_ms = TIMER5_INTERVAL_MS;
    bTimer5Active = true;
  }
}

void stopLogFileFlushing()
{
  if (bTimer5Active) {
    timer5_ms = 0;
    bTimer5Active = false;
  }

  if ( logFile ) {
    logFile.flush();
  }
}

void flushLog() {
    
  if (bTimer5Active && timer5_ms <= 0) {
    
    //Serial.println( "Log flushing" );

    timer5_ms = TIMER5_INTERVAL_MS;

    if ( logFile ) {
      logFile.flush();
    }
    
  }
}

float ax[100], ay[100], az[100];
float gx[100], gy[100], gz[100];

#define DEGtoRAD(x) ((x) * (M_PI / 180.0))

float averageValues(float values[], int count) {
  float sum = 0.0f;
  int i;
  if (count <= 0) {
    return sum;
  }
  if (count == 1) {
    return values[0];
  }
  for (i=0; i<count; ++i) {
    sum += values[i];
  }
  return sum / count;
}

void IMU() {

  if (imuPresent) {

    if (imuSampleReady) {

      imuSampleReady = false;
      imu.getAGT();


#ifdef notdef
      Serial.print(DEGtoRAD(gx[0])); // rad per sec
      Serial.print(",");
      Serial.print(DEGtoRAD(gy[0]));
      Serial.print(",");
      Serial.print(DEGtoRAD(gz[0]));
      Serial.println();
#endif
  
      if (logFile) {
        logFile.print("$PIMU,");
        logFile.print(millis() - ulLogfileOriginMillis);
        
        logFile.print(",");
        logFile.print(imu.accX());  // m/s^2
        logFile.print(",");
        logFile.print(imu.accY());
        logFile.print(",");
        logFile.print(imu.accZ());
        
        logFile.print(",");
        logFile.print(DEGtoRAD(imu.gyrX())); // rad per sec
        logFile.print(",");
        logFile.print(DEGtoRAD(imu.gyrY()));
        logFile.print(",");
        logFile.print(DEGtoRAD(imu.gyrZ()));
        logFile.println();
      }
    }
  }
}