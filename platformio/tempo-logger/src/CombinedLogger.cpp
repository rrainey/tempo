#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>

#include "CombinedLogger.h"
#include "tempo-arduino-pins.h"

extern char nmeaBuffer[100];
extern MicroNMEA nmea;

/*
 * This GNSS class is built more for Arduino coding style than as a purer C++ class.  Definition lies
 * in the CombinedLogger.h file.
 */
extern SFE_UBLOX_GNSS gnss;

/**
 * Extend the BinareyLogger to log dropkick-style jump data in a separate log file.
 */
CombinedLogger::CombinedLogger(SdFs& sd) : BinaryLogger(sd)
{

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

    bFirstPressureSample = true;

    lastTime_ms = millis();
    nNextHSample = 0;
    ulLastHSampleMillis = 0;
    ulLogfileOriginMillis = 0;

    morseBlinker.initialize(RED_LED, 250);
    
    mx = my = mz = 0.0f;
}

BinaryLogger::APIResult CombinedLogger::startLogging(LogfileSlotID slot) { 
    BinaryLogger::APIResult result = BinaryLogger::startLogging(slot);
    if (result == BinaryLogger::APIResult::Success) {
        
    }
    return result;
}

void CombinedLogger::stopLogging() {
    BinaryLogger::stopLogging();
}

void CombinedLogger::loop() {

    uint32_t curTime_ms = millis();

    uint32_t deltaTime_ms = curTime_ms - lastTime_ms;

    if (deltaTime_ms > 0) {
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

    // periodically flush the log file to SD card if active
    flushLog();

    // Call the base class loop() function
    // Which will -- in turn -- invoke sample reporting for both this class and
    // the binary logger
    BinaryLogger::loop();

    // Update application state machine
    if (OPS_MODE == OPS_GROUND_TEST) {
        updateTestStateMachine();
    }
    else {
        updateFlightStateMachine();
    }

}

void CombinedLogger::updateHDot(float H_feet) {

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


/// @brief Accept an IMU sample and update the AHRS algorithm
/// @param pSample incoming IMU sample from ICM42688

// TODO: the handleIMUSample function should also pass the high resolution timestamp
// that is asociated with the incoming sample. We gave the data down in the BinaryLogger,
// but we need to pass it up to the CombinedLogger.
void CombinedLogger::handleIMUSample(icm42688::fifo_packet3* pSample) {

    float aRes = 4.0f / 32768.0f;
    float gRes = 250.0f / 32768.0f;

    float ax, ay, az, gx, gy, gz;
    float imuTemp_C;

    ax = (float)pSample->ax * aRes - accelBias[0];
    ay = (float)pSample->ay * aRes - accelBias[1];
    az = (float)pSample->az * aRes - accelBias[2];

    // Convert the gyro value into degrees per second
    gx = (float)pSample->gx * gRes - gyroBias[0];
    gy = (float)pSample->gy * gRes - gyroBias[1];
    gz = (float)pSample->gz * gRes - gyroBias[2];

    imuTemp_C = ((float)pSample->temp / 2.07f) + 25.0f;

    (void)imuTemp_C;

    // const clock_t timestamp_sec = clock();
    //  deg/sec
    FusionVector gyroscope = {gx, gy, gz};
    // g's
    FusionVector accelerometer = {ax, ay, az};
    // TODO: we use Gauss here, but might need to switch to uT
    FusionVector magnetometer = {mx, my, -mz};

    // Apply calibration
    gyroscope =
        FusionCalibrationInertial(gyroscope, gyroscopeMisalignment,
                                  gyroscopeSensitivity, gyroscopeOffset);

    accelerometer = FusionCalibrationInertial(
        accelerometer, accelerometerMisalignment, accelerometerSensitivity,
        accelerometerOffset);

    if (USE_MAGNETIC_SAMPLING) {
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix,
                                                 hardIronOffset);
    } else {
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
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer,
                     fSampleInterval_sec);
}

void CombinedLogger::handleMagSample(uint32_t sample[3]) {
    mx = (((int32_t)sample[0] - (int32_t)softIronOffset[0])) * softIronScale[0] *
        100000.0f;
    my = (((int32_t)sample[1] - (int32_t)softIronOffset[1])) *
         softIronScale[1] * 100000.0f;
    mz = (((int32_t)sample[2] - (int32_t)softIronOffset[2])) *
         softIronScale[2] * 100000.0f;
}

void CombinedLogger::handleBaroSample(bmp3_data* pData) {
    dStaticPressure_hPa = pData->pressure / 100.0f;
    dBaroTemp_degC = pData->temperature;
    dBaroAltitude_m = 44330.0f * (1.0 - pow(dStaticPressure_hPa / SEALEVELPRESSURE_HPA, 0.1903));
}

void CombinedLogger::handleNMEASentence(const char* pSentence) {
}

void CombinedLogger::startLogFileFlushing() {
    if (!bTimer5Active) {
        timer5_ms = TIMER5_INTERVAL_MS;
        bTimer5Active = true;
    }
}

void CombinedLogger::stopLogFileFlushing() {
    if (bTimer5Active) {
        timer5_ms = 0;
        bTimer5Active = false;
    }

    if (txtLogFile) {
        txtLogFile.flush();
    }
}

void CombinedLogger::flushLog() {
    if (bTimer5Active && timer5_ms <= 0) {
        // Serial.println( "Log flushing" );

        timer5_ms = TIMER5_INTERVAL_MS;

        if (txtLogFile) {
            txtLogFile.flush();
        }
    }
}

void CombinedLogger::updateFlightStateMachine() {

  /**
   * State machine appropriate for flight
   */
  switch (nAppState) {

  case WAIT:
    if (nHDot_fpm > OPS_HDOT_THRESHOLD_FPM) {

      Serial.println("Switching to IN_FLIGHT");

      /*
       * Open both log files
       */
      LogfileSlotID slot;

      if (logManager.findNextLogfileSlot(&slot) != LogfileManager::APIResult::Success) {
        // TODO: add "morse code" error message
        Serial.println("Could not find a log file slot");
        return;
      } 

      // create and open the text log file
      if (logManager.openLogfile(slot, "TXT", txtLogFile) != LogfileManager::APIResult::Success) {

      }

      txtLogFile.println( NMEA_APP_STRING );

      // Activate altitude / battery sensor logging
      bTimer4Active = true;
      timer4_ms = TIMER4_INTERVAL_MS;

      // Activate periodic log file flushing
      startLogFileFlushing();

      // Activate "in flight" LED blinking
      setBlinkState ( BLINK_STATE_LOGGING );

      // Set "time 0" for log file.
      ulLogfileOriginMillis = millis();
      
      nAppState = JumpState::IN_FLIGHT;
    }
    break;

  case JumpState::IN_FLIGHT:
    {
      if (nHDot_fpm <= OPS_HDOT_JUMPING_FPM) {
        Serial.println("Switching to JUMPING");
        nAppState = JUMPING;

        // set nav update rate to 4Hz
        gnss.setMeasurementRate(250);
        gnss.setNavigationRate(1);

        gnss.disableNMEAMessage( UBX_NMEA_GSA, COM_PORT_I2C );
        gnss.disableNMEAMessage (UBX_NMEA_GSV, COM_PORT_I2C );
      }
    }
    break;

  case JumpState::JUMPING:
    {
      if (labs(nHDot_fpm) <= OPS_HDOT_LAND_THRESHOLD_FPM) {
        Serial.println("Switching to LANDED_1");
        nAppState = JumpState::LANDED1;
        timer1_ms = TIMER1_INTERVAL_MS;
        bTimer1Active = true;
      }
    }
    break;

  case JumpState::LANDED1:
    {
      if (nHDot_fpm <= OPS_HDOT_JUMPING_FPM) {
        Serial.println("Switching to JUMPING");
        nAppState = JUMPING;
        bTimer1Active = false;
      }
      else if (labs(nHDot_fpm) >= OPS_HDOT_THRESHOLD_FPM) {
        Serial.println("Switching to IN_FLIGHT");
        nAppState = JumpState::IN_FLIGHT;
        bTimer1Active = false;
      }
      else if (bTimer1Active && timer1_ms <= 0) {

        // Back to 0.5Hz update rate
        gnss.setMeasurementRate(2000);
        gnss.setNavigationRate(1);

        gnss.enableNMEAMessage( UBX_NMEA_GSA, COM_PORT_I2C );
        gnss.enableNMEAMessage (UBX_NMEA_GSV, COM_PORT_I2C );
        
        bTimer4Active = false;
        Serial.println("Switching to STATE_WAIT");
        setBlinkState ( BLINK_STATE_OFF );
        nAppState = WAIT;
        bTimer1Active = false;

        stopLogFileFlushing();
        txtLogFile.close();
      }
    }
    break;
  }
}

void CombinedLogger::sampleAndLogAltitude() {
    double dAlt_ft;
    float dPressure_hPa;

    if (true) {
        // barometer sampled elsewhere
        // see CombinedLogger::handleBaroSample()

        dPressure_hPa = dStaticPressure_hPa;

        if (OPS_MODE != OPS_STATIC_TEST) {
            dAlt_ft = dBaroAltitude_m * 3.28084;

        } else {

            dAlt_ft = 600.0;
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
                {MINtoMS(0), 600},     {MINtoMS(2), 600},
                {MINtoMS(12), 6500},   {MINtoMS(13), 6500},
                {MINtoMS(13.5), 3500}, {MINtoMS(16.5), 600},
                {MINtoMS(19), 600}};

            static int tableSize = sizeof(table) / sizeof(struct _vals);

            int t = millis();

            if (t >= table[tableSize - 1].time_ms || t <= table[0].time_ms) {
                dAlt_ft = 600.0;
            } else {
                int i;
                p = &table[0];
                for (i = 1; i < tableSize - 1; ++i) {
                    prev = p;
                    p = &table[i];

                    if (t < p->time_ms) {
                        if (p->time_ms - prev->time_ms == 0) {
                            Serial.println("divide by zero");
                        }
                        dAlt_ft =
                            prev->alt_ft + (t - prev->time_ms) *
                                               (p->alt_ft - prev->alt_ft) /
                                               (p->time_ms - prev->time_ms);
                        break;
                    }
                }
            }

            // g_atm.SetConditions( dAlt_ft, 0.0 );

            dPressure_hPa = 1000.0;  // TODO hPA pressure
        }

        /*
         * Update based on estimated altitude
         */

        updateHDot(dAlt_ft);

        /*
         * Output a record
         */
        if (nAppState != JumpState::WAIT) {
            txtLogFile.print("$PENV,");
            txtLogFile.print(millis() - ulLogfileOriginMillis);
            txtLogFile.print(",");
            txtLogFile.print(dPressure_hPa);
            txtLogFile.print(",");
            txtLogFile.print(dAlt_ft);
            txtLogFile.print(",");
            txtLogFile.println("-1");  // battery voltage not sampled

        } else {
            // When we're in WAIT mode, we can use the altitude
            // to set ground altitude.
            nHGround_feet = dAlt_ft;
        }
    }
}

void CombinedLogger::setBlinkState(enum BlinkState newState) {
    switch (newState) {
        case BlinkState::BLINK_STATE_OFF:
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            break;

        case BlinkState::BLINK_STATE_LOGGING:
            digitalWrite(GREEN_LED, HIGH);
            digitalWrite(RED_LED, LOW);
            break;

        // Unsupported on tempo V1 hardware
        case BlinkState::BLINK_STATE_BATTERY:
            digitalWrite(GREEN_LED, HIGH);
            morseBlinker.initialize(RED_LED, 250);
            morseBlinker.setOutputCharacter('I');
            break;

        // two blinks
        case BlinkState::BLINK_STATE_NO_SDCARD:
            digitalWrite(RED_LED, HIGH);
            morseBlinker.initialize(GREEN_LED, 250);
            morseBlinker.setOutputCharacter('E');
            break;

        // three blinks
        case BlinkState::BLINK_STATE_BAD_FILESYSTEM:
            digitalWrite(RED_LED, HIGH);
            morseBlinker.initialize(GREEN_LED, 250);
            morseBlinker.setOutputCharacter('O');
            break;

        // four blinks
        case BlinkState::BLINK_STATE_SDCARD_FILE_ERROR:
            digitalWrite(RED_LED, HIGH);
            morseBlinker.initialize(GREEN_LED, 250);
            morseBlinker.setOutputCharacter('H');
            break;

        case BlinkState::BLINK_STATE_INIT_FAILED:
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);
            break;
    }
}

void CombinedLogger::updateTestStateMachine() {
    /**
     * State machine appropriate for ground testing
     */
    switch (nAppState) {
        case JumpState::WAIT:
            if (nmea.isValid() &&
                nmea.getSpeed() >= TEST_SPEED_THRESHOLD_KTS * 1000) {
                Serial.println("Switching to STATE_IN_FLIGHT");

                /*
                 * Open both log files
                 */
                LogfileSlotID slot;

                if (logManager.findNextLogfileSlot(&slot) !=
                    LogfileManager::APIResult::Success) {
                    // TODO: add "morse code" error message
                    Serial.println("Could not find a log file slot");
                    return;
                }

                // create and open the text log file
                if (logManager.openLogfile(slot, "TXT", txtLogFile) !=
                    LogfileManager::APIResult::Success) {
                }

                txtLogFile.println(NMEA_APP_STRING);

                // Activate altitude / battery sensor logging
                bTimer4Active = true;
                timer4_ms = TIMER4_INTERVAL_MS;

                // Activate periodic log file flushing
                startLogFileFlushing();

                // set nav update rate to 4Hz
                gnss.setNavigationFrequency(4);

                // Activate "in flight" LED blinking
                setBlinkState(BLINK_STATE_LOGGING);

                nAppState = JumpState::IN_FLIGHT;
            }
            break;

        case JumpState::IN_FLIGHT:
        case JumpState::JUMPING: {
            if (nmea.isValid() &&
                nmea.getSpeed() < TEST_SPEED_THRESHOLD_KTS * 1000) {
                Serial.println("Switching to STATE_LANDED_1");
                nAppState = JumpState::LANDED1;
                timer1_ms = TIMER1_INTERVAL_MS;
                bTimer1Active = true;
            }
            break;

            case JumpState::LANDED1: {
                if (nmea.isValid() &&
                    nmea.getSpeed() >= TEST_SPEED_THRESHOLD_KTS * 1000) {
                    Serial.println("Switching to STATE_IN_FLIGHT");
                    nAppState = JumpState::IN_FLIGHT;
                    bTimer1Active = false;
                } else if (bTimer1Active && timer1_ms <= 0) {
                    // GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
                    // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
                    bTimer4Active = false;
                    Serial.println("Switching to STATE_WAIT");
                    setBlinkState(BLINK_STATE_OFF);
                    nAppState = JumpState::WAIT;
                    bTimer1Active = false;

                    // set nav update rate to 1Hz
                    gnss.setNavigationFrequency(1);

                    stopLogFileFlushing();
                    txtLogFile.close();
                }

            } break;

                /*
                case JumpState::LANDED2:
                  {

                    if (nmea.isValid() && nmea.getSpeed() >=
                TEST_SPEED_THRESHOLD_KTS*1000) { nAppState =
                JumpState::IN_FLIGHT; bTimer1Active = false;
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
                  */
        }
    }
}
