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

    nNextHGroundSample = 0;

    morseBlinker.initialize(RED_LED, 250);
    
    mx = my = mz = 0.0f;

    nAppState = WAIT;
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

        morseBlinker.loop();

        // periodically flush the log file to SD card if active
        flushLog();

        lastTime_ms = curTime_ms;
    }

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

    updateRateOfClimb();

    /*
     * Log IMU and altitude data
     */
    if (bTimer4Active && timer4_ms <= 0) {

        reportIMU(txtLogFile);

        logAltitude(dStaticPressure_hPa, dBaroAltitude_m * 3.28084);

        timer4_ms = TIMER4_INTERVAL_MS;
    }

    /*
     * Ground altitude estimation
     */
    if (bTimer6Active && timer6_ms <= 0) {
        int nSampleIndex =
            (nNextHSample == 0) ? NUM_H_SAMPLES - 1 : nNextHSample;

        recordGroundAltitude(nHSample[nSampleIndex]);

        timer6_ms = TIMER6_INTERVAL_MS;
    }
}

void CombinedLogger::updateHDot(float H_feet) {
    uint32_t ulMillis = millis();
    int nLastHSample_feet;
    int nInterval_ms = ulMillis - ulLastHSampleMillis;

    /* update HDot every ten seconds */
    if (nInterval_ms > 10000) {
        if (!bFirstPressureSample) {
            if (nNextHSample == 0) {
                nLastHSample_feet = nHSample[NUM_H_SAMPLES - 1];
            } else {
                nLastHSample_feet = nHSample[nNextHSample - 1];
            }
            nHSample[nNextHSample] = H_feet;
            nHDotSample[nNextHSample] =
                (((long)H_feet - nLastHSample_feet) * 60000L) / nInterval_ms;
            nHDot_fpm = nHDotSample[nNextHSample];
        } else {
            bFirstPressureSample = false;
            nHSample[nNextHSample] = H_feet;
            nHDotSample[nNextHSample] = 0;
            nHDot_fpm = 0;
            recordInitialGroundAltitude(H_feet);
        }

        ulLastHSampleMillis = ulMillis;
        if (++nNextHSample >= NUM_H_SAMPLES) {
            nNextHSample = 0;
        }
    }
}

/// @brief Accept an IMU sample and update the AHRS algorithm
/// @param pSample incoming IMU sample from ICM42688
void CombinedLogger::handleIMUSample(longTime_t itime_us, icm42688::fifo_packet3* pSample) {

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

    //  deg/sec, IMU sensor frame
    FusionVector gyroscope = {gx, gy, gz};
    // g's, IMU sensor frame
    FusionVector accelerometer = {ax, ay, az};
    // TODO: we use Gauss here, but might need to switch to uT
    // magnetometer sensor frame
    FusionVector magnetometer = {mx, my, mz};

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

    // We'll execute the Fusion AHRS algorithm in the body frame.
    // Transform sensor frames into body frame before passing to AHRS.
    // Refer to the diagram "images/tempo-v1-fames.png" for sensor/body axes definitions
    FusionVector gyroscopeBodyFrame = FusionAxesSwap(gyroscope, FusionAxesAlignmentPZNYPX);
    FusionVector accelerometerBodyFrame = FusionAxesSwap(accelerometer, FusionAxesAlignmentPZNYPX);

    // FusionAxesAlignmentNZNYPX is not defined as it isn't a proper right-hand-rule set of axes.
    // Still it's what the MMC5983 magnetometer gives us, so we must convert it manually here.
    //FusionVector magnetometerBodyFrame = FusionAxesSwap(magnetometer, FusionAxesAlignmentNZNYPX);
    FusionVector magnetometerBodyFrame;
    magnetometerBodyFrame.axis.x = - magnetometer.axis.z;
    magnetometerBodyFrame.axis.y = - magnetometer.axis.y;
    magnetometerBodyFrame.axis.z = magnetometer.axis.x;


    // Calculate delta time (in seconds) to account for
    // gyroscope sample clock error
    // static clock_t previousTimestamp_sec;
    // const float deltaTime =
    //    (float)(timestamp_sec - previousTimestamp_sec) /
    //    (float)CLOCKS_PER_SEC;
    // previousTimestamp_sec = timestamp_sec;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs, gyroscopeBodyFrame, accelerometerBodyFrame,
                     magnetometerBodyFrame, fSampleInterval_sec);
}

void CombinedLogger::handleMagSample(uint32_t sample[3]) {
    mx = (((int32_t)sample[0] - (int32_t)softIronOffset[0])) * 
        softIronScale[0] * 100000.0f;
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

void CombinedLogger::handleNMEASentence(uint32_t time_ms, const char* pSentence) {

    if (txtLogFile.isOpen()) {
      
      txtLogFile.print( pSentence );

      /*
       * Include time hack for important GNSS messages.  This is
       * designed to allow us to correlate GPS time and millis() time in the output stream.
       */
      if (strncmp( pSentence+3, "GGA", 3) == 0 || strncmp( pSentence+3, "GLL", 3) == 0) {
        char sentence[128];
        sprintf(sentence, "$PTH,%ld*", time_ms);
        logfilePrintSentence( txtLogFile, sentence );
      }
      
      flushLog();
    }
  
    if ( printNMEA ) {
      Serial.print( pSentence );
    }
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

    if (txtLogFile.isOpen()) {
        txtLogFile.flush();
    }
}

void CombinedLogger::flushLog() {
    if (bTimer5Active && timer5_ms <= 0) {
        // Serial.println( "Log flushing" );

        timer5_ms = TIMER5_INTERVAL_MS;

        if (txtLogFile.isOpen()) {
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
        bool errorFlag;
        enterInFlightState(errorFlag);
        if (errorFlag) return;
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
        bTimer1Active = false;
        nAppState = JumpState::IN_FLIGHT;
      }
      else if (bTimer1Active && timer1_ms <= 0) {
          enterWaitState();
      }
    }
    break;
  }
}

void CombinedLogger::enterWaitState() {
    // Back to 0.5Hz update rate
    gnss.setMeasurementRate(2000);
    gnss.setNavigationRate(1);

    gnss.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
    gnss.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);

    bTimer4Active = false;
    Serial.println("Switching to STATE_WAIT");
    setBlinkState(BLINK_STATE_IDLE);
    nAppState = WAIT;
    bTimer1Active = false;

    // Activate surface altitude sampling (and immediately take a sample)
    bTimer6Active = true;
    timer6_ms = 0;

    stopLogFileFlushing();
    txtLogFile.truncate();
    txtLogFile.close();

    BinaryLogger::stopLogging();
}

void CombinedLogger::enterInFlightState(bool& errorFlag) {
    errorFlag = true;
    Serial.println("Switching to IN_FLIGHT");

    /*
     * Open both log files
     */
    LogfileSlotID slot;

    if (logManager.findNextLogfileSlot(&slot) != LogfileManager::APIResult::Success) {
        setBlinkState(BlinkState::BLINK_STATE_SDCARD_FILE_ERROR);
        Serial.println("Could not find an unused log file slot");
        return;
    }

    // create and open the text log file
    if (logManager.openLogfile(slot, "TXT", txtLogFile) != LogfileManager::APIResult::Success) {
        setBlinkState(BlinkState::BLINK_STATE_SDCARD_FILE_ERROR);
        Serial.println("Could not open TXT log file; cannot enter IN_FLIGHT mode");
        return;
    }

    if (startLogging(slot) != BinaryLogger::APIResult::Success) {
        setBlinkState(BlinkState::BLINK_STATE_SDCARD_FILE_ERROR);
        Serial.println("Could not open TBS log file; cannot enter IN_FLIGHT mode");
        return;
    }

    /*
     * Both log files are now open. Write header lines to TXT log file:
     *
     * 1. Application version
     * 2. Estimated surface height (ft, MSL)
     */
    char sentence[128];
    strcpy(sentence, NMEA_APP_STRING);
    logfilePrintSentence( txtLogFile, sentence );

    // Log estimated surface altitude (standard day conditions)
    
    sprintf(sentence, "$PSFC,%d*", getGroundAltitude());
    logfilePrintSentence( txtLogFile, sentence );

    // Activate IMU / altitude / battery sensor logging (10Hz)
    bTimer4Active = true;
    timer4_ms = TIMER4_INTERVAL_MS;

    // Activate periodic log file flushing
    startLogFileFlushing();

    // Activate "in flight" LED blinking
    setBlinkState(BLINK_STATE_LOGGING);

    // Set "time 0" for log file.
    ulLogfileOriginMillis = millis();

    nAppState = JumpState::IN_FLIGHT;
    errorFlag = false;
}

void CombinedLogger::updateRateOfClimb() {

    double dAlt_ft;

    // barometer sampled elsewhere
    // see CombinedLogger::handleBaroSample()

    if (OPS_MODE != OPS_STATIC_TEST) {
        dAlt_ft = dBaroAltitude_m * 3.28084;

    } else {

        dAlt_ft = 600.0;
        /*
         * Simulate altitude based on this schedule:
         *
         * Time (min)     Alt(ft, MSL)
         *     0             600
         *     4             600
         *     7            6500
         *     8            6500
         *     8.5          3500
         *     10.5          600
         *     13            600
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
            {MINtoMS(0), 600},     {MINtoMS(4), 600},       // two minutes at 600 ft - this gives the GPS a chance to lock
            {MINtoMS(7), 6500},    {MINtoMS(8), 6500},
            {MINtoMS(8.5), 3500},    {MINtoMS(10.5), 600},
            {MINtoMS(13), 600}};

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

    }

    /*
        * Update based on estimated altitude
        */
    updateHDot(dAlt_ft);

}

void CombinedLogger::logAltitude(float dPressure_hPa, double dAlt_ft) {
    /*
     * Output a $PENV record (which includes a baro alititude estimate)
     */
    if (txtLogFile.isOpen()) {
        char s1[10], s2[10], s3[10];

        char sentence[128];
        sprintf(sentence, "$PENV,%ld,%s,%s,%s*",
                millis() - ulLogfileOriginMillis,
                dtostrf(dPressure_hPa, 5, 5, s1), dtostrf(dAlt_ft, 3, 3, s2),
                dtostrf(-1.0, 2, 2, s3));
        logfilePrintSentence(txtLogFile, sentence);
    }
}

void CombinedLogger::setBlinkState(enum BlinkState newState) {

    switch (newState) {
        case BlinkState::BLINK_STATE_OFF:
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(RED_LED, LOW);
            break;

        case BlinkState::BLINK_STATE_INITIALIZING:
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(RED_LED, HIGH);
            break;

        case BlinkState::BLINK_STATE_IDLE:
            digitalWrite(RED_LED, LOW);
            morseBlinker.initialize(GREEN_LED, 250);
            morseBlinker.setOutputCharacter('T');
            break;

        case BlinkState::BLINK_STATE_LOGGING:
            digitalWrite(GREEN_LED, HIGH);
            morseBlinker.initialize(RED_LED, 250);
            morseBlinker.setOutputCharacter('T');
            break;

        // Unsupported on tempo V1 hardware
        case BlinkState::BLINK_STATE_BATTERY:
            digitalWrite(GREEN_LED, HIGH);
            morseBlinker.initialize(RED_LED, 250);
            morseBlinker.setOutputCharacter('I');
            break;

        // one dash
        case BlinkState::BLINK_STATE_NO_SDCARD:
            digitalWrite(RED_LED, HIGH);
            morseBlinker.initialize(GREEN_LED, 250);
            morseBlinker.setOutputCharacter('T');
            break;

        // two dashes
        case BlinkState::BLINK_STATE_BAD_FILESYSTEM:
            digitalWrite(RED_LED, HIGH);
            morseBlinker.initialize(GREEN_LED, 250);
            morseBlinker.setOutputCharacter('M');
            break;

        // three dashes
        case BlinkState::BLINK_STATE_SDCARD_FILE_ERROR:
            digitalWrite(RED_LED, HIGH);
            morseBlinker.initialize(GREEN_LED, 250);
            morseBlinker.setOutputCharacter('O');
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
                bool retFlag;
                enterInFlightState(retFlag);
                if (retFlag) return;
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
                    enterWaitState();
                }

            } break;

        }
    }
}
