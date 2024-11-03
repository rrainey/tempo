#ifndef COMBINEDLOGGER_H_
#define COMBINEDLOGGER_H_

#include <Arduino.h>
#include <Fusion.h>
#include <SdFat.h>
#include <icm42688.h>
#include <sdios.h>

#include "BinaryLogger.h"
#include "MorseBlinker.h"

#define USE_MAGNETIC_SAMPLING false

/*
 * Application build configuration
 *
 * The app may be build in any of three forms. OPS_FLIGHT is the normal mode of operation.
 * The other two build configurations are designed for testing/debugging purposes.
 */
#define OPS_FLIGHT       0  // normal mode; altimeter used to detect motion
#define OPS_STATIC_TEST  1  // for testing; time based simulation of vertical motion (preferred test
                            // mode)
#define OPS_GROUND_TEST  2  // for testing; uses GPS horizontal movement as an analogue to altitude
                            // changes

/*
 * Select operating mode for this build of the firmware here
 */
#define OPS_MODE OPS_STATIC_TEST

/*
 * The V1 Tempo board includes a red and green LED that together provide status information.
 * A solid red LED indicates an error condition. A solid green LED indicates normal operation.
 * This enum defines the various valid states of these LEDs.
 */
enum BlinkState {
  BLINK_STATE_OFF = 0,
  BLINK_STATE_INITIALIZING,
  BLINK_STATE_IDLE,
  BLINK_STATE_BATTERY,  // unused on Tempo V1 hardware
  BLINK_STATE_LOGGING,
  BLINK_STATE_NO_SDCARD,
  BLINK_STATE_SDCARD_FILE_ERROR,
  BLINK_STATE_BAD_FILESYSTEM,
  BLINK_STATE_INIT_FAILED
};

/*
 * The version numbers referenced here are a continuation of the versioning scheme originally
 * used in Dropkick log files. These version numbers appear at the start of each TXT log file.
 */
#define APP_STRING "Tempo, version 0.155"
#define LOG_VERSION 2
#define NMEA_APP_STRING "$PVER,\"Tempo, version 0.155\",155"

#define TEST_SPEED_THRESHOLD_KTS 6.0
#define OPS_HDOT_THRESHOLD_FPM 300
#define OPS_HDOT_LAND_THRESHOLD_FPM 100
#define OPS_HDOT_JUMPING_FPM -800

/*
 * Minutes to milliseconds
 */
#define MINtoMS(x) ((x) * 60 * 1000)

/*
 * Air pressure at sea leavel, Standard Day Conditions
 */
#define SEALEVELPRESSURE_HPA 1013.25

/*
 * Automatic jump logging
 * Generate a unique log file for each jump.
 * Log file contains GPS NMEA CSV records along with extra sensor data records
 * in quasi-NMEA format
 *
 * State 0: WAIT - gather baseline surface elevation information; compute
 * HDOT_fps GNSS update rate @ 1Hz
 *
 * State 1: IN_FLIGHT (enter this state when HDOT_fpm indicates >= 300 fpm
 * climb), GNSS update rate @ 1Hz, compute HDOT_fps, start logging if not
 * already)
 *
 * State 2: JUMPING (enter this state whenever HDOT_FPM < -800 fpm)
 *                  GNSS update rate increases to 2Hz, compute HDOT_fps, start
 * logging if not already)
 *
 *
 * State 3: LANDED1  like state 2 - if any conditions are vioated, return to
 * state 2(JUMPING), go to state 0 when timer 1 reaches 60 seconds, set GNSS
 * update rate set to 1Hz, close logfile
 */
typedef enum { WAIT, IN_FLIGHT, JUMPING, LANDED1 } JumpState;

// used to configure the Fusion module
#define SAMPLE_RATE (200)   // IMU samples per second
                            // must match accel/gyro sample rate
#define fSampleInterval_sec (1.0f / SAMPLE_RATE)

class CombinedLogger : public BinaryLogger {

   public:
    /**
     * @brief Constructs a CombinedLogger object.
     *
     * This constructor initializes the BinaryLogger object with the provided
     * SdFs object. It also initializes various member variables to their
     * default values.
     *
     * @param Sd A reference to the SD Card used for logging.
     */
    CombinedLogger(SdFs &sd);

    virtual BinaryLogger::APIResult startLogging(LogfileSlotID slot);

    // Stop logging and close the log file on the SD Card
    virtual void stopLogging();

    /**
     * Control RED and GREEN status LEDs
     * 
     * These LEDs are used to indicate both the general operating status and specific states.
     * An unblinking GREEN means the device is operational. When GREEN is illuminated, the RED LED
     * blinks a status code.
     * An unblinking RED means a fatal error was encounterd. When GREEN is illuminated, the GREEN LED
     * blinks a status code.
     */
    void setBlinkState(enum BlinkState newState);

    MorseBlinker morseBlinker;

    MorseBlinker &getBlinker() { return morseBlinker; }

    // call this exactly once from the main Arduino application loop() function
    virtual void loop();

   protected:
    /**
     * @brief Receive IMU sample, maintain pose state, and log data
     *
     * @param pSample pointer to the IMU sample (from IMU FIFO)
     */
    virtual void handleIMUSample(longTime_t itime_us, icm42688::fifo_packet3 *pSample);

    /**
     * @brief Receive magnetometer sample, maintain pose state, and log data
     *
     * @param sample array of raw XYZ magnetometer samples
     */
    virtual void handleMagSample(uint32_t sample[3]);

    /**
     * @brief Receive Barometer sample, maintain pose state, and log data
     *
     * @param pData barometer sample
     */
    virtual void handleBaroSample(bmp3_data *pData);

    /**
     * @brief Receive Next GNSS NMEA sentence, log it.
     *
     * @param pSentence pointer to buffer containing the next NMEA sentence
     */
    virtual void handleNMEASentence(uint32_t time_ms, const char *pSentence);

    void logfilePrintSentence(FsFile &f, char *s) {
        appendNMEAChecksum(s);
        f.println(s);
    }

    /**
     * Compute an append NMEA sentence checksum
     * The input string can either end with '*', a newline, or just a NULL - all
     * are valid.
     */
    void appendNMEAChecksum(char *pszSentence) {
        int checksum = 0;
        char *c;
        c = ++pszSentence;
        while (*c != '\0' && *c != '\n' && *c != '*') {
            checksum ^= *c++;
        }
        sprintf(c, "*%02X", checksum);
    }

    char *dtostrf(double val, signed char width, unsigned char prec,
                  char *sout) {
        uint32_t iPart = (uint32_t)val;
        uint32_t dPart = (uint32_t)((val - (double)iPart) * pow(10, prec));

        sprintf(sout, "%d.%d", iPart, dPart);
        return sout;
    }

    char *vec2str(char *dest, int size, float x, float y, float z) {
        char a[16], b[16], c[16];
        dtostrf(x, 4, 5, a);
        dtostrf(y, 4, 5, b);
        dtostrf(z, 4, 5, c);
        sprintf(dest, "%s,%s,%s", a, b, c);
        return dest;
    }

    void reportIMU(FsFile &f) {

        if (f) {

            char sentence[128];

            char strGyro[32];
            char strAcc[32];

#define GYROtoDEG(x) ((x)) 
#define ACCELtoMPS2(x) ((x) * 9.81 )

            FusionVector accf = FusionAhrsGetLinearAcceleration(&ahrs);
            FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);

            // gyro rates set to zero for now.
            
            sprintf(sentence, "$PIMU,%ld,%s,%s*",
                    millis() - ulLogfileOriginMillis,
                    vec2str(strAcc, sizeof(strAcc), ACCELtoMPS2(accf.axis.x),
                            ACCELtoMPS2(accf.axis.y), ACCELtoMPS2(accf.axis.z)),
                    vec2str(strGyro, sizeof(strGyro), GYROtoDEG(0.0f),
                            GYROtoDEG(0.0f), GYROtoDEG(0.0f)));

            logfilePrintSentence(f, sentence);

            char qw[10];
            char qx[10];
            char qy[10];
            char qz[10];

            sprintf(sentence, "$PIM2,%ld,%s,%s,%s,%s*",
                    millis() - ulLogfileOriginMillis, dtostrf(q.element.w, 4, 5, qw),
                    dtostrf(q.element.x, 4, 5, qx), dtostrf(q.element.y, 4, 5, qy),
                    dtostrf(q.element.z, 4, 5, qz));

            logfilePrintSentence(f, sentence);
        }
    }

/**
 * Fusion constants and globals
 */
// #define CLOCKS_PER_SEC 1

    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                                0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                         0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    const FusionVector magnetometerZeroes = {0, 0, 0};
    
    // magnetometer reading (units TBD)
    float mx, my, mz;

    float accelBias[3] = {0.0f, 0.0f, 0.0f};
    float gyroBias[3] = {0.0f, 0.0f, 0.0f};

    /*
     * WARNING: These calibration values are specific to my test device
     */
    uint32_t softIronOffset[3] = {139525, 132218, 133466};
    float softIronScale[3] = {0.00052652f, 0.00049533f, 0.00044927f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    boolean bFirstPressureSample;

    uint32_t lastTime_ms;

    JumpState nAppState;

    /*
     * Set up timers
     *
     * Timer 1: used in landing state machine (OFF initially)
     *
     * Timer 2: periodic check of battery state [unused in Tempo V1 hardware]
     *
     * Timer 3: blink controller for RED LED  (OFF initially) [obsolete in Tempo]
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

#define TIMER3_ON_INTERVAL_MS 750
#define TIMER3_OFF_INTERVAL_1_MS 750  // off interval when signaling battery low
#define TIMER3_OFF_INTERVAL_2_MS \
    (3000 - TIMER3_ON_INTERVAL_MS)    // off interval for flight mode

    bool bTimer3Active = false;
    int32_t timer3_ms = 0;

#define TIMER4_INTERVAL_MS 100  // 10 Hz    

    bool bTimer4Active = false;
    int32_t timer4_ms = 0;

#define TIMER5_INTERVAL_MS 10000

    bool bTimer5Active = false;
    int32_t timer5_ms = 0;

#define TIMER6_INTERVAL_MS (1000 * 60 * 5)  // five minutes

    bool bTimer6Active = false;
    int32_t timer6_ms = 0;

    /*
     * most recent Barometer sample
     */
    double dStaticPressure_hPa;
    double dBaroTemp_degC;
    double dBaroAltitude_m;

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

#define NUM_H_SAMPLES 5
    int nHSample[NUM_H_SAMPLES];
    int nHDotSample[NUM_H_SAMPLES];
    int nNextHSample = 0;
    uint32_t ulLastHSampleMillis;
    uint32_t ulLogfileOriginMillis;

    /*
     * Queue of ground altitude samples; we save a series of samples
     * so that when we detect a climbout starting, we can use one further back
     * in time that the very last sample (which might be less accurate)
     */
#define NUM_H_GROUND_SAMPLES 4
    int nHGroundSample[NUM_H_GROUND_SAMPLES];
    int nNextHGroundSample = 0;

    // TXT log file handle
    FsFile txtLogFile;

    // For debugging
    bool printNMEA = false;

    /**
     * @brief record estimated surface altitude (feet, MSL)
     *
     * @param nH_feet_msl
     */
    void recordGroundAltitude(int nH_feet_msl) {
        nHGroundSample[nNextHGroundSample++] = nH_feet_msl;
        if (nNextHGroundSample >= NUM_H_GROUND_SAMPLES) {
            nNextHGroundSample = 0;
        }
    }

    void recordInitialGroundAltitude(int nH_feet_msl) {
        for (int i = 0; i < NUM_H_GROUND_SAMPLES; ++i) {
            recordGroundAltitude(nH_feet_msl);
        }
    }

    int getGroundAltitude() {
        // look three samples back (sampled between 10 and 15 minutes ago)
        int nSelectedSample = nNextHGroundSample - 3;

        if (nSelectedSample < 0) {
            nSelectedSample += NUM_H_GROUND_SAMPLES;
        }
        return nHGroundSample[nSelectedSample];
    }

    /**
     * @brief Use pressure altitude samples to estimate rate of climb
     *
     * Rate of climb is re-estimated every 10 seconds.
     */
    void updateHDot(float H_feet);

    /// @brief Manage application state based on sensor inputs. This function
    /// is called from the main loop() function. It defines the logic used for
    /// the application state machine.
    void updateFlightStateMachine();

    /// @brief Manage application state based on sensor inputs.
    void updateTestStateMachine();

    /// @brief Application state support function: enters the IN_FLIGHT state
    /// when the start of a Jump is detected
    void enterInFlightState(bool &retFlag);

    /// @brief Application state support function: enters the WAIT state
    /// when the end of a Jump is detected
    void enterWaitState();

    /// @brief Maintain rate of climb state based on changes in altitude
    void updateRateOfClimb();

    /// @brief Log altitude and static pressure record to TXT log file
    void logAltitude(float dPressure_hPa, double dAlt_ft);

    void startLogFileFlushing();

    void stopLogFileFlushing();

    void flushLog();
};

#endif