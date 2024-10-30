#ifndef COMBINEDLOGGER_H_
#define COMBINEDLOGGER_H_

#include <Arduino.h>
#include <Fusion.h>
#include <SdFat.h>
#include <icm42688.h>
#include <sdios.h>

#include "BinaryLogger.h"
#include "MorseBlinker.h"

/*
 * Operating mode
 */
#define OPS_FLIGHT 0  // normal mode; altimeter used to detect motion
#define OPS_STATIC_TEST 1  // for testing; time based simulation of vertical motion (preferred test
                           // mode)
#define OPS_GROUND_TEST  2  // for testing; uses GPS horizontal movement as an analogue to altitude
                            // changes

enum BlinkState {
  BLINK_STATE_OFF = 0,
  BLINK_STATE_BATTERY,
  BLINK_STATE_LOGGING,
  BLINK_STATE_NO_SDCARD,
  BLINK_STATE_BAD_FILESYSTEM,
  BLINK_STATE_INIT_FAILED
};

/*
 * Set operating mode for this build of the firmware here
 */
#define OPS_MODE OPS_STATIC_TEST

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
 * update rate set to 1Hz, close logfile log data otherwise
 */
typedef enum { WAIT, IN_FLIGHT, JUMPING, LANDED1 } JumpState;

// used in Fusion code
#define SAMPLE_RATE (200)  // IMU samples per second
// must be set to match accel/gyro samples
float fSampleInterval_sec = 1.0f / SAMPLE_RATE;

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

    BinaryLogger::APIResult startLogging();

    // Stop logging and close the log file on the SD Card
    void stopLogging();

    // call this exactly once from the main Arduino application loop() function
    void loop();

   protected:
    /**
     * @brief Receive IMU sample, maintain pose state, and log data
     *
     * @param pSample pointer to the IMU sample (from IMU FIFO)
     */
    void handleIMUSample(icm42688::fifo_packet3 *pSample);

    /**
     * @brief Receive magnetometer sample, maintain pose state, and log data
     *
     * @param sample array of raw XYZ magnetometer samples
     */
    void handleMagSample(uint32_t sample[3]);

    /**
     * @brief Receive Barometer sample, maintain pose state, and log data
     *
     * @param pData barometer sample
     */
    void handleBaroSample(bmp3_data *pData);

    /**
     * @brief Receive Next GNSS NMEA sentence, log it.
     *
     * @param pSentence pointer to buffer containing the next NMEA sentence
     */
    void handleNMEASentence(const char *pSentence);

/**
 * Fusion constants and globals
 */
#define CLOCKS_PER_SEC 1

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

#define TIMER3_ON_INTERVAL_MS 750
#define TIMER3_OFF_INTERVAL_1_MS 750  // off interval when signaling battery low
#define TIMER3_OFF_INTERVAL_2_MS \
    (3000 - TIMER3_ON_INTERVAL_MS)  // off interval for flight mode

    bool bTimer3Active = false;
    int32_t timer3_ms = 0;

#define TIMER4_INTERVAL_MS 10  // 100 Hz -- twice the IMU sample interval

    bool bTimer4Active = false;
    int32_t timer4_ms = 0;

#define TIMER5_INTERVAL_MS 10000

    bool bTimer5Active = false;
    int32_t timer5_ms = 0;

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
    int nNextHSample;
    uint32_t ulLastHSampleMillis;
    uint32_t ulLogfileOriginMillis;

    MorseBlinker morseBlinker;

    FsFile txtLogFile;

    // For debugging
    bool printNMEA = false;

    /**
     * @brief Use pressure altitude samples to estimate rate of climb
     *
     * Rate of climb is re-estimated every 10 seconds.
     */
    void updateHDot(float H_feet);

    /**
     * Control RED and GREEN status LEDs
     * 
     * These LEDs are used to indicate both the general operating status and specific states.
     * An unblinking GREEN means the device is operational. When GREEN is illuminated, the RED LED
     * blinks a status code.
     * An unblinking RED means a fatal error was encounterd. When GREEN is illuminated, the GREEN LED
     * blinks a status code.
     */
    void setBlinkState(BlinkState newState);

    void updateTestStateMachine();

    void updateFlightStateMachine();

    void sampleAndLogAltitude();

    void startLogFileFlushing();

    void stopLogFileFlushing();
};

#endif