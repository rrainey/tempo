# Tempo Arduino Logger - Altitude Estimation and State Machine Analysis

## Overview
This document analyzes the surface altitude estimator and application state transition algorithms from the Arduino implementation of the Tempo skydiving logger (version 0.155).

## Surface Altitude Estimator

### Implementation
The surface altitude estimation in the current implementation is extremely simple:

```c
// When we're in WAIT mode, we can use the altitude
// to set ground altitude.
nHGround_feet = dAlt_ft;
```

### Algorithm Characteristics
- **Continuous Update**: While in `STATE_WAIT`, the system continuously updates `nHGround_feet` with the current pressure altitude
- **No Filtering**: Raw altitude reading is used directly without averaging or filtering
- **Last Known Value**: Ground altitude is simply the last altitude reading before takeoff
- **No Validation**: No checks for stability or outliers

## Vertical Speed (HDOT) Computation

### Algorithm Implementation
```c
void updateHDot(float H_feet) {
    uint32_t ulMillis = millis();
    int nLastHSample_feet;
    int nInterval_ms = ulMillis - ulLastHSampleMillis;

    /* update HDot every ten seconds */
    if (nInterval_ms > 10000) {
        if (!bFirstPressureSample) {
            // Get previous sample from circular buffer
            if (nNextHSample == 0) {
                nLastHSample_feet = nHSample[NUM_H_SAMPLES-1];
            } else {
                nLastHSample_feet = nHSample[nNextHSample-1];
            }
            
            // Store current altitude
            nHSample[nNextHSample] = H_feet;
            
            // Calculate vertical speed in feet per minute
            nHDotSample[nNextHSample] = (((long) H_feet - nLastHSample_feet) * 60000L) / nInterval_ms;
            nHDot_fpm = nHDotSample[nNextHSample];
        }
        else {
            // First sample - no rate calculation possible
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
```

### Key Features
- **Update Rate**: Every 10 seconds (quite coarse for dynamic activities)
- **Buffer Size**: Circular buffer of 5 samples (`NUM_H_SAMPLES`)
- **Calculation Method**: Simple finite difference: `(current_alt - previous_alt) * 60000 / time_interval_ms`
- **Units**: Feet per minute (fpm)
- **Filtering**: None - raw difference calculation

## State Machine Design

### State Definitions
```c
#define STATE_WAIT       0  // On ground, gathering baseline data
#define STATE_IN_FLIGHT  1  // Aircraft climbing
#define STATE_JUMPING    2  // In freefall
#define STATE_LANDED_1   3  // Just landed, waiting to confirm
```

### Transition Thresholds
```c
#define OPS_HDOT_THRESHOLD_FPM       300   // Climbing threshold
#define OPS_HDOT_LAND_THRESHOLD_FPM  100   // Landing detection
#define OPS_HDOT_JUMPING_FPM         -800  // Freefall detection
```

### State Transition Logic

#### WAIT → IN_FLIGHT
- **Trigger**: Vertical speed > 300 fpm (climbing)
- **Actions**:
  - Open new log file with timestamp
  - Start IMU logging at 100 Hz
  - Enable periodic file flushing
  - Activate LED blinking pattern
  - Record session start time

#### IN_FLIGHT → JUMPING
- **Trigger**: Vertical speed < -800 fpm (rapid descent)
- **Actions**:
  - Increase GNSS update rate from 1 Hz to 4 Hz
  - Disable GSA/GSV NMEA sentences (reduce data overhead)

#### JUMPING → LANDED_1
- **Trigger**: |Vertical speed| < 100 fpm (nearly stationary)
- **Actions**:
  - Start 30-second confirmation timer
  - Continue logging while confirming landing

#### LANDED_1 State Transitions
Three possible outcomes:

1. **LANDED_1 → JUMPING** (False landing)
   - **Trigger**: Vertical speed < -800 fpm
   - **Actions**: Cancel timer, return to freefall state

2. **LANDED_1 → IN_FLIGHT** (False landing)
   - **Trigger**: |Vertical speed| > 300 fpm
   - **Actions**: Cancel timer, return to flight state

3. **LANDED_1 → WAIT** (Confirmed landing)
   - **Trigger**: 30-second timer expires with no motion
   - **Actions**:
     - Reduce GNSS to 0.5 Hz
     - Re-enable GSA/GSV sentences
     - Stop IMU logging
     - Close log file
     - Clear LED indicators

### State Machine Diagram
```
         ┌──────────┐
         │   WAIT   │
         └─────┬────┘
               │ HDOT > 300 fpm
         ┌─────▼────────┐
         │  IN_FLIGHT   │
         └─────┬────────┘
               │ HDOT < -800 fpm
         ┌─────▼────────┐
         │   JUMPING    │
         └─────┬────────┘
               │ |HDOT| < 100 fpm
         ┌─────▼────────┐
         │  LANDED_1    │──┐
         │  (30s timer) │  │ HDOT < -800 fpm
         └─────┬────────┘  │ or |HDOT| > 300 fpm
               │           │
               │ Timer     │
               │ expires   │
         ┌─────▼────────┐  │
         │    WAIT      │◄─┘
         └──────────────┘
```

## Strengths of Current Implementation

1. **Simple and Robust**: Easy to understand and debug
2. **Hysteresis**: Different thresholds prevent state oscillation
3. **Landing Confirmation**: 30-second timer prevents false stops
4. **Resource Management**: Adaptive GNSS rates optimize power/data
5. **Clear State Actions**: Well-defined entry/exit behaviors

## Areas for Improvement

### Surface Altitude Estimation
- Add averaging window during WAIT state
- Implement outlier rejection
- Consider pressure trend analysis
- Add temperature compensation
- Store multiple ground references for different locations

### Vertical Speed Calculation
- Increase update frequency (1 Hz or higher)
- Implement sliding window averaging
- Add Kalman filtering for smoother estimates
- Use accelerometer data for faster response
- Consider GPS vertical velocity as supplementary input

### State Machine Enhancements
- Add altitude-based conditions (minimum altitude for jump detection)
- Implement "CANOPY" state for under-canopy flight
- Add GPS groundspeed to landing detection
- Log state transitions with timestamps and reasons
- Consider barometric pressure rate-of-change
- Add error states for sensor failures

### Additional Considerations
- Implement pre-flight calibration sequence
- Add manual state override capability
- Store state history for post-flight analysis
- Consider wind effects on ground detection
- Add configurable thresholds for different use cases

## Implementation Notes for Zephyr Migration

When porting to the Zephyr-based system:

1. **Use Zephyr's built-in filtering libraries** for altitude and rate calculations
2. **Implement state machine using Zephyr's SM framework** for better structure
3. **Add configuration via Kconfig** for thresholds and timing
4. **Use work queues** for periodic altitude sampling
5. **Implement proper sensor fusion** combining baro, GPS, and IMU data
6. **Add comprehensive logging** of state transitions and decision factors