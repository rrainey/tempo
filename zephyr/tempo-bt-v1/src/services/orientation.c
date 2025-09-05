/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Orientation Tracking Service Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "services/orientation.h"
#include "fusion.h"

LOG_MODULE_REGISTER(orientation, LOG_LEVEL_INF);

/* Module state */
static struct {
    FusionAhrs ahrs;
    orientation_config_t config;
    bool initialized;
    uint64_t last_update_us;
    struct k_mutex lock;
} orientation_state;

/* Default configuration for IMU-only operation */
static const orientation_config_t default_config = {
    .gain = 2.5f,                    /* Higher gain for IMU-only mode */
    .sample_period = 0.0025f,        /* 400Hz default */
    .use_magnetometer = false,       /* No mag on V1 */
    .acceleration_rejection = 10.0f, /* 10 m/s² threshold */
    .magnetic_rejection = 10.0f,     /* Not used without mag */
    .recovery_trigger_period = 5.0f  /* 5 seconds */
};

int orientation_init(const orientation_config_t *config)
{
    LOG_INF("Initializing orientation service");
    
    k_mutex_init(&orientation_state.lock);
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    
    /* Use provided config or defaults */
    if (config) {
        orientation_state.config = *config;
    } else {
        orientation_state.config = default_config;
    }
    
    /* Initialize Fusion AHRS */
    FusionAhrsInitialise(&orientation_state.ahrs);
    
    /* Configure AHRS settings */
    FusionAhrsSettings ahrs_settings = {
        .gain = orientation_state.config.gain,
        .accelerationRejection = orientation_state.config.acceleration_rejection,
        .magneticRejection = orientation_state.config.magnetic_rejection,
        .rejectionTimeout = orientation_state.config.recovery_trigger_period
    };
    
    FusionAhrsSetSettings(&orientation_state.ahrs, &ahrs_settings);
    
    orientation_state.initialized = true;
    orientation_state.last_update_us = 0;
    
    k_mutex_unlock(&orientation_state.lock);
    
    LOG_INF("Orientation service initialized (gain=%.1f, rejection=%.1f m/s²)", 
            orientation_state.config.gain, 
            orientation_state.config.acceleration_rejection);
    
    return 0;
}

int orientation_update(const imu_sample_t *sample)
{
    if (!sample) {
        return -EINVAL;
    }
    
    if (!orientation_state.initialized) {
        return -ENODEV;
    }
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    
    /* Calculate delta time */
    float delta_time;
    if (orientation_state.last_update_us == 0) {
        /* First sample, use configured period */
        delta_time = orientation_state.config.sample_period;
    } else {
        /* Calculate actual delta time */
        uint64_t delta_us = sample->timestamp_us - orientation_state.last_update_us;
        delta_time = delta_us / 1000000.0f;
        
        /* Sanity check - clamp to reasonable range */
        if (delta_time < 0.0001f) {
            delta_time = 0.0001f;  /* 0.1ms minimum */
        } else if (delta_time > 1.0f) {
            delta_time = 1.0f;     /* 1 second maximum */
        }
    }
    
    orientation_state.last_update_us = sample->timestamp_us;
    
    /* Convert units for Fusion:
     * - Gyroscope: rad/s to deg/s
     * - Accelerometer: m/s² to g
     */
    FusionVector gyroscope = {
        .x = sample->gyro_x * (180.0f / M_PI),
        .y = sample->gyro_y * (180.0f / M_PI),
        .z = sample->gyro_z * (180.0f / M_PI)
    };
    
    FusionVector accelerometer = {
        .x = sample->accel_x / 9.80665f,
        .y = sample->accel_y / 9.80665f,
        .z = sample->accel_z / 9.80665f
    };
    
    /* Update AHRS algorithm */
    FusionAhrsUpdateNoMagnetometer(&orientation_state.ahrs, 
                                   gyroscope, 
                                   accelerometer, 
                                   delta_time);
    
    k_mutex_unlock(&orientation_state.lock);
    
    /* Log status periodically */
    static uint32_t update_count = 0;
    if (++update_count % 400 == 0) {  /* Every ~1 second at 400Hz */
        orientation_quaternion_t quat;
        orientation_euler_t euler;
        
        orientation_get_quaternion(&quat);
        orientation_get_euler(&euler);
        
        LOG_DBG("Orientation: Q=[%.3f,%.3f,%.3f,%.3f], RPY=[%.1f,%.1f,%.1f]°",
                quat.w, quat.x, quat.y, quat.z,
                euler.roll, euler.pitch, euler.yaw);
        
        if (FusionAhrsIsInitialising(&orientation_state.ahrs)) {
            LOG_INF("AHRS still initializing...");
        }
    }
    
    return 0;
}

int orientation_get_quaternion(orientation_quaternion_t *quat)
{
    if (!quat) {
        return -EINVAL;
    }
    
    if (!orientation_state.initialized) {
        return -ENODEV;
    }
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    
    FusionQuaternion fusion_quat = FusionAhrsGetQuaternion(&orientation_state.ahrs);
    
    quat->w = fusion_quat.w;
    quat->x = fusion_quat.x;
    quat->y = fusion_quat.y;
    quat->z = fusion_quat.z;
    
    k_mutex_unlock(&orientation_state.lock);
    
    return 0;
}

int orientation_get_euler(orientation_euler_t *euler)
{
    if (!euler) {
        return -EINVAL;
    }
    
    if (!orientation_state.initialized) {
        return -ENODEV;
    }
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    
    FusionEuler fusion_euler = FusionAhrsGetEuler(&orientation_state.ahrs);
    
    euler->roll = fusion_euler.roll;
    euler->pitch = fusion_euler.pitch;
    euler->yaw = fusion_euler.yaw;
    
    k_mutex_unlock(&orientation_state.lock);
    
    return 0;
}

int orientation_reset(void)
{
    if (!orientation_state.initialized) {
        return -ENODEV;
    }
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    
    /* Reset to identity quaternion */
    FusionQuaternion identity = {1.0f, 0.0f, 0.0f, 0.0f};
    FusionAhrsSetQuaternion(&orientation_state.ahrs, identity);
    
    /* Reset timing */
    orientation_state.last_update_us = 0;
    
    k_mutex_unlock(&orientation_state.lock);
    
    LOG_INF("Orientation reset to identity");
    
    return 0;
}

int orientation_set_heading(float heading_deg)
{
    if (!orientation_state.initialized) {
        return -ENODEV;
    }
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    
    /* Get current orientation */
    FusionEuler current_euler = FusionAhrsGetEuler(&orientation_state.ahrs);
    
    /* Calculate heading offset */
    float heading_offset = heading_deg - current_euler.yaw;
    
    /* Create rotation quaternion for heading adjustment */
    float half_angle = FusionDegreesToRadians(heading_offset) * 0.5f;
    FusionQuaternion heading_rotation = {
        .w = cosf(half_angle),
        .x = 0.0f,
        .y = 0.0f,
        .z = sinf(half_angle)
    };
    
    /* Apply heading rotation */
    FusionQuaternion current_quat = FusionAhrsGetQuaternion(&orientation_state.ahrs);
    FusionQuaternion adjusted_quat = FusionQuaternionMultiply(heading_rotation, current_quat);
    FusionAhrsSetQuaternion(&orientation_state.ahrs, adjusted_quat);
    
    k_mutex_unlock(&orientation_state.lock);
    
    LOG_INF("Heading set to %.1f degrees", heading_deg);
    
    return 0;
}

int orientation_get_config(orientation_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    *config = orientation_state.config;
    k_mutex_unlock(&orientation_state.lock);
    
    return 0;
}

int orientation_set_config(const orientation_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }
    
    if (!orientation_state.initialized) {
        return -ENODEV;
    }
    
    k_mutex_lock(&orientation_state.lock, K_FOREVER);
    
    orientation_state.config = *config;
    
    /* Update AHRS settings */
    FusionAhrsSettings ahrs_settings = {
        .gain = config->gain,
        .accelerationRejection = config->acceleration_rejection,
        .magneticRejection = config->magnetic_rejection,
        .rejectionTimeout = config->recovery_trigger_period
    };
    
    FusionAhrsSetSettings(&orientation_state.ahrs, &ahrs_settings);
    
    k_mutex_unlock(&orientation_state.lock);
    
    LOG_INF("Orientation config updated (gain=%.1f, rejection=%.1f m/s²)", 
            config->gain, config->acceleration_rejection);
    
    return 0;
}