/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - RGB LED Service Interface
 */

#ifndef SERVICES_LED_H
#define SERVICES_LED_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief RGB color structure
 */
typedef struct {
    uint8_t r;  /* Red component (0-255) */
    uint8_t g;  /* Green component (0-255) */
    uint8_t b;  /* Blue component (0-255) */
} rgb_color_t;

/* Common color presets */
#define RGB_BLACK   ((rgb_color_t){0, 0, 0})
#define RGB_RED     ((rgb_color_t){255, 0, 0})
#define RGB_GREEN   ((rgb_color_t){0, 255, 0})
#define RGB_BLUE    ((rgb_color_t){0, 0, 255})
#define RGB_YELLOW  ((rgb_color_t){255, 255, 0})
#define RGB_CYAN    ((rgb_color_t){0, 255, 255})
#define RGB_MAGENTA ((rgb_color_t){255, 0, 255})
#define RGB_WHITE   ((rgb_color_t){255, 255, 255})
#define RGB_ORANGE  ((rgb_color_t){255, 128, 0})

/**
 * @brief Initialize the LED service
 * 
 * @return 0 on success, negative error code on failure
 */
int led_service_init(void);

/**
 * @brief Set RGB LED color and state
 * 
 * When state is true, the LED will blink the specified color
 * for 50ms once per second. When false, the LED is off.
 * 
 * @param color RGB color to display
 * @param state true to enable blinking, false to turn off
 * @return 0 on success, negative error code on failure
 */
int set_color_led_state(rgb_color_t color, bool state);

/**
 * @brief Get current LED state
 * 
 * @param color Output: Current color (if state is true)
 * @param state Output: Current state
 * @return 0 on success, negative error code on failure
 */
int led_service_get_state(rgb_color_t *color, bool *state);

/**
 * @brief Set LED override state
 * 
 * When override is enabled, the LED will use the override color
 * instead of the application-set color. This is used for remote
 * control via mcumgr commands.
 * 
 * @param color RGB color to display (ignored if enable is false)
 * @param enable true to enable override, false to return to app control
 * @return 0 on success, negative error code on failure
 */
int led_service_set_override(rgb_color_t color, bool enable);

/**
 * @brief Get current override state
 * 
 * @param color Output: Current override color (if enabled)
 * @param enabled Output: Whether override is active
 * @return 0 on success, negative error code on failure
 */
int led_service_get_override(rgb_color_t *color, bool *enabled);

#endif /* SERVICES_LED_H */