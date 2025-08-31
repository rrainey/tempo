/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - RGB LED Service Implementation
 * 
 * Controls an RGB LED using PWM, providing a 50ms blink once per second
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>

#include "services/led.h"

LOG_MODULE_REGISTER(led_service, LOG_LEVEL_INF);

/* PWM configuration */
#define LED_PWM_NODE DT_NODELABEL(pwm0)
#define LED_PWM_CHANNEL_R 0  /* Red on channel 0 */
#define LED_PWM_CHANNEL_G 1  /* Green on channel 1 */
#define LED_PWM_CHANNEL_B 2  /* Blue on channel 2 */

/* Timing configuration */
#define BLINK_PERIOD_MS    1000  /* 1 second period */ 
#define BLINK_ON_TIME_MS   410   /* 50ms on time */
#define PWM_PERIOD_NS      1000000  /* 1ms PWM period for smooth dimming */ 

/* LED state */
static struct {
    const struct device *pwm_dev;
    rgb_color_t current_color;
    bool enabled;
    struct k_timer blink_timer;
    bool led_on;
    struct k_mutex mutex;
} led_state = {
    .pwm_dev = NULL,
    .current_color = RGB_BLACK,
    .enabled = false,
    .led_on = false
};

/* Forward declaration */
static void blink_timer_handler(struct k_timer *timer);
static void led_off(void);

/* Timer work item for turning LED off */
static struct k_work_delayable led_off_work;

/* Work handler to turn LED off */
static void led_off_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    led_off();
}

/**
 * @brief Set PWM duty cycle for a color channel
 */
static int set_pwm_channel(uint32_t channel, uint8_t brightness)
{
    uint32_t pulse_ns;
    int ret;
    
    pulse_ns = ((uint32_t)brightness * PWM_PERIOD_NS) / 255;
    
    LOG_INF("Setting PWM ch%d: brightness=%d, pulse=%d ns", 
            channel, brightness, pulse_ns);
    
    ret = pwm_set(led_state.pwm_dev, channel, PWM_PERIOD_NS, pulse_ns, 0);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM channel %d: %d", channel, ret);
    }
    
    return ret;
}

/**
 * @brief Turn LED on with current color
 */
static void led_on(void)
{
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    if (led_state.enabled) {
        set_pwm_channel(LED_PWM_CHANNEL_R, led_state.current_color.r >> 3);
        set_pwm_channel(LED_PWM_CHANNEL_G, led_state.current_color.g >> 3);
        set_pwm_channel(LED_PWM_CHANNEL_B, led_state.current_color.b >> 3);
        led_state.led_on = true;
    }
    
    k_mutex_unlock(&led_state.mutex);
}

/**
 * @brief Turn LED off
 */
static void led_off(void)
{
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    set_pwm_channel(LED_PWM_CHANNEL_R, 0);
    set_pwm_channel(LED_PWM_CHANNEL_G, 0);
    set_pwm_channel(LED_PWM_CHANNEL_B, 0);
    led_state.led_on = false;
    
    k_mutex_unlock(&led_state.mutex);
}

/**
 * @brief Blink timer handler
 */
static void blink_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    
    /* Only turn on if currently off */
    if (!led_state.led_on) {
        /* Turn LED on */
        led_on();
        
        /* Schedule LED off after 50ms */
        k_work_reschedule(&led_off_work, K_MSEC(BLINK_ON_TIME_MS));
    }
}

/**
 * @brief Initialize the LED service
 */
int led_service_init(void)
{
    
    /* Get PWM device */
    led_state.pwm_dev = DEVICE_DT_GET(LED_PWM_NODE);
    if (!device_is_ready(led_state.pwm_dev)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }
    
    /* Initialize mutex */
    k_mutex_init(&led_state.mutex);
    
    /* Initialize timer */
    k_timer_init(&led_state.blink_timer, blink_timer_handler, NULL);

    /* Initialize work item */
    k_work_init_delayable(&led_off_work, led_off_work_handler);
    
    /* Turn off all channels initially */
    led_off();
    
    LOG_INF("LED service initialized");
    
    return 0;
}

/**
 * @brief Set RGB LED color and state
 */
int set_color_led_state(rgb_color_t color, bool state)
{
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    /* Update state */
    led_state.current_color = color;
    led_state.enabled = state;
    
    if (state) {
        /* Start blinking timer */
        k_timer_start(&led_state.blink_timer, 
                    K_NO_WAIT,
                    K_MSEC(BLINK_PERIOD_MS));
    } else {
        /* Stop timer and turn off LED */
        k_timer_stop(&led_state.blink_timer);
        k_work_cancel_delayable(&led_off_work);
        led_off();
    }
    
    k_mutex_unlock(&led_state.mutex);
    
    return 0;
}

/**
 * @brief Get current LED state
 */
int led_service_get_state(rgb_color_t *color, bool *state)
{
    if (!color || !state) {
        return -EINVAL;
    }
    
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    *color = led_state.current_color;
    *state = led_state.enabled;
    
    k_mutex_unlock(&led_state.mutex);
    
    return 0;
}