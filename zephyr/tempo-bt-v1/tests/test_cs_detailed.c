/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Detailed CS Pin Test
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(cs_test, LOG_LEVEL_INF);

int test_cs_pin_detailed(void)
{
    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    int ret;
    int value;
    gpio_flags_t flags;
    
    LOG_INF("=== Detailed CS Pin (P0.11) Test ===");
    
    if (!device_is_ready(gpio0)) {
        LOG_ERR("GPIO0 not ready");
        return -ENODEV;
    }
    
    /* First, configure as input to check initial state */
    ret = gpio_pin_configure(gpio0, 11, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure P0.11 as input: %d", ret);
        return ret;
    }
    
    value = gpio_pin_get(gpio0, 11);
    LOG_INF("P0.11 initial state (as input): %d", value);
    
    /* Try different output configurations */
    LOG_INF("Testing different output configurations:");
    
    /* Output push-pull, initially high */
    ret = gpio_pin_configure(gpio0, 11, GPIO_OUTPUT_HIGH);
    if (ret < 0) {
        LOG_ERR("Failed to configure as output high: %d", ret);
        return ret;
    }
    k_msleep(1);
    value = gpio_pin_get(gpio0, 11);
    LOG_INF("Configured as OUTPUT_HIGH, read: %d", value);
    
    /* Try to drive low */
    gpio_pin_set(gpio0, 11, 0);
    k_msleep(1);
    value = gpio_pin_get(gpio0, 11);
    LOG_INF("After set to 0, read: %d", value);
    
    /* Try to drive high */
    gpio_pin_set(gpio0, 11, 1);
    k_msleep(1);
    value = gpio_pin_get(gpio0, 11);
    LOG_INF("After set to 1, read: %d", value);
    
    /* Try open-drain configuration */
    LOG_INF("\nTrying open-drain configuration:");
    ret = gpio_pin_configure(gpio0, 11, GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);
    if (ret < 0) {
        LOG_WRN("Open-drain configuration failed: %d", ret);
    } else {
        /* Open-drain low (active) */
        gpio_pin_set(gpio0, 11, 0);
        k_msleep(1);
        value = gpio_pin_get(gpio0, 11);
        LOG_INF("Open-drain active (0), read: %d", value);
        
        /* Open-drain high (hi-z) */
        gpio_pin_set(gpio0, 11, 1);
        k_msleep(1);
        value = gpio_pin_get(gpio0, 11);
        LOG_INF("Open-drain inactive (1/hi-z), read: %d", value);
    }
    
    /* Check if there's an internal pull resistor */
    LOG_INF("\nChecking pull resistor configurations:");
    
    /* Input with pull-up */
    ret = gpio_pin_configure(gpio0, 11, GPIO_INPUT | GPIO_PULL_UP);
    if (ret == 0) {
        k_msleep(1);
        value = gpio_pin_get(gpio0, 11);
        LOG_INF("Input with pull-up, read: %d", value);
    }
    
    /* Input with pull-down */
    ret = gpio_pin_configure(gpio0, 11, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret == 0) {
        k_msleep(1);
        value = gpio_pin_get(gpio0, 11);
        LOG_INF("Input with pull-down, read: %d", value);
    }
    
    /* Input with no pull */
    ret = gpio_pin_configure(gpio0, 11, GPIO_INPUT);
    if (ret == 0) {
        k_msleep(1);
        value = gpio_pin_get(gpio0, 11);
        LOG_INF("Input with no pull, read: %d", value);
    }
    
    /* Final test: rapid toggling */
    LOG_INF("\nRapid toggle test:");
    ret = gpio_pin_configure(gpio0, 11, GPIO_OUTPUT);
    if (ret == 0) {
        for (int i = 0; i < 10; i++) {
            gpio_pin_set(gpio0, 11, i & 1);
            value = gpio_pin_get(gpio0, 11);
            LOG_INF("Set: %d, Read: %d", i & 1, value);
        }
    }
    
    return 0;
}