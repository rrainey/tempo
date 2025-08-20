/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Test GPIO Forwarder Issue on P0.10/P0.11
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(gpio_test, LOG_LEVEL_INF);

static int test_gpio_pins(void)
{
    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    int ret;
    int value;
    
    LOG_INF("=== Testing GPIO access to P0.10 and P0.11 ===");
    
    if (!device_is_ready(gpio0)) {
        LOG_ERR("GPIO0 not ready");
        return -ENODEV;
    }
    
    /* Test P0.10 (SPI MISO) */
    LOG_INF("Testing P0.10 (SPI MISO)...");
    ret = gpio_pin_configure(gpio0, 10, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure P0.10 as input: %d", ret);
        LOG_ERR("This suggests GPIO forwarder is blocking access!");
        return ret;
    } else {
        value = gpio_pin_get(gpio0, 10);
        LOG_INF("P0.10 configured as input, read value: %d", value);
    }
    
    /* Test P0.11 (SPI CS) */
    LOG_INF("Testing P0.11 (SPI CS)...");
    ret = gpio_pin_configure(gpio0, 11, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure P0.11 as output: %d", ret);
        LOG_ERR("This suggests GPIO forwarder is blocking access!");
        return ret;
    } else {
        LOG_INF("P0.11 configured as output");
        
        /* Try toggling it */
        for (int i = 0; i < 3; i++) {
            gpio_pin_set(gpio0, 11, 0);
            value = gpio_pin_get(gpio0, 11);
            LOG_INF("  Set P0.11 = 0, read = %d", value);
            k_msleep(10);
            
            gpio_pin_set(gpio0, 11, 1);
            value = gpio_pin_get(gpio0, 11);
            LOG_INF("  Set P0.11 = 1, read = %d", value);
            k_msleep(10);
        }
    }
    
    /* Also test some other pins for comparison */
    LOG_INF("\nTesting other GPIO pins for comparison...");
    
    /* Test P1.07 (D6 - IMU INT1) */
    if (device_is_ready(gpio1)) {
        ret = gpio_pin_configure(gpio1, 7, GPIO_INPUT);
        if (ret == 0) {
            value = gpio_pin_get(gpio1, 7);
            LOG_INF("P1.07 (D6) configured as input, read value: %d", value);
        }
    }
    
    /* Test P0.08 (SPI SCK) */
    ret = gpio_pin_configure(gpio0, 8, GPIO_INPUT);
    if (ret == 0) {
        value = gpio_pin_get(gpio0, 8);
        LOG_INF("P0.08 (SPI SCK) configured as input, read value: %d", value);
    }
    
    /* Test P0.09 (SPI MOSI) */
    ret = gpio_pin_configure(gpio0, 9, GPIO_INPUT);
    if (ret == 0) {
        value = gpio_pin_get(gpio0, 9);
        LOG_INF("P0.09 (SPI MOSI) configured as input, read value: %d", value);
    }
    
    return 0;
}

int run_gpio_forwarder_test(void)
{
    LOG_INF("Starting GPIO forwarder test");
    LOG_INF("This test checks if P0.10 and P0.11 are accessible");
    LOG_INF("If these fail, the GPIO forwarder is blocking them");
    
    return test_gpio_pins();
}