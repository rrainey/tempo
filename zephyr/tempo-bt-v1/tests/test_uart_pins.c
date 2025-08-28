/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Test UART pin conflicts
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(uart_test, LOG_LEVEL_INF);

int test_uart_conflicts(void)
{
    LOG_INF("=== Checking UART configurations ===");
    
    /* Check all UART devices */
    const struct device *uart0 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(uart0));
    const struct device *uart1 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(uart1));
    const struct device *uart2 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(uart2));
    
    if (uart0 && device_is_ready(uart0)) {
        LOG_INF("UART0 is ready (console)");
    }
    
    if (uart1 && device_is_ready(uart1)) {
        LOG_INF("UART1 is ready (GNSS)");
    }
    
    if (uart2 && device_is_ready(uart2)) {
        LOG_WRN("UART2 is ready - this might use P0.10/P0.11 for flow control!");
        LOG_WRN("UART2 should be disabled in the overlay");
    }
    
    /* According to README.md:
     * - NET_NORA_TX/RX are the primary GNSS UART
     * - APP_NORA_TX/RX/RTS/CTS are secondary UART with flow control
     * - If APP_NORA uses P0.10/P0.11 for RTS/CTS, that's our conflict
     */
    
    LOG_INF("\nFrom README.md signal mapping:");
    LOG_INF("- SPIM4_CSN -> P0.11 (our CS pin)");
    LOG_INF("- APP_NORA_RTS/CTS might also want P0.10/P0.11");
    LOG_INF("This could explain why P0.11 is being driven by something else");
    
    return 0;
}