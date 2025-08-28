/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Test SPI without CS control
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(spi_no_cs, LOG_LEVEL_INF);

#define ICM42688_REG_WHO_AM_I     0x75
#define ICM42688_WHO_AM_I_VALUE   0x47
#define ICM42688_SPI_READ_BIT     0x80

#define IMU_SPI_NODE DT_NODELABEL(icm42688)

int test_spi_without_cs(void)
{
    const struct device *spi_dev;
    int ret;
    
    LOG_INF("=== Testing SPI without CS control ===");
    LOG_INF("Since P0.11 is stuck low, the device is always selected");
    LOG_INF("This might actually allow communication!");
    
    spi_dev = DEVICE_DT_GET(DT_BUS(IMU_SPI_NODE));
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    
    /* Configure P0.11 as output low to keep CS asserted */
    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    gpio_pin_configure(gpio0, 11, GPIO_OUTPUT_LOW);
    
    /* SPI config without automatic CS control */
    struct spi_config spi_cfg_no_cs = {
        .frequency = 1000000,
        .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
        .cs = NULL  /* No CS control */
    };
    
    /* Test different SPI modes */
    uint32_t modes[] = {
        0,                                  /* Mode 0 */
        SPI_MODE_CPHA,                     /* Mode 1 */
        SPI_MODE_CPOL,                     /* Mode 2 */
        SPI_MODE_CPOL | SPI_MODE_CPHA      /* Mode 3 */
    };
    
    for (int i = 0; i < 4; i++) {
        LOG_INF("\nTesting SPI Mode %d", i);
        spi_cfg_no_cs.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | modes[i];
        
        /* WHO_AM_I read sequence */
        uint8_t tx_data[2] = {ICM42688_REG_WHO_AM_I | ICM42688_SPI_READ_BIT, 0x00};
        uint8_t rx_data[2] = {0xFF, 0xFF};
        
        struct spi_buf tx_buf = {.buf = tx_data, .len = 2};
        struct spi_buf rx_buf = {.buf = rx_data, .len = 2};
        struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
        struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
        
        ret = spi_transceive(spi_dev, &spi_cfg_no_cs, &tx_set, &rx_set);
        if (ret < 0) {
            LOG_ERR("SPI transceive failed: %d", ret);
        } else {
            LOG_INF("TX: [0x%02X, 0x%02X]", tx_data[0], tx_data[1]);
            LOG_INF("RX: [0x%02X, 0x%02X]", rx_data[0], rx_data[1]);
            if (rx_data[1] == ICM42688_WHO_AM_I_VALUE) {
                LOG_INF("SUCCESS! WHO_AM_I = 0x%02X (correct!)", rx_data[1]);
                return 0;
            }
        }
        
        k_msleep(10);
    }
    
    /* Also try with a dummy transaction first */
    LOG_INF("\nTrying with dummy transaction first:");
    uint8_t dummy_tx[2] = {0xFF, 0xFF};
    uint8_t dummy_rx[2];
    struct spi_buf dummy_tx_buf = {.buf = dummy_tx, .len = 2};
    struct spi_buf dummy_rx_buf = {.buf = dummy_rx, .len = 2};
    struct spi_buf_set dummy_tx_set = {.buffers = &dummy_tx_buf, .count = 1};
    struct spi_buf_set dummy_rx_set = {.buffers = &dummy_rx_buf, .count = 1};
    
    spi_cfg_no_cs.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8); /* Mode 0 */
    spi_transceive(spi_dev, &spi_cfg_no_cs, &dummy_tx_set, &dummy_rx_set);
    k_msleep(1);
    
    /* Now try WHO_AM_I again */
    uint8_t tx_data[2] = {ICM42688_REG_WHO_AM_I | ICM42688_SPI_READ_BIT, 0x00};
    uint8_t rx_data[2] = {0xFF, 0xFF};
    struct spi_buf tx_buf = {.buf = tx_data, .len = 2};
    struct spi_buf rx_buf = {.buf = rx_data, .len = 2};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
    
    ret = spi_transceive(spi_dev, &spi_cfg_no_cs, &tx_set, &rx_set);
    if (ret == 0) {
        LOG_INF("After dummy: RX = [0x%02X, 0x%02X]", rx_data[0], rx_data[1]);
    }
    
    return -ENODEV;
}