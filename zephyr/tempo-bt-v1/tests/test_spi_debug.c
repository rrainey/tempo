/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * SPI Debug Test for ICM-42688
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(spi_debug, LOG_LEVEL_INF);

/* ICM-42688 WHO_AM_I register */
#define ICM42688_REG_WHO_AM_I     0x75
#define ICM42688_WHO_AM_I_VALUE   0x47
#define ICM42688_SPI_READ_BIT     0x80

/* SPI device from devicetree */
#define IMU_SPI_NODE DT_NODELABEL(icm42688)

/* Get the CS GPIO spec separately for newer Zephyr */
static const struct gpio_dt_spec cs_gpio_spec = GPIO_DT_SPEC_GET(DT_BUS(IMU_SPI_NODE), cs_gpios);

static void hexdump(const char *label, const uint8_t *data, size_t len)
{
    LOG_INF("%s:", label);
    for (size_t i = 0; i < len; i += 16) {
        char hex_str[50] = {0};
        char *ptr = hex_str;
        size_t j;
        
        for (j = 0; j < 16 && (i + j) < len; j++) {
            ptr += sprintf(ptr, "%02X ", data[i + j]);
        }
        LOG_INF("  %04X: %s", i, hex_str);
    }
}

static int test_spi_raw(const struct device *spi_dev, const struct spi_config *spi_cfg)
{
    int ret;
    uint8_t tx_data[4];
    uint8_t rx_data[4];
    
    LOG_INF("=== Testing raw SPI communication ===");
    
    /* Test 1: Simple WHO_AM_I read */
    memset(tx_data, 0, sizeof(tx_data));
    memset(rx_data, 0xFF, sizeof(rx_data));
    tx_data[0] = ICM42688_REG_WHO_AM_I | ICM42688_SPI_READ_BIT;
    
    struct spi_buf tx_buf = {.buf = tx_data, .len = 2};
    struct spi_buf rx_buf = {.buf = rx_data, .len = 2};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
    
    LOG_INF("Sending WHO_AM_I read command...");
    hexdump("TX", tx_data, 2);
    
    ret = spi_transceive(spi_dev, spi_cfg, &tx_set, &rx_set);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d", ret);
        return ret;
    }
    
    hexdump("RX", rx_data, 2);
    LOG_INF("WHO_AM_I result: 0x%02X (expected 0x%02X)", rx_data[1], ICM42688_WHO_AM_I_VALUE);
    
    /* Test 2: Try with longer transaction */
    LOG_INF("\nTest 2: Extended read (4 bytes)");
    memset(tx_data, 0, sizeof(tx_data));
    memset(rx_data, 0xFF, sizeof(rx_data));
    tx_data[0] = ICM42688_REG_WHO_AM_I | ICM42688_SPI_READ_BIT;
    
    tx_buf.len = 4;
    rx_buf.len = 4;
    
    ret = spi_transceive(spi_dev, spi_cfg, &tx_set, &rx_set);
    if (ret < 0) {
        LOG_ERR("Extended SPI transceive failed: %d", ret);
        return ret;
    }
    
    hexdump("Extended RX", rx_data, 4);
    
    return 0;
}

static int test_different_modes(const struct device *spi_dev)
{
    int ret;
    struct spi_config configs[] = {
        {
            .frequency = 1000000,
            .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
            .cs = {
                .gpio = cs_gpio_spec,
                .delay = 0
            }
        },
        {
            .frequency = 1000000,
            .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8),
            .cs = {
                .gpio = cs_gpio_spec,
                .delay = 0
            }
        },
        {
            .frequency = 1000000,
            .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_WORD_SET(8),
            .cs = {
                .gpio = cs_gpio_spec,
                .delay = 0
            }
        },
        {
            .frequency = 1000000,
            .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8),
            .cs = {
                .gpio = cs_gpio_spec,
                .delay = 0
            }
        }
    };
    
    const char *mode_names[] = {"Mode 0", "Mode 1", "Mode 2", "Mode 3"};
    
    LOG_INF("=== Testing different SPI modes ===");
    
    for (int i = 0; i < ARRAY_SIZE(configs); i++) {
        LOG_INF("\nTesting %s (CPOL=%d, CPHA=%d):", 
                mode_names[i],
                (configs[i].operation & SPI_MODE_CPOL) ? 1 : 0,
                (configs[i].operation & SPI_MODE_CPHA) ? 1 : 0);
        
        ret = test_spi_raw(spi_dev, &configs[i]);
        if (ret == 0) {
            LOG_INF("%s completed successfully", mode_names[i]);
        }
        
        k_msleep(10);  /* Small delay between tests */
    }
    
    return 0;
}

static int test_cs_gpio_access(void)
{
    const struct gpio_dt_spec *cs_gpio = &cs_gpio_spec;
    int ret;
    int read_val;
    
    LOG_INF("=== Testing CS GPIO Access (P0.11) ===");
    
    if (!gpio_is_ready_dt(cs_gpio)) {
        LOG_ERR("CS GPIO device not ready!");
        return -ENODEV;
    }
    
    LOG_INF("CS GPIO device ready: port=%s, pin=%d", 
            cs_gpio->port->name, cs_gpio->pin);
    
    /* Configure as output inactive (high for active-low CS) */
    ret = gpio_pin_configure_dt(cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure CS pin as output: %d", ret);
        return ret;
    }
    
    /* Test toggling CS */
    LOG_INF("Testing CS toggle sequence...");
    for (int i = 0; i < 5; i++) {
        /* Set CS low (active) */
        ret = gpio_pin_set_dt(cs_gpio, 0);
        if (ret < 0) {
            LOG_ERR("Failed to set CS low: %d", ret);
            return ret;
        }
        
        /* Read back the value */
        read_val = gpio_pin_get_dt(cs_gpio);
        LOG_INF("  CS set low, readback = %d", read_val);
        
        k_usleep(10);
        
        /* Set CS high (inactive) */
        ret = gpio_pin_set_dt(cs_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to set CS high: %d", ret);
            return ret;
        }
        
        /* Read back the value */
        read_val = gpio_pin_get_dt(cs_gpio);
        LOG_INF("  CS set high, readback = %d", read_val);
        
        k_usleep(10);
    }
    
    LOG_INF("CS GPIO access test completed successfully");
    return 0;
}

int run_spi_debug_test(void)
{
    const struct device *spi_dev;
    int ret;
    
    LOG_INF("Starting SPI debug test for ICM-42688");
    
    /* First test CS GPIO access */
    ret = test_cs_gpio_access();
    if (ret < 0) {
        LOG_ERR("CS GPIO access test failed - this suggests P0.11 conflict!");
        LOG_ERR("Check if GPIO forwarder is properly disabled");
        return ret;
    }
    
    /* Get SPI device */
    spi_dev = DEVICE_DT_GET(DT_BUS(IMU_SPI_NODE));
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    LOG_INF("SPI device ready: %s", spi_dev->name);
    
    /* Configure CS GPIO manually for testing */
    if (gpio_is_ready_dt(&cs_gpio_spec)) {
        ret = gpio_pin_configure_dt(&cs_gpio_spec, GPIO_OUTPUT_INACTIVE);
        if (ret == 0) {
            LOG_INF("CS GPIO configured: port=%s, pin=%d", 
                    cs_gpio_spec.port->name, cs_gpio_spec.pin);
        }
    }
    
    /* Allow device to power up */
    k_msleep(100);
    
    /* Test different SPI modes */
    test_different_modes(spi_dev);
    
    /* Test manual CS control */
    LOG_INF("\n=== Testing manual CS control ===");
    if (gpio_is_ready_dt(&cs_gpio_spec)) {
        /* Pull CS low manually */
        gpio_pin_set_dt(&cs_gpio_spec, 1);  /* Active low, so 1 = inactive */
        k_usleep(10);
        gpio_pin_set_dt(&cs_gpio_spec, 0);  /* Active low, so 0 = active */
        k_usleep(10);
        
        /* Try SPI without automatic CS */
        struct spi_config manual_cfg = {
            .frequency = 1000000,
            .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_CS_ACTIVE_HIGH,
            .cs = {0}  /* No automatic CS */
        };
        
        uint8_t tx_data[2] = {ICM42688_REG_WHO_AM_I | ICM42688_SPI_READ_BIT, 0};
        uint8_t rx_data[2] = {0xFF, 0xFF};
        
        struct spi_buf tx_buf = {.buf = tx_data, .len = 2};
        struct spi_buf rx_buf = {.buf = rx_data, .len = 2};
        struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
        struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
        
        ret = spi_transceive(spi_dev, &manual_cfg, &tx_set, &rx_set);
        
        /* Pull CS high manually */
        k_usleep(10);
        gpio_pin_set_dt(&cs_gpio_spec, 1);  /* Inactive */
        
        if (ret == 0) {
            LOG_INF("Manual CS result: 0x%02X", rx_data[1]);
        }
    }
    
    LOG_INF("\nSPI debug test completed");
    
    return 0;
}