/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - IMU Service Implementation (ICM-42688-V)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include "services/imu.h"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

/* ICM-42688 Register Addresses */
#define ICM42688_REG_WHO_AM_I           0x75
#define ICM42688_REG_DEVICE_CONFIG      0x11
#define ICM42688_REG_PWR_MGMT0          0x4E
#define ICM42688_REG_GYRO_CONFIG0       0x4F
#define ICM42688_REG_ACCEL_CONFIG0      0x50
#define ICM42688_REG_TEMP_DATA1         0x1D
#define ICM42688_REG_ACCEL_DATA_X1      0x1F
#define ICM42688_REG_GYRO_DATA_X1       0x25
#define ICM42688_REG_INT_CONFIG         0x14
#define ICM42688_REG_INT_CONFIG1        0x64
#define ICM42688_REG_INT_SOURCE0        0x65

/* ICM-42688 Constants */
#define ICM42688_WHO_AM_I_VALUE         0x47
#define ICM42688_SPI_READ_BIT           0x80

/* SPI device configuration */
#define IMU_SPI_NODE DT_NODELABEL(icm42688)
#define IMU_SPI_BUS DT_BUS(IMU_SPI_NODE)

/* Device and configuration */
static const struct device *spi_dev;
static struct spi_config spi_cfg = {
    .frequency = 1000000,  /* 1 MHz - conservative for initial comms */
    .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA |
                 SPI_WORD_SET(8) | SPI_LINES_SINGLE,
    .cs = {
        .gpio = GPIO_DT_SPEC_GET(DT_BUS(IMU_SPI_NODE), cs_gpios),
        .delay = 0,
    },
};

/* Alternative SPI configs to try */
static struct spi_config spi_cfg_mode0 = {
    .frequency = 1000000,  
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
    .cs = {
        .gpio = GPIO_DT_SPEC_GET(DT_BUS(IMU_SPI_NODE), cs_gpios),
        .delay = 0,
    },
};

/* GPIO for interrupt - define directly since DT binding doesn't support it */
#define IMU_INT1_GPIO_NODE DT_NODELABEL(gpio1)
#define IMU_INT1_GPIO_PIN 7  /* P1.07 (D6) */
static const struct gpio_dt_spec int1_gpio = {
    .port = DEVICE_DT_GET(IMU_INT1_GPIO_NODE),
    .pin = IMU_INT1_GPIO_PIN,
    .dt_flags = GPIO_ACTIVE_HIGH
};

/* Current configuration */
static imu_config_t current_config = {
    .accel_odr_hz = 400,
    .gyro_odr_hz = 400,
    .accel_range_g = 16,
    .gyro_range_dps = 2000,
    .fifo_enabled = false
};

/* Callback */
static imu_data_callback_t data_callback = NULL;

/* SPI read single register */
static int icm42688_read_reg(uint8_t reg, uint8_t *data)
{
    /* ICM-42688 expects:
     * - First byte: register address with MSB set for read
     * - Second byte: dummy byte for reading data
     */
    uint8_t tx_buf[2] = {reg | ICM42688_SPI_READ_BIT, 0x00};
    uint8_t rx_buf[2] = {0x00, 0x00};
    
    struct spi_buf tx_bufs[] = {
        {
            .buf = tx_buf,
            .len = 2
        }
    };
    
    struct spi_buf rx_bufs[] = {
        {
            .buf = rx_buf,
            .len = 2
        }
    };
    
    struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = 1
    };
    
    struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = 1
    };
    
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d", ret);
        return ret;
    }
    
    /* Data is in the second byte of the response */
    *data = rx_buf[1];
    LOG_DBG("Read reg 0x%02X = 0x%02X (rx[0]=0x%02X)", reg, *data, rx_buf[0]);
    
    return 0;
}

/* SPI write single register */
static int icm42688_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg & ~ICM42688_SPI_READ_BIT, data};
    
    struct spi_buf tx_bufs[] = {
        {
            .buf = tx_buf,
            .len = 2
        }
    };
    
    struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = 1
    };
    
    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
    }
    
    /* Small delay after write */
    k_usleep(50);
    
    return ret;
}

/* Try different SPI configurations */
static int try_spi_configs(uint8_t *who_am_i)
{
    int ret;
    
    /* Try Mode 3 (CPOL=1, CPHA=1) - original config */
    LOG_INF("Trying SPI Mode 3 (CPOL=1, CPHA=1)...");
    ret = icm42688_read_reg(ICM42688_REG_WHO_AM_I, who_am_i);
    if (ret == 0 && *who_am_i == ICM42688_WHO_AM_I_VALUE) {
        LOG_INF("Mode 3 successful!");
        return 0;
    }
    LOG_WRN("Mode 3: WHO_AM_I = 0x%02X", *who_am_i);
    
    /* Try Mode 0 (CPOL=0, CPHA=0) */
    LOG_INF("Trying SPI Mode 0 (CPOL=0, CPHA=0)...");
    spi_cfg = spi_cfg_mode0;
    ret = icm42688_read_reg(ICM42688_REG_WHO_AM_I, who_am_i);
    if (ret == 0 && *who_am_i == ICM42688_WHO_AM_I_VALUE) {
        LOG_INF("Mode 0 successful!");
        return 0;
    }
    LOG_WRN("Mode 0: WHO_AM_I = 0x%02X", *who_am_i);
    
    return -ENODEV;
}

int imu_init(void)
{
    int ret;
    uint8_t who_am_i;
    
    LOG_INF("Starting IMU initialization...");
    
    /* Get SPI device */
    spi_dev = DEVICE_DT_GET(DT_BUS(IMU_SPI_NODE));
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    LOG_INF("SPI device ready");
    
    /* Configure CS GPIO if needed */
    if (spi_cfg.cs.gpio.port) {
        if (!gpio_is_ready_dt(&spi_cfg.cs.gpio)) {
            LOG_ERR("CS GPIO not ready");
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&spi_cfg.cs.gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure CS GPIO: %d", ret);
            return ret;
        }
        LOG_INF("CS GPIO configured");
    }
    
    /* Configure interrupt GPIO */
    if (!gpio_is_ready_dt(&int1_gpio)) {
        LOG_ERR("INT1 GPIO not ready");
        return -ENODEV;
    }
    LOG_INF("INT1 GPIO ready");
    
    ret = gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure INT1 GPIO: %d", ret);
        return ret;
    }
    LOG_INF("INT1 GPIO configured");
    
    /* Power-up delay */
    k_msleep(100);
    
    /* Try a soft reset first */
    LOG_INF("Performing soft reset...");
    ret = icm42688_write_reg(ICM42688_REG_DEVICE_CONFIG, 0x01);
    if (ret < 0) {
        LOG_WRN("Soft reset write failed: %d", ret);
    }
    
    /* Wait for reset to complete - ICM-42688 needs ~1ms */
    k_msleep(10);
    
    /* Read WHO_AM_I with different SPI configurations */
    ret = try_spi_configs(&who_am_i);
    if (ret < 0) {
        LOG_ERR("Failed to communicate with ICM-42688");
        
        /* Additional debug: try raw SPI transaction */
        LOG_INF("Attempting raw SPI debug...");
        uint8_t debug_tx[] = {0xF5, 0x00};  /* WHO_AM_I with read bit */
        uint8_t debug_rx[2] = {0, 0};
        
        struct spi_buf debug_tx_buf = {.buf = debug_tx, .len = 2};
        struct spi_buf debug_rx_buf = {.buf = debug_rx, .len = 2};
        struct spi_buf_set debug_tx_set = {.buffers = &debug_tx_buf, .count = 1};
        struct spi_buf_set debug_rx_set = {.buffers = &debug_rx_buf, .count = 1};
        
        ret = spi_transceive(spi_dev, &spi_cfg, &debug_tx_set, &debug_rx_set);
        LOG_INF("Raw SPI result: ret=%d, rx=[0x%02X, 0x%02X]", ret, debug_rx[0], debug_rx[1]);
        
        return -ENODEV;
    }
    
    LOG_INF("ICM-42688 initialized successfully (WHO_AM_I: 0x%02X)", who_am_i);
    
    /* Configure power management - turn on accelerometer and gyroscope */
    ret = icm42688_write_reg(ICM42688_REG_PWR_MGMT0, 0x0F);  /* Accel + Gyro in LN mode */
    if (ret < 0) {
        LOG_ERR("Failed to configure power management: %d", ret);
        return ret;
    }
    
    /* Small delay for sensors to start */
    k_msleep(1);
    
    LOG_INF("IMU power management configured");
    
    return 0;
}

int imu_configure(const imu_config_t *config)
{
    /* TODO: Implement configuration in next task */
    current_config = *config;
    return 0;
}

void imu_register_callback(imu_data_callback_t callback)
{
    data_callback = callback;
}

int imu_start(void)
{
    /* TODO: Implement in next task */
    return 0;
}

int imu_stop(void)
{
    /* TODO: Implement in next task */
    return 0;
}

int imu_get_config(imu_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }
    
    *config = current_config;
    return 0;
}