/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Barometer Service Implementation (BMP390)
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

#include "services/baro.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(baro, LOG_LEVEL_DBG);

/* BMP390 I2C Address */
#define BMP390_I2C_ADDR         0x76

/* BMP390 Register Addresses */
#define BMP390_REG_CHIP_ID      0x00
#define BMP390_REG_ERR_REG      0x02
#define BMP390_REG_STATUS       0x03
#define BMP390_REG_DATA         0x04  /* Pressure and temperature data */
#define BMP390_REG_EVENT        0x10
#define BMP390_REG_INT_STATUS   0x11
#define BMP390_REG_FIFO_LENGTH  0x12
#define BMP390_REG_FIFO_DATA    0x14
#define BMP390_REG_FIFO_WM      0x15
#define BMP390_REG_FIFO_CONFIG  0x17
#define BMP390_REG_INT_CTRL     0x19
#define BMP390_REG_IF_CONF      0x1A
#define BMP390_REG_PWR_CTRL     0x1B
#define BMP390_REG_OSR          0x1C  /* Oversampling */
#define BMP390_REG_ODR          0x1D  /* Output data rate */
#define BMP390_REG_CONFIG       0x1F
#define BMP390_REG_CALIB_DATA   0x31  /* Calibration data start */
#define BMP390_REG_CMD          0x7E

/* BMP390 Commands */
#define BMP390_CMD_SOFT_RESET   0xB6
#define BMP390_CMD_FIFO_FLUSH   0xB0

/* BMP390 Constants */
#define BMP390_CHIP_ID          0x60

/* Power control bits */
#define BMP390_PWR_PRESS_EN     BIT(0)
#define BMP390_PWR_TEMP_EN      BIT(1)
#define BMP390_PWR_MODE_POS     4
#define BMP390_PWR_MODE_MASK    (0x3 << BMP390_PWR_MODE_POS)
#define BMP390_PWR_MODE_SLEEP   (0x0 << BMP390_PWR_MODE_POS)
#define BMP390_PWR_MODE_FORCED  (0x1 << BMP390_PWR_MODE_POS)
#define BMP390_PWR_MODE_NORMAL  (0x3 << BMP390_PWR_MODE_POS)

/* Status bits */
#define BMP390_STATUS_CMD_RDY   BIT(4)
#define BMP390_STATUS_DRDY_PRESS BIT(5)
#define BMP390_STATUS_DRDY_TEMP BIT(6)

/* Interrupt control bits */
#define BMP390_INT_CTRL_DRDY_EN BIT(6)
#define BMP390_INT_CTRL_LEVEL   BIT(1)
#define BMP390_INT_CTRL_OD      BIT(0)

/* I2C device */
static const struct device *i2c_dev;

/* GPIO for interrupt */
static const struct gpio_dt_spec int_gpio = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 11,  /* P1.11 (D11) */
    .dt_flags = GPIO_ACTIVE_HIGH
};

/* Calibration data structure */
struct bmp390_calib_data {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t   par_t3;
    int16_t  par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    int8_t   par_p10;
    int8_t   par_p11;
} __packed;

static struct bmp390_calib_data calib_data;

/* Current configuration */
static baro_config_t current_config = {
    .odr_hz = 4,  /* 4 Hz default */
    .pressure_oversampling = 8,
    .temperature_oversampling = 1,
    .iir_filter_coeff = 3,
    .enable_data_ready_int = true
};

/* Sea level pressure for altitude calculation */
static float sea_level_pressure_pa = 101325.0f;  /* Standard atmosphere */

/* Callback and work items */
static baro_data_callback_t data_callback = NULL;
static struct gpio_callback int_cb_data;
static struct k_work int_work;
static struct k_work_delayable sample_work;

/* Data ready flag */
static volatile bool data_ready = false;

/* Thread for barometer sampling */
K_THREAD_STACK_DEFINE(baro_thread_stack, 2048);
static struct k_thread baro_thread_data;
static k_tid_t baro_thread_id = NULL;
static struct k_sem baro_sem;

/* I2C read register */
static int bmp390_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_burst_read(i2c_dev, BMP390_I2C_ADDR, reg, data, len);
}

/* I2C write register */
static int bmp390_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_write(i2c_dev, buf, sizeof(buf), BMP390_I2C_ADDR);
}

/* BMP390 Compensation - Based on Bosch BMP3 driver */

/* Calculate temperature compensation */
static void calc_temp_comp(uint32_t uncomp_temp, float *temp, struct bmp390_calib_data *calib)
{
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - (256 * calib->par_t1));
    partial_data2 = calib->par_t2 * partial_data1;
    partial_data2 = partial_data2 + (partial_data1 * partial_data1) * calib->par_t3;
    *temp = partial_data2 / 5120.0f;
}

/* Calculate pressure compensation */
static void calc_press_comp(uint32_t uncomp_press, float temp, float *pressure, struct bmp390_calib_data *calib)
{
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    partial_data1 = temp * temp;
    partial_data2 = partial_data1 / 64.0f;
    partial_data3 = (partial_data2 * temp) / 256.0f;
    partial_data1 = (calib->par_p8 * partial_data3) / 32.0f;
    partial_data2 = (calib->par_p7 * partial_data2) * 16.0f;
    partial_data3 = (calib->par_p6 * temp) * 4194304.0f;
    partial_data4 = (calib->par_p5) * 140737488355328.0f;
    partial_out1 = partial_data1 + partial_data2 + partial_data3 + partial_data4;

    partial_data1 = (calib->par_p4 * partial_data3) / 32.0f;
    partial_data2 = (calib->par_p3 * partial_data2) * 4.0f;
    partial_data3 = ((float)calib->par_p2 - 16384.0f) * temp * 2097152.0f;
    partial_data4 = ((float)calib->par_p1 - 16384.0f) * 70368744177664.0f;
    partial_out2 = uncomp_press * (partial_data1 + partial_data2 + partial_data3 + partial_data4);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = (calib->par_p11 * partial_data1) / 65536.0f;
    partial_data3 = (partial_data2 * uncomp_press) / 128.0f;
    partial_data4 = (partial_out1 / 4.0f) + partial_data3;
    partial_data1 = ((calib->par_p10 * partial_data1) * uncomp_press) / 8192.0f;
    partial_data1 = ((partial_data1 / 48.0f) + (partial_out2 / 8192.0f)) / 524288.0f;

    *pressure = partial_data1 + partial_data4;
}

/* Calculate altitude from pressure */
static float calculate_altitude(float pressure_pa)
{
    /* Barometric formula */
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pressure_pa, 0.1903f));
}

/* Read sensor data */
static int read_sensor_data(baro_sample_t *sample)
{
    uint8_t data[6];
    int ret;
    
    /* Read pressure and temperature data */
    ret = bmp390_read_reg(BMP390_REG_DATA, data, sizeof(data));
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }
    
    /* Extract raw values (LSB first) */
    uint32_t raw_press = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    uint32_t raw_temp = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);
    
    /* Apply compensation */
    float temp_c;
    float press_pa;
    
    calc_temp_comp(raw_temp, &temp_c, &calib_data);
    calc_press_comp(raw_press, temp_c, &press_pa, &calib_data);
    
    sample->temperature_c = temp_c;
    sample->pressure_pa = press_pa;
    sample->altitude_m = calculate_altitude(press_pa);
    
    sample->temperature_valid = true;
    sample->pressure_valid = true;
    sample->timestamp_us = time_now_us();
    
    LOG_DBG("Raw: P=%u, T=%u", raw_press, raw_temp);
    LOG_DBG("Compensated: P=%.2f Pa, T=%.2f C, Alt=%.2f m", 
            sample->pressure_pa, sample->temperature_c, sample->altitude_m);
    
    return 0;
}

/* Interrupt handler */
static void bmp390_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    k_work_submit(&int_work);
}

/* Interrupt work handler */
static void int_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* Signal the sampling thread */
    data_ready = true;
    k_sem_give(&baro_sem);
}

/* Periodic sampling work */
static void sample_work_handler(struct k_work *work)
{
    /* Trigger a measurement in forced mode */
    data_ready = true;
    k_sem_give(&baro_sem);
    
    /* Reschedule based on ODR */
    if (current_config.odr_hz > 0) {
        k_work_reschedule(&sample_work, K_MSEC(1000 / current_config.odr_hz));
    }
}

/* Barometer sampling thread */
static void baro_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    baro_sample_t sample;
    
    LOG_INF("Barometer thread started");
    
    while (1) {
        /* Wait for data ready signal */
        k_sem_take(&baro_sem, K_FOREVER);
        
        if (!data_ready) {
            continue;
        }
        
        data_ready = false;
        
        /* Read sensor data */
        if (read_sensor_data(&sample) == 0) {
            /* Call the callback if registered */
            if (data_callback) {
                data_callback(&sample);
            }
        }
    }
}

int baro_init(void)
{
    int ret;
    uint8_t chip_id;
    
    LOG_INF("Initializing BMP390 barometer...");
    
    /* Get I2C device */
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    /* Configure interrupt GPIO if available */
    if (gpio_is_ready_dt(&int_gpio)) {
        ret = gpio_pin_configure_dt(&int_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT GPIO: %d", ret);
            return ret;
        }
        
        /* Configure interrupt */
        gpio_init_callback(&int_cb_data, bmp390_int_handler, BIT(int_gpio.pin));
        ret = gpio_add_callback(int_gpio.port, &int_cb_data);
        if (ret < 0) {
            LOG_ERR("Failed to add GPIO callback: %d", ret);
            return ret;
        }
        
        ret = gpio_pin_interrupt_configure_dt(&int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
            return ret;
        }
        
        LOG_INF("Interrupt GPIO configured on P1.11");
    }
    
    /* Initialize work items */
    k_work_init(&int_work, int_work_handler);
    k_work_init_delayable(&sample_work, sample_work_handler);
    k_sem_init(&baro_sem, 0, 1);
    
    /* Soft reset */
    ret = bmp390_write_reg(BMP390_REG_CMD, BMP390_CMD_SOFT_RESET);
    if (ret < 0) {
        LOG_ERR("Failed to reset BMP390: %d", ret);
        return ret;
    }
    
    /* Wait for reset to complete */
    k_msleep(10);
    
    /* Read and verify chip ID */
    ret = bmp390_read_reg(BMP390_REG_CHIP_ID, &chip_id, 1);
    if (ret < 0) {
        LOG_ERR("Failed to read chip ID: %d", ret);
        return ret;
    }
    
    if (chip_id != BMP390_CHIP_ID) {
        LOG_ERR("Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BMP390_CHIP_ID);
        return -ENODEV;
    }
    
    LOG_INF("BMP390 detected (ID: 0x%02X)", chip_id);
    
    /* Read calibration data */
    ret = bmp390_read_reg(BMP390_REG_CALIB_DATA, (uint8_t *)&calib_data, sizeof(calib_data));
    if (ret < 0) {
        LOG_ERR("Failed to read calibration data: %d", ret);
        return ret;
    }
    
    LOG_INF("Calibration data loaded");
    LOG_DBG("Calib T: T1=%u, T2=%u, T3=%d", calib_data.par_t1, calib_data.par_t2, calib_data.par_t3);
    LOG_DBG("Calib P: P1=%d, P2=%d, P3=%d, P4=%d", 
            calib_data.par_p1, calib_data.par_p2, calib_data.par_p3, calib_data.par_p4);
    LOG_DBG("Calib P: P5=%u, P6=%u, P7=%d, P8=%d", 
            calib_data.par_p5, calib_data.par_p6, calib_data.par_p7, calib_data.par_p8);
    LOG_DBG("Calib P: P9=%d, P10=%d, P11=%d", 
            calib_data.par_p9, calib_data.par_p10, calib_data.par_p11);
    
    /* Sanity check calibration data */
    if (calib_data.par_t1 == 0 || calib_data.par_t1 == 0xFFFF) {
        LOG_ERR("Invalid calibration data - T1=%u", calib_data.par_t1);
        return -EINVAL;
    }
    
    /* Configure sensor with defaults */
    ret = baro_configure(&current_config);
    if (ret < 0) {
        LOG_ERR("Failed to configure sensor: %d", ret);
        return ret;
    }
    
    /* Create sampling thread */
    baro_thread_id = k_thread_create(&baro_thread_data, baro_thread_stack,
                                     K_THREAD_STACK_SIZEOF(baro_thread_stack),
                                     baro_thread, NULL, NULL, NULL,
                                     CONFIG_MAIN_THREAD_PRIORITY + 1, 0, K_NO_WAIT);
    k_thread_name_set(baro_thread_id, "baro");
    
    LOG_INF("BMP390 initialized successfully");
    
    return 0;
}

int baro_configure(const baro_config_t *config)
{
    int ret;
    uint8_t osr_reg = 0;
    uint8_t odr_reg = 0;
    uint8_t config_reg = 0;
    uint8_t int_ctrl_reg = 0;
    
    if (!config) {
        return -EINVAL;
    }
    
    /* Configure oversampling */
    switch (config->pressure_oversampling) {
        case 1:  osr_reg |= (0 << 0); break;
        case 2:  osr_reg |= (1 << 0); break;
        case 4:  osr_reg |= (2 << 0); break;
        case 8:  osr_reg |= (3 << 0); break;
        case 16: osr_reg |= (4 << 0); break;
        case 32: osr_reg |= (5 << 0); break;
        default: return -EINVAL;
    }
    
    switch (config->temperature_oversampling) {
        case 1:  osr_reg |= (0 << 3); break;
        case 2:  osr_reg |= (1 << 3); break;
        case 4:  osr_reg |= (2 << 3); break;
        case 8:  osr_reg |= (3 << 3); break;
        case 16: osr_reg |= (4 << 3); break;
        case 32: osr_reg |= (5 << 3); break;
        default: return -EINVAL;
    }
    
    /* Configure ODR */
    if (config->odr_hz <= 1) {
        odr_reg = 0x00;  /* 200 Hz / 200 = 1 Hz */
    } else if (config->odr_hz <= 2) {
        odr_reg = 0x01;  /* 200 Hz / 100 = 2 Hz */
    } else if (config->odr_hz <= 4) {
        odr_reg = 0x02;  /* 200 Hz / 50 = 4 Hz */
    } else if (config->odr_hz <= 8) {
        odr_reg = 0x03;  /* 200 Hz / 25 = 8 Hz */
    } else {
        odr_reg = 0x04;  /* 200 Hz / 12.5 = 16 Hz */
    }
    
    /* Configure IIR filter */
    switch (config->iir_filter_coeff) {
        case 0:   config_reg = (0 << 1); break;
        case 1:   config_reg = (1 << 1); break;
        case 3:   config_reg = (2 << 1); break;
        case 7:   config_reg = (3 << 1); break;
        case 15:  config_reg = (4 << 1); break;
        case 31:  config_reg = (5 << 1); break;
        case 63:  config_reg = (6 << 1); break;
        case 127: config_reg = (7 << 1); break;
        default:  return -EINVAL;
    }
    
    /* Configure interrupt */
    if (config->enable_data_ready_int) {
        int_ctrl_reg = BMP390_INT_CTRL_DRDY_EN | BMP390_INT_CTRL_LEVEL;
    }
    
    /* Write configuration registers */
    ret = bmp390_write_reg(BMP390_REG_OSR, osr_reg);
    if (ret < 0) return ret;
    
    ret = bmp390_write_reg(BMP390_REG_ODR, odr_reg);
    if (ret < 0) return ret;
    
    ret = bmp390_write_reg(BMP390_REG_CONFIG, config_reg);
    if (ret < 0) return ret;
    
    ret = bmp390_write_reg(BMP390_REG_INT_CTRL, int_ctrl_reg);
    if (ret < 0) return ret;
    
    /* Update current configuration */
    current_config = *config;
    
    LOG_INF("BMP390 configured: ODR=%d Hz, P_OSR=%d, T_OSR=%d, IIR=%d",
            config->odr_hz, config->pressure_oversampling,
            config->temperature_oversampling, config->iir_filter_coeff);
    
    return 0;
}

void baro_register_callback(baro_data_callback_t callback)
{
    data_callback = callback;
}

int baro_start(void)
{
    int ret;
    
    /* Enable pressure and temperature measurement in normal mode */
    ret = bmp390_write_reg(BMP390_REG_PWR_CTRL,
                          BMP390_PWR_PRESS_EN | BMP390_PWR_TEMP_EN | BMP390_PWR_MODE_NORMAL);
    if (ret < 0) {
        LOG_ERR("Failed to start measurements: %d", ret);
        return ret;
    }
    
    /* Start periodic sampling if no interrupt */
    if (!current_config.enable_data_ready_int || !gpio_is_ready_dt(&int_gpio)) {
        k_work_reschedule(&sample_work, K_MSEC(1000 / current_config.odr_hz));
    }
    
    LOG_INF("BMP390 measurements started");
    
    return 0;
}

int baro_stop(void)
{
    int ret;
    
    /* Cancel periodic work */
    k_work_cancel_delayable(&sample_work);
    
    /* Put sensor in sleep mode */
    ret = bmp390_write_reg(BMP390_REG_PWR_CTRL, BMP390_PWR_MODE_SLEEP);
    if (ret < 0) {
        LOG_ERR("Failed to stop measurements: %d", ret);
        return ret;
    }
    
    LOG_INF("BMP390 measurements stopped");
    
    return 0;
}

int baro_get_config(baro_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }
    
    *config = current_config;
    return 0;
}

void baro_set_sea_level_pressure(float pressure_pa)
{
    sea_level_pressure_pa = pressure_pa;
    LOG_INF("Sea level pressure set to %.2f Pa", pressure_pa);
}