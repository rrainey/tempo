/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Main Application Entry
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>

#include "app_init.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    int ret;
    
    printk("boot\n");
    LOG_INF("hello");
    LOG_INF("Tempo-BT V1 started successfully");
    
    /* Check if QSPI flash device is ready */
    const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25r64));
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("QSPI flash device not ready");
    } else {
        LOG_INF("QSPI flash device is ready");
        
        /* Get flash parameters */
        const struct flash_parameters *flash_params = flash_get_parameters(flash_dev);
        LOG_INF("Flash write block size: %d", flash_params->write_block_size);
        
        /* Get flash size */
        uint64_t flash_size;
        int res = flash_get_size(flash_dev, &flash_size);
        if (res == 0) {
            LOG_INF("Flash size: %llu bytes", flash_size);
        }
        
        /* Try to get page info */
        struct flash_pages_info info;
        ret = flash_get_page_info_by_offs(flash_dev, 0, &info);
        if (ret == 0) {
            LOG_INF("Flash page size: %d, start offset: 0x%x", 
                    info.size, (unsigned int)info.start_offset);
        }
    }
    
    /* Initialize storage */
    ret = app_storage_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize storage: %d", ret);
        /* Continue anyway for now */
    }
    
    /* Log periodic heartbeat */
    while (1) {
        k_sleep(K_SECONDS(5));
        LOG_DBG("Heartbeat - system running");
    }
    
    return 0;
}