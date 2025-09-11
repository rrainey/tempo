/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Custom mcumgr Handlers
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
//#include <zephyr/mgmt/mcumgr/util/zcbor_bulk.h>

#include <zcbor_encode.h>
#include <zcbor_decode.h>

#include "services/storage.h"
#include "services/logger.h"

LOG_MODULE_REGISTER(mcumgr_custom, LOG_LEVEL_INF);

/* Custom group ID */
#define MGMT_GROUP_ID_TEMPO    64

/* Command IDs */
#define TEMPO_MGMT_ID_SESSION_LIST     0
#define TEMPO_MGMT_ID_SESSION_INFO     1
#define TEMPO_MGMT_ID_STORAGE_INFO     2
#define TEMPO_MGMT_ID_LED_CONTROL      3

/* List callback context */
struct list_context {
    zcbor_state_t *zse;
    int count;
    bool error;
};


static int list_callback(const char *path, bool is_dir, size_t size, void *ctx)
{
    struct list_context *context = (struct list_context *)ctx;

    if (context->error) {
        return -1;  /* Stop on error */
    }

    if (is_dir) {
        /* Extract relative path */
        const char *rel_path = path;
        if (strncmp(path, "/lfs/logs/", 10) == 0) {
            rel_path = path + 10;
        }

        /* Start session object */
        bool ok = zcbor_map_start_encode(context->zse, 3) &&
                  zcbor_tstr_put_lit(context->zse, "name") &&
                  zcbor_tstr_put_term(context->zse, rel_path, 10) &&
                  zcbor_tstr_put_lit(context->zse, "is_dir") &&
                  zcbor_bool_put(context->zse, true) &&
                  zcbor_tstr_put_lit(context->zse, "size") &&
                  zcbor_uint32_put(context->zse, (uint32_t)size) &&
                  zcbor_map_end_encode(context->zse, 3);

        if (!ok) {
            context->error = true;
            return -1;  /* Stop on error */
        }

        context->count++;
    }

    return 0;  /* Continue */
}

/* List logging sessions */
static int tempo_mgmt_session_list(struct smp_streamer *ctxt)
{
    zcbor_state_t *zse = ctxt->writer->zs;
    struct list_context list_ctx;
    bool ok;

    /* Start map */
    ok = zcbor_tstr_put_lit(zse, "sessions") &&
         zcbor_list_start_encode(zse, 20);
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }

    /* Initialize list context */
    list_ctx.zse = zse;
    list_ctx.count = 0;
    list_ctx.error = false;

    /* List sessions from storage */
    storage_list_dir("/lfs/logs", list_callback, &list_ctx);

    if (list_ctx.error) {
        return MGMT_ERR_EMSGSIZE;
    }

    /* End sessions array and add count */
    ok = zcbor_list_end_encode(zse, 20) &&
         zcbor_tstr_put_lit(zse, "count") &&
         zcbor_int32_put(zse, list_ctx.count);

    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }

    return 0;
}

/* Get storage info */
static int tempo_mgmt_storage_info(struct smp_streamer *ctxt)
{
    zcbor_state_t *zse = ctxt->writer->zs;
    uint64_t free_bytes, total_bytes;
    int ret;

    ret = storage_get_free_space(&free_bytes, &total_bytes);
    if (ret != 0) {
        return MGMT_ERR_EUNKNOWN;
    }

    uint32_t used_percent = (uint32_t)(100 - (free_bytes * 100 / total_bytes));

    bool ok = zcbor_tstr_put_lit(zse, "free_bytes") &&
              zcbor_uint64_put(zse, free_bytes) &&
              zcbor_tstr_put_lit(zse, "total_bytes") &&
              zcbor_uint64_put(zse, total_bytes) &&
              zcbor_tstr_put_lit(zse, "used_percent") &&
              zcbor_uint32_put(zse, used_percent);

    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }

    return 0;
}

/* Add these to your existing mcumgr_custom.c file */

#include "services/led.h"

/* Add new command ID after the existing ones */
#define TEMPO_MGMT_ID_LED_CONTROL      3

/* LED control command handler */
static int tempo_mgmt_led_control(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;  /* Decoder for request */
    zcbor_state_t *zse = ctxt->writer->zs;  /* Encoder for response */
    
    bool ok;
    bool enable = false;
    uint32_t r = 0, g = 0, b = 0;
    bool has_enable = false;
    bool has_r = false, has_g = false, has_b = false;
    struct zcbor_string key;
    size_t decoded;
    
    /* Start decoding the map */
    ok = zcbor_map_start_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to start decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    /* Decode each key-value pair */
    while (zcbor_tstr_decode(zsd, &key)) {
        if (key.len == 6 && memcmp(key.value, "enable", 6) == 0) {
            ok = zcbor_bool_decode(zsd, &enable);
            if (ok) has_enable = true;
        }
        else if (key.len == 1 && key.value[0] == 'r') {
            ok = zcbor_uint32_decode(zsd, &r);
            if (ok) has_r = true;
        }
        else if (key.len == 1 && key.value[0] == 'g') {
            ok = zcbor_uint32_decode(zsd, &g);
            if (ok) has_g = true;
        }
        else if (key.len == 1 && key.value[0] == 'b') {
            ok = zcbor_uint32_decode(zsd, &b);
            if (ok) has_b = true;
        }
        else {
            /* Skip unknown keys */
            ok = zcbor_any_skip(zsd, NULL);
        }
        
        if (!ok) {
            LOG_ERR("Failed to decode value");
            return MGMT_ERR_EINVAL;
        }
    }
    
    ok = zcbor_map_end_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to end decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    LOG_INF("LED control: enable=%d, R=%d G=%d B=%d", 
            enable, (uint8_t)r, (uint8_t)g, (uint8_t)b);
    
    /* Validate color values */
    if (r > 255 || g > 255 || b > 255) {
        LOG_ERR("Invalid color values");
        return MGMT_ERR_EINVAL;
    }
    
    /* Apply the command */
    rgb_color_t color = {
        .r = (uint8_t)r,
        .g = (uint8_t)g,
        .b = (uint8_t)b
    };
    
    int ret = led_service_set_override(color, enable);
    if (ret != 0) {
        return MGMT_ERR_EUNKNOWN;
    }
    
    /* Build response with current state */
    rgb_color_t current_color;
    bool override_enabled;
    led_service_get_override(&current_color, &override_enabled);
    
    ok = zcbor_tstr_put_lit(zse, "enabled") &&
         zcbor_bool_put(zse, override_enabled) &&
         zcbor_tstr_put_lit(zse, "r") &&
         zcbor_uint32_put(zse, current_color.r) &&
         zcbor_tstr_put_lit(zse, "g") &&
         zcbor_uint32_put(zse, current_color.g) &&
         zcbor_tstr_put_lit(zse, "b") &&
         zcbor_uint32_put(zse, current_color.b);
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }
    
    return 0;
}

/* Command handlers table */
static const struct mgmt_handler tempo_mgmt_handlers[] = {
    [TEMPO_MGMT_ID_SESSION_LIST] = {
        .mh_read = tempo_mgmt_session_list,
        .mh_write = NULL,
    },
    [TEMPO_MGMT_ID_SESSION_INFO] = {
        .mh_read = NULL,  /* TODO: Implement */
        .mh_write = NULL,
    },
    [TEMPO_MGMT_ID_STORAGE_INFO] = {
        .mh_read = tempo_mgmt_storage_info,
        .mh_write = NULL,
    },
    [TEMPO_MGMT_ID_LED_CONTROL] = {
        .mh_read = NULL,
        .mh_write = tempo_mgmt_led_control,  /* Write handler for control */
    },
};

static struct mgmt_group tempo_mgmt_group = {
    .mg_handlers = tempo_mgmt_handlers,
    .mg_handlers_count = ARRAY_SIZE(tempo_mgmt_handlers),
    .mg_group_id = MGMT_GROUP_ID_TEMPO,
};

/* Register custom handlers */
void tempo_mgmt_register(void)
{
    mgmt_register_group(&tempo_mgmt_group);
    LOG_INF("Tempo custom mcumgr handlers registered");
}