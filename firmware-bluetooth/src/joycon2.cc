#include "joycon2.h"
#include <bluetooth/gatt_dm.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(joycon2, LOG_LEVEL_DBG);

// Joy-Con 2 connection management
#define MAX_JOYCON2 2
struct joycon2_connection joycon2_conns[MAX_JOYCON2];
int joycon2_count = 0;

// Subscribe params must stay valid for the connection lifetime
static struct bt_gatt_subscribe_params joycon2_sub_params[MAX_JOYCON2];

static void joycon2_notify_cb(struct bt_conn* conn, struct bt_gatt_subscribe_params* params,
                              const void* data, uint16_t len) {
    if (!data || len == 0) return;
    uint8_t conn_idx = bt_conn_index(conn);
    uint16_t interface = conn_idx << 8;
    const uint8_t* buf = (const uint8_t*)data;
    joycon2_decode_and_handle_report(buf, len, interface);
}

// Device info storage per connection
struct joycon2_device_info joycon2_device_info_per_conn[CONFIG_BT_MAX_CONN] = {0};

// Define report_type here since it's needed for the message queue
struct report_type {
    uint16_t interface;
    uint8_t len;
    uint8_t data[65];
};

// Forward declarations for report handling
extern struct k_msgq report_q;

// Helper function to get 16-bit little-endian value
static inline uint16_t get_le16(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

// Callback for bt_data_parse - checks for Nintendo manufacturer ID (0x057E)
static bool manufacturer_data_cb(struct bt_data* data, void* user_data) {
    bool* found = (bool*)user_data;
    if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 2) {
        uint16_t company_id = sys_get_le16(data->data);
        if (company_id == JOYCON2_MANUFACTURER_ID) {
            *found = true;
            return false;  // Stop parsing
        }
    }
    return true;  // Continue parsing
}

bool joycon2_is_device(const struct bt_scan_device_info* device_info) {
    if (!device_info || !device_info->adv_data) {
        return false;
    }
    // Parse advertisement data using bt_data_parse (adv_data is struct net_buf_simple*)
    bool found_nintendo = false;
    bt_data_parse(device_info->adv_data, manufacturer_data_cb, &found_nintendo);
    return found_nintendo;
}

void joycon2_store_device_info(const struct bt_scan_device_info* device_info, uint8_t conn_idx) {
    if (conn_idx < CONFIG_BT_MAX_CONN) {
        // Store a pointer to the device info (const cast needed for compatibility)
        joycon2_device_info_per_conn[conn_idx].device_info = (struct bt_scan_device_info*)device_info;
        joycon2_device_info_per_conn[conn_idx].is_joycon2 = true;
        LOG_INF("Joy-Con 2 device info stored for connection %d", conn_idx);
    }
}

bool joycon2_is_connection(uint8_t conn_idx) {
    return (conn_idx < CONFIG_BT_MAX_CONN && joycon2_device_info_per_conn[conn_idx].is_joycon2);
}

void joycon2_discover_characteristics(struct bt_conn* conn, void* dm) {
    LOG_INF("Joy-Con 2 discovery characteristics called");
    
    if (joycon2_count < MAX_JOYCON2) {
        joycon2_conns[joycon2_count].conn = conn;
        joycon2_conns[joycon2_count].input_handle = 0;
        joycon2_conns[joycon2_count].write_handle = 0;
        joycon2_conns[joycon2_count].sensors_enabled = false;
        
        // Parse GATT discovery data to find Joy-Con characteristics
        const struct bt_gatt_dm* gatt_dm = (const struct bt_gatt_dm*)dm;
        const struct bt_gatt_dm_attr* attr = NULL;
        while (NULL != (attr = bt_gatt_dm_attr_next(gatt_dm, attr))) {
            if (attr->uuid->type == BT_UUID_TYPE_128) {
                const struct bt_uuid_128* uuid = (const struct bt_uuid_128*)attr->uuid;
                
                // Check for input report characteristic
                if (memcmp(uuid->val, JOYCON2_INPUT_REPORT_UUID_VAL, 16) == 0) {
                    joycon2_conns[joycon2_count].input_handle = attr->handle;
                    LOG_INF("Found Joy-Con input report characteristic at handle: 0x%04x", attr->handle);
                }
                
                // Check for write command characteristic
                if (memcmp(uuid->val, JOYCON2_WRITE_COMMAND_UUID_VAL, 16) == 0) {
                    joycon2_conns[joycon2_count].write_handle = attr->handle;
                    LOG_INF("Found Joy-Con write command characteristic at handle: 0x%04x", attr->handle);
                }
            }
        }
        
        joycon2_count++;
        LOG_INF("Joy-Con 2 connection added with input_handle: 0x%04x, write_handle: 0x%04x", 
                joycon2_conns[joycon2_count-1].input_handle, 
                joycon2_conns[joycon2_count-1].write_handle);
    }
}

void joycon2_enable_sensors(struct bt_conn* conn, uint16_t write_handle) {
    static const uint8_t enable_imu_cmd1[] = { 0x0c, 0x91, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };
    static const uint8_t enable_imu_cmd2[] = { 0x0c, 0x91, 0x01, 0x04, 0x00, 0x04, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };
    int err;

    err = bt_gatt_write_without_response(conn, write_handle, enable_imu_cmd1, sizeof(enable_imu_cmd1));
    if (err) {
        LOG_ERR("Failed to send Joy-Con 2 IMU enable cmd1: %d", err);
    } else {
        LOG_INF("Sent Joy-Con 2 IMU enable cmd1");
    }
    
    k_sleep(K_MSEC(100));
    
    err = bt_gatt_write_without_response(conn, write_handle, enable_imu_cmd2, sizeof(enable_imu_cmd2));
    if (err) {
        LOG_ERR("Failed to send Joy-Con 2 IMU enable cmd2: %d", err);
    } else {
        LOG_INF("Sent Joy-Con 2 IMU enable cmd2");
    }
    
    // Mark sensors as enabled for this connection
    for (int i = 0; i < joycon2_count; i++) {
        if (joycon2_conns[i].conn == conn) {
            joycon2_conns[i].sensors_enabled = true;
            break;
        }
    }
}

// Convert Joy-Con format to standard HID gamepad report for XAC compatibility
// Joy-Con/Nintendo report layout: [report_id], buttons[2-3], stick data...
// Output: report_id 1, 2 bytes buttons, 4 bytes axes - minimal gamepad
static void joycon2_to_standard_gamepad(const uint8_t* in, uint8_t in_len,
                                         uint8_t* out, uint8_t* out_len) {
    *out_len = 0;
    if (in_len < 2) return;
    out[0] = 0x01;  // Report ID 1 - standard gamepad
    uint16_t buttons = 0;
    // Buttons: byte 0 is often report_id, bytes 1-2 are buttons (adapt to format)
    if (in_len >= 3) {
        buttons = (uint16_t)in[1] | ((uint16_t)in[2] << 8);
    } else {
        buttons = in[1];
    }
    out[1] = buttons & 0xFF;
    out[2] = (buttons >> 8) & 0xFF;
    uint8_t lx = 0x80, ly = 0x80;
    if (in_len >= 6) {
        lx = in[4];
        ly = in[5];
    } else if (in_len >= 5) {
        lx = in[3];
        ly = in[4];
    }
    out[3] = lx;
    out[4] = ly;
    out[5] = 0x80;  // RX center
    out[6] = 0x80;  // RY center
    *out_len = 7;
}

void joycon2_decode_and_handle_report(const uint8_t* data, uint8_t len, uint16_t interface) {
    if (len < 2) {
        LOG_ERR("[Joy-Con 2] Report too short: %d", len);
        return;
    }

    struct report_type decoded_report;
    decoded_report.interface = interface;
    uint8_t out_len;

    joycon2_to_standard_gamepad(data, len, decoded_report.data, &out_len);
    if (out_len == 0) return;  // Decode failed, skip
    decoded_report.len = out_len;

    if (k_msgq_put(&report_q, &decoded_report, K_NO_WAIT)) {
        LOG_ERR("[Joy-Con 2] Failed to queue report");
    }
}

void joycon2_subscribe_to_input(struct bt_conn* conn, uint16_t input_handle) {
    struct joycon2_connection* jc = joycon2_find_connection(conn);
    if (!jc || input_handle == 0) return;
    int idx = jc - joycon2_conns;
    if (idx < 0 || idx >= MAX_JOYCON2) return;

    struct bt_gatt_subscribe_params* params = &joycon2_sub_params[idx];
    memset(params, 0, sizeof(*params));
    params->value_handle = input_handle;
    params->ccc_handle = input_handle + 1;  // CCC typically follows value
    params->value = BT_GATT_CCC_NOTIFY;
    params->notify = joycon2_notify_cb;

    int err = bt_gatt_subscribe(conn, params);
    if (err) {
        LOG_ERR("Joy-Con 2 subscribe failed: %d (try ccc_handle %u)", err, params->ccc_handle);
    } else {
        LOG_INF("Joy-Con 2 subscribed to input notifications on handle 0x%04x", input_handle);
    }
}

// Joy-Con 2 service UUID for GATT discovery
static struct bt_uuid_128 joycon2_svc_uuid = BT_UUID_INIT_128(
    0xd2, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82,
    0x49, 0xad, 0x89, 0xfe, 0xab, 0x7d, 0xe9, 0xbe);

// Weak stub - main.cc overrides with strong symbol to reschedule scan
__attribute__((weak)) void joycon2_notify_scan_restart(void) {}

static void joycon2_discovery_completed(struct bt_gatt_dm* dm, void* context) {
    struct bt_conn* conn = (struct bt_conn*)context;
    LOG_INF("Joy-Con 2 GATT discovery completed");
    joycon2_discover_characteristics(conn, dm);
    struct joycon2_connection* jc = joycon2_find_connection(conn);
    if (jc) {
        if (jc->write_handle != 0) {
            joycon2_enable_sensors(conn, jc->write_handle);
        }
        if (jc->input_handle != 0) {
            joycon2_subscribe_to_input(conn, jc->input_handle);
        }
    }
    bt_gatt_dm_data_release(dm);
    joycon2_notify_scan_restart();
}

static void joycon2_discovery_service_not_found(struct bt_conn* conn, void* context) {
    LOG_ERR("Joy-Con 2 service not found");
    joycon2_notify_scan_restart();
}

static void joycon2_discovery_error(struct bt_conn* conn, int err, void* context) {
    LOG_ERR("Joy-Con 2 discovery error: %d", err);
    joycon2_notify_scan_restart();
}

static const struct bt_gatt_dm_cb joycon2_discovery_cb = {
    .completed = joycon2_discovery_completed,
    .service_not_found = joycon2_discovery_service_not_found,
    .error_found = joycon2_discovery_error,
};

void joycon2_gatt_discover(struct bt_conn* conn) {
    LOG_INF("Starting Joy-Con 2 GATT discovery");
    int err = bt_gatt_dm_start(conn, (struct bt_uuid*)&joycon2_svc_uuid,
                               &joycon2_discovery_cb, conn);
    if (err) {
        LOG_ERR("Joy-Con 2 GATT discovery start failed: %d", err);
    }
}

void joycon2_cleanup_connection(uint8_t conn_idx) {
    if (conn_idx >= CONFIG_BT_MAX_CONN) return;
    joycon2_device_info_per_conn[conn_idx].device_info = NULL;
    joycon2_device_info_per_conn[conn_idx].is_joycon2 = false;
    // Remove from joycon2_conns - find and clear the slot
    for (int i = 0; i < joycon2_count; i++) {
        if (joycon2_conns[i].conn && bt_conn_index(joycon2_conns[i].conn) == conn_idx) {
            joycon2_conns[i].conn = NULL;
            joycon2_conns[i].input_handle = 0;
            joycon2_conns[i].write_handle = 0;
            joycon2_conns[i].sensors_enabled = false;
            // Compact array: move last entry to this slot
            if (i < joycon2_count - 1) {
                joycon2_conns[i] = joycon2_conns[joycon2_count - 1];
                joycon2_conns[joycon2_count - 1].conn = NULL;
            }
            joycon2_count--;
            LOG_INF("Joy-Con 2 connection removed for index %d", conn_idx);
            break;
        }
    }
}

// Helper function to find Joy-Con 2 connection by bt_conn
struct joycon2_connection* joycon2_find_connection(struct bt_conn* conn) {
    for (int i = 0; i < joycon2_count; i++) {
        if (joycon2_conns[i].conn == conn) {
            return &joycon2_conns[i];
        }
    }
    return NULL;
}

// Helper function to check if a connection is Joy-Con 2
bool joycon2_is_connection_by_conn(struct bt_conn* conn) {
    return joycon2_find_connection(conn) != NULL;
} 