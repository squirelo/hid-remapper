#include "joycon2.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(joycon2, LOG_LEVEL_DBG);

// Joy-Con 2 GATT UUIDs - using manual initialization to avoid macro issues
static const struct bt_uuid_128 joycon2_input_report_uuid = {
    .uuid = { BT_UUID_TYPE_128 },
    .val = JOYCON2_INPUT_REPORT_UUID_VAL
};
static const struct bt_uuid_128 joycon2_write_command_uuid = {
    .uuid = { BT_UUID_TYPE_128 },
    .val = JOYCON2_WRITE_COMMAND_UUID_VAL
};

// Joy-Con 2 connection management
#define MAX_JOYCON2 2
struct joycon2_connection joycon2_conns[MAX_JOYCON2];
int joycon2_count = 0;

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

bool joycon2_is_device(const struct bt_scan_device_info* device_info) {
    // For now, we'll use a simplified approach since the scan API has changed
    // We'll rely on the main scanning logic to identify Joy-Con 2 devices
    // This function will be called from the main scan callback
    return false; // Placeholder - will be implemented when scan API is available
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
    // For now, we'll use a simplified approach since the GATT discovery API has changed
    // This function will be called from the main discovery callback
    LOG_INF("Joy-Con 2 discovery characteristics called");
    
    // We'll need to implement proper GATT discovery later
    // For now, just log that we're here
    if (joycon2_count < MAX_JOYCON2) {
        joycon2_conns[joycon2_count].conn = conn;
        joycon2_conns[joycon2_count].input_handle = 0;  // Will be set later
        joycon2_conns[joycon2_count].write_handle = 0;  // Will be set later
        joycon2_conns[joycon2_count].sensors_enabled = false;
        joycon2_count++;
        LOG_INF("Joy-Con 2 connection added (handles to be discovered)");
    }
}

void joycon2_enable_sensors(struct bt_conn* conn, uint16_t write_handle) {
    static const uint8_t enable_imu_cmd1[] = { 0x0c, 0x91, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };
    static const uint8_t enable_imu_cmd2[] = { 0x0c, 0x91, 0x01, 0x04, 0x00, 0x04, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };
    struct bt_gatt_write_params params = {0};
    int err;

    params.handle = write_handle;
    params.offset = 0;
    params.data = enable_imu_cmd1;
    params.length = sizeof(enable_imu_cmd1);
    params.func = NULL;
    err = bt_gatt_write(conn, &params);
    if (err) {
        LOG_ERR("Failed to send Joy-Con 2 IMU enable cmd1: %d", err);
    } else {
        LOG_INF("Sent Joy-Con 2 IMU enable cmd1");
    }
    
    k_sleep(K_MSEC(100));
    
    params.data = enable_imu_cmd2;
    params.length = sizeof(enable_imu_cmd2);
    err = bt_gatt_write(conn, &params);
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

void joycon2_decode_and_handle_report(const uint8_t* data, uint8_t len, uint16_t interface) {
    if (len < 2) {
        LOG_ERR("[Joy-Con 2] Report too short: %d", len);
        return;
    }

    uint8_t report_id = data[0];
    const uint8_t* report_data = data + 1;
    uint8_t report_len = len - 1;

    LOG_INF("[Joy-Con 2] Decoding report ID: %d, len: %d, interface: 0x%04x", report_id, report_len, interface);

    // Create a standard HID report structure for the remapper
    struct report_type decoded_report;
    decoded_report.interface = interface;
    decoded_report.len = report_len + 1;
    decoded_report.data[0] = report_id;
    
    // Copy the report data
    if (report_len > 0 && report_len <= sizeof(decoded_report.data) - 1) {
        memcpy(decoded_report.data + 1, report_data, report_len);
        
        // Process the decoded report through the normal remapper pipeline
        if (k_msgq_put(&report_q, &decoded_report, K_NO_WAIT)) {
            LOG_ERR("[Joy-Con 2] Failed to queue decoded report");
        } else {
            LOG_INF("[Joy-Con 2] Successfully queued decoded report");
        }
    } else {
        LOG_ERR("[Joy-Con 2] Invalid report length: %d", report_len);
    }
}

void joycon2_cleanup_connection(uint8_t conn_idx) {
    if (conn_idx < CONFIG_BT_MAX_CONN) {
        joycon2_device_info_per_conn[conn_idx].device_info = NULL;
        joycon2_device_info_per_conn[conn_idx].is_joycon2 = false;
        LOG_INF("Joy-Con 2 device info cleared for connection %d", conn_idx);
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