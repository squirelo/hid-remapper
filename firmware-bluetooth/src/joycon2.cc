#include "joycon2.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(joycon2, LOG_LEVEL_DBG);

// Joy-Con 2 GATT UUIDs - using manual initialization to avoid macro issues
static const struct bt_uuid_128 joycon2_input_report_uuid = {
    .uuid = { BT_UUID_TYPE_128 },
    .val = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};
static const struct bt_uuid_128 joycon2_write_command_uuid = {
    .uuid = { BT_UUID_TYPE_128 },
    .val = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
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

// Helper function to get 16-bit little-endian value
static inline uint16_t get_le16(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

bool joycon2_is_device(const struct bt_scan_device_info* device_info) {
    // Check manufacturer data for Nintendo Joy-Con
    // Use the correct Zephyr API to access advertisement data
    const struct bt_le_scan_recv_info* recv_info = device_info->recv_info;
    
    // In Zephyr, advertisement data is accessed through net_buf_simple
    // We need to parse the advertisement data to find manufacturer specific data
    // For now, let's use a simpler approach - check by device name
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(recv_info->addr, addr, sizeof(addr));
    
    LOG_INF("Checking device at %s for Joy-Con 2", addr);
    
    // TODO: Implement proper advertisement data parsing using net_buf_simple
    // The advertisement data should be accessed through a separate parameter
    // in the scan callback function, not through recv_info->data
    
    return false;
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
    LOG_INF("[Joy-Con 2] Received report ID: %d, len: %d, interface: 0x%04x", report_id, len, interface);

    // For debugging, log the first few bytes of the report
    if (len >= 8) {
        LOG_INF("[Joy-Con 2] Report data: %02x %02x %02x %02x %02x %02x %02x %02x...", 
                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    }

    // Create a standard HID report structure for the remapper
    struct report_type decoded_report;
    decoded_report.interface = interface;
    decoded_report.len = len;
    
    // Copy the entire report data as-is
    if (len <= sizeof(decoded_report.data)) {
        memcpy(decoded_report.data, data, len);
        
        // Process the report through the normal remapper pipeline
        if (k_msgq_put(&report_q, &decoded_report, K_NO_WAIT)) {
            LOG_ERR("[Joy-Con 2] Failed to queue report");
        } else {
            LOG_INF("[Joy-Con 2] Successfully queued report (len: %d)", len);
        }
    } else {
        LOG_ERR("[Joy-Con 2] Report too large: %d bytes", len);
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