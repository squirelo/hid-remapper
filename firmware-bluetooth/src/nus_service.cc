#include "nus_service.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <bluetooth/services/nus.h>
#include <string.h>

LOG_MODULE_REGISTER(nus_service, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// Report type structure (matches main.cc)
struct report_type {
    uint16_t interface;
    uint8_t len;
    uint8_t data[65];
};

// Forward declaration - report_q is defined in main.cc
// K_MSGQ_DEFINE creates a struct k_msgq variable
extern struct k_msgq report_q;

static struct bt_conn* nus_current_conn = NULL;

// Process incoming NUS data as gamepad report
static void process_nus_data(const uint8_t* data, uint16_t len) {
    if (data == NULL || len == 0) {
        LOG_WRN("Invalid NUS data");
        return;
    }

    // Ensure we have at least a report ID
    if (len < 1) {
        LOG_WRN("NUS data too short");
        return;
    }

    // Ensure data fits in report_type structure
    if (len > 65) {
        LOG_WRN("NUS data too long: %d bytes", len);
        return;
    }

    struct report_type buf;
    buf.interface = NUS_GAMEPAD_INTERFACE;
    buf.len = len;
    
    // Copy data: first byte is report ID, rest is report data
    memcpy(buf.data, data, len);

    // Queue to report_q for processing by remapper
    if (k_msgq_put(&report_q, &buf, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to queue NUS report data");
    } else {
        LOG_DBG("Queued NUS gamepad report: len=%d, report_id=0x%02x", len, data[0]);
    }
}

// NUS data receive callback
static void bt_nus_receive_cb(struct bt_conn* conn, const uint8_t* data, uint16_t len) {
    char addr[BT_ADDR_LE_STR_LEN];
    
    // Track this as a NUS connection if not already tracked
    if (nus_current_conn != conn) {
        nus_current_conn = bt_conn_ref(conn);
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        LOG_INF("NUS connection established: %s", addr);
    }
    
    // Process the data as gamepad report
    // Expected format: first byte is report ID, rest is report data
    // This matches the format used by hogp_notify_cb: data[0] = report ID, data[1..] = report data
    if (len > 0) {
        process_nus_data(data, len);
    }
}

// NUS callback structure
static struct bt_nus_cb nus_cb = {
    .received = bt_nus_receive_cb,
};

// Check if a connection is a NUS (peripheral) connection
bool nus_is_nus_connection(struct bt_conn* conn) {
    return (nus_current_conn == conn);
}

// Handle NUS connection (call from main connection callbacks)
void nus_handle_connected(struct bt_conn* conn) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("NUS connected: %s", addr);
    nus_current_conn = bt_conn_ref(conn);
}

void nus_handle_disconnected(struct bt_conn* conn) {
    if (nus_current_conn == conn) {
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        LOG_INF("NUS disconnected: %s", addr);
        bt_conn_unref(nus_current_conn);
        nus_current_conn = NULL;
    }
}

// Advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// Scan response data with NUS UUID
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

// Initialize NUS service and start advertising
int nus_service_init() {
    int err;
    
    // Initialize NUS service
    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("Failed to initialize NUS service (err: %d)", err);
        return err;
    }
    
    LOG_INF("NUS service initialized");
    
    // Start advertising
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Failed to start advertising (err: %d)", err);
        return err;
    }
    
    LOG_INF("NUS advertising started");
    
    return 0;
}

