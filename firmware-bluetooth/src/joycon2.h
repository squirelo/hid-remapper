#ifndef JOYCON2_H
#define JOYCON2_H

#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/hogp.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

// Joy-Con 2 GATT UUIDs
static const uint8_t JOYCON2_INPUT_REPORT_UUID_VAL[16] = {0xbe,0xe9,0x7d,0xab,0xfe,0x89,0xad,0x49,0x82,0x8f,0x11,0x8f,0x09,0xdf,0x7f,0xd2};
static const uint8_t JOYCON2_WRITE_COMMAND_UUID_VAL[16] = {0xc9,0x4a,0x9d,0x64,0xb7,0x8e,0x6c,0x4e,0xaf,0x44,0x1e,0xa5,0x4f,0xe5,0xf0,0x05};

// Joy-Con 2 identification constants
#define JOYCON2_MANUFACTURER_ID 0x057E
#define JOYCON2_LEFT_NAME "Joy-Con (L)"
#define JOYCON2_RIGHT_NAME "Joy-Con (R)"

// Joy-Con 2 connection structure
struct joycon2_connection {
    struct bt_conn* conn;
    uint16_t input_handle;
    uint16_t input_ccc_handle;
    uint16_t write_handle;
    struct bt_gatt_subscribe_params notify_sub_params;
    bool sensors_enabled;
};

// Function declarations
bool joycon2_is_device(const struct bt_scan_device_info* device_info);
void joycon2_mark_candidate_addr(const bt_addr_le_t* addr);
void joycon2_assign_connection(struct bt_conn* conn);
bool joycon2_is_connection(uint8_t conn_idx);
void joycon2_discover_characteristics(struct bt_conn* conn, const struct bt_gatt_dm* dm);
int joycon2_subscribe_input(struct bt_conn* conn);
void joycon2_enable_sensors(struct bt_conn* conn, uint16_t write_handle);
void joycon2_decode_and_handle_report(const uint8_t* data, uint8_t len, uint16_t interface);
void joycon2_cleanup_connection(uint8_t conn_idx);

// Helper function declarations
struct joycon2_connection* joycon2_find_connection(struct bt_conn* conn);
bool joycon2_is_connection_by_conn(struct bt_conn* conn);

// External variables
extern struct joycon2_connection joycon2_conns[];
extern int joycon2_count;

#endif // JOYCON2_H
