#include "joycon2.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(joycon2, LOG_LEVEL_DBG);

struct joycon2_candidate_addr {
    bt_addr_le_t addr;
    bool in_use;
};

struct joycon2_connection joycon2_conns[CONFIG_BT_MAX_CONN] = {0};
int joycon2_count = 0;

static bool joycon2_per_conn[CONFIG_BT_MAX_CONN] = {0};
static struct joycon2_candidate_addr joycon2_candidates[CONFIG_BT_MAX_CONN] = {0};

struct report_type {
    uint16_t interface;
    uint8_t len;
    uint8_t data[65];
};

extern struct k_msgq report_q;

struct adv_identity {
    bool nintendo_mfg_seen;
    char name[32];
};

static inline uint16_t get_le16(const uint8_t* buf) {
    return (uint16_t) buf[0] | ((uint16_t) buf[1] << 8);
}

static inline uint8_t scale_12bit_to_u8(uint16_t v) {
    if (v > 4095) {
        v = 4095;
    }
    return (uint8_t) ((v * 255U) / 4095U);
}

static void recount_joycon2_connections() {
    int count = 0;
    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (joycon2_per_conn[i]) {
            count++;
        }
    }
    joycon2_count = count;
}

static void parse_adv_entry(const struct bt_data* ad, struct adv_identity* identity) {
    if ((ad == NULL) || (identity == NULL)) {
        return;
    }

    if ((ad->type == BT_DATA_MANUFACTURER_DATA) && (ad->data_len >= 2)) {
        if (get_le16(ad->data) == JOYCON2_MANUFACTURER_ID) {
            identity->nintendo_mfg_seen = true;
        }
        return;
    }

    if ((ad->type == BT_DATA_NAME_COMPLETE) || (ad->type == BT_DATA_NAME_SHORTENED)) {
        size_t n = MIN((size_t) ad->data_len, sizeof(identity->name) - 1);
        memcpy(identity->name, ad->data, n);
        identity->name[n] = '\0';
    }
}

template <typename T>
static auto get_adv_data_len(const T* device_info, int) -> decltype(device_info->adv_data_len, size_t()) {
    return device_info->adv_data_len;
}

template <typename T>
static size_t get_adv_data_len(const T* device_info, long) {
    (void) device_info;
    return 0;
}

static void parse_adv_data(const struct bt_data* adv_data, size_t adv_data_len, struct adv_identity* identity) {
    if ((adv_data == NULL) || (identity == NULL)) {
        return;
    }

    for (size_t i = 0; i < adv_data_len; i++) {
        parse_adv_entry(&adv_data[i], identity);
    }
}

static bool parse_adv_data_cb(struct bt_data* data, void* user_data) {
    parse_adv_entry(data, (struct adv_identity*) user_data);
    return true;
}

static void parse_adv_data(struct net_buf_simple* adv_data, size_t adv_data_len, struct adv_identity* identity) {
    (void) adv_data_len;
    if ((adv_data == NULL) || (identity == NULL)) {
        return;
    }

    struct net_buf_simple local = *adv_data;
    bt_data_parse(&local, parse_adv_data_cb, identity);
}

static void parse_adv_data(const struct net_buf_simple* adv_data, size_t adv_data_len, struct adv_identity* identity) {
    parse_adv_data((struct net_buf_simple*) adv_data, adv_data_len, identity);
}

bool joycon2_is_device(const struct bt_scan_device_info* device_info) {
    if ((device_info == NULL) || (device_info->recv_info == NULL) || (device_info->recv_info->addr == NULL)) {
        return false;
    }

    struct adv_identity identity = {0};
    parse_adv_data(device_info->adv_data, get_adv_data_len(device_info, 0), &identity);

    bool name_match = (strstr(identity.name, JOYCON2_LEFT_NAME) != NULL) ||
                      (strstr(identity.name, JOYCON2_RIGHT_NAME) != NULL) ||
                      (strstr(identity.name, "Joy-Con") != NULL);
    bool is_joycon2 = identity.nintendo_mfg_seen || name_match;

    if (is_joycon2) {
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
        LOG_INF("Joy-Con 2 candidate detected at %s (name='%s', nintendo_mfg=%d)",
                addr, identity.name, identity.nintendo_mfg_seen);
    }

    return is_joycon2;
}

void joycon2_mark_candidate_addr(const bt_addr_le_t* addr) {
    if (addr == NULL) {
        return;
    }

    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (joycon2_candidates[i].in_use && bt_addr_le_eq(&joycon2_candidates[i].addr, addr)) {
            return;
        }
    }

    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (!joycon2_candidates[i].in_use) {
            bt_addr_le_copy(&joycon2_candidates[i].addr, addr);
            joycon2_candidates[i].in_use = true;
            return;
        }
    }

    LOG_WRN("Joy-Con 2 candidate list full; dropping scan match");
}

void joycon2_assign_connection(struct bt_conn* conn) {
    if (conn == NULL) {
        return;
    }

    uint8_t conn_idx = bt_conn_index(conn);
    if (conn_idx >= CONFIG_BT_MAX_CONN) {
        return;
    }

    joycon2_per_conn[conn_idx] = false;

    const bt_addr_le_t* dst = bt_conn_get_dst(conn);
    if (dst == NULL) {
        recount_joycon2_connections();
        return;
    }

    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (joycon2_candidates[i].in_use && bt_addr_le_eq(&joycon2_candidates[i].addr, dst)) {
            joycon2_per_conn[conn_idx] = true;
            joycon2_candidates[i].in_use = false;
            break;
        }
    }

    recount_joycon2_connections();

    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(dst, addr, sizeof(addr));
    LOG_INF("Connection %u (%s) Joy-Con 2=%d", conn_idx, addr, joycon2_per_conn[conn_idx]);
}

bool joycon2_is_connection(uint8_t conn_idx) {
    return (conn_idx < CONFIG_BT_MAX_CONN) && joycon2_per_conn[conn_idx];
}

static uint8_t joycon2_notify_cb(struct bt_conn* conn, struct bt_gatt_subscribe_params* params, const void* data, uint16_t length) {
    (void) params;
    if ((conn == NULL) || (data == NULL)) {
        return BT_GATT_ITER_STOP;
    }

    uint8_t len = (length > UINT8_MAX) ? UINT8_MAX : (uint8_t) length;
    joycon2_decode_and_handle_report((const uint8_t*) data, len, (uint16_t) (bt_conn_index(conn) << 8));
    return BT_GATT_ITER_CONTINUE;
}

void joycon2_discover_characteristics(struct bt_conn* conn, const struct bt_gatt_dm* dm) {
    if ((conn == NULL) || (dm == NULL)) {
        return;
    }

    uint8_t conn_idx = bt_conn_index(conn);
    if (!joycon2_is_connection(conn_idx)) {
        return;
    }

    struct joycon2_connection* joycon2_conn = &joycon2_conns[conn_idx];
    memset(joycon2_conn, 0, sizeof(*joycon2_conn));
    joycon2_conn->conn = conn;

    bool looking_for_input_ccc = false;

    const struct bt_gatt_dm_attr* attr = NULL;
    while (NULL != (attr = bt_gatt_dm_attr_next(dm, attr))) {
        if ((attr->uuid != NULL) && (attr->uuid->type == BT_UUID_TYPE_128)) {
            const struct bt_uuid_128* uuid = BT_UUID_128(attr->uuid);
            if (memcmp(uuid->val, JOYCON2_INPUT_REPORT_UUID_VAL, sizeof(JOYCON2_INPUT_REPORT_UUID_VAL)) == 0) {
                joycon2_conn->input_handle = attr->handle;
                looking_for_input_ccc = true;
                continue;
            }
            if (memcmp(uuid->val, JOYCON2_WRITE_COMMAND_UUID_VAL, sizeof(JOYCON2_WRITE_COMMAND_UUID_VAL)) == 0) {
                joycon2_conn->write_handle = attr->handle;
                continue;
            }
        }

        if (looking_for_input_ccc && (attr->handle > joycon2_conn->input_handle)) {
            if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC) == 0) {
                joycon2_conn->input_ccc_handle = attr->handle;
                looking_for_input_ccc = false;
            } else if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC) == 0) {
                looking_for_input_ccc = false;
            }
        }
    }

    LOG_INF("Joy-Con 2 handles conn=%u input=0x%04x ccc=0x%04x write=0x%04x",
            conn_idx,
            joycon2_conn->input_handle,
            joycon2_conn->input_ccc_handle,
            joycon2_conn->write_handle);
}

int joycon2_subscribe_input(struct bt_conn* conn) {
    struct joycon2_connection* joycon2_conn = joycon2_find_connection(conn);
    if ((joycon2_conn == NULL) || (joycon2_conn->input_handle == 0)) {
        return -EINVAL;
    }

    if (joycon2_conn->notify_sub_params.notify != NULL) {
        return 0;
    }

    if (joycon2_conn->input_ccc_handle == 0) {
        joycon2_conn->input_ccc_handle = joycon2_conn->input_handle + 1;
    }

    memset(&joycon2_conn->notify_sub_params, 0, sizeof(joycon2_conn->notify_sub_params));
    joycon2_conn->notify_sub_params.notify = joycon2_notify_cb;
    joycon2_conn->notify_sub_params.value = BT_GATT_CCC_NOTIFY;
    joycon2_conn->notify_sub_params.value_handle = joycon2_conn->input_handle;
    joycon2_conn->notify_sub_params.ccc_handle = joycon2_conn->input_ccc_handle;

    int err = bt_gatt_subscribe(conn, &joycon2_conn->notify_sub_params);
    if (err == 0) {
        LOG_INF("Subscribed Joy-Con 2 input notifications (value=0x%04x ccc=0x%04x)",
                joycon2_conn->input_handle,
                joycon2_conn->input_ccc_handle);
    } else {
        LOG_ERR("Failed Joy-Con 2 subscribe: %d", err);
    }
    return err;
}

void joycon2_enable_sensors(struct bt_conn* conn, uint16_t write_handle) {
    static const uint8_t enable_imu_cmd1[] = {0x0c, 0x91, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
    static const uint8_t enable_imu_cmd2[] = {0x0c, 0x91, 0x01, 0x04, 0x00, 0x04, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};

    if ((conn == NULL) || (write_handle == 0)) {
        return;
    }

    int err = bt_gatt_write_without_response(conn, write_handle, enable_imu_cmd1, sizeof(enable_imu_cmd1), false);
    if (err) {
        LOG_ERR("Failed Joy-Con 2 IMU enable cmd1: %d", err);
        return;
    }

    k_sleep(K_MSEC(50));

    err = bt_gatt_write_without_response(conn, write_handle, enable_imu_cmd2, sizeof(enable_imu_cmd2), false);
    if (err) {
        LOG_ERR("Failed Joy-Con 2 IMU enable cmd2: %d", err);
        return;
    }

    struct joycon2_connection* joycon2_conn = joycon2_find_connection(conn);
    if (joycon2_conn != NULL) {
        joycon2_conn->sensors_enabled = true;
    }
}

static void queue_report_raw(uint16_t interface, const uint8_t* data, uint8_t len) {
    if ((data == NULL) || (len == 0) || (len > sizeof(((struct report_type*) 0)->data))) {
        return;
    }

    struct report_type report = {
        .interface = interface,
        .len = len,
    };
    memcpy(report.data, data, len);
    if (k_msgq_put(&report_q, &report, K_NO_WAIT)) {
        LOG_ERR("Failed to queue Joy-Con 2 report");
    }
}

void joycon2_decode_and_handle_report(const uint8_t* data, uint8_t len, uint16_t interface) {
    if ((data == NULL) || (len == 0)) {
        return;
    }

    const uint8_t report_id = data[0];
    if ((report_id != 0x30) || (len < 12)) {
        queue_report_raw(interface, data, len);
        return;
    }

    const uint8_t b0 = data[3];
    const uint8_t b1 = data[4];
    const uint8_t b2 = data[5];

    const bool dpad_down = (b2 & BIT(0)) != 0;
    const bool dpad_up = (b2 & BIT(1)) != 0;
    const bool dpad_right = (b2 & BIT(2)) != 0;
    const bool dpad_left = (b2 & BIT(3)) != 0;

    uint8_t hat = 8;  // neutral
    if (dpad_up && dpad_right) {
        hat = 1;
    } else if (dpad_right && dpad_down) {
        hat = 3;
    } else if (dpad_down && dpad_left) {
        hat = 5;
    } else if (dpad_left && dpad_up) {
        hat = 7;
    } else if (dpad_up) {
        hat = 0;
    } else if (dpad_right) {
        hat = 2;
    } else if (dpad_down) {
        hat = 4;
    } else if (dpad_left) {
        hat = 6;
    }

    uint16_t lx12 = (uint16_t) data[6] | (((uint16_t) data[7] & 0x0F) << 8);
    uint16_t ly12 = (((uint16_t) data[7] >> 4) & 0x0F) | ((uint16_t) data[8] << 4);
    uint16_t rx12 = (uint16_t) data[9] | (((uint16_t) data[10] & 0x0F) << 8);
    uint16_t ry12 = (((uint16_t) data[10] >> 4) & 0x0F) | ((uint16_t) data[11] << 4);

    struct report_type normalized = {
        .interface = interface,
        .len = 16,
        .data = {0},
    };

    normalized.data[0] = 0x01;  // normalized report ID
    normalized.data[1] = 0;
    normalized.data[2] = 0;
    normalized.data[3] = hat;
    normalized.data[4] = scale_12bit_to_u8(lx12);
    normalized.data[5] = scale_12bit_to_u8(ly12);
    normalized.data[6] = scale_12bit_to_u8(rx12);
    normalized.data[7] = scale_12bit_to_u8(ry12);
    normalized.data[8] = (b2 & BIT(7)) ? 0xFF : 0x00;   // ZL -> LT
    normalized.data[9] = (b0 & BIT(7)) ? 0xFF : 0x00;   // ZR -> RT

    normalized.data[1] |= (b0 & BIT(3)) ? BIT(0) : 0;   // A
    normalized.data[1] |= (b0 & BIT(2)) ? BIT(1) : 0;   // B
    normalized.data[1] |= (b0 & BIT(1)) ? BIT(2) : 0;   // X
    normalized.data[1] |= (b0 & BIT(0)) ? BIT(3) : 0;   // Y
    normalized.data[1] |= (b2 & BIT(6)) ? BIT(4) : 0;   // L
    normalized.data[1] |= (b0 & BIT(6)) ? BIT(5) : 0;   // R
    normalized.data[1] |= (b2 & BIT(7)) ? BIT(6) : 0;   // ZL
    normalized.data[1] |= (b0 & BIT(7)) ? BIT(7) : 0;   // ZR

    normalized.data[2] |= (b1 & BIT(0)) ? BIT(0) : 0;   // minus/select
    normalized.data[2] |= (b1 & BIT(1)) ? BIT(1) : 0;   // plus/start
    normalized.data[2] |= (b1 & BIT(3)) ? BIT(2) : 0;   // L3
    normalized.data[2] |= (b1 & BIT(2)) ? BIT(3) : 0;   // R3
    normalized.data[2] |= (b1 & BIT(4)) ? BIT(4) : 0;   // home
    normalized.data[2] |= (b1 & BIT(5)) ? BIT(5) : 0;   // capture

    if (len >= 25) {
        int16_t gx = (int16_t) get_le16(&data[19]);
        int16_t gy = (int16_t) get_le16(&data[21]);
        int16_t gz = (int16_t) get_le16(&data[23]);
        normalized.data[10] = (uint8_t) (gx & 0xFF);
        normalized.data[11] = (uint8_t) ((gx >> 8) & 0xFF);
        normalized.data[12] = (uint8_t) (gy & 0xFF);
        normalized.data[13] = (uint8_t) ((gy >> 8) & 0xFF);
        normalized.data[14] = (uint8_t) (gz & 0xFF);
        normalized.data[15] = (uint8_t) ((gz >> 8) & 0xFF);
    }

    if (k_msgq_put(&report_q, &normalized, K_NO_WAIT)) {
        LOG_ERR("Failed to queue normalized Joy-Con 2 report");
    }
}

void joycon2_cleanup_connection(uint8_t conn_idx) {
    if (conn_idx >= CONFIG_BT_MAX_CONN) {
        return;
    }

    struct joycon2_connection* joycon2_conn = &joycon2_conns[conn_idx];
    if ((joycon2_conn->conn != NULL) && (joycon2_conn->notify_sub_params.notify != NULL)) {
        (void) bt_gatt_unsubscribe(joycon2_conn->conn, &joycon2_conn->notify_sub_params);
    }

    memset(joycon2_conn, 0, sizeof(*joycon2_conn));
    joycon2_per_conn[conn_idx] = false;
    recount_joycon2_connections();
}

struct joycon2_connection* joycon2_find_connection(struct bt_conn* conn) {
    if (conn == NULL) {
        return NULL;
    }

    uint8_t conn_idx = bt_conn_index(conn);
    if ((conn_idx >= CONFIG_BT_MAX_CONN) || !joycon2_per_conn[conn_idx]) {
        return NULL;
    }

    struct joycon2_connection* joycon2_conn = &joycon2_conns[conn_idx];
    if (joycon2_conn->conn != conn) {
        return NULL;
    }
    return joycon2_conn;
}

bool joycon2_is_connection_by_conn(struct bt_conn* conn) {
    return joycon2_find_connection(conn) != NULL;
}
