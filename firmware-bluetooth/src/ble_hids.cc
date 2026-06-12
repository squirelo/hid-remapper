#include "ble_hids.h"

#include "descriptor_parser.h"
#include "globals.h"
#include "our_descriptor.h"
#include "types.h"
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ble_hids, LOG_LEVEL_DBG);

#define BASE_USB_HID_SPEC_VERSION 0x0101
#define BLE_HIDS_REPORT_MAX_LEN 64

#define hids_obj (*ble_hids_get_instance())

static struct bt_conn* hids_host_conn;
static K_SEM_DEFINE(hids_notify_sem, 1, 1);
static uint8_t report_id_to_index[256];
static uint8_t input_report_count = 0;
static bool hids_notifications_enabled = false;

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn* conn) {
    ARG_UNUSED(conn);
    LOG_DBG("Protocol mode event: %u", evt);
}

static void hids_notif_handler(enum bt_hids_notify_evt evt) {
    if (evt == BT_HIDS_CCCD_EVT_NOTIFY_ENABLED) {
        hids_notifications_enabled = true;
        k_sem_give(&hids_notify_sem);
        LOG_INF("BLE HID notifications enabled");
    } else if (evt == BT_HIDS_CCCD_EVT_NOTIFY_DISABLED) {
        hids_notifications_enabled = false;
        k_sem_give(&hids_notify_sem);
        LOG_INF("BLE HID notifications disabled");
    }
}

static void hids_outp_rep_handler(struct bt_hids_rep* rep, struct bt_conn* conn, bool write) {
    ARG_UNUSED(conn);
    if (!write) {
        return;
    }
    LOG_DBG("BLE HID output report len=%u", rep->size);
}

static void hids_notify_complete(struct bt_conn* conn, void* user_data) {
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&hids_notify_sem);
}

static uint16_t appearance_for_descriptor(void) {
    switch (our_descriptor_number) {
        case 2:
            return 0x03C4;
        default:
            return 0x03C1;
    }
}

static bool descriptor_has_keyboard(void) {
    return our_descriptor_number == 0 || our_descriptor_number == 1;
}

static bool descriptor_has_mouse(void) {
    return our_descriptor_number == 0 || our_descriptor_number == 1;
}

void ble_hids_init(void) {
    memset(report_id_to_index, 0xFF, sizeof(report_id_to_index));
    input_report_count = 0;

    std::unordered_map<uint8_t, std::unordered_map<uint32_t, usage_def_t>> input_usages;
    std::unordered_map<uint8_t, std::unordered_map<uint32_t, usage_def_t>> output_usages;
    std::unordered_map<uint8_t, std::unordered_map<uint32_t, usage_def_t>> feature_usages;
    bool has_report_id = false;

    auto report_sizes_map = parse_descriptor(
        input_usages,
        output_usages,
        feature_usages,
        has_report_id,
        our_descriptor->descriptor,
        our_descriptor->descriptor_length);

    struct bt_hids_init_param hids_init = {};
    hids_init.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
    hids_init.info.b_country_code = 0x00;
    hids_init.info.flags = BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE;
    hids_init.pm_evt_handler = hids_pm_evt_handler;
    hids_init.is_kb = descriptor_has_keyboard();
    hids_init.is_mouse = descriptor_has_mouse();

    if (our_descriptor->descriptor_length > UINT8_MAX) {
        LOG_WRN("Report map length %u exceeds BLE HIDS uint8 limit; truncating",
                our_descriptor->descriptor_length);
    }
    hids_init.rep_map.data = our_descriptor->descriptor;
    hids_init.rep_map.size = (uint8_t) MIN((uint32_t) our_descriptor->descriptor_length, (uint32_t) UINT8_MAX);

    for (auto const& [report_id, size] : report_sizes_map[ReportType::INPUT]) {
        if (input_report_count >= CONFIG_BT_HIDS_INPUT_REP_MAX) {
            LOG_WRN("Too many BLE input reports; skipping id=%u", report_id);
            continue;
        }
        if (size == 0 || size > BLE_HIDS_REPORT_MAX_LEN) {
            LOG_WRN("Skipping BLE input report id=%u size=%u", report_id, size);
            continue;
        }
        struct bt_hids_inp_rep* rep = &hids_init.inp_rep_group_init.reports[input_report_count];
        rep->id = report_id;
        rep->size = (uint8_t) size;
        rep->handler = hids_notif_handler;
        rep->perm = BT_GATT_PERM_READ_ENCRYPT;
        report_id_to_index[report_id] = input_report_count;
        input_report_count++;
    }
    hids_init.inp_rep_group_init.cnt = input_report_count;

    uint8_t output_count = 0;
    for (auto const& [report_id, size] : report_sizes_map[ReportType::OUTPUT]) {
        if (output_count >= CONFIG_BT_HIDS_OUTPUT_REP_MAX) {
            break;
        }
        if (size == 0 || size > BLE_HIDS_REPORT_MAX_LEN) {
            continue;
        }
        struct bt_hids_outp_feat_rep* rep = &hids_init.outp_rep_group_init.reports[output_count];
        rep->id = report_id;
        rep->size = (uint8_t) size;
        rep->handler = hids_outp_rep_handler;
        rep->perm = BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT;
        output_count++;
    }
    hids_init.outp_rep_group_init.cnt = output_count;

    uint8_t feature_count = 0;
    for (auto const& [report_id, size] : report_sizes_map[ReportType::FEATURE]) {
        if (feature_count >= CONFIG_BT_HIDS_FEATURE_REP_MAX) {
            break;
        }
        if (size == 0 || size > BLE_HIDS_REPORT_MAX_LEN) {
            continue;
        }
        struct bt_hids_outp_feat_rep* rep = &hids_init.feat_rep_group_init.reports[feature_count];
        rep->id = report_id;
        rep->size = (uint8_t) size;
        rep->handler = hids_outp_rep_handler;
        rep->perm = BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT;
        feature_count++;
    }
    hids_init.feat_rep_group_init.cnt = feature_count;

    int err = bt_hids_init(&hids_obj, &hids_init);
    if (err) {
        LOG_ERR("bt_hids_init failed: %d", err);
        return;
    }

    LOG_INF("BLE HIDS initialized with %u input reports", input_report_count);
}

void ble_hids_on_connected(struct bt_conn* conn) {
    if (!hids_host_conn) {
        hids_host_conn = bt_conn_ref(conn);
    }
    int err = bt_hids_connected(&hids_obj, conn);
    if (err) {
        LOG_WRN("bt_hids_connected failed: %d", err);
    }
}

void ble_hids_on_disconnected(struct bt_conn* conn) {
    bt_hids_disconnected(&hids_obj, conn);
    if (conn == hids_host_conn) {
        bt_conn_unref(hids_host_conn);
        hids_host_conn = NULL;
        hids_notifications_enabled = false;
        k_sem_give(&hids_notify_sem);
    }
}

bool ble_hids_host_connected(void) {
    return hids_host_conn != NULL && hids_notifications_enabled;
}

bool ble_hids_try_send_report(const uint8_t* report_with_id, uint8_t len) {
    if (!hids_host_conn || !hids_notifications_enabled || input_report_count == 0) {
        return false;
    }

    if (k_sem_take(&hids_notify_sem, K_NO_WAIT) != 0) {
        return false;
    }

    const uint8_t* payload = report_with_id;
    uint8_t payload_len = len;
    uint8_t report_id = 0;

    if (payload_len > 0 && payload[0] != 0) {
        report_id = payload[0];
        payload++;
        payload_len--;
    }

    uint8_t rep_index = report_id_to_index[report_id];
    if (rep_index == 0xFF) {
        k_sem_give(&hids_notify_sem);
        return false;
    }

    int err = bt_hids_inp_rep_send(&hids_obj, hids_host_conn, rep_index, payload, payload_len,
                                   hids_notify_complete);
    if (err) {
        k_sem_give(&hids_notify_sem);
        LOG_DBG("bt_hids_inp_rep_send failed: %d", err);
        return false;
    }

    return true;
}

uint16_t ble_hids_appearance(void) {
    return appearance_for_descriptor();
}
