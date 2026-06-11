#include "ble_central.h"

#include "ble_button_led.h"
#include "ble_nus.h"
#include "ble_shared.h"
#include "descriptor_parser.h"
#include "globals.h"
#include "remapper.h"

#include <string.h>

#include <pico/cyw43_arch.h>

extern "C" {
#include "ble/gatt-service/hids_client.h"
#include "ble/le_device_db.h"
#include "ble/sm.h"
#include "btstack.h"
#include "btstack_run_loop.h"
#include "gap.h"
}

struct central_slot_t {
    hci_con_handle_t con_handle;
    uint16_t hids_cid;
    uint8_t conn_idx;
    bd_addr_t addr;
    bd_addr_type_t addr_type;
    bool in_use;
    bool hids_ready;
};

static central_slot_t central_slots[BLE_MAX_CENTRAL_CONNECTIONS];
static uint8_t hids_descriptor_storage[4096];
static bool scanning = false;
static bool peers_only = true;
static bool stack_ready = false;
static bool scan_pending = false;
static btstack_timer_source_t scan_timer;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static int8_t conn_idx_for_cid(uint16_t hids_cid) {
    for (int i = 0; i < BLE_MAX_CENTRAL_CONNECTIONS; i++) {
        if (central_slots[i].in_use && central_slots[i].hids_cid == hids_cid) {
            return central_slots[i].conn_idx;
        }
    }
    return -1;
}

static int8_t conn_idx_for_handle(hci_con_handle_t handle) {
    for (int i = 0; i < BLE_MAX_CENTRAL_CONNECTIONS; i++) {
        if (central_slots[i].in_use && central_slots[i].con_handle == handle) {
            return central_slots[i].conn_idx;
        }
    }
    return -1;
}

static central_slot_t* alloc_slot(hci_con_handle_t handle) {
    for (int i = 0; i < BLE_MAX_CENTRAL_CONNECTIONS; i++) {
        if (!central_slots[i].in_use) {
            central_slots[i].in_use = true;
            central_slots[i].con_handle = handle;
            central_slots[i].hids_cid = 0;
            central_slots[i].conn_idx = (uint8_t) i;
            central_slots[i].hids_ready = false;
            return &central_slots[i];
        }
    }
    return nullptr;
}

static void free_slot_by_handle(hci_con_handle_t handle) {
    int8_t idx = conn_idx_for_handle(handle);
    if (idx < 0) {
        return;
    }
    central_slots[idx].in_use = false;
    central_slots[idx].con_handle = HCI_CON_HANDLE_INVALID;
    central_slots[idx].hids_cid = 0;
    central_slots[idx].hids_ready = false;
}

static int count_central_connections(void) {
    int count = 0;
    for (int i = 0; i < BLE_MAX_CENTRAL_CONNECTIONS; i++) {
        if (central_slots[i].in_use) {
            count++;
        }
    }
    return count;
}

static void update_status_led(void) {
    ble_status_led_set_blink_count(count_central_connections());
    if (scanning) {
        ble_status_led_set_mode(peers_only ? BleLedMode::BLINK : BleLedMode::ON);
    } else {
        ble_status_led_set_mode(BleLedMode::BLINK);
    }
}

static bool advertisement_report_contains_hids(uint8_t* advertisement_report) {
    const uint8_t* adv_data = gap_event_advertising_report_get_data(advertisement_report);
    uint8_t adv_len = gap_event_advertising_report_get_data_length(advertisement_report);

    ad_context_t context;
    for (ad_iterator_init(&context, adv_len, adv_data); ad_iterator_has_more(&context); ad_iterator_next(&context)) {
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t data_size = ad_iterator_get_data_len(&context);
        const uint8_t* data = ad_iterator_get_data(&context);
        switch (data_type) {
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                for (int i = 0; i + 1 < data_size; i += 2) {
                    uint16_t type = little_endian_read_16(data, i);
                    if (type == ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE) {
                        return true;
                    }
                }
                break;
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
                for (int i = 0; i + 15 < data_size; i += 16) {
                    uint16_t uuid16 = little_endian_read_16(data, i + 12);
                    if (uuid16 == ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE) {
                        return true;
                    }
                }
                break;
            default:
                break;
        }
    }
    return false;
}

static void populate_whitelist(void) {
    gap_whitelist_clear();
    int bonded = le_device_db_count();
    for (int i = 0; i < bonded; i++) {
        int addr_type = 0;
        bd_addr_t addr = { 0 };
        sm_key_t irk = { 0 };
        le_device_db_info(i, &addr_type, addr, irk);
        gap_whitelist_add((bd_addr_type_t) addr_type, addr);
    }
}

static bool scan_setup_filters(void) {
    int bonded_count = le_device_db_count();
    int conn_count = count_central_connections();

    if (peers_only && bonded_count > 0) {
        if (conn_count >= bonded_count) {
            return false;
        }
        populate_whitelist();
    }

    return true;
}

static void scan_stop(void) {
    if (scanning) {
        gap_stop_scan();
        scanning = false;
        update_status_led();
    }
}

static void scan_start(void) {
    if (!stack_ready) {
        scan_pending = true;
        return;
    }

    if (scanning) {
        scan_stop();
    }

    if (!scan_setup_filters()) {
        scanning = false;
        update_status_led();
        return;
    }

    uint8_t filter_policy = 0;
    if (peers_only && le_device_db_count() > 0) {
        filter_policy = 1;
    } else {
        peers_only = false;
    }

    gap_set_scan_params(0, 0x0030, 0x0030, filter_policy);
    gap_start_scan();
    scanning = true;
    update_status_led();
}

static void scan_timer_handler(struct btstack_timer_source* ts) {
    scan_start();
}

static void schedule_scan_start(void) {
    scan_timer.process = scan_timer_handler;
    btstack_run_loop_set_timer(&scan_timer, BLE_SCAN_DELAY_MS);
    btstack_run_loop_add_timer(&scan_timer);
}

static void schedule_scan_stop_and_restart(void) {
    scan_stop();
    schedule_scan_start();
}

static uint8_t find_bond_index_for_addr(bd_addr_type_t addr_type, const bd_addr_t addr) {
    for (int i = 0; i < le_device_db_count(); i++) {
        int stored_type = 0;
        bd_addr_t stored_addr = { 0 };
        sm_key_t irk = { 0 };
        le_device_db_info(i, &stored_type, stored_addr, irk);
        if ((stored_type == addr_type) && (memcmp(stored_addr, addr, 6) == 0)) {
            return (uint8_t) (i + 1);
        }
    }
    return 0;
}

static void hids_emit_descriptor(uint16_t hids_cid) {
    int8_t conn_idx = conn_idx_for_cid(hids_cid);
    if (conn_idx < 0) {
        return;
    }

    const uint8_t* descriptor = hids_client_descriptor_storage_get_descriptor_data(hids_cid, 0);
    uint16_t descriptor_len = hids_client_descriptor_storage_get_descriptor_len(hids_cid, 0);
    if ((descriptor == nullptr) || (descriptor_len == 0)) {
        return;
    }

    struct ble_descriptor_t item = {
        .size = descriptor_len,
        .conn_idx = (uint8_t) conn_idx,
    };
    if (descriptor_len > sizeof(item.data)) {
        descriptor_len = sizeof(item.data);
        item.size = descriptor_len;
    }
    memcpy(item.data, descriptor, descriptor_len);
    ble_descriptor_queue_try_add(&item);
}

static void hids_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) {
        return;
    }

    switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
        case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED: {
            uint16_t hids_cid = gattservice_subevent_hid_service_connected_get_hids_cid(packet);
            uint8_t status = gattservice_subevent_hid_service_connected_get_status(packet);
            int8_t conn_idx = conn_idx_for_cid(hids_cid);
            if ((conn_idx < 0) || (status != ERROR_CODE_SUCCESS)) {
                break;
            }
            central_slots[conn_idx].hids_ready = true;

            uint8_t bond_idx = find_bond_index_for_addr(central_slots[conn_idx].addr_type, central_slots[conn_idx].addr);

            device_connected_callback((uint16_t) conn_idx << 8, 1, 1, bond_idx);
            hids_emit_descriptor(hids_cid);
            schedule_scan_stop_and_restart();
            break;
        }
        case GATTSERVICE_SUBEVENT_HID_REPORT: {
            uint16_t hids_cid = gattservice_subevent_hid_report_get_hids_cid(packet);
            int8_t conn_idx = conn_idx_for_cid(hids_cid);
            if (conn_idx < 0) {
                break;
            }

            if (scanning) {
                schedule_scan_stop_and_restart();
            } else {
                schedule_scan_start();
            }

            uint8_t report_id = gattservice_subevent_hid_report_get_report_id(packet);
            uint16_t report_len = gattservice_subevent_hid_report_get_report_len(packet);
            const uint8_t* report = gattservice_subevent_hid_report_get_report(packet);

            struct ble_report_t item = {
                .interface = (uint16_t) conn_idx << 8,
                .external_report_id = 0,
                .len = (uint8_t) ((report_len > sizeof(item.data)) ? sizeof(item.data) : report_len),
            };
            if (report_len > 0) {
                memcpy(item.data, report, item.len);
            } else {
                item.data[0] = report_id;
                item.len = 1;
            }
            ble_report_queue_try_add(&item);
            ble_status_led_activity_flash();
            break;
        }
        default:
            break;
    }
}

static void start_hids_client_if_needed(hci_con_handle_t handle) {
    central_slot_t* slot = nullptr;
    for (int i = 0; i < BLE_MAX_CENTRAL_CONNECTIONS; i++) {
        if (central_slots[i].in_use && central_slots[i].con_handle == handle) {
            slot = &central_slots[i];
            break;
        }
    }
    if ((slot == nullptr) || (slot->hids_cid != 0)) {
        return;
    }

    uint16_t hids_cid = 0;
    uint8_t status = hids_client_connect(handle, hids_packet_handler, HID_PROTOCOL_MODE_REPORT, &hids_cid);
    if (status == ERROR_CODE_SUCCESS) {
        slot->hids_cid = hids_cid;
    }
}

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                stack_ready = true;
                if (le_device_db_count() == 0) {
                    peers_only = false;
                }
                schedule_scan_start();
                scan_pending = false;
            } else {
                stack_ready = false;
                scanning = false;
            }
            break;
        case GAP_EVENT_ADVERTISING_REPORT: {
            if (!scanning) {
                return;
            }
            if (!advertisement_report_contains_hids(packet)) {
                return;
            }
            if (count_central_connections() >= BLE_MAX_CENTRAL_CONNECTIONS) {
                return;
            }

            bd_addr_t addr;
            gap_event_advertising_report_get_address(packet, addr);
            bd_addr_type_t addr_type = (bd_addr_type_t) gap_event_advertising_report_get_address_type(packet);

            scan_stop();
            gap_connect(addr, addr_type);
            break;
        }
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE: {
                    if (ble_nus_is_peripheral_connection(packet_type, packet, size)) {
                        return;
                    }

                    hci_con_handle_t handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    if (hci_subevent_le_connection_complete_get_status(packet) != ERROR_CODE_SUCCESS) {
                        schedule_scan_start();
                        break;
                    }

                    central_slot_t* slot = alloc_slot(handle);
                    if (slot == nullptr) {
                        gap_disconnect(handle);
                        schedule_scan_start();
                        break;
                    }

                    hci_subevent_le_connection_complete_get_peer_address(packet, slot->addr);
                    slot->addr_type = (bd_addr_type_t) hci_subevent_le_connection_complete_get_peer_address_type(packet);

                    scanning = false;
                    update_status_led();
                    sm_request_pairing(handle);
                    gap_request_connection_parameter_update(handle, 6, 9, 0, 400);
                    break;
                }
                default:
                    break;
            }
            break;
        case SM_EVENT_JUST_WORKS_REQUEST:
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            if (sm_event_pairing_complete_get_status(packet) == ERROR_CODE_SUCCESS) {
                peers_only = true;
                start_hids_client_if_needed(sm_event_pairing_complete_get_handle(packet));
            }
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            if (sm_event_reencryption_complete_get_status(packet) == ERROR_CODE_SUCCESS) {
                peers_only = true;
                start_hids_client_if_needed(sm_event_reencryption_complete_get_handle(packet));
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            int8_t conn_idx = conn_idx_for_handle(handle);
            if (conn_idx < 0) {
                break;
            }

            uint16_t hids_cid = central_slots[conn_idx].hids_cid;
            if (hids_cid != 0) {
                hids_client_disconnect(hids_cid);
            }

            ble_disconnected_queue_try_add((uint8_t) conn_idx);
            free_slot_by_handle(handle);
            update_status_led();
            schedule_scan_start();
            break;
        }
        default:
            break;
    }
}

void ble_central_init(void) {
    memset(central_slots, 0, sizeof(central_slots));
    hids_client_init(hids_descriptor_storage, sizeof(hids_descriptor_storage));

    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(0x01);  // bonding
    sm_set_encryption_key_size_range(7, 16);

    gatt_client_init();
}

void ble_central_pair_new_device(void) {
    peers_only = false;
    schedule_scan_start();
}

void ble_central_clear_bonds(void) {
    scan_stop();

    for (int i = 0; i < BLE_MAX_CENTRAL_CONNECTIONS; i++) {
        if (central_slots[i].in_use) {
            gap_disconnect(central_slots[i].con_handle);
        }
    }

    int max = le_device_db_max_count();
    for (int i = max - 1; i >= 0; i--) {
        le_device_db_remove(i);
    }

    gap_whitelist_clear();
    peers_only = false;
    schedule_scan_start();
}

void ble_central_poll(void) {
}

int ble_central_connected_count(void) {
    return count_central_connections();
}

bool ble_central_is_scanning(void) {
    return scanning;
}
