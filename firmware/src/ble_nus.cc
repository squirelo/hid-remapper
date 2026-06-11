#include "ble_nus.h"

#include "ble_button_led.h"
#include "ble_shared.h"
#include "crc.h"
#include "descriptor_parser.h"
#include "globals.h"
#include "remapper.h"

#include <string.h>

#include <pico/cyw43_arch.h>

extern "C" {
#include "ble/att_server.h"
#include "ble/gatt-service/nordic_spp_service_server.h"
#include "ble/sm.h"
#include "btstack.h"
#include "gap.h"
}

#define NUS_PROTOCOL_VERSION 1
#define NUS_PACKET_BUFFER_SIZE 512
#define SLIP_END 0300
#define SLIP_ESC 0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335

struct __attribute__((packed)) nus_packet_t {
    uint8_t protocol_version;
    uint8_t our_descriptor_number;
    uint8_t len;
    uint8_t report_id;
    uint8_t data[0];
};

static uint8_t nus_packet_buffer[NUS_PACKET_BUFFER_SIZE];
static uint16_t nus_bytes_read = 0;
static bool nus_escaped = false;
static bool nus_overflowed = false;
static hci_con_handle_t nus_conn = HCI_CON_HANDLE_INVALID;
static bool nus_advertising = false;

static const uint8_t nus_virtual_gamepad_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x15, 0x00, 0x25, 0x01, 0x35, 0x00, 0x45, 0x01,
    0x75, 0x01, 0x95, 0x0E, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0E, 0x81, 0x02, 0x95, 0x02, 0x81, 0x01,
    0x05, 0x01, 0x25, 0x0F, 0x46, 0x3B, 0x01, 0x75, 0x04, 0x95, 0x01, 0x65, 0x14, 0x09, 0x39, 0x81,
    0x42, 0x65, 0x00, 0x95, 0x01, 0x81, 0x01, 0x26, 0xFF, 0x00, 0x46, 0xFF, 0x00, 0x09, 0x30, 0x09,
    0x31, 0x09, 0x32, 0x09, 0x35, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x75, 0x08, 0x95, 0x01, 0x81,
    0x01, 0xC0,
};

static uint8_t adv_data[] = {
    0x02, 0x01, 0x06,
    0x11, 0x07, 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
};

static uint8_t scan_rsp_data[] = {
    0x0c, 0x09, 'H', 'I', 'D', ' ', 'R', 'e', 'm', 'a', 'p', 'p', 'e', 'r',
};

static void nus_handle_packet(const uint8_t* data, uint16_t len) {
    static uint8_t last_descriptor_warning = 0xff;

    if (len < sizeof(nus_packet_t)) {
        return;
    }

    const nus_packet_t* msg = (const nus_packet_t*) data;
    uint16_t payload_len = len - sizeof(nus_packet_t);

    if ((msg->protocol_version != NUS_PROTOCOL_VERSION) ||
        (msg->len != payload_len) ||
        (payload_len > 64) ||
        (msg->our_descriptor_number >= NOUR_DESCRIPTORS) ||
        ((msg->report_id == 0) && (payload_len >= 64))) {
        return;
    }

    if ((msg->our_descriptor_number != our_descriptor_number) &&
        (msg->our_descriptor_number != last_descriptor_warning)) {
        last_descriptor_warning = msg->our_descriptor_number;
    }

    struct ble_report_t report = {
        .interface = BLE_NUS_VIRTUAL_INTERFACE,
        .external_report_id = msg->report_id,
        .len = (uint8_t) payload_len,
    };
    memcpy(report.data, msg->data, payload_len);
    ble_report_queue_try_add(&report);
}

static void nus_packet_append(uint8_t c) {
    if (nus_bytes_read >= sizeof(nus_packet_buffer)) {
        nus_overflowed = true;
        return;
    }
    nus_packet_buffer[nus_bytes_read++] = c;
}

static void nus_process_byte(uint8_t c) {
    if (nus_escaped) {
        switch (c) {
            case SLIP_ESC_END:
                nus_packet_append(SLIP_END);
                break;
            case SLIP_ESC_ESC:
                nus_packet_append(SLIP_ESC);
                break;
            default:
                nus_packet_append(c);
                break;
        }
        nus_escaped = false;
        return;
    }

    switch (c) {
        case SLIP_END:
            if (!nus_overflowed && (nus_bytes_read > 4)) {
                uint32_t crc = crc32(nus_packet_buffer, nus_bytes_read - 4);
                uint32_t received_crc =
                    (nus_packet_buffer[nus_bytes_read - 4] << 0) |
                    (nus_packet_buffer[nus_bytes_read - 3] << 8) |
                    (nus_packet_buffer[nus_bytes_read - 2] << 16) |
                    (nus_packet_buffer[nus_bytes_read - 1] << 24);
                if (crc == received_crc) {
                    nus_handle_packet(nus_packet_buffer, nus_bytes_read - 4);
                }
            }
            nus_bytes_read = 0;
            nus_escaped = false;
            nus_overflowed = false;
            break;
        case SLIP_ESC:
            if (!nus_overflowed) {
                nus_escaped = true;
            }
            break;
        default:
            nus_packet_append(c);
            break;
    }
}

static void nus_process_bytes(const uint8_t* data, uint16_t len) {
    ble_status_led_activity_flash();
    for (uint16_t i = 0; i < len; i++) {
        nus_process_byte(data[i]);
    }
}

static void nus_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    if (packet_type == RFCOMM_DATA_PACKET) {
        nus_process_bytes(packet, size);
        return;
    }

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_GATTSERVICE_META:
            switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
                case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
                    nus_conn = gattservice_subevent_spp_service_connected_get_con_handle(packet);
                    sm_request_pairing(nus_conn);
                    break;
                case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
                    if (nus_conn == gattservice_subevent_spp_service_disconnected_get_con_handle(packet)) {
                        nus_conn = HCI_CON_HANDLE_INVALID;
                        nus_bytes_read = 0;
                        nus_escaped = false;
                        nus_overflowed = false;
                        ble_nus_start_advertising();
                    }
                    break;
                default:
                    break;
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            if (nus_conn == hci_event_disconnection_complete_get_connection_handle(packet)) {
                nus_conn = HCI_CON_HANDLE_INVALID;
                nus_bytes_read = 0;
                nus_escaped = false;
                nus_overflowed = false;
                ble_nus_start_advertising();
            }
            break;
        default:
            break;
    }
}

void ble_nus_init_virtual_device(void) {
    parse_descriptor(BLE_NUS_VIRTUAL_VID, BLE_NUS_VIRTUAL_PID,
                     nus_virtual_gamepad_descriptor,
                     sizeof(nus_virtual_gamepad_descriptor),
                     BLE_NUS_VIRTUAL_INTERFACE, 0);
    device_connected_callback(BLE_NUS_VIRTUAL_INTERFACE, BLE_NUS_VIRTUAL_VID, BLE_NUS_VIRTUAL_PID, 0);
    their_descriptor_updated = true;
}

void ble_nus_init(void) {
    nordic_spp_service_server_init(nus_packet_handler);
    ble_nus_init_virtual_device();
    ble_nus_start_advertising();
}

void ble_nus_start_advertising(void) {
    if (nus_advertising) {
        return;
    }
    gap_advertisements_set_data(sizeof(adv_data), adv_data);
    gap_scan_response_set_data(sizeof(scan_rsp_data), scan_rsp_data);
    gap_advertisements_set_params(0x0030, 0x0030, 0, 0, NULL, 0x07, 0);
    gap_advertisements_enable(1);
    nus_advertising = true;
}

void ble_nus_stop_advertising(void) {
    gap_advertisements_enable(0);
    nus_advertising = false;
}

void ble_nus_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    nus_packet_handler(packet_type, channel, packet, size);
}

bool ble_nus_is_peripheral_connection(uint8_t packet_type, uint8_t* packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET || size == 0) {
        return false;
    }
    if (hci_event_packet_get_type(packet) != HCI_EVENT_LE_META) {
        return false;
    }
    if (hci_event_le_meta_get_subevent_code(packet) != HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
        return false;
    }
    return hci_subevent_le_connection_complete_get_role(packet) == 1;
}
