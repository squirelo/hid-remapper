#include "usb_cdc_input.h"

#include "our_descriptor.h"
#include "slip_input.h"
#include "virtual_input.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_cdc_input, LOG_LEVEL_DBG);

#define SLIP_PROTOCOL_VERSION 1

struct __attribute__((packed)) slip_hid_packet_t {
    uint8_t protocol_version;
    uint8_t our_descriptor_number;
    uint8_t len;
    uint8_t report_id;
    uint8_t data[0];
};

static const struct device* cdc_uart;

static void slip_packet_handler(const uint8_t* data, uint16_t len, void* user_data) {
    ARG_UNUSED(user_data);

    if (len < sizeof(slip_hid_packet_t)) {
        LOG_WRN("CDC packet too small: %u", len);
        return;
    }

    const slip_hid_packet_t* msg = (const slip_hid_packet_t*) data;
    uint16_t payload_len = len - sizeof(slip_hid_packet_t);

    if ((msg->protocol_version != SLIP_PROTOCOL_VERSION) ||
        (msg->len != payload_len) ||
        (payload_len > 64) ||
        (msg->our_descriptor_number >= NOUR_DESCRIPTORS) ||
        ((msg->report_id == 0) && (payload_len >= 64))) {
        LOG_WRN("Invalid CDC packet");
        return;
    }

    queue_virtual_hid_report(CDC_VIRTUAL_INTERFACE, msg->report_id, msg->data, (uint8_t) payload_len);
}

void usb_cdc_input_init(void) {
    cdc_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_cdc_acm_uart));
    if (!device_is_ready(cdc_uart)) {
        LOG_WRN("CDC ACM UART not ready");
        cdc_uart = NULL;
        return;
    }
    slip_input_reset();
    LOG_INF("USB CDC input ready");
}

void usb_cdc_input_poll(void) {
    if (!cdc_uart) {
        return;
    }

    uint8_t byte;
    while (uart_poll_in(cdc_uart, &byte) == 0) {
        slip_input_process_bytes(&byte, 1, slip_packet_handler, NULL);
    }
}
