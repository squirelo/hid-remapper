#include "slip_input.h"

#include "crc.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(slip_input, LOG_LEVEL_DBG);

#define SLIP_PACKET_BUFFER_SIZE 512
#define SLIP_END 0300
#define SLIP_ESC 0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335

static uint8_t slip_packet_buffer[SLIP_PACKET_BUFFER_SIZE];
static uint16_t slip_bytes_read = 0;
static bool slip_escaped = false;
static bool slip_overflowed = false;

void slip_input_reset(void) {
    slip_bytes_read = 0;
    slip_escaped = false;
    slip_overflowed = false;
}

static void slip_packet_append(uint8_t c) {
    if (slip_bytes_read >= sizeof(slip_packet_buffer)) {
        if (!slip_overflowed) {
            LOG_WRN("SLIP packet too large; dropping until frame end");
        }
        slip_overflowed = true;
        return;
    }
    slip_packet_buffer[slip_bytes_read++] = c;
}

static void slip_process_byte(uint8_t c, slip_input_packet_cb_t cb, void* user_data) {
    if (slip_escaped) {
        switch (c) {
            case SLIP_ESC_END:
                slip_packet_append(SLIP_END);
                break;
            case SLIP_ESC_ESC:
                slip_packet_append(SLIP_ESC);
                break;
            default:
                slip_packet_append(c);
                break;
        }
        slip_escaped = false;
        return;
    }

    switch (c) {
        case SLIP_END:
            if (!slip_overflowed && (slip_bytes_read > 4)) {
                uint32_t crc = crc32(slip_packet_buffer, slip_bytes_read - 4);
                uint32_t received_crc =
                    (slip_packet_buffer[slip_bytes_read - 4] << 0) |
                    (slip_packet_buffer[slip_bytes_read - 3] << 8) |
                    (slip_packet_buffer[slip_bytes_read - 2] << 16) |
                    (slip_packet_buffer[slip_bytes_read - 1] << 24);
                if (crc == received_crc) {
                    cb(slip_packet_buffer, slip_bytes_read - 4, user_data);
                } else {
                    LOG_WRN("SLIP CRC mismatch");
                }
            }
            slip_bytes_read = 0;
            slip_escaped = false;
            slip_overflowed = false;
            break;
        case SLIP_ESC:
            if (!slip_overflowed) {
                slip_escaped = true;
            }
            break;
        default:
            slip_packet_append(c);
            break;
    }
}

void slip_input_process_bytes(const uint8_t* data, uint16_t len, slip_input_packet_cb_t cb, void* user_data) {
    for (uint16_t i = 0; i < len; i++) {
        slip_process_byte(data[i], cb, user_data);
    }
}
