#ifndef SLIP_INPUT_H_
#define SLIP_INPUT_H_

#include <stdint.h>

typedef void (*slip_input_packet_cb_t)(const uint8_t* data, uint16_t len, void* user_data);

void slip_input_reset(void);
void slip_input_process_bytes(const uint8_t* data, uint16_t len, slip_input_packet_cb_t cb, void* user_data);

#endif
