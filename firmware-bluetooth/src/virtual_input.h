#ifndef VIRTUAL_INPUT_H_
#define VIRTUAL_INPUT_H_

#include <stdint.h>

void queue_virtual_hid_report(uint16_t interface, uint8_t report_id, const uint8_t* payload, uint8_t payload_len);

#endif
