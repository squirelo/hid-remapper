#ifndef _JOYCON_H_
#define _JOYCON_H_

#include <stdint.h>

// Original Bluetooth Joy-Cons only. A matching descriptor is probed for its
// controller type before joining one L/R pair. Non-Joy-Cons use generic HID.
bool joycon_attach(uint16_t cid, uint16_t interface, uint8_t port, const uint8_t* descriptor, uint16_t len);
bool joycon_report(uint16_t interface, const uint8_t* report, uint16_t len);
bool joycon_detach(uint16_t interface);
void joycon_task();

#endif
