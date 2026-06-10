#ifndef _BLE_NUS_H_
#define _BLE_NUS_H_

#include <stdint.h>

void ble_nus_init(void);
void ble_nus_start_advertising(void);
void ble_nus_stop_advertising(void);
void ble_nus_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
bool ble_nus_is_peripheral_connection(uint8_t packet_type, uint8_t* packet, uint16_t size);

#endif
