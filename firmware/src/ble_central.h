#ifndef _BLE_CENTRAL_H_
#define _BLE_CENTRAL_H_

#include <stdbool.h>
#include <stdint.h>

void ble_central_init(void);
void ble_central_pair_new_device(void);
void ble_central_clear_bonds(void);
void ble_central_poll(void);
int ble_central_connected_count(void);
bool ble_central_is_scanning(void);

#endif
