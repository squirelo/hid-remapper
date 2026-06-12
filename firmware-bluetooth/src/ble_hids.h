#ifndef BLE_HIDS_H_

#define BLE_HIDS_H_



#include <stdint.h>



struct bt_conn;

struct bt_hids;



#ifdef __cplusplus

extern "C" {

#endif



struct bt_hids* ble_hids_get_instance(void);



void ble_hids_init(void);

void ble_hids_on_connected(struct bt_conn* conn);

void ble_hids_on_disconnected(struct bt_conn* conn);

bool ble_hids_try_send_report(const uint8_t* report_with_id, uint8_t len);

bool ble_hids_host_connected(void);

uint16_t ble_hids_appearance(void);



#ifdef __cplusplus

}

#endif



#endif

