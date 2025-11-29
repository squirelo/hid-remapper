#ifndef NUS_SERVICE_H
#define NUS_SERVICE_H

#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>

// Interface identifier for NUS gamepad (reserved value to avoid conflicts with BLE Central gamepads)
#define NUS_GAMEPAD_INTERFACE 0xFF00

// Initialize NUS service and start advertising
int nus_service_init();

// Check if a connection is a NUS (peripheral) connection
bool nus_is_nus_connection(struct bt_conn* conn);

// Handle NUS connection (call from main connection callbacks)
void nus_handle_connected(struct bt_conn* conn);
void nus_handle_disconnected(struct bt_conn* conn);

#endif // NUS_SERVICE_H

