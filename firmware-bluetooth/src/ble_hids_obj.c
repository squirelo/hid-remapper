#include <bluetooth/services/hids.h>

BT_HIDS_DEF(hids_obj,
            64, 64, 64, 64, 64, 64, 64, 64,
            64, 64, 64, 64,
            64, 64, 64, 64);

struct bt_hids* ble_hids_get_instance(void) {
    return &hids_obj;
}
