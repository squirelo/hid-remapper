#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(remapper, LOG_LEVEL_DBG);

#define CHK(X) ({ int err = X; if (err != 0) { LOG_ERR("%s returned %d (%s:%d)", #X, err, __FILE__, __LINE__); } err == 0; })

typedef struct {
    uint16_t buttons;
    uint8_t hat;
    int8_t x;
    int8_t y;
    int8_t z;
    int8_t rz;
} hid_gamepad_report_t;
// HID report descriptor for gamepad
static const uint8_t hid_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x35, 0x00,        //   Physical Minimum (0)
    0x45, 0x01,        //   Physical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x10,        //   Report Count (16)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x10,        //   Usage Maximum (0x10)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x25, 0x07,        //   Logical Maximum (7)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
    0x09, 0x39,        //   Usage (Hat switch)
    0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x65, 0x00,        //   Unit (None)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x46, 0xFF, 0x00,  //   Physical Maximum (255)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};

static const struct device* hid_dev;
/ Add this forward declaration before the attrs array
static ssize_t write_cb(struct bt_conn* conn, const struct bt_gatt_attr* attr,
                        const void* buf, uint16_t len, uint16_t offset, uint8_t flags);

static struct bt_uuid_128 uart_service_uuid = BT_UUID_INIT_128(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static struct bt_uuid_128 uart_rx_uuid = BT_UUID_INIT_128(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static struct bt_uuid_128 uart_tx_uuid = BT_UUID_INIT_128(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static struct bt_gatt_attr attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&uart_service_uuid),
    BT_GATT_CHARACTERISTIC(&uart_rx_uuid.uuid, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&uart_tx_uuid.uuid, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
};

static struct bt_gatt_service uart_svc = BT_GATT_SERVICE(attrs);

static void connected(struct bt_conn* conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
    } else {
        LOG_INF("Connected");
    }
}

static void disconnected(struct bt_conn* conn, uint8_t reason) {
    LOG_INF("Disconnected (reason %u)", reason);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static ssize_t write_cb(struct bt_conn* conn, const struct bt_gatt_attr* attr,
                        const void* buf, uint16_t len, uint16_t offset, uint8_t flags) {
    LOG_INF("Received data, len %d", len);
    
    // Process the received gamepad data and send it to USB HID
    if (len == sizeof(hid_gamepad_report_t)) {
        const hid_gamepad_report_t* gp = (const hid_gamepad_report_t*)buf;
        uint8_t report_with_id[sizeof(hid_gamepad_report_t) + 1];
        report_with_id[0] = 0; // Assuming report ID 0, adjust if needed
        memcpy(report_with_id + 1, gp, sizeof(hid_gamepad_report_t));
        
        if (hid_dev0) {
            CHK(hid_int_ep_write(hid_dev0, report_with_id, sizeof(report_with_id), NULL));
        }
    }

    return len;
}

static int set_report_cb(const struct device* dev, struct usb_setup_packet* setup, int32_t* len, uint8_t** data) {
    LOG_INF("Set report callback");
    return 0;
}

static int get_report_cb(const struct device* dev, struct usb_setup_packet* setup, int32_t* len, uint8_t** data) {
    LOG_INF("Get report callback");
    return 0;
}

static void int_in_ready_cb(const struct device* dev) {
    LOG_INF("INT IN ready");
}

static const struct hid_ops ops = {
    .get_report = get_report_cb,
    .set_report = set_report_cb,
    .int_in_ready = int_in_ready_cb,
};

static void bt_ready(int err) {
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    CHK(bt_gatt_service_register(&uart_svc));

    struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
        BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);

    CHK(bt_le_adv_start(&adv_param, NULL, 0, NULL, 0));
    LOG_INF("Advertising started");
}



int main(void) {
    int err;

    LOG_INF("Starting HID Remapper Bluetooth");

    hid_dev = device_get_binding("HID_0");
    if (hid_dev == NULL) {
        LOG_ERR("Cannot get USB HID Device");
        return -1;
    }

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), &ops);
    CHK(usb_hid_init(hid_dev));

    CHK(usb_enable(NULL));

    bt_conn_cb_register(&conn_callbacks);

    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return -1;
    }

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;  // This line will never be reached, but it satisfies the compiler
}