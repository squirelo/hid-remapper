#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(remapper, LOG_LEVEL_DBG);

#define CHK(X) ({ int err = X; if (err != 0) { LOG_ERR("%s returned %d (%s:%d)", #X, err, __FILE__, __LINE__); } err == 0; })

// HID report descriptor for gamepad
static const uint8_t hid_report_desc[] = {
    // TUD_HID_REPORT_DESC_GAMEPAD()
    // Add the appropriate HID report descriptor for gamepad here
};

static const struct device* hid_dev;

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
                           BT_GATT_PERM_WRITE, NULL, NULL, NULL),
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
        hid_int_ep_write(hid_dev, (const uint8_t*)gp, sizeof(hid_gamepad_report_t), NULL);
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

void main(void) {
    int err;

    LOG_INF("Starting HID Remapper Bluetooth");

    hid_dev = device_get_binding("HID_0");
    if (hid_dev == NULL) {
        LOG_ERR("Cannot get USB HID Device");
        return;
    }

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), &ops);
    CHK(usb_hid_init(hid_dev));

    CHK(usb_enable(NULL));

    bt_conn_cb_register(&conn_callbacks);

    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    while (1) {
        k_sleep(K_FOREVER);
    }
}