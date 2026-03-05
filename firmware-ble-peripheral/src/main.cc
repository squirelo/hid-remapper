#include <errno.h>
#include <stddef.h>
#include <cstring>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/types.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>

#include "descriptors.h"

LOG_MODULE_REGISTER(ble_hid, LOG_LEVEL_DBG);

#define CHK(X) ({ int err = X; if (err != 0) { LOG_ERR("%s returned %d (%s:%d)", #X, err, __FILE__, __LINE__); } err == 0; })

// ---------------------------------------------------------------------------
// BLE NUS UUIDs (Nordic UART Service for Web Bluetooth compatibility)
// ---------------------------------------------------------------------------
static struct bt_uuid_128 nus_service_uuid = BT_UUID_INIT_128(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static struct bt_uuid_128 nus_rx_uuid = BT_UUID_INIT_128(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

static struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

// ---------------------------------------------------------------------------
// BLE command protocol (received on NUS RX characteristic)
//
// Byte 0: command
//   0x01 = switch descriptor (byte 1 = descriptor index 0..5)
//   0x02 = send HID report  (byte 1 = report_id, bytes 2..N = report data)
//   0x03 = query current descriptor (response sent on TX notify)
// ---------------------------------------------------------------------------
enum BleCmd : uint8_t {
    CMD_SWITCH_DESCRIPTOR = 0x01,
    CMD_SEND_REPORT       = 0x02,
    CMD_QUERY_DESCRIPTOR  = 0x03,
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static uint8_t current_descriptor_number = 0;
static const descriptor_def_t* current_descriptor = &descriptors[0];
static volatile bool descriptor_switch_pending = false;
static uint8_t pending_descriptor_number = 0;

static const struct device* hid_dev;
static K_SEM_DEFINE(usb_sem, 1, 1);
static bool usb_ready = false;

static struct bt_conn* ble_conn;

// ---------------------------------------------------------------------------
// Hardware: button and LEDs
// ---------------------------------------------------------------------------
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#define LED1_NODE DT_ALIAS(led1)
#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button_cb_data;
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const int CLEAR_BONDS_HOLD_MS = 3000;

// ---------------------------------------------------------------------------
// LED helpers
// ---------------------------------------------------------------------------
enum class LedMode : uint8_t { OFF = 0, ON = 1, BLINK = 2 };

static atomic_t led_mode_val = ATOMIC_INIT((atomic_val_t)LedMode::OFF);
static atomic_t led_blink_count = ATOMIC_INIT(0);
static int led_blinks_left = 0;
static bool next_blink_state = true;

static void led_work_fn(struct k_work* work);
static K_WORK_DELAYABLE_DEFINE(led_work, led_work_fn);

static void led_work_fn(struct k_work* work) {
    LedMode mode = (LedMode)atomic_get(&led_mode_val);
    switch (mode) {
        case LedMode::OFF:
            gpio_pin_set_dt(&led1, false);
            break;
        case LedMode::ON:
            gpio_pin_set_dt(&led1, true);
            break;
        case LedMode::BLINK: {
            int next_ms = 0;
            if (next_blink_state) {
                if (led_blinks_left > 0) {
                    led_blinks_left--;
                    gpio_pin_set_dt(&led1, true);
                    next_blink_state = false;
                    next_ms = 100;
                } else {
                    led_blinks_left = atomic_get(&led_blink_count);
                    gpio_pin_set_dt(&led1, false);
                    next_ms = 1000;
                }
            } else {
                gpio_pin_set_dt(&led1, false);
                next_blink_state = true;
                next_ms = 100;
            }
            k_work_reschedule(&led_work, K_MSEC(next_ms));
            break;
        }
    }
}

static void set_led_mode(LedMode m) {
    if (atomic_set(&led_mode_val, (atomic_val_t)m) != (atomic_val_t)m) {
        k_work_reschedule(&led_work, K_NO_WAIT);
    }
}

static void activity_led_off_fn(struct k_work* work) { gpio_pin_set_dt(&led0, false); }
static K_WORK_DELAYABLE_DEFINE(activity_led_off_work, activity_led_off_fn);

static void activity_flash() {
    gpio_pin_set_dt(&led0, true);
    k_work_reschedule(&activity_led_off_work, K_MSEC(50));
}

// ---------------------------------------------------------------------------
// USB HID
// ---------------------------------------------------------------------------
extern struct usb_device_descriptor __usb_descriptor_start[];

static void int_in_ready_cb(const struct device* dev) {
    k_sem_give(&usb_sem);
}

static int get_report_cb(const struct device* dev, struct usb_setup_packet* setup,
                         int32_t* len, uint8_t** data) {
    LOG_DBG("get_report");
    *len = 0;
    return 0;
}

static int set_report_cb(const struct device* dev, struct usb_setup_packet* setup,
                         int32_t* len, uint8_t** data) {
    LOG_DBG("set_report len=%d", *len);
    return 0;
}

static const struct hid_ops hid_ops = {
    .get_report = get_report_cb,
    .set_report = set_report_cb,
    .int_in_ready = int_in_ready_cb,
};

static void status_cb(enum usb_dc_status_code status, const uint8_t* param) {
    (void)param;
    (void)status;
}

static bool send_hid_report(const uint8_t* report, uint8_t len) {
    if (!usb_ready || !hid_dev) return false;
    if (k_sem_take(&usb_sem, K_NO_WAIT) != 0) return false;
    if (!CHK(hid_int_ep_write(hid_dev, report, len, NULL))) {
        k_sem_give(&usb_sem);
        return false;
    }
    return true;
}

static void usb_init_with_descriptor() {
    hid_dev = device_get_binding("HID_0");
    if (!hid_dev) {
        LOG_ERR("Cannot get USB HID device");
        return;
    }

    usb_hid_register_device(hid_dev,
                            current_descriptor->descriptor,
                            current_descriptor->descriptor_length,
                            &hid_ops);
    CHK(usb_hid_init(hid_dev));
    CHK(usb_enable(status_cb));
    usb_ready = true;

    LOG_INF("USB HID active with descriptor %d (VID=0x%04X PID=0x%04X)",
            current_descriptor->idx,
            current_descriptor->vid,
            current_descriptor->pid);
}

static void apply_vid_pid() {
    if (current_descriptor->vid && current_descriptor->pid) {
        struct usb_device_descriptor* dd = __usb_descriptor_start;
        dd->idVendor  = current_descriptor->vid;
        dd->idProduct = current_descriptor->pid;
    }
}

// Soft-reconnect: cycle USB to present a different descriptor to the host.
// Falls back to reboot if the USB stack doesn't cooperate.
static void switch_usb_descriptor(uint8_t new_idx) {
    if (new_idx >= NOUR_DESCRIPTORS || new_idx == current_descriptor_number) return;

    LOG_INF("Switching descriptor %d -> %d", current_descriptor_number, new_idx);

    current_descriptor_number = new_idx;
    current_descriptor = &descriptors[new_idx];

    // Persist choice so it survives power-cycles
    settings_save_one("hid/desc", &current_descriptor_number, sizeof(current_descriptor_number));

    // Try soft-reconnect
    usb_ready = false;
    int err = usb_disable();
    if (err && err != -EALREADY) {
        LOG_WRN("usb_disable returned %d, rebooting", err);
        sys_reboot(SYS_REBOOT_WARM);
        return;
    }

    k_sleep(K_MSEC(100));

    apply_vid_pid();
    usb_hid_register_device(hid_dev,
                            current_descriptor->descriptor,
                            current_descriptor->descriptor_length,
                            &hid_ops);

    err = usb_hid_init(hid_dev);
    if (err && err != -EALREADY) {
        LOG_WRN("usb_hid_init returned %d after re-register, rebooting", err);
        sys_reboot(SYS_REBOOT_WARM);
        return;
    }

    err = usb_enable(status_cb);
    if (err && err != -EALREADY) {
        LOG_WRN("usb_enable returned %d, rebooting", err);
        sys_reboot(SYS_REBOOT_WARM);
        return;
    }

    k_sem_give(&usb_sem);
    usb_ready = true;

    LOG_INF("USB re-enabled with descriptor %d", new_idx);
}

// ---------------------------------------------------------------------------
// BLE NUS service
// ---------------------------------------------------------------------------
static void handle_ble_command(const uint8_t* buf, uint16_t len);

static ssize_t nus_rx_write_cb(struct bt_conn* conn, const struct bt_gatt_attr* attr,
                                const void* buf, uint16_t len, uint16_t offset, uint8_t flags) {
    handle_ble_command((const uint8_t*)buf, len);
    return len;
}

static void nus_ccc_changed(const struct bt_gatt_attr* attr, uint16_t value) {
    LOG_DBG("NUS CCC changed: %d", value);
}

BT_GATT_SERVICE_DEFINE(nus_service,
    BT_GATT_PRIMARY_SERVICE(&nus_service_uuid),
    BT_GATT_CHARACTERISTIC(&nus_rx_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, nus_rx_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(nus_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void handle_ble_command(const uint8_t* buf, uint16_t len) {
    if (len < 1) return;
    activity_flash();

    switch (buf[0]) {
        case CMD_SWITCH_DESCRIPTOR:
            if (len >= 2 && buf[1] < NOUR_DESCRIPTORS) {
                pending_descriptor_number = buf[1];
                descriptor_switch_pending = true;
                LOG_INF("BLE: descriptor switch requested -> %d", buf[1]);
            }
            break;

        case CMD_SEND_REPORT:
            if (len >= 2) {
                send_hid_report(buf + 1, len - 1);
            }
            break;

        case CMD_QUERY_DESCRIPTOR: {
            if (!ble_conn) break;
            uint8_t resp = current_descriptor_number;
            const struct bt_gatt_attr* tx_attr = &nus_service.attrs[4];
            bt_gatt_notify(ble_conn, tx_attr, &resp, 1);
            break;
        }

        default:
            LOG_WRN("Unknown BLE command 0x%02X", buf[0]);
            break;
    }
}

// ---------------------------------------------------------------------------
// BLE advertising
// ---------------------------------------------------------------------------
static void start_advertising() {
    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
        .peer = NULL,
    };

    // Main ad: flags + UUID only (20 bytes) - must stay under 31 bytes for connectable display
    static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
            0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
            0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e),
    };

    // Name in scan response to avoid overflowing main ad (31 byte limit)
    static const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, "PlayAbility Receiver", sizeof("PlayAbility Receiver") - 1),
    };

    CHK(bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd)));
    LOG_INF("Advertising started");
}

// ---------------------------------------------------------------------------
// BLE connection callbacks
// ---------------------------------------------------------------------------
static void ble_connected(struct bt_conn* conn, uint8_t err) {
    if (err) {
        LOG_ERR("BLE connect failed (err %u)", err);
        return;
    }
    ble_conn = bt_conn_ref(conn);
    set_led_mode(LedMode::ON);
    LOG_INF("BLE connected");
}

static void ble_disconnected(struct bt_conn* conn, uint8_t reason) {
    LOG_INF("BLE disconnected (reason %u)", reason);
    if (ble_conn) {
        bt_conn_unref(ble_conn);
        ble_conn = NULL;
    }
    set_led_mode(LedMode::BLINK);
    start_advertising();
}

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected = ble_connected,
    .disconnected = ble_disconnected,
};

static void auth_cancel(struct bt_conn* conn) { LOG_WRN("auth cancel"); }
static struct bt_conn_auth_cb auth_cbs = { .cancel = auth_cancel };

// ---------------------------------------------------------------------------
// Button: short press cycles descriptor, long press clears bonds
// ---------------------------------------------------------------------------
static int64_t button_pressed_at;

static void button_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    int state = gpio_pin_get(dev, button.pin);
    if (state) {
        button_pressed_at = k_uptime_get();
    } else {
        if (k_uptime_get() - button_pressed_at > CLEAR_BONDS_HOLD_MS) {
            static auto addr_any = BT_ADDR_LE_ANY[0];
            CHK(bt_unpair(BT_ID_DEFAULT, &addr_any));
            LOG_INF("Bonds cleared");
        } else {
            uint8_t next = (current_descriptor_number + 1) % NOUR_DESCRIPTORS;
            pending_descriptor_number = next;
            descriptor_switch_pending = true;
            LOG_INF("Button: switching to descriptor %d", next);
        }
    }
}

// ---------------------------------------------------------------------------
// Settings: persist current descriptor choice
// ---------------------------------------------------------------------------
static int settings_set_cb(const char* name, size_t len, settings_read_cb read_cb, void* cb_arg) {
    if (len != 1) return -EINVAL;
    uint8_t val;
    if (read_cb(cb_arg, &val, 1) != 1) return -EIO;
    if (val < NOUR_DESCRIPTORS) {
        current_descriptor_number = val;
        current_descriptor = &descriptors[val];
        LOG_INF("Loaded descriptor %d from settings", val);
    }
    return 0;
}

static struct settings_handler settings_handlers = {
    .name = "hid",
    .h_set = settings_set_cb,
};

// ---------------------------------------------------------------------------
// Init helpers
// ---------------------------------------------------------------------------
static void button_init() {
    if (!device_is_ready(button.port)) { LOG_ERR("button not ready"); return; }
    CHK(gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW));
    CHK(gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH));
    gpio_init_callback(&button_cb_data, button_cb, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
}

static void leds_init() {
    if (device_is_ready(led0.port)) { CHK(gpio_pin_configure_dt(&led0, GPIO_OUTPUT)); gpio_pin_set_dt(&led0, false); }
    if (device_is_ready(led1.port)) { CHK(gpio_pin_configure_dt(&led1, GPIO_OUTPUT)); gpio_pin_set_dt(&led1, false); }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    LOG_INF("PlayAbility Receiver starting");

    // Early LED blink: if you see this, the app at least reached main()
    leds_init();
    gpio_pin_set_dt(&led0, true);
    k_sleep(K_MSEC(200));
    gpio_pin_set_dt(&led0, false);
    gpio_pin_set_dt(&led1, true);
    k_sleep(K_MSEC(200));
    gpio_pin_set_dt(&led1, false);
    k_sleep(K_MSEC(200));

    button_init();

    CHK(bt_conn_auth_cb_register(&auth_cbs));
    CHK(bt_enable(NULL));

    CHK(settings_subsys_init());
    CHK(settings_register(&settings_handlers));
    settings_load();

    apply_vid_pid();
    usb_init_with_descriptor();

    start_advertising();
    set_led_mode(LedMode::BLINK);
    atomic_set(&led_blink_count, current_descriptor_number + 1);

    LOG_INF("Ready. Active descriptor: %d", current_descriptor_number);

    while (true) {
        if (descriptor_switch_pending) {
            descriptor_switch_pending = false;
            switch_usb_descriptor(pending_descriptor_number);
            atomic_set(&led_blink_count, current_descriptor_number + 1);
        }

        k_sleep(K_MSEC(1));
    }

    return 0;
}
