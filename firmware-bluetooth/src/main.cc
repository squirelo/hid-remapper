
#include <errno.h>
#include <stddef.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/types.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/console/console.h>


#include "config.h"
#include "descriptor_parser.h"
#include "globals.h"
#include "our_descriptor.h"
#include "platform.h"
#include "remapper.h"

LOG_MODULE_REGISTER(remapper, LOG_LEVEL_DBG);

#define CHK(X) ({ int err = X; if (err != 0) { LOG_ERR("%s returned %d (%s:%d)", #X, err, __FILE__, __LINE__); } err == 0; })

static const int SCAN_DELAY_MS = 1000;
static const int CLEAR_BONDS_BUTTON_PRESS_MS = 3000;

// these macros don't work in C++ when used directly ("taking address of temporary array")
static auto const BT_UUID_HIDS_ = (struct bt_uuid_16) BT_UUID_INIT_16(BT_UUID_HIDS_VAL);
static auto BT_ADDR_LE_ANY_ = BT_ADDR_LE_ANY[0];
static auto BT_CONN_LE_CREATE_CONN_ = BT_CONN_LE_CREATE_CONN[0];

#define GAMEPAD_SERVICE_UUID        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define GAMEPAD_CHARACTERISTIC_UUID BT_UUID_128_ENCODE(0x87654321, 0x4321, 0x8765, 0x4321, 0xfedcba987654)

static struct bt_uuid_128 gamepad_service_uuid = BT_UUID_INIT_128(GAMEPAD_SERVICE_UUID);
static struct bt_uuid_128 gamepad_char_uuid = BT_UUID_INIT_128(GAMEPAD_CHARACTERISTIC_UUID);

static struct bt_gatt_service gamepad_svc;
static struct bt_gatt_chr_def gamepad_chars[2];
static struct bt_gatt_service_static gamepad_service;

struct report_type {
    uint8_t conn_idx;
    uint8_t len;
    uint8_t data[65];
};

struct descriptor_type {
    uint16_t size;
    uint8_t conn_idx;
    uint8_t data[512];
};

struct hogp_ready_type {
    struct bt_hogp* hogp;
};

struct disconnected_type {
    uint8_t conn_idx;
};

K_MSGQ_DEFINE(report_q, sizeof(struct report_type), 16, 4);
K_MSGQ_DEFINE(descriptor_q, sizeof(struct descriptor_type), 2, 4);
K_MSGQ_DEFINE(hogp_ready_q, sizeof(struct hogp_ready_type), CONFIG_BT_MAX_CONN, 4);
K_MSGQ_DEFINE(disconnected_q, sizeof(struct disconnected_type), CONFIG_BT_MAX_CONN, 4);
ATOMIC_DEFINE(tick_pending, 1);

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

static K_SEM_DEFINE(usb_sem0, 1, 1);
static K_SEM_DEFINE(usb_sem1, 1, 1);
static K_SEM_DEFINE(usb_sem_gamepad, 1, 1);

static struct k_mutex mutexes[(uint8_t) MutexId::N];

static const struct device* hid_dev0;
static const struct device* hid_dev1;  // config interface
static const struct device* hid_dev_gamepad;  // gamepad interface

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
#define UART_BUFFER_SIZE 64

static const struct device *uart_dev;
static uint8_t uart_buffer[UART_BUFFER_SIZE];
static size_t uart_buffer_pos;

struct gamepad_report {
    int8_t x;
    int8_t y;
    int8_t z;
    int8_t rz;
    int8_t rx;
    int8_t ry;
    uint8_t hat;
    uint32_t buttons;
} __packed;

static struct gamepad_report gp_report;

static ssize_t write_gamepad(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset + len > sizeof(gp_report)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(((uint8_t *)&gp_report) + offset, buf, len);
    
    // Send the updated gamepad report over USB
    send_gamepad_report(hid_dev_gamepad);

    return len;
}

static void send_gamepad_report(const struct device *dev) {
    if (k_sem_take(&usb_sem_gamepad, K_NO_WAIT) == 0) {
        hid_int_ep_write(dev, (uint8_t *)&gp_report, sizeof(gp_report), NULL);
    }
}

static void gamepad_service_init(void)
{
    gamepad_chars[0] = BT_GATT_CHARACTERISTIC(&gamepad_char_uuid.uuid,
                                              BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_gamepad, NULL);
    gamepad_chars[1] = BT_GATT_CHARACTERISTIC_END;

    BT_GATT_SERVICE_DEFINE(gamepad_svc,
        BT_GATT_PRIMARY_SERVICE(&gamepad_service_uuid),
        gamepad_chars[0],
        gamepad_chars[1],
    );
}

static void activity_led_off_work_fn(struct k_work* work) {
    gpio_pin_set_dt(&led0, false);
}
static K_WORK_DELAYABLE_DEFINE(activity_led_off_work, activity_led_off_work_fn);

enum class LedMode {
    OFF = 0,
    ON = 1,
    BLINK = 2,
};

static atomic_t led_blink_count = ATOMIC_INIT(0);
static atomic_t led_mode = (atomic_t) ATOMIC_INIT(LedMode::OFF);
static int led_blinks_left = 0;
static bool next_blink_state = true;

static void led_work_fn(struct k_work* work);
static K_WORK_DELAYABLE_DEFINE(led_work, led_work_fn);

static void led_work_fn(struct k_work* work) {
    LedMode my_led_mode = (LedMode) atomic_get(&led_mode);
    switch (my_led_mode) {
        case LedMode::OFF:
            gpio_pin_set_dt(&led1, false);
            break;
        case LedMode::ON:
            gpio_pin_set_dt(&led1, true);
            break;
        case LedMode::BLINK: {
            int next_work = 0;
            if (next_blink_state) {
                if (led_blinks_left > 0) {
                    led_blinks_left--;
                    gpio_pin_set_dt(&led1, true);
                    next_blink_state = false;
                    next_work = 100;
                } else {
                    led_blinks_left = atomic_get(&led_blink_count);
                    gpio_pin_set_dt(&led1, false);
                    next_work = 1000;
                }
            } else {
                gpio_pin_set_dt(&led1, false);
                next_blink_state = true;
                next_work = 100;
            }
            k_work_reschedule(&led_work, K_MSEC(next_work));
            break;
        }
    }
}

static void set_led_mode(LedMode led_mode_) {
    if (atomic_set(&led_mode, (atomic_val_t) led_mode_) != (atomic_val_t) led_mode_) {
        k_work_reschedule(&led_work, K_NO_WAIT);
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        LOG_ERR("Failed to connect to %s (%u)", addr, err);
        return;
    }

    LOG_INF("Connected: %s", addr);
    set_led_mode(LedMode::ON);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", addr, reason);

    set_led_mode(LedMode::BLINK);

    // Restart advertising
    int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    }
}

static void security_changed(struct bt_conn* conn, bt_security_t level, enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_INF("%s, level=%u.", addr, level);
    } else {
        LOG_ERR("security failed: %s, level=%u, err=%d", addr, level, err);
    }
}

static void le_param_updated(struct bt_conn* conn, uint16_t interval, uint16_t latency, uint16_t timeout) {
    LOG_INF("interval=%u, latency=%u, timeout=%u", interval, latency, timeout);
}

static bool le_param_req(struct bt_conn* conn, struct bt_le_conn_param* param) {
    LOG_INF("interval_min=%d, interval_max=%d, latency=%d, timeout=%d", param->interval_min, param->interval_max, param->latency, param->timeout);
    param->interval_max = param->interval_min;
    return true;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_req = le_param_req,
    .le_param_updated = le_param_updated,
    .security_changed = security_changed,
};

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    gamepad_service_init();

    // Prepare advertising data
    static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, GAMEPAD_SERVICE_UUID),
    };

    // Start advertising
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
    set_led_mode(LedMode::BLINK);
}

static void bt_init()
{
    if (!CHK(bt_enable(bt_ready))) {
        return;
    }
}

static int64_t button_pressed_at;

static void button_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    int button_state = gpio_pin_get(dev, button.pin);
    if (button_state) {
        button_pressed_at = k_uptime_get();
    } else {
        if (k_uptime_get() - button_pressed_at > CLEAR_BONDS_BUTTON_PRESS_MS) {
            clear_bonds();
        } else {
            pair_new_device();
        }
    }
}

static int set_report_cb(const struct device* dev, struct usb_setup_packet* setup, int32_t* len, uint8_t** data) {
    uint8_t request_value[2];

    // report_id, report_type
    sys_put_le16(setup->wValue, request_value);

    LOG_INF("report_id=%d, report_type=%d, len=%d", request_value[0], request_value[1], *len);
    LOG_HEXDUMP_DBG((*data), (uint32_t) *len, "");

    if ((request_value[0] > 0) && (*len > 0)) {
        if (dev == hid_dev0) {
            handle_set_report0(request_value[0], (*data) + 1, (*len) - 1);
        } else if (dev == hid_dev1) {
            handle_set_report1(request_value[0], (*data) + 1, (*len) - 1);
        }
    } else {
        LOG_ERR("no report ID?");
    }

    return 0;
};

static int get_report_cb(const struct device* dev, struct usb_setup_packet* setup, int32_t* len, uint8_t** data) {
    uint8_t request_value[2];

    sys_put_le16(setup->wValue, request_value);

    LOG_INF("report_id=%d, %d, len=%d", request_value[0], request_value[1], *len);

    *data[0] = request_value[0];
    if (dev == hid_dev0) {
        *len = handle_get_report0(request_value[0], (*data) + 1, CONFIG_SIZE);
    } else if (dev == hid_dev1) {
        *len = handle_get_report1(request_value[0], (*data) + 1, CONFIG_SIZE);
    }
    (*len)++;

    return 0;
};
static void uart_cb(const struct device *dev, void *user_data);
static void process_uart_data(uint8_t *data, size_t len);

static void uart_cb(const struct device *dev, void *user_data) {
    uint8_t data;
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uart_fifo_read(dev, &data, 1);
            uart_buffer[uart_buffer_pos++] = data;

            if (uart_buffer_pos == UART_BUFFER_SIZE || data == '\n') {
                uart_buffer[uart_buffer_pos] = '\0';
                process_uart_data(uart_buffer, uart_buffer_pos);
                uart_buffer_pos = 0;
            }
        }
    }
}

static void process_uart_data(uint8_t *data, size_t len) {
    // Process the data received from UART
    // For example, print it to the console or parse commands
    printk("Received data: %s\n", data);

    // Add your specific data handling code here
}

static void uart_init(void) {
    uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready\n");
        return;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);
}

static void int_in_ready_cb0(const struct device* dev) {
    k_sem_give(&usb_sem0);
}

static void int_in_ready_cb1(const struct device* dev) {
    k_sem_give(&usb_sem1);
}

static const struct hid_ops ops0 = {
    .get_report = get_report_cb,
    .set_report = set_report_cb,
    .int_in_ready = int_in_ready_cb0,
};

static const struct hid_ops ops1 = {
    .get_report = get_report_cb,
    .set_report = set_report_cb,
    .int_in_ready = int_in_ready_cb1,
};

static bool do_send_report(uint8_t interface, const uint8_t* report_with_id, uint8_t len) {
    if (report_with_id[0] == 0) {
        report_with_id++;
        len--;
    }
    if (interface == 0) {
        return CHK(hid_int_ep_write(hid_dev0, report_with_id, len, NULL));
    }
    if (interface == 1) {
        return CHK(hid_int_ep_write(hid_dev1, report_with_id, len, NULL));
    }
}

static void button_init() {
    if (!device_is_ready(button.port)) {
        LOG_ERR("button device %s is not ready", button.port->name);
        return;
    }

    if (!CHK(gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW))) {
        return;
    }

    if (!CHK(gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH))) {
        return;
    }

    gpio_init_callback(&button_cb_data, button_cb, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
}

static void leds_init() {
    if (device_is_ready(led0.port)) {
        CHK(gpio_pin_configure_dt(&led0, GPIO_OUTPUT));
        gpio_pin_set_dt(&led0, false);
    } else {
        LOG_ERR("led0 device %s is not ready", led0.port->name);
    }

    if (device_is_ready(led1.port)) {
        CHK(gpio_pin_configure_dt(&led1, GPIO_OUTPUT));
        gpio_pin_set_dt(&led1, false);
    } else {
        LOG_ERR("led1 device %s is not ready", led1.port->name);
    }
}

static void status_cb(enum usb_dc_status_code status, const uint8_t* param) {
    if (status == USB_DC_SOF) {
        atomic_set_bit(tick_pending, 0);
    }
}

extern struct usb_device_descriptor __usb_descriptor_start[];

static void descriptor_init() {
    our_descriptor = &our_descriptors[our_descriptor_number];
    if ((our_descriptor->vid != 0) && (our_descriptor->pid != 0)) {
        struct usb_device_descriptor* device_descriptor = __usb_descriptor_start;
        device_descriptor->idVendor = our_descriptor->vid;
        device_descriptor->idProduct = our_descriptor->pid;
    }
}

static void usb_init() {
    hid_dev0 = device_get_binding("HID_0");
    if (hid_dev0 == NULL) {
        LOG_ERR("Cannot get USB HID Device 0.");
        return;
    }

    hid_dev1 = device_get_binding("HID_1");
    if (hid_dev1 == NULL) {
        LOG_ERR("Cannot get USB HID Device 1.");
        return;
    }

    hid_dev_gamepad = device_get_binding("HID_2");
    if (hid_dev_gamepad == NULL) {
        LOG_ERR("Cannot get USB HID Device for Gamepad.");
        return;
    }

    usb_hid_register_device(hid_dev0, our_descriptor->descriptor, our_descriptor->descriptor_length, &ops0);
    usb_hid_register_device(hid_dev1, config_report_descriptor, config_report_descriptor_length, &ops1);
    usb_hid_register_device(hid_dev_gamepad, hid_gamepad_report_desc, sizeof(hid_gamepad_report_desc), &ops_gamepad);

    CHK(usb_hid_init(hid_dev0));
    CHK(usb_hid_init(hid_dev1));
    CHK(usb_hid_init(hid_dev_gamepad));

    CHK(usb_enable(status_cb));
}

static bool peers_only = false;
static struct k_work_delayable scan_start_work;
static struct k_work clear_bonds_work;

void scan_init(void);

int main(void)
{
    LOG_INF("Starting Bluetooth Gamepad Peripheral");

    my_mutexes_init();
    button_init();
    leds_init();
    CHK(settings_subsys_init());
    CHK(settings_register(&our_settings_handlers));
    settings_load();
    descriptor_init();
    usb_init();
    bt_init();
    uart_init();

    while (true) {
        // Handle Bluetooth reports
        struct report_type incoming_report;
        while (!k_msgq_get(&report_q, &incoming_report, K_NO_WAIT)) {
            // Update the gamepad report with the received data
            memcpy(&gp_report, incoming_report.data, MIN(sizeof(gp_report), incoming_report.len));
            
            // Send the updated report over USB
            send_gamepad_report(hid_dev_gamepad);
        }

        // Handle other USB HID reports if needed
        if (!k_sem_take(&usb_sem0, K_NO_WAIT)) {
            if (!send_report(do_send_report)) {
                k_sem_give(&usb_sem0);
            }
        }
        if (!k_sem_take(&usb_sem1, K_NO_WAIT)) {
            if (!send_monitor_report(do_send_report)) {
                k_sem_give(&usb_sem1);
            }
        }

        // Handle other tasks as needed

        k_sleep(K_MSEC(1));
    }

    return 0;
}

static struct settings_handler our_settings_handlers = {
    .name = "remapper",
    .h_set = remapper_settings_set,
};

static const uint8_t hid_gamepad_report_desc[] = {
    // Add your gamepad HID report descriptor here
};

static const struct hid_ops ops_gamepad = {
    .get_report = get_report_cb,
    .set_report = set_report_cb,
    .int_in_ready = int_in_ready_cb_gamepad,
};

static void int_in_ready_cb_gamepad(const struct device *dev)
{
    k_sem_give(&usb_sem_gamepad);
}

void reset_to_bootloader() {
    sys_reboot(DFU_MAGIC_UF2_RESET);
}

void flash_b_side() {
}

void pair_new_device() {
    peers_only = false;
    k_work_reschedule(&scan_start_work, K_MSEC(SCAN_DELAY_MS));
}

void clear_bonds() {
    k_work_submit(&clear_bonds_work);
}

void my_mutexes_init() {
    for (int i = 0; i < (int8_t) MutexId::N; i++) {
        k_mutex_init(&mutexes[i]);
    }
}

void my_mutex_enter(MutexId id) {
    k_mutex_lock(&mutexes[(uint8_t) id], K_FOREVER);
}

void my_mutex_exit(MutexId id) {
    k_mutex_unlock(&mutexes[(uint8_t) id]);
}

uint64_t get_time() {
    return k_uptime_get() * 1000;  // XXX precision?
}

void interval_override_updated() {
}

void queue_out_report(uint16_t interface, uint8_t report_id, const uint8_t* buffer, uint8_t len) {
    // TODO
}

void queue_set_feature_report(uint16_t interface, uint8_t report_id, const uint8_t* buffer, uint8_t len) {
    // TODO
}

void queue_get_feature_report(uint16_t interface, uint8_t report_id, uint8_t len) {
    // TODO
}

void set_gpio_inout_masks(uint32_t in_mask, uint32_t out_mask) {
}
