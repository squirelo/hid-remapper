#include "gpio.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "globals.h"
#include "remapper.h"

LOG_MODULE_REGISTER(remapper_gpio, LOG_LEVEL_DBG);

struct logical_gpio_pin_t {
    const struct device* port;
    gpio_pin_t pin;
};

#define GPIO_PIN(port_, pin_) { DEVICE_DT_GET(DT_NODELABEL(port_)), pin_ }

#if defined(CONFIG_BOARD_SEEED_XIAO_NRF52840)

// GPIO usages use the labels printed on the XIAO header (D0-D10), not the
// nRF52840 port/pin numbers. D0 is reserved for the pairing/clear-bonds input.
static const logical_gpio_pin_t logical_gpio_pins[] = {
    GPIO_PIN(gpio0, 2),   // D0  = P0.02 (pairing input)
    GPIO_PIN(gpio0, 3),   // D1  = P0.03
    GPIO_PIN(gpio0, 28),  // D2  = P0.28
    GPIO_PIN(gpio0, 29),  // D3  = P0.29
    GPIO_PIN(gpio0, 4),   // D4  = P0.04
    GPIO_PIN(gpio0, 5),   // D5  = P0.05
    GPIO_PIN(gpio1, 11),  // D6  = P1.11
    GPIO_PIN(gpio1, 12),  // D7  = P1.12
    GPIO_PIN(gpio1, 13),  // D8  = P1.13
    GPIO_PIN(gpio1, 14),  // D9  = P1.14
    GPIO_PIN(gpio1, 15),  // D10 = P1.15
};

static constexpr uint32_t BOARD_GPIO_VALID_PINS_MASK = GENMASK(10, 1);

#elif defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE_SENSE)

// GPIO usages follow the Arduino header labels: D0-D13 followed by A0-A7 as
// D14-D21. D2 is reserved for the pairing/clear-bonds input.
static const logical_gpio_pin_t logical_gpio_pins[] = {
    GPIO_PIN(gpio1, 10),  // D0  / RX
    GPIO_PIN(gpio1, 3),   // D1  / TX
    GPIO_PIN(gpio1, 11),  // D2  (pairing input)
    GPIO_PIN(gpio1, 12),  // D3
    GPIO_PIN(gpio1, 15),  // D4
    GPIO_PIN(gpio1, 13),  // D5
    GPIO_PIN(gpio1, 14),  // D6
    GPIO_PIN(gpio0, 23),  // D7
    GPIO_PIN(gpio0, 21),  // D8
    GPIO_PIN(gpio0, 27),  // D9
    GPIO_PIN(gpio1, 2),   // D10
    GPIO_PIN(gpio1, 1),   // D11 / MOSI
    GPIO_PIN(gpio1, 8),   // D12 / MISO
    GPIO_PIN(gpio0, 13),  // D13 / SCK
    GPIO_PIN(gpio0, 4),   // D14 / A0
    GPIO_PIN(gpio0, 5),   // D15 / A1
    GPIO_PIN(gpio0, 30),  // D16 / A2
    GPIO_PIN(gpio0, 29),  // D17 / A3
    GPIO_PIN(gpio0, 31),  // D18 / A4 / SDA
    GPIO_PIN(gpio0, 2),   // D19 / A5 / SCL
    GPIO_PIN(gpio0, 28),  // D20 / A6
    GPIO_PIN(gpio0, 3),   // D21 / A7
};

static constexpr uint32_t BOARD_GPIO_VALID_PINS_MASK = GENMASK(21, 0) & ~BIT(2);

#else

// Other Bluetooth boards retain their previous behavior until a logical
// header-to-nRF pin map is defined for them.
static const logical_gpio_pin_t logical_gpio_pins[] = {
    GPIO_PIN(gpio0, 0),
};
static constexpr uint32_t BOARD_GPIO_VALID_PINS_MASK = 0;

#endif

static uint32_t gpio_valid_pins_mask;
static uint32_t gpio_in_mask;
static uint32_t gpio_out_mask;
static uint32_t prev_gpio_state;
static uint32_t configured_output_mask;
static uint64_t last_gpio_change[32];
static bool set_gpio_dir_pending;

void gpio_pins_init() {
    gpio_valid_pins_mask = BOARD_GPIO_VALID_PINS_MASK;

    for (uint8_t logical_pin = 0; logical_pin < ARRAY_SIZE(logical_gpio_pins); logical_pin++) {
        uint32_t bit = BIT(logical_pin);
        if (!(gpio_valid_pins_mask & bit)) {
            continue;
        }

        const logical_gpio_pin_t& pin = logical_gpio_pins[logical_pin];
        if (!device_is_ready(pin.port)) {
            LOG_ERR("GPIO %u device %s is not ready", logical_pin, pin.port->name);
            gpio_valid_pins_mask &= ~bit;
        }
    }
}

void set_gpio_inout_masks(uint32_t in_mask, uint32_t out_mask) {
    // Match the non-Bluetooth firmware: input wins if a pin is used in both
    // directions, and every valid non-output pin is monitored as an input.
    gpio_out_mask = (out_mask & ~in_mask) & gpio_valid_pins_mask;
    gpio_in_mask = gpio_valid_pins_mask & ~gpio_out_mask;
    set_gpio_dir_pending = true;
}

static void set_gpio_dir() {
    for (uint8_t logical_pin = 0; logical_pin < ARRAY_SIZE(logical_gpio_pins); logical_pin++) {
        uint32_t bit = BIT(logical_pin);
        if (!(gpio_valid_pins_mask & bit)) {
            continue;
        }

        const logical_gpio_pin_t& pin = logical_gpio_pins[logical_pin];
        gpio_flags_t flags = GPIO_INPUT;
        if (gpio_in_mask & bit) {
            flags |= GPIO_PULL_UP;
        }

        int err = gpio_pin_configure(pin.port, pin.pin, flags);
        if (err != 0) {
            LOG_ERR("gpio_pin_configure(GPIO %u input) returned %d", logical_pin, err);
        }
    }

    // Output direction is applied by write_gpio(), just like on RP2040.
    configured_output_mask = 0;
}

void apply_pending_gpio_direction() {
    if (set_gpio_dir_pending && !suspended) {
        set_gpio_dir();
        set_gpio_dir_pending = false;
    }
}

bool read_gpio(uint64_t now) {
    uint32_t gpio_state = 0;

    for (uint8_t logical_pin = 0; logical_pin < ARRAY_SIZE(logical_gpio_pins); logical_pin++) {
        uint32_t bit = BIT(logical_pin);
        if (!(gpio_in_mask & bit)) {
            continue;
        }

        const logical_gpio_pin_t& pin = logical_gpio_pins[logical_pin];
        int value = gpio_pin_get_raw(pin.port, pin.pin);
        if (value < 0) {
            LOG_ERR("gpio_pin_get_raw(GPIO %u) returned %d", logical_pin, value);
        } else if (value != 0) {
            gpio_state |= bit;
        }
    }

    uint32_t changed = prev_gpio_state ^ gpio_state;
    if (changed != 0) {
        for (uint8_t logical_pin = 0; logical_pin < ARRAY_SIZE(logical_gpio_pins); logical_pin++) {
            uint32_t bit = BIT(logical_pin);
            if (!(changed & bit)) {
                continue;
            }

            if (last_gpio_change[logical_pin] + gpio_debounce_time <= now) {
                uint32_t usage = GPIO_USAGE_PAGE | logical_pin;
                int32_t state = !(gpio_state & bit);  // active low
                set_input_state(usage, state, state);
                if (monitor_enabled) {
                    monitor_usage(usage, state, 0);
                }
                last_gpio_change[logical_pin] = now;
            } else {
                // Ignore this transition and retain the previous stable state.
                gpio_state ^= bit;
                changed ^= bit;
            }
        }
        prev_gpio_state = gpio_state;
    }

    return changed != 0;
}

void write_gpio() {
    if (suspended) {
        return;
    }

    uint32_t value =
        (uint32_t) gpio_out_state[0] |
        ((uint32_t) gpio_out_state[1] << 8) |
        ((uint32_t) gpio_out_state[2] << 16) |
        ((uint32_t) gpio_out_state[3] << 24);

    for (uint8_t logical_pin = 0; logical_pin < ARRAY_SIZE(logical_gpio_pins); logical_pin++) {
        uint32_t bit = BIT(logical_pin);
        if (!(gpio_out_mask & bit)) {
            continue;
        }

        const logical_gpio_pin_t& pin = logical_gpio_pins[logical_pin];
        bool active = value & bit;
        int err = 0;

        if (gpio_output_mode == 0) {
            if (configured_output_mask & bit) {
                err = gpio_pin_set_raw(pin.port, pin.pin, active);
            } else {
                err = gpio_pin_configure(
                    pin.port,
                    pin.pin,
                    active ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW);
                if (err == 0) {
                    configured_output_mask |= bit;
                }
            }
        } else if (active) {
            if (configured_output_mask & bit) {
                err = gpio_pin_set_raw(pin.port, pin.pin, 0);
            } else {
                err = gpio_pin_configure(pin.port, pin.pin, GPIO_OUTPUT_LOW);
                if (err == 0) {
                    configured_output_mask |= bit;
                }
            }
        } else if (configured_output_mask & bit) {
            // Open-drain compatibility mode: inactive is high impedance.
            err = gpio_pin_configure(pin.port, pin.pin, GPIO_INPUT);
            if (err == 0) {
                configured_output_mask &= ~bit;
            }
        }

        if (err != 0) {
            LOG_ERR("writing GPIO %u returned %d", logical_pin, err);
        }
    }

    memset(gpio_out_state, 0, sizeof(gpio_out_state));
}
