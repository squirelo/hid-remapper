#include "ble_button_led.h"

#include "ble_shared.h"

#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <pico/cyw43_arch.h>

#ifndef BLE_PAIRING_BUTTON_PIN
#define BLE_PAIRING_BUTTON_PIN 22
#endif

static BleLedMode led_mode = BleLedMode::OFF;
static int led_blink_count = 0;
static int led_blinks_left = 0;
static bool next_blink_state = true;
static uint64_t led_next_change_us = 0;
static uint64_t activity_off_us = 0;
static bool activity_override = false;

static void set_led_gpio(bool on) {
    if (activity_override) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
        return;
    }
    switch (led_mode) {
        case BleLedMode::OFF:
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            break;
        case BleLedMode::ON:
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            break;
        case BleLedMode::BLINK:
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
            break;
    }
}

void ble_status_led_set_mode(BleLedMode mode) {
    led_mode = mode;
    if (led_mode == BleLedMode::BLINK) {
        led_blinks_left = led_blink_count;
        next_blink_state = true;
        led_next_change_us = time_us_64();
    }
    if (!activity_override) {
        set_led_gpio(led_mode == BleLedMode::ON);
    }
}

void ble_status_led_set_blink_count(int count) {
    led_blink_count = count;
    if (led_mode == BleLedMode::BLINK) {
        led_blinks_left = led_blink_count;
    }
}

void ble_status_led_activity_flash(void) {
    activity_override = true;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    activity_off_us = time_us_64() + 50000;
}

extern void pair_new_device(void);
extern void clear_bonds(void);

static int64_t button_pressed_at = 0;

void ble_button_led_init(void) {
    gpio_init(BLE_PAIRING_BUTTON_PIN);
    gpio_set_dir(BLE_PAIRING_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BLE_PAIRING_BUTTON_PIN);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
}

void ble_button_led_poll(void) {
    uint64_t now = time_us_64();

    if (activity_override && now >= activity_off_us) {
        activity_override = false;
        set_led_gpio(led_mode == BleLedMode::ON);
    }

    if (!activity_override && led_mode == BleLedMode::BLINK && now >= led_next_change_us) {
        if (next_blink_state) {
            if (led_blinks_left > 0) {
                led_blinks_left--;
                set_led_gpio(true);
                next_blink_state = false;
                led_next_change_us = now + 100000;
            } else {
                led_blinks_left = led_blink_count;
                set_led_gpio(false);
                led_next_change_us = now + 1000000;
            }
        } else {
            set_led_gpio(false);
            next_blink_state = true;
            led_next_change_us = now + 100000;
        }
    }

    static bool prev_button = false;
    bool button = !gpio_get(BLE_PAIRING_BUTTON_PIN);
    if (button && !prev_button) {
        button_pressed_at = time_us_64() / 1000;
    } else if (!button && prev_button) {
        int64_t held_ms = (int64_t) (time_us_64() / 1000) - button_pressed_at;
        if (held_ms > BLE_CLEAR_BONDS_BUTTON_PRESS_MS) {
            clear_bonds();
        } else {
            pair_new_device();
        }
    }
    prev_button = button;
}
