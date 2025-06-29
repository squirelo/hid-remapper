#pragma once

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// External references to LED and work items from main.cc
extern const struct gpio_dt_spec led0;
extern struct k_work_delayable activity_led_off_work;

// IMU function declarations
bool imu_init(); 