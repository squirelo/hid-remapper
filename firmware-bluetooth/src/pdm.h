#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "globals.h"

extern const struct gpio_dt_spec led0;
extern struct k_work_delayable activity_led_off_work;

bool pdm_init();
void pdm_recalibrate();
void pdm_stop(); 