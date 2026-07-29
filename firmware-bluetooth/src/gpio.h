#pragma once

#include <stdint.h>

void gpio_pins_init();
bool read_gpio(uint64_t now);
void write_gpio();
void apply_pending_gpio_direction();
