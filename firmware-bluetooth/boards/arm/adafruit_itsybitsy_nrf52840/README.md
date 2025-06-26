# Adafruit ItsyBitsy nRF52840

This directory contains the board configuration for the Adafruit ItsyBitsy nRF52840.

## Hardware

- **MCU**: nRF52840 (ARM Cortex-M4)
- **Flash**: 1MB
- **RAM**: 256KB
- **LED**: Connected to GPIO0.6
- **Button**: Connected to GPIO0.29 (with internal pull-up)
- **UART**: TX=GPIO0.24, RX=GPIO0.25
- **I2C**: SDA=GPIO0.12, SCL=GPIO0.11
- **SPI**: SCK=GPIO0.14, MOSI=GPIO0.13, MISO=GPIO0.15

## Features

- Bluetooth Low Energy (BLE)
- USB Device support
- ADC support
- I2C support
- SPI support
- QSPI flash support
- PWM support
- Watchdog support
- Counter support

## Building

To build for this board, use:

```bash
west build -b adafruit_itsybitsy_nrf52840 <your_project>
```

## Flashing

The board supports various flashing methods:

- J-Link
- pyOCD
- Black Magic Probe
- nRFjprog

Example with J-Link:
```bash
west flash
``` 