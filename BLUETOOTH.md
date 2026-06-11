# Bluetooth version

_(Please note this is experimental and hasn't been tested with a lot of devices.)_

There's a special version of the remapper that takes inputs from Bluetooth devices and translates them to USB. It's different from most Bluetooth USB dongles in that from the computer's point of view it is a USB mouse/keyboard, so it requires no special drivers. The remapping functionality works as usual. You can connect multiple devices to it at the same time.

**It only works with Bluetooth Low Energy devices, not Bluetooth Classic.** You can usually tell that a device uses Bluetooth LE if the documentation says it requires Bluetooth version 4 or 5, but some devices say they use Bluetooth 4 or 5 and still use Bluetooth Classic (which technically isn't wrong as Bluetooth Classic is still part of the newer spec).

![HID Remapper Bluetooth](images/bluetooth.jpg)

The Bluetooth version of the remapper is available for Nordic nRF52840 boards and for Raspberry Pi Pico W / Pico 2 W.

### nRF52840 boards

Precompiled binaries are available for:

* [Adafruit Feather nRF52840 Express](https://www.adafruit.com/product/4062)
* [Seeed Studio Xiao nRF52840](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html)

To flash the [nRF firmware](firmware-bluetooth), first put the board in flashing mode by double clicking the reset button quickly. A drive should appear on your computer. Copy the [UF2 file that matches your board](https://github.com/jfedor2/hid-remapper/releases/latest) to that drive and that's it. If you want to flash a newer version of the firmware in the future, you can also put the board in firmware flashing mode using the HID Remapper [web configuration tool](https://www.remapper.org/config/).

### Raspberry Pi Pico W / Pico 2 W

Precompiled binaries are available for:

* [Raspberry Pi Pico W](https://www.raspberrypi.com/products/raspberry-pi-pico/)
* [Raspberry Pi Pico 2 W](https://www.raspberrypi.com/products/raspberry-pi-pico-2/)

To flash the [Pico firmware](firmware), hold the BOOTSEL button, plug the board into USB, and release BOOTSEL. A drive should appear on your computer. Copy the `remapper_bluetooth_pico_w.uf2` or `remapper_bluetooth_pico2_w.uf2` file from the [latest release](https://github.com/jfedor2/hid-remapper/releases/latest) to that drive.

To build from source, initialize the Pico SDK nested submodules (BTstack, CYW43 driver, etc.) and Python 3 must be on your PATH (used to generate the GATT header):

```bash
git submodule update --init --recursive firmware/pico-sdk
cd firmware && mkdir -p build-pico_w && cd build-pico_w
PICO_BOARD=pico_w cmake .. && make remapper_bluetooth
```

For Pico 2 W, use `PICO_BOARD=pico2_w` instead.

The Pico W boards do not have a built-in user button. To pair or clear bonds without the web configuration tool, short **GPIO 22** to GND: a short press starts pairing a new device, holding for more than 3 seconds clears all paired devices.

To connect Bluetooth devices to the remapper, you need to put the device in pairing mode. This is device-specific, but usually involves holding a button for a few seconds. Then you also need to put HID Remapper in pairing mode. You do this by either pressing the "user switch" button on the board or by clicking the "Pair new device" button on the web configuration tool (the Xiao and Pico W boards don't have a user button so you have to either do it through the web interface or by shorting a GPIO pin to GND: pin 0 on the Xiao, GPIO 22 on the Pico W). The remapper will also automatically enter pairing mode if no devices are currently paired.

You can tell the remapper is in pairing mode if the onboard LED is lit constantly. When it's not in pairing mode, the LED will be blinking, with the number of blinks per cycle corresponding to the number of currently connected devices.

To make the remapper forget all currently paired devices, hold the "user switch" button for over 3 seconds, or click the "Forget all devices" button on the web configuration tool (or short the pairing pin to GND for over 3 seconds on the Seeed Xiao or Pico W boards).

## BLE GATT peripheral input

The Bluetooth firmware also advertises a Nordic UART Service compatible GATT
peripheral while continuing to scan for Bluetooth LE HID devices as a central.
Reports written to this service are treated as a separate virtual input device,
so they can be remapped together with connected BLE HID devices.

* Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
* Write characteristic: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
* Notify characteristic: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

The write characteristic requires an encrypted BLE connection, so clients may
need to pair before reports can be written.

Writes are SLIP-style framed packets with a little-endian CRC32 trailer:
start `0xc0`, escaped payload bytes, four CRC bytes, end `0xc0`.
The decoded payload is:

| Byte | Meaning |
| ---: | --- |
| 0 | Protocol version, currently `1` |
| 1 | Advisory output descriptor number |
| 2 | HID report payload length |
| 3 | HID report ID |
| 4..N | HID report payload bytes, without the report ID |

The descriptor number in the packet does not switch the USB descriptor at
runtime. Choose the USB descriptor through the normal HID Remapper configuration
so it is active when the board enumerates over USB.

## Known issues

* Quirks mechanism for fixing broken report descriptors doesn't work.
* Reconnects could be faster if we cached attributes/report descriptor.
