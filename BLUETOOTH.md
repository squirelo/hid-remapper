# Bluetooth version

_(Please note this is experimental and hasn't been tested with a lot of devices.)_

There's a special version of the remapper that takes inputs from Bluetooth devices and can output remapped HID over USB and over Bluetooth LE. From a computer's point of view the USB side is a normal USB mouse/keyboard/gamepad, so it requires no special drivers. The remapping functionality works as usual. You can connect multiple BLE input devices at the same time.

**It only works with Bluetooth Low Energy devices, not Bluetooth Classic.** You can usually tell that a device uses Bluetooth LE if the documentation says it requires Bluetooth version 4 or 5, but some devices say they use Bluetooth 4 or 5 and still use Bluetooth Classic (which technically isn't wrong as Bluetooth Classic is still part of the newer spec).

![HID Remapper Bluetooth](images/bluetooth.jpg)

The Bluetooth version of the remapper is available for Nordic nRF52840 boards.

### nRF52840 boards

Precompiled binaries are available for:

* [Adafruit Feather nRF52840 Express](https://www.adafruit.com/product/4062)
* [Seeed Studio Xiao nRF52840](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html)

To flash the [firmware](firmware-bluetooth), first put the board in flashing mode by double clicking the reset button quickly. A drive should appear on your computer. Copy the [UF2 file that matches your board](https://github.com/jfedor2/hid-remapper/releases/latest) to that drive and that's it. If you want to flash a newer version of the firmware in the future, you can also put the board in firmware flashing mode using the HID Remapper [web configuration tool](https://www.remapper.org/config/).

To build from source with west / NCS v2.2:

```bash
cd firmware-bluetooth
west build -b seeed_xiao_nrf52840
```

To connect Bluetooth devices to the remapper, you need to put the device in pairing mode. This is device-specific, but usually involves holding a button for a few seconds. Then you also need to put HID Remapper in pairing mode. You do this by either pressing the "user switch" button on the board or by clicking the "Pair new device" button on the web configuration tool (the Xiao board doesn't have a user button so you have to either do it through the web interface or by shorting pin 0 to GND). The remapper will also automatically enter pairing mode if no devices are currently paired.

You can tell the remapper is in pairing mode if the blue LED is lit constantly. When it's not in pairing mode, the blue LED will be blinking, with the number of blinks per cycle corresponding to the number of currently connected devices.

To make the remapper forget all currently paired devices, hold the "user switch" button for over 3 seconds, or click the "Forget all devices" button on the web configuration tool (or short pin 0 to GND for over 3 seconds on the Seeed Xiao board).

## BLE HID output

The nRF52840 firmware also advertises a standard BLE Human Interface Device service (HIDS, UUID `0x1812`) while continuing to scan for BLE HID input devices as a central. Remapped reports are sent to a paired BLE host (PC, iPad, Android, etc.) through HIDS notifications in addition to the existing USB HID output.

Pair the board from the host's Bluetooth settings like any other keyboard, mouse, or gamepad. The active output descriptor (keyboard/mouse, gamepad, etc.) is selected through the normal HID Remapper configuration before use. The Device Information Service includes a PnP ID when the descriptor defines a USB vendor/product ID override.

**Note:** The original Nintendo Switch does not accept BLE HID controllers. Use wired USB output for Switch; use BLE HID output for PC/tablet/phone hosts.

When a BLE host is connected, report pacing follows the host connection interval (typically 7.5–15 ms). Relative usages such as mouse movement and scroll deltas accumulate between notifications.

Peripheral advertising includes both the HIDS UUID and the Nordic UART Service UUID below. Any device that connects as a peripheral client can see both services.

## BLE GATT peripheral input (NUS)

The Bluetooth firmware also advertises a Nordic UART Service compatible GATT peripheral while continuing to scan for Bluetooth LE HID devices as a central. Reports written to this service are treated as a separate virtual input device, so they can be remapped together with connected BLE HID devices.

* Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
* Write characteristic: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
* Notify characteristic: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

The write characteristic requires an encrypted BLE connection, so clients may need to pair before reports can be written.

Writes are SLIP-style framed packets with a little-endian CRC32 trailer: start `0xc0`, escaped payload bytes, four CRC bytes, end `0xc0`. The decoded payload is:

| Byte | Meaning |
| ---: | --- |
| 0 | Protocol version, currently `1` |
| 1 | Advisory output descriptor number |
| 2 | HID report payload length |
| 3 | HID report ID |
| 4..N | HID report payload bytes, without the report ID |

The descriptor number in the packet does not switch the active output descriptor at runtime. Choose the output descriptor through the normal HID Remapper configuration so it is active when the board enumerates over USB or BLE.

## USB CDC serial input

The firmware exposes a USB CDC ACM serial port using the same SLIP packet format as the BLE NUS input. A desktop application can send virtual HID reports over serial; they are remapped and emitted on USB and/or BLE HID output.

Example using the included helper script:

```bash
pip install pyserial
python firmware-bluetooth/scripts/cdc_hid_send.py COM7 --descriptor 0 --report-id 1 --payload "00 00"
```

Replace `COM7` with the CDC serial port assigned to the board.

## USB MIDI input (iOS)

When the board is connected to an iOS device over USB, it enumerates as a class-compliant USB-MIDI device. MIDI messages received from iOS are mapped through the remapper core using the same MIDI usage mapping as the USB-host firmware builds. Configure MIDI-to-HID mappings in the web configuration tool as usual.

iOS ignores the other composite USB interfaces (HID configuration, CDC). The board is powered by the iOS device or adapter in this setup.

## Known issues

* Quirks mechanism for fixing broken report descriptors doesn't work.
* Reconnects could be faster if we cached attributes/report descriptor.
* Nordic HIDS report map size is limited to 255 bytes; very large output descriptors may not work over BLE.
