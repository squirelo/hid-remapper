# Original Joy-Con pair on Pico W / Pico 2 W

The Bluetooth Classic firmware (`remapper_bt`) automatically combines one
original Nintendo Joy-Con (L) and one Joy-Con (R) into one gamepad input. Each
half still has its own Bluetooth connection, but its controls update the same
logical controller in HID Remapper. Either half can reconnect first. A missing
half has released buttons and a centered stick.

## Using it

1. Flash the build for your board: `remapper_bt.uf2` for Pico W, or
   `remapper_bt_pico2_w.uf2` for Pico 2 W.
2. Put one Joy-Con in pairing mode with its rail SYNC button, then use **Pair new
   device** in the HID Remapper configuration tool. Repeat for the other half;
   pairing mode stops after each successful connection.
3. Keep your existing emulated output device selection. Set **Input labels** to
   **gamepad** for readable control names. Map the combined controls using source
   port **Any**, rather than the old individual Bluetooth ports. Standard gamepad
   usages also work with the existing unmapped-input passthrough.

Both halves show player LED 1 once initialized. Left supplies the left stick,
D-pad, L/ZL, minus, capture and L-stick click; right supplies the right stick,
ABXY, R/ZR, plus, home and R-stick click. Capture is Button 14. Rail buttons are
Button 15 (left SL), 16 (left SR), 17 (right SL), and 18 (right SR), available for
custom mappings even when the selected output has fewer buttons.

This is one automatic L/R pair. Additional controllers of an already occupied
side retain generic HID handling. Joy-Con 2 and controllers with different
report descriptors are outside this implementation. Motion sensors, rumble,
NFC and IR are not forwarded.

## Implementation

`firmware/src/joycon.cc` contains recognition, initialization, calibration and
combining. The hooks in `main_bt.cc` pass descriptor/report/disconnect events to
it and run its nonblocking command retries. The existing mapping engine and
USB descriptors are unchanged.

Recognition requires the exact 170-byte original Nintendo Bluetooth descriptor,
followed by a device-info reply confirming type 1 (L) or 2 (R). Other types fall
back to generic HID before changing their report mode. Joy-Cons are switched
from simple `0x3F` reports to full `0x30` reports. User stick calibration is used
when valid, otherwise factory calibration is read; invalid or unavailable
calibration falls back to approximate center/range values with a UART diagnostic.

The pair has a separate virtual input interface and no physical source port.
Each report replaces only its originating half's state. Disconnect clears that
half and publishes the remaining state. Buffered physical reports are discarded
on disconnect before a device slot can be reused. The virtual descriptor stays
registered until reboot so releases are observed even when both halves leave.

Protocol references:

- [Report formats](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md)
- [Subcommands](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_subcommands_notes.md)
- [Calibration](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/spi_flash_notes.md)
- [Original descriptor dump](https://github.com/backupbrew/switchbrew/blob/master/Joy-Con.md)

## Validation

Run the host protocol tests (C++17 compiler with AddressSanitizer and
UndefinedBehaviorSanitizer):

```sh
sh firmware/tests/run_joycon_tests.sh
```

Build the existing Bluetooth target with the repository's pinned dependencies:

```sh
git submodule update --init
git -C firmware/pico-sdk submodule update --init lib/cyw43-driver lib/lwip lib/btstack
cmake -S firmware -B /tmp/remapper-pico-w -DPICO_BOARD=pico_w
cmake --build /tmp/remapper-pico-w --target remapper_bt
cmake -S firmware -B /tmp/remapper-pico2-w -DPICO_BOARD=pico2_w
cmake --build /tmp/remapper-pico2-w --target remapper_bt
```

Hardware verification remains necessary: move both sticks simultaneously, hold
a button on one half while releasing buttons on the other, disconnect each half
while controls are held, then reconnect in reverse order. Check that neither a
release nor a reconnect clears the other half's inputs. UART diagnostics use the
existing GP0 / 921600-baud logging configuration.
