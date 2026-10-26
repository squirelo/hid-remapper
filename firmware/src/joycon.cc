// Original Joy-Con L/R -> one gamepad input for the existing mapping engine.
// Protocol and calibration:
// https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
// Original Bluetooth descriptor (identical on both halves):
// https://github.com/backupbrew/switchbrew/blob/master/Joy-Con.md
#include <algorithm>
#include <cstdio>
#include <cstring>

extern "C" {
#include "btstack.h"
}

#include "descriptor_parser.h"
#include "joycon.h"
#include "platform.h"
#include "remapper.h"

namespace {

// An exact match limits the probe to the known Nintendo protocol. Device info
// below distinguishes Joy-Cons from other controllers sharing this descriptor.
constexpr uint8_t nintendo_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x06, 0x01, 0xFF,
    0x85, 0x21, 0x09, 0x21, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02,
    0x85, 0x30, 0x09, 0x30, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02,
    0x85, 0x31, 0x09, 0x31, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02,
    0x85, 0x32, 0x09, 0x32, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02,
    0x85, 0x33, 0x09, 0x33, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02,
    0x85, 0x3F, 0x05, 0x09, 0x19, 0x01, 0x29, 0x10, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x10, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00,
    0x25, 0x07, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x05, 0x09, 0x75, 0x04,
    0x95, 0x01, 0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x33,
    0x09, 0x34, 0x16, 0x00, 0x00, 0x27, 0xFF, 0xFF, 0x00, 0x00,
    0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0x06, 0x01, 0xFF,
    0x85, 0x01, 0x09, 0x01, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02,
    0x85, 0x10, 0x09, 0x10, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02,
    0x85, 0x11, 0x09, 0x11, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02,
    0x85, 0x12, 0x09, 0x12, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0xC0,
};

// Canonical HID Remapper gamepad usages. No report ID; 18 buttons, a hat,
// and LX/LY/RX/RY. This describes an INPUT, not another USB output device.
constexpr uint8_t pair_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x12, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x12, 0x81, 0x02,
    0x75, 0x06, 0x95, 0x01, 0x81, 0x03,
    0x05, 0x01, 0x09, 0x39, 0x25, 0x07, 0x75, 0x04, 0x81, 0x42,
    0x75, 0x04, 0x81, 0x03,
    0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35,
    0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0xC0,
};
constexpr uint16_t pair_interface = 0xFE00;  // outside physical slots and OUR_OUT_INTERFACE
constexpr uint64_t retry_us = 500000;
constexpr uint64_t command_gap_us = 25000;
constexpr unsigned max_attempts = 3;

enum class Stage { INFO, USER_CAL, FACTORY_CAL, MODE, LED, READY, FAILED, GENERIC };
struct Axis {
    int center = 2000;
    int below = 1500;
    int above = 1500;
};
struct Joycon {
    uint16_t cid = 0;
    uint16_t interface = 0;
    uint8_t port = 0;
    uint8_t side = 0;  // device-info type: 1=L, 2=R
    Stage stage = Stage::INFO;
    uint8_t sequence = 0;
    uint8_t attempts = 0;
    uint8_t send_failures = 0;
    uint64_t next_send = 0;
    // BTstack retains the payload pointer until CAN_SEND_NOW. Keep each setup
    // command intact, even when a timeout advances to the next calibration read.
    uint8_t tx[5][48] = {};
    uint8_t command_index = 0;
    uint8_t* payload() { return tx[command_index]; }
    Axis cal[2];
    uint32_t buttons = 0;
    uint8_t dpad = 0;
    uint8_t x = 128;
    uint8_t y = 128;
};
Joycon devices[4];  // matches main_bt.cc's maximum physical connections
bool pair_registered = false;

Joycon* find(uint16_t interface) {
    for (auto& jc : devices) {
        if (jc.cid && jc.interface == interface) {
            return &jc;
        }
    }
    return nullptr;
}

uint32_t calibration_address(const Joycon& jc) {
    if (jc.stage == Stage::USER_CAL) {
        return jc.side == 1 ? 0x8010 : 0x801B;
    }
    return jc.side == 1 ? 0x603D : 0x6046;
}

void start_command(Joycon& jc, Stage stage) {
    jc.stage = stage;
    jc.command_index = static_cast<uint8_t>(stage);
    jc.attempts = 0;
    jc.send_failures = 0;
    jc.next_send = get_time() + command_gap_us;
    memset(jc.payload(), 0, sizeof(jc.tx[0]));
    jc.payload()[0] = jc.sequence++ & 0x0F;
    const uint8_t neutral_rumble[] = { 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
    memcpy(jc.payload() + 1, neutral_rumble, sizeof(neutral_rumble));
    switch (stage) {
        case Stage::INFO:
            jc.payload()[9] = 0x02;
            break;
        case Stage::USER_CAL:
        case Stage::FACTORY_CAL: {
            jc.payload()[9] = 0x10;
            uint32_t address = calibration_address(jc);
            for (int i = 0; i < 4; i++) {
                jc.payload()[10 + i] = address >> (8 * i);
            }
            jc.payload()[14] = stage == Stage::USER_CAL ? 11 : 9;
            break;
        }
        case Stage::MODE:
            jc.payload()[9] = 0x03;
            jc.payload()[10] = 0x30;
            break;
        case Stage::LED:
            jc.payload()[9] = 0x30;
            jc.payload()[10] = 1;  // both halves show player 1
            break;
        default:
            break;
    }
}

// The descriptor also occurs on other Nintendo controllers. They retain the
// existing generic path, as do extra controllers of an already claimed side.
void use_generic(Joycon& jc) {
    device_connected_callback(jc.interface, 1, 1, jc.port);
    parse_descriptor(1, 1, nintendo_descriptor, sizeof(nintendo_descriptor), jc.interface, 0);
    jc.stage = Stage::GENERIC;
    jc.side = 0;
}

void emit_pair() {
    if (!pair_registered) {
        return;
    }
    uint8_t report[] = { 0, 0, 0, 8, 128, 128, 128, 128 };
    uint32_t buttons = 0;
    for (const auto& jc : devices) {
        if (!jc.cid || !jc.side) {
            continue;
        }
        buttons |= jc.buttons;
        unsigned stick = jc.side == 1 ? 4 : 6;
        report[stick] = jc.x;
        report[stick + 1] = jc.y;
        if (jc.side == 1) {
            int dx = !!(jc.dpad & 2) - !!(jc.dpad & 4);
            int dy = !!(jc.dpad & 8) - !!(jc.dpad & 1);
            constexpr uint8_t hats[] = { 7, 0, 1, 6, 8, 2, 5, 4, 3 };
            report[3] = hats[(dy + 1) * 3 + dx + 1];
        }
    }
    for (int i = 0; i < 3; i++) {
        report[i] = buttons >> (8 * i);
    }
    handle_received_report(report, sizeof(report), pair_interface);
}

int unpack_axis(const uint8_t* data, int axis) {
    return axis == 0 ? data[0] | ((data[1] & 0x0F) << 8) : (data[1] >> 4) | (data[2] << 4);
}

bool read_calibration(Joycon& jc, const uint8_t* data) {
    Axis cal[2];
    for (int axis = 0; axis < 2; axis++) {
        cal[axis].center = unpack_axis(data + (jc.side == 1 ? 3 : 0), axis);
        cal[axis].below = unpack_axis(data + (jc.side == 1 ? 6 : 3), axis);
        cal[axis].above = unpack_axis(data + (jc.side == 1 ? 0 : 6), axis);
        if (cal[axis].below <= 0 || cal[axis].above <= 0 ||
            cal[axis].center < cal[axis].below || cal[axis].center + cal[axis].above > 4095) {
            return false;
        }
    }
    std::copy(cal, cal + 2, jc.cal);
    return true;
}

uint8_t scale_axis(int raw, const Axis& cal, bool invert) {
    int delta = raw - cal.center;
    int span = delta < 0 ? cal.below : cal.above;
    if (invert) {
        delta = -delta;
    }
    return std::clamp(128 + delta * (delta < 0 ? 128 : 127) / span, 0, 255);
}

void decode_input(Joycon& jc, const uint8_t* report) {
    uint8_t shared = report[4];
    uint8_t buttons = report[jc.side == 1 ? 5 : 3];
    jc.buttons = 0;
    auto button = [&](bool pressed, int number) {
        if (pressed) {
            jc.buttons |= 1u << (number - 1);
        }
    };
    if (jc.side == 1) {
        jc.dpad = ((buttons & 1) ? 8 : 0) | ((buttons & 2) ? 1 : 0) |
            ((buttons & 4) ? 2 : 0) | ((buttons & 8) ? 4 : 0);
        button(buttons & 0x40, 5);   // L
        button(buttons & 0x80, 7);   // ZL
        button(shared & 0x01, 9);   // minus
        button(shared & 0x08, 11);  // L stick
        button(shared & 0x20, 14);  // capture
        button(buttons & 0x20, 15); // L-SL
        button(buttons & 0x10, 16); // L-SR
    } else {
        button(buttons & 0x01, 1);  // Y / west
        button(buttons & 0x04, 2);  // B / south
        button(buttons & 0x08, 3);  // A / east
        button(buttons & 0x02, 4);  // X / north
        button(buttons & 0x40, 6);  // R
        button(buttons & 0x80, 8);  // ZR
        button(shared & 0x02, 10); // plus
        button(shared & 0x04, 12); // R stick
        button(shared & 0x10, 13); // home
        button(buttons & 0x20, 17); // R-SL
        button(buttons & 0x10, 18); // R-SR
    }
    const uint8_t* stick = report + (jc.side == 1 ? 6 : 9);
    jc.x = scale_axis(unpack_axis(stick, 0), jc.cal[0], false);
    jc.y = scale_axis(unpack_axis(stick, 1), jc.cal[1], true);
    emit_pair();
}

void command_failed(Joycon& jc) {
    printf("[joycon] interface %u: command 0x%02x failed\n", jc.interface, jc.payload()[9]);
    switch (jc.stage) {
        case Stage::INFO:
            use_generic(jc); // no mode changes made
            break;
        case Stage::USER_CAL:
            start_command(jc, Stage::FACTORY_CAL);
            break;
        case Stage::FACTORY_CAL:
            printf("[joycon] using approximate stick calibration\n");
            start_command(jc, Stage::MODE);
            break;
        case Stage::LED:
            jc.stage = Stage::READY; // lights are optional
            break;
        default:
            jc.stage = Stage::FAILED;
            break;
    }
}

}  // namespace

bool joycon_attach(uint16_t cid, uint16_t interface, uint8_t port, const uint8_t* descriptor, uint16_t len) {
    if (len != sizeof(nintendo_descriptor) || memcmp(descriptor, nintendo_descriptor, len) != 0) {
        return false;
    }
    if (find(interface)) {
        return true;
    }
    for (auto& jc : devices) {
        if (!jc.cid) {
            jc = Joycon{};
            jc.cid = cid;
            jc.interface = interface;
            jc.port = port;
            start_command(jc, Stage::INFO);
            return true;
        }
    }
    return false;
}

bool joycon_report(uint16_t interface, const uint8_t* report, uint16_t len) {
    Joycon* jc = find(interface);
    if (!jc || jc->stage == Stage::GENERIC) {
        return false;
    }
    // The mode ACK can be lost while full reports are already arriving.
    if (len >= 49 && report[0] == 0x30 && jc->stage == Stage::MODE && jc->attempts) {
        start_command(*jc, Stage::LED);
    }
    if (len >= 49 && report[0] == 0x30 && (jc->stage == Stage::LED || jc->stage == Stage::READY)) {
        decode_input(*jc, report);
    }
    if (len < 15 || report[0] != 0x21 || report[14] != jc->payload()[9] || !jc->attempts ||
        jc->stage == Stage::READY || jc->stage == Stage::FAILED) {
        return true;
    }
    if (!(report[13] & 0x80)) {
        command_failed(*jc);
        return true;
    }
    switch (jc->stage) {
        case Stage::INFO: {
            if (len < 18) {
                break;
            }
            uint8_t side = report[17];
            bool claimed = false;
            for (const auto& other : devices) {
                claimed |= other.cid && other.side == side;
            }
            if ((side != 1 && side != 2) || claimed) {
                use_generic(*jc);
                break;
            }
            jc->side = side;
            if (!pair_registered) {
                // No physical port: the pair is mapped using source port Any.
                device_connected_callback(pair_interface, 1, 1, 0);
                parse_descriptor(1, 1, pair_descriptor, sizeof(pair_descriptor), pair_interface, 0);
                pair_registered = true;
            }
            printf("[joycon] interface %u joins pair as %s\n", interface, side == 1 ? "left" : "right");
            start_command(*jc, Stage::USER_CAL);
            break;
        }
        case Stage::USER_CAL:
        case Stage::FACTORY_CAL: {
            uint8_t count = jc->stage == Stage::USER_CAL ? 11 : 9;
            if (len < 20 + count || report[19] != count || memcmp(report + 15, jc->payload() + 10, 4) != 0) {
                break;
            }
            bool user = jc->stage == Stage::USER_CAL;
            bool valid = (!user || (report[20] == 0xB2 && report[21] == 0xA1)) &&
                read_calibration(*jc, report + (user ? 22 : 20));
            if (user && !valid) {
                start_command(*jc, Stage::FACTORY_CAL);
            } else {
                if (!valid) {
                    printf("[joycon] invalid factory calibration; using approximate defaults\n");
                }
                start_command(*jc, Stage::MODE);
            }
            break;
        }
        case Stage::MODE:
            start_command(*jc, Stage::LED);
            break;
        case Stage::LED:
            jc->stage = Stage::READY;
            break;
        default:
            break;
    }
    return true;
}

bool joycon_detach(uint16_t interface) {
    Joycon* jc = find(interface);
    if (!jc) {
        return false;
    }
    bool generic = jc->stage == Stage::GENERIC;
    *jc = Joycon{};
    if (generic) {
        return false;
    }
    // Keep the virtual descriptor until reboot, even with neither half present.
    // The neutral report releases controls without discarding the pair's state
    // slots before the mapping engine has observed those releases.
    emit_pair();
    return true;
}

void joycon_task() {
    uint64_t now = get_time();
    for (auto& jc : devices) {
        if (!jc.cid || jc.stage == Stage::READY || jc.stage == Stage::FAILED || jc.stage == Stage::GENERIC || now < jc.next_send) {
            continue;
        }
        if (jc.attempts >= max_attempts) {
            command_failed(jc);
            continue;
        }
        // Interrupt DATA/OUTPUT (0xA2), not control SET_REPORT.
        jc.next_send = now + retry_us;
        uint8_t status = hid_host_send_report(jc.cid, 0x01, jc.payload(), sizeof(jc.tx[0]));
        if (status == ERROR_CODE_SUCCESS) {
            jc.attempts++;
            jc.send_failures = 0;
        } else if (++jc.send_failures >= max_attempts) {
            printf("[joycon] transport error 0x%02x\n", status);
            command_failed(jc);
        }
    }
}
