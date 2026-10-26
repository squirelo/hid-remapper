#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <vector>

#include "btstack.h"
#include "descriptor_parser.h"
#include "joycon.h"
#include "remapper.h"

// Original Bluetooth descriptor from:
// https://github.com/backupbrew/switchbrew/blob/master/Joy-Con.md
constexpr uint8_t descriptor[] = {
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
static_assert(sizeof(descriptor) == 170);

struct Command {
    uint16_t cid;
    std::vector<uint8_t> data;
    const uint8_t* retained;
};
struct Registration {
    uint16_t interface;
    uint8_t port;
};
struct Descriptor {
    uint16_t interface;
    std::vector<uint8_t> data;
};
static uint64_t now_us = 1000000;
static std::vector<Command> commands;
static std::vector<Registration> registrations;
static std::vector<Descriptor> descriptors;
static std::vector<std::array<uint8_t, 8>> reports;
static uint8_t next_send_status = ERROR_CODE_SUCCESS;
constexpr uint16_t pair_interface = 0xFE00;

uint64_t get_time() { return now_us; }

uint8_t hid_host_send_report(uint16_t cid, uint16_t id, const uint8_t* report, uint8_t len) {
    assert(id == 0x01 && len == 48);
    constexpr uint8_t neutral[] = { 0, 1, 0x40, 0x40, 0, 1, 0x40, 0x40 };
    assert(std::equal(std::begin(neutral), std::end(neutral), report + 1));
    commands.push_back({ cid, { report, report + len }, report });
    uint8_t status = next_send_status;
    next_send_status = ERROR_CODE_SUCCESS;
    return status;
}

void device_connected_callback(uint16_t interface, uint16_t, uint16_t, uint8_t port) {
    registrations.push_back({ interface, port });
}

void parse_descriptor(uint16_t, uint16_t, const uint8_t* data, int len, uint16_t interface, uint8_t) {
    descriptors.push_back({ interface, { data, data + len } });
}

void handle_received_report(const uint8_t* data, int len, uint16_t interface, uint8_t id) {
    assert(interface == pair_interface && id == 0 && len == 8);
    std::array<uint8_t, 8> report;
    std::copy(data, data + len, report.begin());
    reports.push_back(report);
}

static void tick(uint64_t elapsed = 25000) {
    now_us += elapsed;
    joycon_task();
}

static const Command& last_command(uint16_t cid) {
    auto it = std::find_if(commands.rbegin(), commands.rend(), [=](const Command& command) {
        return command.cid == cid;
    });
    assert(it != commands.rend());
    return *it;
}

static void expect_command(uint16_t cid, uint8_t subcommand) {
    assert(last_command(cid).data[9] == subcommand);
}

static void expect_retained(const Command& command) {
    // BTstack retains the supplied pointer until CAN_SEND_NOW, including while
    // the adapter times out and starts another command or falls back to HID.
    assert(std::equal(command.data.begin(), command.data.end(), command.retained));
}

static void attach(uint16_t cid, uint16_t interface) {
    assert(joycon_attach(cid, interface, 2, descriptor, sizeof(descriptor)));
}

static std::array<uint8_t, 49> reply(uint8_t subcommand, uint8_t ack = 0x80) {
    std::array<uint8_t, 49> packet{};
    packet[0] = 0x21;
    packet[13] = ack;
    packet[14] = subcommand;
    return packet;
}

static void receive(uint16_t interface, const std::array<uint8_t, 49>& packet) {
    assert(joycon_report(interface, packet.data(), packet.size()));
}

static void pack(uint8_t* data, unsigned x, unsigned y) {
    data[0] = x;
    data[1] = (x >> 8) | ((y & 0x0F) << 4);
    data[2] = y >> 4;
}

static std::vector<uint8_t> calibration(uint8_t side, bool user) {
    std::vector<uint8_t> bytes(user ? 11 : 9);
    if (user) {
        bytes[0] = 0xB2;
        bytes[1] = 0xA1;
    }
    uint8_t* data = bytes.data() + (user ? 2 : 0);
    pack(data + (side == 1 ? 3 : 0), 2000, 2100); // center
    pack(data + (side == 1 ? 6 : 3), 1000, 1100); // distance to minimum
    pack(data + (side == 1 ? 0 : 6), 1200, 1300); // distance to maximum
    return bytes;
}

static std::array<uint8_t, 49> spi_reply(uint32_t address, const std::vector<uint8_t>& data) {
    auto packet = reply(0x10, 0x90);
    for (unsigned i = 0; i < 4; i++) {
        packet[15 + i] = address >> (8 * i);
    }
    packet[19] = data.size();
    std::copy(data.begin(), data.end(), packet.begin() + 20);
    return packet;
}

static void expect_spi(uint16_t cid, uint32_t address, uint8_t count) {
    expect_command(cid, 0x10);
    const auto& data = last_command(cid).data;
    for (unsigned i = 0; i < 4; i++) {
        assert(data[10 + i] == ((address >> (8 * i)) & 0xFF));
    }
    assert(data[14] == count);
}

static void finish_initialization(uint16_t cid, uint16_t interface) {
    expect_command(cid, 0x03);
    assert(last_command(cid).data[10] == 0x30);
    receive(interface, reply(0x03));
    tick();
    expect_command(cid, 0x30);
    assert(last_command(cid).data[10] == 1);
    receive(interface, reply(0x30));
    size_t sent = commands.size();
    tick();
    assert(commands.size() == sent);
}

static void initialize(uint16_t cid, uint16_t interface, uint8_t side, bool use_user) {
    tick();
    expect_command(cid, 0x02);
    auto info = reply(0x02, 0x82);
    info[17] = side;
    receive(interface, info);
    tick();
    const uint32_t user_address = side == 1 ? 0x8010 : 0x801B;
    expect_spi(cid, user_address, 11);

    // A stale response, short response, or wrong byte count cannot advance SPI initialization.
    size_t sent = commands.size();
    receive(interface, spi_reply(user_address + 1, calibration(side, true)));
    auto short_reply = spi_reply(user_address, calibration(side, true));
    assert(joycon_report(interface, short_reply.data(), 30));
    short_reply[19] = 10;
    receive(interface, short_reply);
    tick();
    assert(commands.size() == sent);

    receive(interface, spi_reply(user_address, use_user ? calibration(side, true) : std::vector<uint8_t>(11, 0xFF)));
    tick();
    if (!use_user) {
        uint32_t factory_address = side == 1 ? 0x603D : 0x6046;
        expect_spi(cid, factory_address, 9);
        receive(interface, spi_reply(factory_address, calibration(side, false)));
        tick();
    }
    finish_initialization(cid, interface);
}

static std::array<uint8_t, 49> input(uint8_t side, unsigned x, unsigned y, uint8_t buttons, uint8_t shared) {
    std::array<uint8_t, 49> packet{};
    packet[0] = 0x30;
    packet[4] = shared;
    packet[side == 1 ? 5 : 3] = buttons;
    pack(packet.data() + (side == 1 ? 6 : 9), x, y);
    return packet;
}

static uint32_t buttons(std::initializer_list<unsigned> numbers) {
    uint32_t result = 0;
    for (unsigned number : numbers) {
        result |= 1u << (number - 1);
    }
    return result;
}

static void expect_report(uint32_t pressed, uint8_t hat, uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry) {
    assert(!reports.empty());
    const std::array<uint8_t, 8> expected = {
        uint8_t(pressed), uint8_t(pressed >> 8), uint8_t(pressed >> 16), hat, lx, ly, rx, ry
    };
    assert(reports.back() == expected);
}

static void test_pair(bool right_first) {
    // Side identity, not slot/port/connection order, decides each half's controls.
    uint16_t left = right_first ? 0x0300 : 0x0100;
    uint16_t right = right_first ? 0x0100 : 0x0300;
    attach(41, right_first ? right : left);
    initialize(41, right_first ? right : left, right_first ? 2 : 1, true);
    attach(42, right_first ? left : right);
    initialize(42, right_first ? left : right, right_first ? 1 : 2, false);
    assert(registrations.size() == 1 && descriptors.size() == 1);
    assert(registrations[0].interface == pair_interface && registrations[0].port == 0);

    // Exercise the asymmetric calibration layouts, endpoint scaling and Y inversion.
    auto left_pressed = input(1, 1000, 3400, 0xF6, 0x29);
    auto right_pressed = input(2, 3200, 1000, 0xFF, 0x16);
    receive(left, left_pressed);
    receive(right, right_pressed);
    expect_report(0x3FFFF, 1, 0, 0, 255, 255);

    // Ignore truncated/unknown packets without changing either half's held state.
    size_t received = reports.size();
    for (unsigned len = 0; len < left_pressed.size(); len++) {
        assert(joycon_report(left, left_pressed.data(), len));
    }
    assert(joycon_report(left, nullptr, 0));
    auto unknown = left_pressed;
    unknown[0] = 0x3F;
    receive(left, unknown);
    assert(!joycon_report(0x5500, left_pressed.data(), left_pressed.size()));
    assert(reports.size() == received);

    receive(right, input(2, 2000, 2100, 0, 0));
    const uint32_t left_buttons = buttons({ 5, 7, 9, 11, 14, 15, 16 });
    expect_report(left_buttons, 1, 0, 0, 128, 128);
    receive(right, right_pressed);
    assert(joycon_detach(right));
    expect_report(left_buttons, 1, 0, 0, 128, 128);
    receive(left, input(1, 2000, 2100, 0, 0));
    expect_report(0, 8, 128, 128, 128, 128);
    receive(left, left_pressed);
    assert(joycon_detach(left));
    expect_report(0, 8, 128, 128, 128, 128);
    assert(!joycon_detach(left));
}

static void test_calibration_timeout() {
    constexpr uint16_t cid = 50, interface = 0x0200;
    attach(cid, interface);
    tick();
    auto info = reply(0x02, 0x82);
    info[17] = 1;
    receive(interface, info);
    tick();
    expect_spi(cid, 0x8010, 11);
    Command user_read = last_command(cid);
    for (int i = 0; i < 3; i++) {
        tick(500000);
        expect_retained(user_read);
    }
    tick();
    expect_spi(cid, 0x603D, 9);
    Command factory_read = last_command(cid);
    assert(factory_read.retained != user_read.retained);
    for (int i = 0; i < 3; i++) {
        tick(500000);
        expect_retained(user_read);
        expect_retained(factory_read);
    }
    tick();
    expect_command(cid, 0x03);
    // Full reports prove that mode switching succeeded even if its ACK was lost.
    receive(interface, input(1, 2000, 2000, 0x40, 0));
    expect_report(buttons({ 5 }), 8, 128, 128, 128, 128);
    tick();
    expect_command(cid, 0x30);
    receive(interface, reply(0x30));
    expect_retained(user_read);
    expect_retained(factory_read);
    assert(joycon_detach(interface));
}

static void test_generic_fallback() {
    constexpr uint8_t generic_descriptor[] = { 0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0xC0 };
    size_t sent = commands.size();
    assert(!joycon_attach(51, 0x0100, 2, generic_descriptor, sizeof(generic_descriptor)));
    tick();
    assert(commands.size() == sent);

    // A Pro Controller can share the descriptor; preserve its original generic path.
    attach(52, 0x0100);
    tick();
    auto info = reply(0x02, 0x82);
    info[17] = 3;
    receive(0x0100, info);
    assert(!joycon_report(0x0100, info.data(), info.size()));
    assert(descriptors.back().interface == 0x0100);
    assert(descriptors.back().data == std::vector<uint8_t>(std::begin(descriptor), std::end(descriptor)));
    assert(registrations.back().port == 2);
    expect_retained(last_command(52));
    assert(!joycon_detach(0x0100));

    // An unresponsive candidate receives a bounded probe, then returns to generic HID.
    sent = commands.size();
    attach(53, 0x0200);
    tick();
    Command probe = last_command(53);
    for (int i = 0; i < 3; i++) {
        tick(500000);
        expect_retained(probe);
    }
    assert(commands.size() == sent + 3);
    assert(!joycon_report(0x0200, info.data(), info.size()));
    assert(descriptors.back().interface == 0x0200);
    assert(!joycon_detach(0x0200));

    // A transient BTstack rejection retries the same packet. Unsent commands
    // must not accept a reply or consume the reply retry budget.
    attach(54, 0x0100);
    next_send_status = 0x0C;
    tick();
    Command rejected = last_command(54);
    size_t registered = registrations.size();
    receive(0x0100, info);
    assert(registrations.size() == registered);
    tick(500000);
    assert(last_command(54).retained == rejected.retained);
    assert(last_command(54).data == rejected.data);
    receive(0x0100, info);
    assert(!joycon_report(0x0100, info.data(), info.size()));
    expect_retained(rejected);
    assert(!joycon_detach(0x0100));
}

int main() {
    test_pair(false);
    test_pair(true);
    test_calibration_timeout();
    test_generic_fallback();
    puts("Joy-Con tests passed");
}
