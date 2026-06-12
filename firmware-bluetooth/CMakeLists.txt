cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/usb_composite.overlay)

add_compile_definitions(PERSISTED_CONFIG_SIZE=2048)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

set(REMAPPER_SRC ../firmware/src)

target_include_directories(app PRIVATE ${REMAPPER_SRC} src)

target_sources(app PRIVATE
    src/main.cc
    src/imu.cc
    src/slip_input.cc
    src/ble_hids.cc
    src/ble_hids_obj.c
    src/usb_cdc_input.cc
    ${REMAPPER_SRC}/config.cc
    ${REMAPPER_SRC}/crc.cc
    ${REMAPPER_SRC}/descriptor_parser.cc
    ${REMAPPER_SRC}/globals.cc
    ${REMAPPER_SRC}/interval_override.cc
    ${REMAPPER_SRC}/our_descriptor.cc
    ${REMAPPER_SRC}/quirks.cc
    ${REMAPPER_SRC}/remapper.cc
    ${REMAPPER_SRC}/ps_auth.cc
)
