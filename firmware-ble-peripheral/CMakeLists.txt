cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_hid_peripheral)

target_sources(app PRIVATE
    src/main.cc
    src/descriptors.cc
)
