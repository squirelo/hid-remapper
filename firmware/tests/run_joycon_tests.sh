#!/bin/sh
set -eu

cd "$(dirname "$0")"
binary=$(mktemp "${TMPDIR:-/tmp}/joycon-tests.XXXXXX")
trap 'rm -f "$binary"' EXIT HUP INT TERM
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -Istubs -I../src joycon_test.cc ../src/joycon.cc -o "$binary"
"$binary"
