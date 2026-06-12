#!/usr/bin/env python3
"""Send SLIP-framed virtual HID reports to the nRF52840 remapper over USB CDC."""

from __future__ import annotations

import argparse
import struct
import sys
import zlib

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    raise

END = 0xC0
ESC = 0xDB
ESC_END = 0xDC
ESC_ESC = 0xDD

PROTOCOL_VERSION = 1


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def slip_encode(payload: bytes) -> bytes:
    frame = bytearray([END])
    for byte in payload:
        if byte == END:
            frame.extend([ESC, ESC_END])
        elif byte == ESC:
            frame.extend([ESC, ESC_ESC])
        else:
            frame.append(byte)
    frame.extend(struct.pack("<I", crc32(payload)))
    frame.append(END)
    return bytes(frame)


def build_packet(descriptor_number: int, report_id: int, report_payload: bytes) -> bytes:
    header = struct.pack(
        "<BBBB",
        PROTOCOL_VERSION,
        descriptor_number,
        len(report_payload),
        report_id,
    )
    return header + report_payload


def parse_hex_bytes(text: str) -> bytes:
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    parts = [part for part in cleaned.split() if part]
    return bytes(int(part, 16) for part in parts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port for the CDC ACM interface")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--descriptor", type=int, default=0, help="Advisory output descriptor number")
    parser.add_argument("--report-id", type=int, default=1, help="HID report ID")
    parser.add_argument(
        "--payload",
        default="00",
        help="Report payload bytes in hex, without the report ID",
    )
    args = parser.parse_args()

    payload = parse_hex_bytes(args.payload)
    packet = build_packet(args.descriptor, args.report_id, payload)
    frame = slip_encode(packet)

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        ser.write(frame)
        ser.flush()

    print(f"Sent {len(frame)} byte SLIP frame to {args.port}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
