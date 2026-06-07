#!/usr/bin/env python3
"""Diagnose which data libcsp computes CRC32 over."""
import zlib, struct

# Packet #2 from sniffer (111 bytes), split into header+payload (no CRC)
raw = bytes.fromhex(
    "84128001"
    "544c4d3a4e4f524d414c3a3030303030303a36383030306d733a31363030306d56"
    "3a3530306d413a3130303a3130303a3130303a3130303a3130303a3130303a3235"
    "433a32353a32353a32353a32353a32353a3235433a313030306d6d3a303a303a30"
    "3a303a30"
)
hdr_be  = raw[:4]          # wire format  : 84 12 80 01
payload = raw[4:]          # 103 bytes TLM text
hdr_le  = hdr_be[::-1]    # LE in memory : 01 80 12 84

expected = "12e4b6c0"

print(f"header BE : {hdr_be.hex()}")
print(f"header LE : {hdr_le.hex()}")
print(f"payload   : {len(payload)} bytes")
print(f"expected  : {expected}\n")

for label, d in [
    ("pay-only",         payload),
    ("BE-hdr + payload", raw),
    ("LE-hdr + payload", hdr_le + payload),
    ("payload + BE-hdr", payload + hdr_be),
    ("payload + LE-hdr", payload + hdr_le),
]:
    val = zlib.crc32(d) & 0xFFFFFFFF
    result = struct.pack(">I", val).hex()
    match  = "  ← MATCH" if result == expected else ""
    print(f"  {label:25s}: {result}{match}")
