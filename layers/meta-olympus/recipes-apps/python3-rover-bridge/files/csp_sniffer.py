#!/usr/bin/env python3
"""
csp_sniffer.py — Captura y decodifica los bytes exactos que libcsp envía por UDP.

Escucha en :9001 (TLM port) y muestra el formato wire real sin asumir nada.
Prueba big-endian Y little-endian para encontrar qué formato usa libcsp.

Uso:
  python3 csp_sniffer.py          # escuchar en :9001
  python3 csp_sniffer.py --send 192.168.18.245  # también enviar un CMD para provocar TLM
"""

import argparse
import select
import socket
import struct
import zlib
import time


CMD_PORT = 9000
TLM_PORT = 9001

CSP_ADDR_GCS = 1
CSP_ADDR_HLC = 2
CSP_PORT_CMD = 11


def csp_pack_v1(src, dst, dport, sport, payload: bytes) -> bytes:
    """CSPv1 big-endian — lo que gcs_mock.py envía."""
    header = (
        ((2    & 0x03) << 30) |
        ((src  & 0x1F) << 25) |
        ((dst  & 0x1F) << 20) |
        ((dport & 0x3F) << 14) |
        ((sport & 0x3F) <<  8) |
        0b10  # FLAG_CRC32
    )
    raw = struct.pack(">I", header) + payload
    return raw + struct.pack(">I", zlib.crc32(raw) & 0xFFFFFFFF)


def try_csp_parse(data: bytes, label: str):
    """Intenta parsear como CSP con header big-endian y little-endian."""
    if len(data) < 8:
        print(f"  [{label}] Demasiado corto ({len(data)} bytes)")
        return

    # Big-endian header
    hdr_be = struct.unpack(">I", data[:4])[0]
    pri_be    = (hdr_be >> 30) & 0x03
    src_be    = (hdr_be >> 25) & 0x1F
    dst_be    = (hdr_be >> 20) & 0x1F
    dport_be  = (hdr_be >> 14) & 0x3F
    sport_be  = (hdr_be >>  8) & 0x3F
    flags_be  = (hdr_be >>  0) & 0xFF
    # CSP_FCRC32 = bit 0 (0x01) in libcsp 4.x — NOT bit 1 (0x02=RDP)
    has_crc   = bool(flags_be & 0x01)

    print(f"\n  [{label}] {len(data)} bytes raw: {data.hex()}")
    print(f"  Header BE:  pri={pri_be} src={src_be} dst={dst_be} "
          f"dport={dport_be} sport={sport_be} flags=0x{flags_be:02x} crc_flag={has_crc}")

    # Always try CRC verification on last 4 bytes regardless of flag
    if len(data) >= 8:
        raw_be = data[:-4]
        crc_recv = data[-4:]
        crc_val = zlib.crc32(raw_be) & 0xFFFFFFFF
        crc_calc_be = struct.pack(">I", crc_val)
        crc_calc_le = struct.pack("<I", crc_val)
        if has_crc:
            print(f"  CRC recv:      {crc_recv.hex()}")
            print(f"  CRC calc (BE): {crc_calc_be.hex()} — {'✓ MATCH' if crc_recv == crc_calc_be else '✗ MISMATCH'}")
            print(f"  CRC calc (LE): {crc_calc_le.hex()} — {'✓ MATCH' if crc_recv == crc_calc_le else '✗ MISMATCH'}")
            payload = raw_be[4:]
        else:
            print(f"  (no CRC flag — last 4 bytes as CRC anyway for diagnosis)")
            print(f"  CRC recv:      {crc_recv.hex()}")
            print(f"  CRC calc (BE): {crc_calc_be.hex()} — {'✓ MATCH' if crc_recv == crc_calc_be else '✗ MISMATCH'}")
            print(f"  CRC calc (LE): {crc_calc_le.hex()} — {'✓ MATCH' if crc_recv == crc_calc_le else '✗ MISMATCH'}")
            payload = data[4:]
        print(f"  Payload ({len(payload)} B): {payload[:80]!r}")

    # Little-endian header (por si libcsp usa LE)
    hdr_le = struct.unpack("<I", data[:4])[0]
    pri_le   = (hdr_le >> 30) & 0x03
    src_le   = (hdr_le >> 25) & 0x1F
    dst_le   = (hdr_le >> 20) & 0x1F
    dport_le = (hdr_le >> 14) & 0x3F
    sport_le = (hdr_le >>  8) & 0x3F
    flags_le = (hdr_le >>  0) & 0xFF
    print(f"  Header LE:  pri={pri_le} src={src_le} dst={dst_le} "
          f"dport={dport_le} sport={sport_le} flags=0x{flags_le:02x}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--send", metavar="RPI5_IP",
                        help="Enviar un CMD de prueba para provocar TLM")
    parser.add_argument("--port", type=int, default=TLM_PORT)
    parser.add_argument("--count", type=int, default=5,
                        help="Número de paquetes a capturar (default: 5)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.setblocking(False)

    print(f"Escuchando en :{args.port} — capturando {args.count} paquetes")

    if args.send:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cmd = csp_pack_v1(CSP_ADDR_GCS, CSP_ADDR_HLC, CSP_PORT_CMD, 0, b"PING")
        tx.sendto(cmd, (args.send, CMD_PORT))
        print(f"CMD PING enviado a {args.send}:{CMD_PORT}")
        tx.close()

    received = 0
    deadline = time.monotonic() + 30.0
    while received < args.count and time.monotonic() < deadline:
        r, _, _ = select.select([sock], [], [], 1.0)
        if not r:
            continue
        try:
            data, addr = sock.recvfrom(2048)
        except OSError:
            continue
        received += 1
        print(f"\n{'─'*60}")
        print(f"Paquete #{received} desde {addr[0]}:{addr[1]}")
        try_csp_parse(data, "RAW")

    sock.close()
    print(f"\n{'═'*60}")
    print(f"Capturados {received} paquetes.")


if __name__ == "__main__":
    main()
