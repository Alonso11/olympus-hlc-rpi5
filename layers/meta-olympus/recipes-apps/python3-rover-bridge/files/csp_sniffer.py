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
import time


CMD_PORT = 9000
TLM_PORT = 9001

CSP_ADDR_GCS = 1
CSP_ADDR_HLC = 2
CSP_PORT_CMD = 11


def _crc32c(data: bytes) -> int:
    """CRC-32C (Castagnoli) — matches libcsp 4.x csp_crc32_append."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


def csp_pack_v1(src, dst, dport, sport, payload: bytes) -> bytes:
    """CSPv1 big-endian with CRC-32C over payload-only — matches libcsp 4.x."""
    header = (
        ((2     & 0x03) << 30) |
        ((src   & 0x1F) << 25) |
        ((dst   & 0x1F) << 20) |
        ((dport & 0x3F) << 14) |
        ((sport & 0x3F) <<  8) |
        0x01  # FCRC32 flag
    )
    hdr_bytes = struct.pack(">I", header)
    crc = struct.pack(">I", _crc32c(payload))
    return hdr_bytes + payload + crc


def try_csp_parse(data: bytes, label: str):
    """Parsea y verifica un paquete CSP (CRC-32C sobre payload-only)."""
    if len(data) < 8:
        print(f"  [{label}] Demasiado corto ({len(data)} bytes)")
        return

    hdr_be = struct.unpack(">I", data[:4])[0]
    pri    = (hdr_be >> 30) & 0x03
    src    = (hdr_be >> 25) & 0x1F
    dst    = (hdr_be >> 20) & 0x1F
    dport  = (hdr_be >> 14) & 0x3F
    sport  = (hdr_be >>  8) & 0x3F
    flags  = (hdr_be >>  0) & 0xFF

    payload  = data[4:-4]
    crc_recv = data[-4:]
    crc_calc = struct.pack(">I", _crc32c(payload))
    crc_ok   = crc_recv == crc_calc

    print(f"\n  [{label}] {len(data)} bytes  crc={'✓ OK' if crc_ok else '✗ FAIL'}")
    print(f"  Header: pri={pri} src={src} dst={dst} dport={dport} sport={sport} flags=0x{flags:02x}")
    print(f"  CRC recv={crc_recv.hex()} calc={crc_calc.hex()}")
    print(f"  Payload ({len(payload)} B): {payload[:100]!r}")


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
