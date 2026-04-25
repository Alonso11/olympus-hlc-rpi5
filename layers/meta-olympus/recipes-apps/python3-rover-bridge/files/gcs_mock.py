#!/usr/bin/env python3
"""
gcs_mock.py — GCS simulator para probar GCSSource / LibcspGCSSource.

Transporte: UDP punto-a-punto RPi5 ↔ laptop (WiFi).
  CMD → RPi5_IP:9000  (paquete CSP src=GCS=1, dst=HLC=2, dport=CMD=11)
  TLM ← RPi5_IP:9001  (paquete CSP src=HLC=2, dst=GCS=1, dport=TM=10)

Compatible con ambas fuentes:
  GCSSource        — raw sockets + CSP header manual en el HLC
  LibcspGCSSource  — libcsp_py3 + csp_if_udp en el HLC (--use-libcsp)

Uso:
  # Lab (mDNS — cualquier red):
  python3 gcs_mock.py olympus-rover.local

  # Field (AP hotspot del rover):
  python3 gcs_mock.py 192.168.100.1

  # Sin CSP (modo legado):
  python3 gcs_mock.py <IP> --no-csp
"""

import argparse
import select
import socket
import struct
import time


# ── CSP (mismo formato que olympus_hlc/csp.py) ───────────────────────────────

def _crc32c(data: bytes) -> int:
    """CRC-32C (Castagnoli) — matches libcsp 4.x csp_crc32_append."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


PRIO_NORM  = 2
FLAG_CRC32 = 0x01  # CSP_FCRC32 bit in CSPv1 (libcsp 4.x)

CSP_ADDR_GCS  = 1
CSP_ADDR_HLC  = 2
CSP_PORT_TM   = 10
CSP_PORT_CMD  = 11
CSP_PORT_HB   = 1


def csp_pack(src: int, dst: int, dport: int, sport: int, payload: bytes) -> bytes:
    """Wire: 4B header (BE) + payload + 4B CRC-32C (BE, over payload-only)."""
    header = (
        ((PRIO_NORM & 0x03) << 30) |
        ((src   & 0x1F) << 25) |
        ((dst   & 0x1F) << 20) |
        ((dport & 0x3F) << 14) |
        ((sport & 0x3F) <<  8) |
        FLAG_CRC32
    )
    hdr_bytes = struct.pack(">I", header)
    crc = struct.pack(">I", _crc32c(payload))
    return hdr_bytes + payload + crc


def csp_unpack(data: bytes) -> "tuple[int | None, bytes | None]":
    if len(data) < 8:
        return None, None
    payload  = data[4:-4]
    crc_recv = data[-4:]
    crc_calc = struct.pack(">I", _crc32c(payload))
    if crc_calc != crc_recv:
        return None, None
    header = struct.unpack(">I", data[:4])[0]
    return header, payload


def csp_dst_port(header: int) -> int:
    return (header >> 14) & 0x3F


# ── GCS mock ─────────────────────────────────────────────────────────────────

class GCSMock:

    def __init__(self, rpi_ip: str, cmd_port: int, tlm_port: int, csp: bool):
        self._rpi_ip   = rpi_ip
        self._cmd_port = cmd_port
        self._tlm_port = tlm_port
        self._csp      = csp
        self._tlm_count = 0
        self._hb_count  = 0

        # Socket de envío/recepción TLM (bind en tlm_port para recibir respuestas)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", tlm_port))
        self._sock.setblocking(False)
        print(f"[GCS] Escuchando TLM en :{tlm_port}  |  CMD → {rpi_ip}:{cmd_port}")
        print(f"[GCS] Modo: {'CSP+CRC32' if csp else 'ASCII (--no-csp)'}\n")

    def send_cmd(self, cmd: str) -> None:
        payload = cmd.encode()
        if self._csp:
            msg = csp_pack(CSP_ADDR_GCS, CSP_ADDR_HLC, CSP_PORT_CMD, 0, payload)
        else:
            msg = f"CMD:{cmd}\n".encode()
        self._sock.sendto(msg, (self._rpi_ip, self._cmd_port))
        print(f"[GCS → HLC] {cmd}")

    def drain_tlm(self, timeout_s: float = 0.3) -> None:
        """Lee todos los paquetes UDP disponibles durante timeout_s segundos."""
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            ready, _, _ = select.select([self._sock], [], [], remaining)
            if not ready:
                break
            try:
                data, addr = self._sock.recvfrom(1024)
            except OSError:
                break

            if self._csp:
                header, payload = csp_unpack(data)
                if header is None:
                    print(f"[GCS] CRC-32 INVÁLIDO desde {addr[0]} — paquete descartado")
                    continue
                dport = csp_dst_port(header)
                text = payload.decode(errors="replace") if payload else ""
                if dport == CSP_PORT_TM:
                    self._tlm_count += 1
                    print(f"[HLC → GCS] TLM #{self._tlm_count}: {text}")
                elif dport == CSP_PORT_HB:
                    self._hb_count += 1
                    print(f"[HLC → GCS] HB_REQ #{self._hb_count}: {text}")
                else:
                    print(f"[HLC → GCS] CSP port={dport}: {text}")
            else:
                line = data.decode(errors="replace").strip()
                if line.startswith("TLM:"):
                    self._tlm_count += 1
                    print(f"[HLC → GCS] TLM #{self._tlm_count}: {line}")
                else:
                    print(f"[HLC → GCS] {line}")

    def close(self) -> None:
        self._sock.close()


# ── Secuencia de test ─────────────────────────────────────────────────────────

def run_tests(gcs: GCSMock) -> None:
    PASS = "✓"
    FAIL = "✗"

    def step(label: str) -> None:
        print(f"\n{'─'*60}")
        print(f"  TEST: {label}")
        print(f"{'─'*60}")

    # ── Test 1: Conexión inicial ──────────────────────────────────────────────
    step("T1 — Conexión inicial: EXP:40:40")
    gcs.send_cmd("EXP:40:40")
    gcs.drain_tlm(2.0)
    print(f"[{PASS if gcs._tlm_count > 0 else FAIL}] TLM recibidos: {gcs._tlm_count}")

    # ── Test 2: STB ───────────────────────────────────────────────────────────
    step("T2 — STB (detener)")
    gcs.send_cmd("STB")
    gcs.drain_tlm(2.0)

    # ── Test 3: PING ──────────────────────────────────────────────────────────
    step("T3 — PING keepalive")
    gcs.send_cmd("PING")
    gcs.drain_tlm(1.0)

    # ── Test 4: link_lost (pausa > 10s) ──────────────────────────────────────
    step("T4 — Silencio 17s → RPi5 debe detectar link_lost y enviar HB_REQ")
    print("[GCS] Sin enviar comandos — esperando 17s (link_lost@10s + primer retry@15s)...")
    t4_hb_before = gcs._hb_count
    gcs.drain_tlm(17.0)
    t4_hb_after = gcs._hb_count
    received_hb = t4_hb_after > t4_hb_before
    print(f"[{'✓' if received_hb else '✗ WARN'}] HB_REQ de reconexión recibidos: {t4_hb_after - t4_hb_before}")
    print("  (Si no se reciben HB_REQ, verificar que CommLinkMonitor está activo en el HLC)")

    # ── Test 5: link_restored ─────────────────────────────────────────────────
    step("T5 — link_restored: retomar con EXP")
    gcs.send_cmd("EXP:40:40")
    gcs.drain_tlm(2.0)
    print(f"[{PASS}] Comando enviado — ver log HLC para 'link_restored'")

    # ── Test 6: ciclo de falla FLT → RST ─────────────────────────────────────
    step("T6 — FLT seguido de RST (recuperación de falla)")
    gcs.send_cmd("FLT")
    gcs.drain_tlm(1.0)
    gcs.send_cmd("RST")
    gcs.drain_tlm(1.5)
    print(f"[{PASS}] Ver log HLC para transición FAULT → STANDBY")

    # ── Test 7: CRC corrupto ─────────────────────────────────────────────────
    if isinstance(gcs, GCSMock) and gcs._csp:
        step("T7 — Paquete CSP con CRC corrupto (debe ser descartado por HLC)")
        payload = b"EXP:99:99"
        bad_pkt  = csp_pack(CSP_ADDR_GCS, CSP_ADDR_HLC, CSP_PORT_CMD, 0, payload)
        # Corromper el último byte del CRC
        bad_pkt = bad_pkt[:-1] + bytes([bad_pkt[-1] ^ 0xFF])
        gcs._sock.sendto(bad_pkt, (gcs._rpi_ip, gcs._cmd_port))
        print("[GCS] Paquete corrompido enviado — HLC debe loguear 'CSP CRC-32 inválido'")
        gcs.drain_tlm(1.0)

    # ── Resumen ───────────────────────────────────────────────────────────────
    print(f"\n{'═'*60}")
    print(f"  RESUMEN")
    print(f"{'═'*60}")
    print(f"  TLM recibidos total : {gcs._tlm_count}")
    print(f"  HB_REQ recibidos    : {gcs._hb_count}")
    print(f"\n  Verificar en el log HLC (/var/log/olympus/):")
    print(f"    - link_lost  → forzó STB")
    print(f"    - link_restored → retomó comando")
    print(f"    - CSP CRC-32 inválido (test 7)")
    print(f"    - Transición FAULT → STANDBY (test 6)")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="GCS mock para test GCSSource Olympus HLC")
    parser.add_argument("rpi_ip", help="IP de la RPi5 (ej. 192.168.1.50)")
    parser.add_argument("--port-cmd", type=int, default=9000,
                        help="Puerto UDP CMD en RPi5 (default: 9000)")
    parser.add_argument("--port-tlm", type=int, default=9001,
                        help="Puerto UDP TLM en laptop (default: 9001)")
    parser.add_argument("--no-csp", action="store_true",
                        help="Usar protocolo ASCII sin CSP (CSP_ENABLED=False en HLC)")
    args = parser.parse_args()

    gcs = GCSMock(args.rpi_ip, args.port_cmd, args.port_tlm, csp=not args.no_csp)
    try:
        run_tests(gcs)
    except KeyboardInterrupt:
        print("\n[GCS] Interrumpido por usuario.")
    finally:
        gcs.close()
        print("[GCS] Socket cerrado.")


if __name__ == "__main__":
    main()
