#!/usr/bin/env python3
"""
gcs_drive.py — Cliente GCS interactivo para control del rover Olympus.

Flujo completo:
  Laptop ─CSP/UDP─► RPi5:9000 ─UART─► Arduino Mega (LLC)
  Laptop ◄─CSP/UDP─ RPi5:9001 ◄─UART─ Arduino Mega (TLM)

Uso:
    python3 gcs_drive.py <RPi5_IP>
    python3 gcs_drive.py 192.168.68.68

Modos HLC compatibles:
    --mode gcs         Control manual puro (GCS dirige)
    --mode vision-gcs  Control supervisorio (YOLO + overrides GCS)

Comandos LLC directos:
    EXP:L:R      Mover (L/R = -100..100, positivo = adelante)
    STB          Detener (Standby)
    RST          Recuperar desde FAULT → STANDBY
    PING         Keepalive manual (se envía automático cada 5s)
    FLT          Forzar FAULT (test)
    CLB:L:R      Modo escalada

Comandos de modo supervisorio (solo --mode vision-gcs):
    MODE:AUTO    Ceder control al YOLO (autónomo)
    MODE:TELEOP  Tomar control manual

Atajos de movimiento:
    w / fwd      EXP:50:50   (adelante)
    s / bck      EXP:-50:-50 (atrás)
    a / left     EXP:-40:40  (giro izquierda)
    d / right    EXP:40:-40  (giro derecha)
    [espacio]    STB         (frenar)
    r            RST

Atajos de modo:
    auto         MODE:AUTO
    teleop       MODE:TELEOP

    q / quit     Salir
"""

import socket
import struct
import sys
import threading
import time

# ── CSP constants (matches olympus_hlc/csp.py) ───────────────────────────────
PRIO_NORM     = 2
FLAG_CRC32    = 0x01
CSP_ADDR_GCS  = 1
CSP_ADDR_HLC  = 2
CSP_PORT_TM   = 10
CSP_PORT_CMD  = 11
CSP_PORT_HB   = 1
CMD_UDP_PORT  = 9000
TLM_UDP_PORT  = 9001
PING_INTERVAL = 5.0   # seconds between auto-PINGs


def _crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


def csp_pack(payload: bytes) -> bytes:
    header = (
        ((PRIO_NORM   & 0x03) << 30) |
        ((CSP_ADDR_GCS & 0x1F) << 25) |
        ((CSP_ADDR_HLC & 0x1F) << 20) |
        ((CSP_PORT_CMD & 0x3F) << 14) |
        FLAG_CRC32
    )
    return struct.pack(">I", header) + payload + struct.pack(">I", _crc32c(payload))


def csp_unpack(data: bytes):
    if len(data) < 8:
        return None, None
    payload = data[4:-4]
    if struct.pack(">I", _crc32c(payload)) != data[-4:]:
        return None, None
    header = struct.unpack(">I", data[:4])[0]
    dport  = (header >> 14) & 0x3F
    return dport, payload.decode(errors="replace")


SHORTCUTS = {
    # Movement
    "w":      "EXP:50:50",
    "fwd":    "EXP:50:50",
    "s":      "EXP:-50:-50",
    "bck":    "EXP:-50:-50",
    "a":      "EXP:-40:40",
    "left":   "EXP:-40:40",
    "d":      "EXP:40:-40",
    "right":  "EXP:40:-40",
    " ":      "STB",
    "stop":   "STB",
    "r":      "RST",
    # Supervisory mode (vision-gcs only)
    "auto":   "MODE:AUTO",
    "teleop": "MODE:TELEOP",
}


class GCSDrive:

    def __init__(self, rpi_ip: str):
        self._rpi_ip  = rpi_ip
        self._running = True
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", TLM_UDP_PORT))
        self._sock.settimeout(1.0)
        self._last_ping = time.monotonic()
        self._tlm_count = 0

    def send(self, cmd: str) -> None:
        pkt = csp_pack(cmd.encode())
        self._sock.sendto(pkt, (self._rpi_ip, CMD_UDP_PORT))
        print(f"\r[TX] {cmd:<30}", flush=True)
        self._last_ping = time.monotonic()

    def _rx_loop(self) -> None:
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            dport, text = csp_unpack(data)
            if dport is None or text is None:
                print("\r[RX] CRC INVALIDO — paquete descartado", flush=True)
                continue
            if dport == CSP_PORT_TM:
                self._tlm_count += 1
                parts = text.strip().split(":")
                if len(parts) >= 3:
                    state = parts[1] if len(parts) > 1 else "?"
                    ts    = parts[3] if len(parts) > 3 else "?"
                    dist_near = parts[-3] if len(parts) > 3 else "?"
                    dist_far  = parts[-1] if len(parts) > 1 else "?"
                    print(f"\r[TLM #{self._tlm_count}] state={state} t={ts} near={dist_near} far={dist_far}    ", flush=True)
                else:
                    print(f"\r[TLM #{self._tlm_count}] {text.strip()}", flush=True)
            elif dport == CSP_PORT_HB:
                print(f"\r[HB_REQ] {text.strip()} — respondiendo PING", flush=True)
                self.send("PING")

    def _ping_loop(self) -> None:
        while self._running:
            time.sleep(0.5)
            if time.monotonic() - self._last_ping >= PING_INTERVAL:
                self.send("PING")

    def run(self) -> None:
        print(f"\n=== GCS Drive — {self._rpi_ip} ===")
        print("Movimiento : w/s/a/d=mover  [espacio]=stop  r=RST  q=salir")
        print("Modo       : auto=MODE:AUTO  teleop=MODE:TELEOP")
        print("Directo    : EXP:60:60  STB  RST  FLT  CLB:30:30  MODE:AUTO\n")

        # Enviar RST inicial para salir de FAULT
        self.send("RST")
        time.sleep(0.3)

        rx_thread   = threading.Thread(target=self._rx_loop,   daemon=True)
        ping_thread = threading.Thread(target=self._ping_loop,  daemon=True)
        rx_thread.start()
        ping_thread.start()

        try:
            while self._running:
                try:
                    line = input("> ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break

                if not line:
                    continue
                if line in ("q", "quit", "exit"):
                    break

                cmd = SHORTCUTS.get(line, line.upper())
                self.send(cmd)
        finally:
            self._running = False
            self.send("STB")   # frenar antes de salir
            time.sleep(0.3)
            self._sock.close()
            print("\n[GCS] Desconectado. Motores detenidos.")


def main() -> None:
    if len(sys.argv) < 2:
        print("Uso: python3 gcs_drive.py <RPi5_IP>")
        print("     python3 gcs_drive.py 192.168.68.68")
        sys.exit(1)
    GCSDrive(sys.argv[1]).run()


if __name__ == "__main__":
    main()
