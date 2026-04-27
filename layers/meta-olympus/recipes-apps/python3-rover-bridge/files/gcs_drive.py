#!/usr/bin/env python3
"""
gcs_drive.py — Cliente GCS interactivo para control del rover Olympus.

Usa libcsp_py3 nativo (mismo transporte que el HLC en RPi5).

Flujo:
  Laptop ─CSP/UDP─► RPi5:9000 ─UART─► Arduino Mega (LLC)
  Laptop ◄─CSP/UDP─ RPi5:9001 ◄─UART─ Arduino Mega (TLM)

Uso:
    python3 gcs_drive.py <RPi5_IP>
    python3 gcs_drive.py 192.168.X.X

Modos HLC compatibles:
    --mode gcs         Control manual puro (GCS dirige)
    --mode vision-gcs  Control supervisorio (YOLO + overrides GCS)

Atajos de movimiento:
    w / fwd      EXP:25:25   (adelante)
    s / bck      EXP:-25:-25 (atrás)
    a / left     EXP:-20:20  (giro izquierda)
    d / right    EXP:20:-20  (giro derecha)
    [espacio]    STB         (frenar)
    r            RST

Atajos de modo:
    auto         MODE:AUTO
    teleop       MODE:TELEOP

    q / quit     Salir
"""

import os
import sys
import threading
import time

# ── libcsp_py3: buscar .so junto a este script o en LD_LIBRARY_PATH ──────────
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
# libcsp.so debe cargarse antes que libcsp_py3.so (dependencia dinámica)
import ctypes as _ct
_ct.CDLL(os.path.join(_HERE, "libcsp.so"), _ct.RTLD_GLOBAL)
import libcsp_py3 as csp  # noqa: E402

# ── Constantes CSP (ICD-CSP-001) ─────────────────────────────────────────────
CSP_ADDR_GCS  = 1
CSP_ADDR_HLC  = 2
CSP_PORT_CMD  = 11
CSP_PORT_TM   = 10
CSP_PORT_HB   = 1
CMD_UDP_PORT  = 9000   # HLC escucha aquí (RPi5 RX)
TLM_UDP_PORT  = 9001   # GCS escucha aquí (Laptop RX)
PING_INTERVAL = 5.0

SHORTCUTS = {
    "w":      "EXP:25:25",
    "fwd":    "EXP:25:25",
    "s":      "EXP:-25:-25",
    "bck":    "EXP:-25:-25",
    "a":      "EXP:-20:20",
    "left":   "EXP:-20:20",
    "d":      "EXP:20:-20",
    "right":  "EXP:20:-20",
    " ":      "STB",
    "stop":   "STB",
    "r":      "RST",
    "auto":   "MODE:AUTO",
    "teleop": "MODE:TELEOP",
}


class GCSDrive:

    def __init__(self, rpi_ip: str):
        self._rpi_ip   = rpi_ip
        self._running  = True
        self._tlm_count = 0
        self._last_ping = time.monotonic()

        # Nodo GCS = 1, espejo del HLC (nodo=2)
        csp.init("olympus-gcs", "Laptop", "1.0", 1)

        # GCS: escucha TLM en 9001, envía CMDs a rpi_ip:9000
        csp.udp_init(
            CSP_ADDR_GCS,
            rpi_ip,
            TLM_UDP_PORT,   # lport — laptop escucha aquí
            CMD_UDP_PORT,   # rport — envía a RPi5:9000
            True,
        )

        csp.rtable_load("0/0 UDP")
        csp.route_start_task()

        # Socket servidor para recibir TLM y HB_REQ del HLC
        self._sock = csp.socket()
        csp.bind(self._sock, csp.CSP_ANY)
        csp.listen(self._sock, 5)

    def send(self, cmd: str) -> None:
        payload = cmd.encode()
        pkt = csp.buffer_get(len(payload))
        if pkt is None:
            print(f"\r[ERROR] No hay buffers CSP disponibles", flush=True)
            return
        csp.packet_set_data(pkt, payload)
        try:
            csp.sendto(2, CSP_ADDR_HLC, CSP_PORT_CMD, 0, csp.CSP_O_CRC32, pkt)
        except Exception as exc:
            csp.buffer_free(pkt)
            print(f"\r[ERROR] sendto: {exc}", flush=True)
            return
        print(f"\r[TX] {cmd:<30}", flush=True)
        self._last_ping = time.monotonic()

    def _rx_loop(self) -> None:
        while self._running:
            conn = csp.accept(self._sock, 200)
            if conn is None:
                continue
            dport = csp.conn_dport(conn)
            pkt   = csp.read(conn, 100)
            csp.close(conn)
            if pkt is None:
                continue
            data = bytes(csp.packet_get_data(pkt))
            csp.buffer_free(pkt)
            text = data.decode("utf-8", errors="replace").strip()

            if dport == CSP_PORT_TM:
                self._tlm_count += 1
                parts = text.split(":")
                if len(parts) >= 4:
                    state     = parts[1] if len(parts) > 1 else "?"
                    ts        = parts[3] if len(parts) > 3 else "?"
                    dist_near = parts[-3] if len(parts) > 3 else "?"
                    dist_far  = parts[-1]
                    print(f"\r[TLM #{self._tlm_count}] state={state} t={ts} near={dist_near} far={dist_far}    ", flush=True)
                else:
                    print(f"\r[TLM #{self._tlm_count}] {text}", flush=True)
            elif dport == CSP_PORT_HB:
                print(f"\r[HB_REQ] {text} — respondiendo PING", flush=True)
                self.send("PING")

    def _ping_loop(self) -> None:
        while self._running:
            time.sleep(0.5)
            if time.monotonic() - self._last_ping >= PING_INTERVAL:
                self.send("PING")

    def run(self) -> None:
        print(f"\n=== GCS Drive (libcsp) — {self._rpi_ip} ===")
        print("Movimiento : w/s/a/d=mover  [espacio]=stop  r=RST  q=salir")
        print("Modo       : auto=MODE:AUTO  teleop=MODE:TELEOP")
        print("Directo    : EXP:60:60  STB  RST  FLT  CLB:30:30  MODE:AUTO\n")

        self.send("RST")
        time.sleep(0.3)

        rx_thread   = threading.Thread(target=self._rx_loop,  daemon=True)
        ping_thread = threading.Thread(target=self._ping_loop, daemon=True)
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
            self.send("STB")
            time.sleep(0.3)
            print("\n[GCS] Desconectado. Motores detenidos.")


def main() -> None:
    if len(sys.argv) < 2:
        print("Uso: python3 gcs_drive.py <RPi5_IP>")
        sys.exit(1)
    GCSDrive(sys.argv[1]).run()


if __name__ == "__main__":
    main()
