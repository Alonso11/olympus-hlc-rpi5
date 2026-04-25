# olympus_hlc/sources/gcs_libcsp.py — LibcspGCSSource: libcsp 4.2 nativo via UDP
#
# Transporte: csp_if_udp — WiFi punto-a-punto RPi5 ↔ GCS.
# No requiere zmqproxy ni zeromq. El binding udp_init lo añade el patch
# 0001-add-udp-python-binding.patch en la receta libcsp_4.2.bb.
#
# Arquitectura de red CSP (ICD-CSP-001):
#
#   GCS (laptop)                         RPi5
#   libcsp node=1                        libcsp node=2  (este módulo)
#   csp_if_udp                           csp_if_udp
#     TX → RPi5_IP:9000  ─── WiFi ───→  lport=9000  (RX comandos)
#     RX ← RPi5_IP:9001  ←───────────── rport=9001  (TX telemetría)
#
# ── Añadir radio UHF en el futuro ────────────────────────────────────────────
# Cuando llegue el radio-módem UART (tipo AX100/ES920 a /dev/ttyUSB1):
#
#   csp.kiss_init("/dev/ttyUSB1", CSP_ADDR_HLC, baudrate=9600, is_default=False)
#   csp.rtable_load(f"0/0 UDP\n{CSP_ADDR_GCS_UHF}/8 KISS")
#
# No hay que tocar nada más. El router CSP decide por dirección destino:
#   - Paquetes a CSP_ADDR_GCS (1)          → por UDP  (WiFi)
#   - Paquetes a CSP_ADDR_GCS_UHF (3)      → por KISS (radio)
# ─────────────────────────────────────────────────────────────────────────────
#
# Activar:  python3 -m olympus_hlc --mode gcs [--gcs-host <IP_GCS>]
# Requiere: imagen Yocto con libcsp (CSP_IF_UDP=ON + patch udp_init)

import time

from ..interfaces import CommandSource
from ..monitors import CommLinkMonitor
from ..config import (
    CSP_ADDR_HLC, CSP_ADDR_GCS,
    CSP_PORT_CMD, CSP_PORT_TM, CSP_PORT_HB,
    GCS_LISTEN_PORT, GCS_REPLY_PORT,
)

_CSP_PRIO_NORM = 2


def _import_libcsp():
    try:
        import libcsp_py3  # noqa: PLC0415
        return libcsp_py3
    except ImportError as exc:
        raise ImportError(
            "libcsp_py3 no encontrado. "
            "Verificar imagen Yocto: receta libcsp_4.2.bb con "
            "CSP_IF_UDP=ON y patch 0001-add-udp-python-binding."
        ) from exc


class LibcspGCSSource(CommandSource):
    """
    Fuente de comandos GCS usando libcsp 4.2 nativo con csp_if_udp (WiFi).

    Inicialización (una sola vez al arrancar el HLC):
      1. csp.init()           — configura el nodo CSP local (addr=HLC=2)
      2. csp.udp_init()       — registra interfaz UDP; lanza hilo RX en lport
      3. csp.rtable_load()    — ruta todos los paquetes por UDP
      4. csp.route_start_task() — lanza pthread interno de routing
      5. csp.socket/bind/listen — servidor CSP en CSP_ANY

    Para añadir UHF: csp.kiss_init() + actualizar rtable_load(). Sin más cambios.
    """

    def __init__(self, gcs_host: str = ""):
        self._csp = _import_libcsp()
        csp = self._csp

        csp.init("olympus-hlc", "RPi5", "2.16", 1)  # version=1: wire format 4B big-endian (5-bit addr)

        # Interfaz UDP — enlace WiFi punto-a-punto con el GCS.
        # lport: puerto donde la RPi5 escucha comandos entrantes (GCS → RPi5).
        # rport: puerto del GCS donde va la telemetría (RPi5 → GCS).
        #
        # gcs_host="" activa peer learning dinámico: el primer CMD recibido
        # actualiza el TX peer automáticamente (patch 0002). No hace falta
        # saber la IP del GCS de antemano — funciona en cualquier red.
        csp.udp_init(
            CSP_ADDR_HLC,     # dirección CSP local
            gcs_host,         # IP GCS: "" = aprender del primer CMD
            GCS_LISTEN_PORT,  # lport=9000 — RX comandos
            GCS_REPLY_PORT,   # rport=9001 — TX telemetría
            True,             # is_default=True
        )

        csp.rtable_load("0/0 UDP")
        csp.route_start_task()

        self._sock = csp.socket()
        csp.bind(self._sock, csp.CSP_ANY)
        csp.listen(self._sock, 5)

        self._last_recv = time.monotonic()
        self._probe_seq = 0

        print(f"[LibcspGCSSource] node={CSP_ADDR_HLC} UDP "
              f"lport={GCS_LISTEN_PORT} → {gcs_host}:{GCS_REPLY_PORT}")

    # ── CommandSource interface ───────────────────────────────────────────────

    def next_command(self, log=None) -> "str | None":
        csp = self._csp
        conn = csp.accept(self._sock, 0)  # pycsp es METH_VARARGS — no admite kwargs
        if conn is None:
            return None

        dport = csp.conn_dport(conn)
        pkt   = csp.read(conn, 100)  # pycsp METH_VARARGS — no admite kwargs
        csp.close(conn)

        if pkt is None:
            return None

        self._last_recv = time.monotonic()
        data = bytes(csp.packet_get_data(pkt))
        csp.buffer_free(pkt)

        if dport == CSP_PORT_HB:
            return None  # heartbeat — solo actualiza last_recv

        if dport != CSP_PORT_CMD:
            return None

        cmd = data.decode("utf-8", errors="replace").strip()
        if log:
            log.info("COMM", f"CSP CMD (UDP node {CSP_ADDR_GCS}→{CSP_ADDR_HLC}): {cmd!r}")
        return cmd

    def on_tlm(self, raw_tlm: str) -> None:
        """Envía frame TLM al GCS como paquete CSP via UDP (downlink SRS-020)."""
        self._send_payload(CSP_ADDR_GCS, CSP_PORT_TM, raw_tlm.encode())

    @property
    def last_recv_time(self) -> float:
        return self._last_recv

    def send_probe(self) -> None:
        """Envía HB_REQ al GCS (CommLinkMonitor, SRS-013)."""
        self._probe_seq += 1
        self._send_payload(
            CSP_ADDR_GCS, CSP_PORT_HB,
            f"HB_REQ:{self._probe_seq}".encode(),
        )

    def make_link_monitor(self) -> CommLinkMonitor:
        return CommLinkMonitor()

    def close(self) -> None:
        # csp.close() es para connections, no sockets. El socket se libera
        # automáticamente por el destructor del capsule cuando el GC lo recolecta.
        self._sock = None

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send_payload(self, dst: int, dport: int, payload: bytes) -> None:
        csp = self._csp
        pkt = csp.buffer_get(len(payload))
        if pkt is None:
            return
        csp.packet_set_data(pkt, payload)
        try:
            csp.sendto(_CSP_PRIO_NORM, dst, dport, 0, csp.CSP_O_CRC32, pkt)
        except Exception:
            csp.buffer_free(pkt)
