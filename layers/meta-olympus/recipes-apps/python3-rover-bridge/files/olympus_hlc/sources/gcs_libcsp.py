# olympus_hlc/sources/gcs_libcsp.py — LibcspGCSSource: libcsp 4.2 nativo
#
# Transportes (ICD-CSP-001):
#   Primario:  csp_if_udp  — WiFi UDP punto-a-punto RPi5 ↔ GCS (node=1)
#   Respaldo:  csp_if_kiss — radio UHF serie /dev/ttyTNC ↔ GCS UHF (node=3)
#
# Arquitectura de red CSP:
#
#   GCS (laptop)                         RPi5
#   libcsp node=1 (WiFi)                 libcsp node=2  (este módulo)
#   csp_if_udp                           csp_if_udp
#     TX → RPi5_IP:9000  ─── WiFi ───→  lport=9000  (RX comandos)
#     RX ← RPi5_IP:9001  ←───────────── rport=9001  (TX telemetría)
#
#   GCS UHF  (node=3)                    RPi5
#   csp_if_kiss (/dev/ttyTNC)            csp_if_kiss (/dev/ttyTNC)
#     TX / RX  ─── UHF ────────────────── RX / TX   (respaldo automático)
#
# Conmutación WiFi → UHF automática:
#   Si no se recibe ningún paquete del GCS durante GCS_LINK_LOST_S segundos,
#   _activate_uhf() recarga la tabla de rutas y envía TLM/HB por KISS con
#   RDP (CSP_O_RDP | CSP_O_CRC32). Al llegar un paquete UDP del GCS se vuelve
#   automáticamente a WiFi (_deactivate_uhf).
#
# RDP preconfigurado (global libcsp, inactivo en WiFi):
#   window=3, conn_timeout=2000ms, packet_timeout=500ms,
#   delayed_acks=1, ack_timeout=100ms, ack_delay_count=2.
#
# Activar:  python3 -m olympus_hlc --mode gcs [--gcs-host <IP_GCS>]
# Requiere: imagen Yocto con libcsp (CSP_IF_UDP=ON, CSP_IF_KISS=ON,
#           CSP_USE_RDP=ON, CSP_ENABLE_KISS_CRC=ON, patch 0001-add-udp-python-binding)

import time

from ..interfaces import CommandSource
from ..monitors import CommLinkMonitor
from ..config import (
    CSP_ADDR_HLC, CSP_ADDR_GCS, CSP_ADDR_GCS_UHF,
    CSP_PORT_CMD, CSP_PORT_TM, CSP_PORT_HB,
    GCS_LISTEN_PORT, GCS_REPLY_PORT, GCS_LINK_LOST_S,
    CSP_UHF_DEVICE, CSP_UHF_BAUD,
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
            "CSP_IF_UDP=ON, CSP_IF_KISS=ON, CSP_USE_RDP=ON "
            "y patch 0001-add-udp-python-binding."
        ) from exc


class LibcspGCSSource(CommandSource):
    """
    Fuente de comandos GCS usando libcsp 4.2 nativo.

    Transportes:
      - Primario:  csp_if_udp  (WiFi, CSP_O_CRC32)
      - Respaldo:  csp_if_kiss (UHF serie, CSP_O_RDP | CSP_O_CRC32)

    La conmutación WiFi → UHF ocurre cuando no se reciben paquetes del GCS
    en GCS_LINK_LOST_S segundos. La vuelta a WiFi es automática al recibir
    cualquier paquete UDP del GCS (last_recv actualizado).
    """

    def __init__(self, gcs_host: str = ""):
        self._csp = _import_libcsp()
        csp = self._csp

        csp.init("olympus-hlc", "RPi5", "2.16", 1)  # version=1: wire format 4B big-endian (5-bit addr)

        # Interfaz UDP — enlace WiFi primario.
        # gcs_host="" activa peer learning dinámico (patch 0002).
        csp.udp_init(
            CSP_ADDR_HLC,
            gcs_host,
            GCS_LISTEN_PORT,
            GCS_REPLY_PORT,
            True,             # is_default=True
        )

        # Parámetros RDP globales — sólo se activan con CSP_O_RDP (enlace UHF).
        # Los paquetes UDP (WiFi) usan únicamente CSP_O_CRC32; RDP no interviene.
        csp.rdp_set_opt(3, 2000, 500, 1, 100, 2)

        # Interfaz KISS / UHF — opcional: no-op si el dispositivo no existe.
        self._uhf_available = False
        try:
            csp.kiss_init(CSP_UHF_DEVICE, CSP_ADDR_HLC, CSP_UHF_BAUD)
            self._uhf_available = True
            print(f"[LibcspGCSSource] UHF KISS listo en {CSP_UHF_DEVICE}@{CSP_UHF_BAUD}")
        except Exception as exc:
            print(f"[LibcspGCSSource] UHF no disponible ({exc}) — sólo WiFi")

        csp.rtable_load("0/0 UDP")
        csp.route_start_task()

        self._sock = csp.socket()
        csp.bind(self._sock, csp.CSP_ANY)
        csp.listen(self._sock, 5)

        self._last_recv = time.monotonic()
        self._probe_seq = 0
        self._uhf_active = False

        print(f"[LibcspGCSSource] node={CSP_ADDR_HLC} UDP "
              f"lport={GCS_LISTEN_PORT} → {gcs_host}:{GCS_REPLY_PORT}")

    # ── Propiedades de estado ─────────────────────────────────────────────────

    @property
    def _gcs_dst(self) -> int:
        """Dirección CSP destino del GCS según enlace activo."""
        return CSP_ADDR_GCS_UHF if self._uhf_active else CSP_ADDR_GCS

    # ── CommandSource interface ───────────────────────────────────────────────

    def next_command(self, log=None) -> "str | None":
        self._check_link_switch()

        csp = self._csp
        conn = csp.accept(self._sock, 0)  # pycsp METH_VARARGS — no admite kwargs
        if conn is None:
            return None

        dport = csp.conn_dport(conn)
        pkt   = csp.read(conn, 100)       # pycsp METH_VARARGS — no admite kwargs
        csp.close(conn)

        if pkt is None:
            return None

        data = bytes(csp.packet_get_data(pkt))
        csp.buffer_free(pkt)

        # Only GCS-originated packets (CMD or HB) prove the link is alive.
        if dport == CSP_PORT_HB:
            self._last_recv = time.monotonic()
            return None

        if dport != CSP_PORT_CMD:
            return None

        self._last_recv = time.monotonic()
        cmd = data.decode("utf-8", errors="replace").strip()
        if log:
            log.info("COMM", f"CSP CMD (node {CSP_ADDR_GCS}→{CSP_ADDR_HLC}): {cmd!r}")
        return cmd

    def on_tlm(self, raw_tlm: str) -> None:
        """Envía frame TLM al GCS como paquete CSP (downlink SRS-020)."""
        self._send_payload(self._gcs_dst, CSP_PORT_TM, raw_tlm.encode())

    @property
    def last_recv_time(self) -> float:
        return self._last_recv

    def send_probe(self) -> None:
        """Envía HB_REQ al GCS (CommLinkMonitor, SRS-013)."""
        self._probe_seq += 1
        self._send_payload(
            self._gcs_dst, CSP_PORT_HB,
            f"HB_REQ:{self._probe_seq}".encode(),
        )

    def make_link_monitor(self) -> CommLinkMonitor:
        return CommLinkMonitor()

    def close(self) -> None:
        # csp.close() es para connections, no sockets. El socket se libera
        # automáticamente por el destructor del capsule cuando el GC lo recolecta.
        self._sock = None

    # ── Conmutación de enlace ─────────────────────────────────────────────────

    def _check_link_switch(self) -> None:
        elapsed = time.monotonic() - self._last_recv
        if not self._uhf_active and elapsed > GCS_LINK_LOST_S and self._uhf_available:
            self._activate_uhf()
        elif self._uhf_active and elapsed <= GCS_LINK_LOST_S:
            self._deactivate_uhf()

    def _activate_uhf(self) -> None:
        self._csp.rtable_load(f"0/0 UDP\n{CSP_ADDR_GCS_UHF}/8 KISS")
        self._uhf_active = True
        print("[LibcspGCSSource] link_lost → UHF/KISS activo (RDP+CRC32)")

    def _deactivate_uhf(self) -> None:
        self._csp.rtable_load("0/0 UDP")
        self._uhf_active = False
        print("[LibcspGCSSource] WiFi restaurado → volviendo a UDP")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send_payload(self, dst: int, dport: int, payload: bytes) -> None:
        csp = self._csp
        pkt = csp.buffer_get(len(payload))
        if pkt is None:
            return
        csp.packet_set_data(pkt, payload)
        flags = (csp.CSP_O_RDP | csp.CSP_O_CRC32) if self._uhf_active else csp.CSP_O_CRC32
        try:
            csp.sendto(_CSP_PRIO_NORM, dst, dport, 0, flags, pkt)
        except Exception:
            csp.buffer_free(pkt)
