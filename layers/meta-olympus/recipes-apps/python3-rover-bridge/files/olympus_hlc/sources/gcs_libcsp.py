# olympus_hlc/sources/gcs_libcsp.py — LibcspGCSSource: real libcsp 4.2 + ZMQ
#
# Usa libcsp_py3 (Python bindings nativos del paquete libcsp compilado en Yocto).
# Transporte: csp_zmqproxy corriendo en localhost del RPi5 (systemd).
# GCS conecta al zmqproxy del RPi5 via WiFi usando libcsp_py3 + zmqhub_init.
#
# Arquitectura de red CSP (ICD-CSP-001):
#
#   ┌──────────────────────────────────────────────────────┐
#   │  RPi5                                                │
#   │  ┌─────────────────┐     ┌──────────────────────┐   │
#   │  │ LibcspGCSSource │────▶│  csp_zmqproxy        │   │
#   │  │ (CSP node=2)    │◀────│  XSUB :6000          │   │
#   │  └─────────────────┘     │  XPUB :7000          │◀──┼── GCS WiFi
#   └──────────────────────────┴──────────────────────┘   │
#
#   HLC: zmqhub_init(CSP_ADDR_HLC=2, "localhost")
#   GCS: zmqhub_init(CSP_ADDR_GCS=1, "<rpi5_ip>")
#
# Activar:  python3 -m olympus_hlc --mode gcs --use-libcsp
# Requiere: imagen Yocto con libcsp (CSP_HAVE_LIBZMQ=ON) + csp-zmqproxy.service

import time

from ..interfaces import CommandSource
from ..monitors import CommLinkMonitor
from ..config import (
    CSP_ADDR_HLC, CSP_ADDR_GCS,
    CSP_PORT_CMD, CSP_PORT_TM, CSP_PORT_HB,
    GCS_LINK_LOST_S, ZMQ_PROXY_HOST,
)

# Constantes CSP — espejean los valores de libcsp_py3 para evitar importarlos
# a nivel de módulo (el import puede fallar en el host de desarrollo).
_CSP_PRIO_NORM = 2
_CSP_O_NONE    = 0


def _import_libcsp():
    """
    Importa libcsp_py3. Falla con mensaje claro si no está disponible.

    El módulo se llama 'libcsp_py3' en el binding C (PyInit_libcsp_py3).
    La imagen Yocto lo instala en site-packages via la receta libcsp_4.2.bb.
    """
    try:
        import libcsp_py3  # noqa: PLC0415
        return libcsp_py3
    except ImportError as exc:
        raise ImportError(
            "libcsp_py3 no encontrado. "
            "Verificar imagen Yocto: receta libcsp_4.2.bb con "
            "CSP_HAVE_LIBZMQ=ON y csp-zmqproxy.service activo."
        ) from exc


class LibcspGCSSource(CommandSource):
    """
    Fuente de comandos GCS usando libcsp 4.2 nativo + ZMQ como transporte WiFi.

    Inicialización (una sola vez al arrancar el HLC):
      1. csp.init()             — configura el nodo CSP local (addr=HLC)
      2. csp.zmqhub_init()      — conecta al zmqproxy local (WiFi hub)
      3. csp.rtable_load()      — ruta todos los paquetes por ZMQHUB
      4. csp.route_start_task() — lanza pthread interno de routing
      5. csp.socket/bind/listen — servidor CSP en CSP_PORT_CMD

    Para añadir UHF en el futuro: csp.kiss_init() + entrada adicional en rtable.
    Sin cambios en este código.
    """

    def __init__(self, zmq_host: str = ZMQ_PROXY_HOST):
        self._csp = _import_libcsp()
        csp = self._csp

        # 1. Nodo CSP local
        csp.init("olympus-hlc", "RPi5", "2.16")

        # 2. Interfaz ZMQ — conecta al zmqproxy en localhost (csp_zmqproxy.service)
        #    addr  = dirección CSP local (para filtros ZMQ SUB por destino)
        #    host  = IP del zmqproxy (localhost en RPi5, <rpi5_ip> en GCS)
        csp.zmqhub_init(CSP_ADDR_HLC, zmq_host)

        # 3. Ruta por defecto: todos los destinos CSP via ZMQHUB
        csp.rtable_load("0/0 ZMQHUB")

        # 4. Lanzar tarea de routing (pthread detached — corre hasta el final)
        csp.route_start_task()

        # 5. Socket servidor: escucha en CSP_ANY para recibir en cualquier puerto
        #    y despachar por dport dentro de next_command().
        self._sock = csp.socket()
        csp.bind(self._sock, csp.CSP_ANY)
        csp.listen(self._sock, 5)

        self._last_recv = time.monotonic()
        self._probe_seq = 0

        print(f"[LibcspGCSSource] node={CSP_ADDR_HLC} zmqproxy={zmq_host}:6000/7000")

    # ── CommandSource interface ───────────────────────────────────────────────

    def next_command(self, log=None) -> "str | None":
        """
        Acepta conexiones CSP entrantes (non-blocking, timeout=0 ms).

        Despacha por puerto de destino:
          CSP_PORT_CMD → retorna payload como string de comando MSM.
          CSP_PORT_HB  → actualiza last_recv, retorna None.
          otros        → descarta, retorna None.
        """
        csp = self._csp
        conn = csp.accept(self._sock, timeout_ms=0)
        if conn is None:
            return None

        dport = csp.conn_dport(conn)
        pkt   = csp.read(conn, timeout_ms=100)
        csp.close(conn)

        if pkt is None:
            return None

        self._last_recv = time.monotonic()
        data   = bytes(csp.packet_get_data(pkt))
        csp.buffer_free(pkt)

        if dport == CSP_PORT_HB:
            return None  # heartbeat — solo actualiza last_recv

        if dport != CSP_PORT_CMD:
            return None  # servicio desconocido

        cmd = data.decode("utf-8", errors="replace").strip()
        if log:
            log.info("COMM", f"CSP CMD (ZMQ node {CSP_ADDR_GCS}→{CSP_ADDR_HLC}): {cmd!r}")
        return cmd

    def on_tlm(self, raw_tlm: str) -> None:
        """Envía frame TLM al GCS como paquete CSP via ZMQ (downlink SRS-020)."""
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
        try:
            self._csp.close(self._sock)
        except Exception:
            pass

    # ── Helpers internos ─────────────────────────────────────────────────────

    def _send_payload(self, dst: int, dport: int, payload: bytes) -> None:
        """
        Encapsula payload en un paquete CSP y lo envía vía libcsp sendto.

        Flujo:
          buffer_get()       — aloca paquete del pool CSP (256 B, compile-time)
          packet_set_data()  — copia payload al paquete
          sendto()           — entrega al router CSP → ZMQHUB → GCS

        Si el pool está agotado, la función retorna silenciosamente.
        El tamaño máximo del payload es CSP_BUFFER_SIZE − CSP_HEADER_SIZE ≈ 252 B.
        """
        csp = self._csp
        pkt = csp.buffer_get(len(payload))
        if pkt is None:
            return
        csp.packet_set_data(pkt, payload)
        try:
            csp.sendto(_CSP_PRIO_NORM, dst, dport, 0, _CSP_O_NONE, pkt)
        except Exception:
            csp.buffer_free(pkt)
