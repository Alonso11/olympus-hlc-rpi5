# olympus_hlc/sources/gcs_libcsp.py — GCSSource backed by native libcsp (optional)
#
# Implementa CommandSource usando el módulo Python de libcsp (bakeado por
# meta-olympus/recipes-connectivity/libcsp/libcsp_4.2.bb).
#
# Cuándo usar este backend en lugar de GCSSource (UDP raw + CSPPacket custom):
#   - Cuando el GCS también usa libcsp (nodo CSP real, no solo UDP+CSP header)
#   - Cuando se añade interfaz UHF: rtable_load() redirige transparentemente
#   - Cuando se necesita RDP (reliable datagram) para comandos críticos
#
# La máquina de estados (HlcEngine, RoverMSM) no cambia nada — solo la capa
# de transporte cambia aquí. El resto del sistema ve exactamente la misma
# interfaz CommandSource que GCSSource.
#
# Activación: en olympus_controller.yaml poner `use_libcsp_native: true`
# (o pasar --use-libcsp en el CLI cuando se implemente).
#
# Requisito: libcsp Python bindings disponibles en site-packages (csp / _csp).
# Si el import falla, lanza ImportError con instrucciones claras.

import time

from ..interfaces import CommandSource
from ..monitors import CommLinkMonitor
from ..config import (
    CSP_ADDR_HLC, CSP_ADDR_GCS,
    CSP_PORT_CMD, CSP_PORT_TM, CSP_PORT_HB,
    GCS_LISTEN_PORT,
)


def _import_libcsp():
    """Importa el módulo libcsp con mensaje claro si no está disponible."""
    try:
        import csp  # noqa: PLC0415
        return csp
    except ImportError as exc:
        raise ImportError(
            "libcsp Python bindings no encontrados. "
            "Verificar que la imagen Yocto incluye el paquete libcsp "
            "(recipes-connectivity/libcsp/libcsp_4.2.bb con "
            "CSP_ENABLE_PYTHON3_BINDINGS=ON). "
            "En desarrollo usar GCSSource en su lugar."
        ) from exc


class LibcspGCSSource(CommandSource):
    """
    Fuente de comandos GCS usando libcsp nativo (socket API real).

    Diferencias respecto a GCSSource (UDP raw):
      - Usa csp.socket() en lugar de socket.SOCK_DGRAM
      - libcsp gestiona el header CSP internamente (sin CSPPacket.unpack)
      - Soporta routing: añadir interfaz UHF → rtable_load() → transparente
      - Soporta RDP: cambiar CSP_SO_RDPOPT para entregas garantizadas
      - Conexión orientada: acepta conn por puerto, lee paquetes, cierra conn

    El TLM downlink usa csp.sendto() (connectionless) hacia CSP_ADDR_GCS:CSP_PORT_TM
    para mantener compatibilidad con GCS no-orientado a conexión.
    """

    def __init__(self,
                 gcs_ip:    str = "0.0.0.0",
                 gcs_port:  int = GCS_LISTEN_PORT,
                 iface:     str = "udp"):
        """
        Args:
            gcs_ip:   IP del nodo GCS para routing (o "0.0.0.0" = aprender dinámicamente).
            gcs_port: Puerto UDP donde escucha el GCS (para rtable_load).
            iface:    Interfaz CSP a registrar ("udp" es la única soportada ahora).
        """
        self._csp = _import_libcsp()

        # Inicializar nodo CSP
        self._csp.init(
            addr=CSP_ADDR_HLC,
            hostname="olympus-hlc",
            model="RPi5",
            revision="v1.0",
            version=2,
        )
        self._csp.buffer_init(count=20, size=256)

        # Registrar interfaz UDP
        if iface == "udp":
            # Formato: "peer_ip:peer_port" para IF_UDP
            peer = f"{gcs_ip}:{gcs_port}" if gcs_ip != "0.0.0.0" else f"0.0.0.0:{gcs_port}"
            self._csp.add_interface(self._csp.IF_UDP, peer)
            if gcs_ip != "0.0.0.0":
                self._csp.rtable_load(f"{CSP_ADDR_GCS}/0 UDP")
        else:
            raise ValueError(f"Interfaz CSP no soportada: {iface!r}. Usar 'udp'.")

        # Socket de servidor — escucha en CSP_PORT_CMD
        self._sock_cmd = self._csp.socket()
        self._csp.bind(self._sock_cmd, CSP_PORT_CMD)
        self._csp.listen(self._sock_cmd, 5)

        # Estado
        self._last_recv   = time.monotonic()
        self._gcs_addr    = None   # Se aprende del primer paquete recibido
        self._probe_seq   = 0

        print(f"[LibcspGCSSource] CSP node={CSP_ADDR_HLC} "
              f"iface={iface} port_cmd={CSP_PORT_CMD} port_tm={CSP_PORT_TM}")

    # ── CommandSource interface ───────────────────────────────────────────────

    def next_command(self, log=None) -> "str | None":
        """
        Acepta una conexión entrante en CSP_PORT_CMD (no-bloqueante, timeout=0).
        Lee el payload, actualiza last_recv y retorna la cadena de comando MSM.
        """
        # accept con timeout 0 ms → no-bloqueante
        conn = self._csp.accept(self._sock_cmd, timeout_ms=0)
        if conn is None:
            return None

        pkt = self._csp.read(conn, timeout_ms=100)
        if pkt is None:
            self._csp.close(conn)
            return None

        # Aprender dirección del GCS del primer paquete
        src_addr = self._csp.conn_src(conn)
        if self._gcs_addr != src_addr:
            self._gcs_addr = src_addr
            if log:
                log.info("COMM", f"GCS aprendido: CSP addr={src_addr}")
            # Añadir ruta si no estaba en rtable
            try:
                self._csp.rtable_load(f"{src_addr}/0 UDP")
            except Exception:
                pass  # Ya existe o no es crítico

        self._last_recv = time.monotonic()
        cmd = bytes(pkt.data[:pkt.length]).decode("utf-8", errors="replace").strip()
        self._csp.buffer_free(pkt)
        self._csp.close(conn)

        if log:
            log.info("COMM", f"CSP CMD recibido: {cmd!r} de addr={self._gcs_addr}")
        return cmd if cmd else None

    def on_tlm(self, raw_tlm: str) -> None:
        """Envía TLM al GCS encapsulado en CSP (downlink, connectionless)."""
        if self._gcs_addr is None:
            return
        self._forward_tlm(raw_tlm)

    @property
    def last_recv_time(self) -> float:
        return self._last_recv

    def send_probe(self) -> None:
        """Envía HB_REQ al GCS vía CSP (CommLinkMonitor, SRS-013)."""
        if self._gcs_addr is None:
            return
        self._probe_seq += 1
        payload = f"HB_REQ:{self._probe_seq}".encode()
        try:
            self._csp.sendto(
                self._gcs_addr, CSP_PORT_HB, 0,  # dst, dport, sport
                CSP_ADDR_HLC,                     # src
                payload,
                timeout_ms=200,
            )
        except Exception:
            pass

    def make_link_monitor(self) -> CommLinkMonitor:
        return CommLinkMonitor()

    def close(self) -> None:
        try:
            self._csp.close(self._sock_cmd)
        except Exception:
            pass

    # ── Métodos internos ──────────────────────────────────────────────────────

    def _forward_tlm(self, raw_tlm: str) -> None:
        """Envía TLM como paquete CSP connectionless hacia CSP_ADDR_GCS:CSP_PORT_TM."""
        try:
            self._csp.sendto(
                self._gcs_addr, CSP_PORT_TM, 0,
                CSP_ADDR_HLC,
                raw_tlm.encode(),
                timeout_ms=200,
            )
        except Exception:
            pass
