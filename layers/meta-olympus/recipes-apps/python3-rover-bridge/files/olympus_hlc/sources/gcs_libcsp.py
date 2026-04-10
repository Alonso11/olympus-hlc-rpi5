# olympus_hlc/sources/gcs_libcsp.py — Multi-link CSP Source (WiFi + UHF/KISS)
#
# v3.2: Soporte Multi-Link con RDP (Reliable Datagram Protocol).
#
# ── ATRIBUCIÓN Y REFERENCIAS ──────────────────────────────────────────────────
# Basado en la arquitectura de comunicaciones propuesta en el Trabajo de 
# Graduación (TEC):
#   "ELANav Comms: Propuesta de diseño de un enlace de comunicación 
#    para robots de exploración lunar" (2025).
#   Repositorio de referencia: https://github.com/Tobiasfonseca/libcsp-ELANav
#
# Contribuciones adaptadas:
#   - Uso de RDP para garantizar la entrega en enlaces UHF ruidosos (SRS-001).
#   - Estrategia de direccionamiento estático para nodos Rover y GCS.
#   - Implementación del driver KISS para interfaces TNC USB.
# ──────────────────────────────────────────────────────────────────────────────

import time
import logging

from ..interfaces import CommandSource
from ..monitors import CommLinkMonitor
from ..config import (
    CSP_ADDR_HLC, CSP_ADDR_GCS,
    CSP_PORT_CMD, CSP_PORT_TM, CSP_PORT_HB,
    GCS_LISTEN_PORT, GCS_LINK_LOST_S
)

def _import_libcsp():
    """Importa el módulo libcsp con mensaje claro si no está disponible."""
    try:
        import csp  # noqa: PLC0415
        return csp
    except ImportError as exc:
        raise ImportError(
            "libcsp Python bindings no encontrados. "
            "Verificar que la imagen Yocto incluye el paquete libcsp."
        ) from exc


class LibcspGCSSource(CommandSource):
    """
    Fuente de comandos GCS con soporte Multi-Link y RDP (Reliable Datagram).
    
    Gestiona dinámicamente el rtable para priorizar WiFi (UDP) y 
    conmutar a UHF (KISS) en caso de pérdida de enlace, utilizando 
    protocolos confiables para comandos críticos (Arquitectura ELANav Comms).
    """

    def __init__(self,
                 gcs_ip:    str = "0.0.0.0",
                 gcs_port:  int = GCS_LISTEN_PORT,
                 tnc_dev:   str = "/dev/ttyTNC"):
        
        self._csp = _import_libcsp()

        # 1. Inicializar nodo CSP (v2 compatible con libcsp 4.x)
        self._csp.init(
            addr=CSP_ADDR_HLC,
            hostname="olympus-hlc",
            model="RPi5",
            version=2,
        )
        self._csp.buffer_init(count=30, size=256)

        # 2. Configuración RDP (Reliable Datagram Protocol) - Propuesta ELANav
        # window=3, timeout=500ms, conn_timeout=2000ms
        # Asegura que los comandos lleguen aunque haya interferencia en UHF.
        self._rdp_options = {
            'window': 3,
            'conn_timeout': 2000,
            'packet_timeout': 500,
            'ack_timeout': 200,
            'ack_count': 1
        }

        # 3. Registrar interfaces
        # UDP (WiFi)
        peer = f"{gcs_ip}:{gcs_port}" if gcs_ip != "0.0.0.0" else f"0.0.0.0:{gcs_port}"
        self._csp.add_interface(self._csp.IF_UDP, peer)
        
        # KISS (UHF)
        self._has_uhf = False
        try:
            self._csp.kiss_init(tnc_dev, 115200, "KISS")
            self._has_uhf = True
            print(f"[Libcsp] Interfaz KISS detectada en {tnc_dev}")
        except Exception as e:
            print(f"[Libcsp] Advertencia: UHF no disponible ({e})")

        # 4. Routing Inicial (WiFi por defecto)
        self._current_link = "UDP"
        self._csp.rtable_load(f"{CSP_ADDR_GCS}/0 UDP")

        # 5. Socket de Servidor con RDP habilitado
        self._sock_cmd = self._csp.socket(self._csp.SO_RDP) # Forzar RDP para comandos
        self._csp.bind(self._sock_cmd, CSP_PORT_CMD)
        self._csp.listen(self._sock_cmd, 5)

        # Estado
        self._last_recv   = time.monotonic()
        self._gcs_addr    = CSP_ADDR_GCS
        self._monitor     = CommLinkMonitor(timeout_s=GCS_LINK_LOST_S)

    def next_command(self, log=None) -> "str | None":
        """Revisa comandos y gestiona la conmutación de rutas."""
        self._manage_routing(log)

        # Aceptar conexión (libcsp gestiona el handshake RDP automáticamente)
        conn = self._csp.accept(self._sock_cmd, timeout_ms=0)
        if conn is None:
            return None

        pkt = self._csp.read(conn, timeout_ms=100)
        if pkt is None:
            self._csp.close(conn)
            return None

        self._last_recv = time.monotonic()
        self._monitor.update()

        cmd = bytes(pkt.data[:pkt.length]).decode("utf-8", errors="replace").strip()
        self._csp.buffer_free(pkt)
        self._csp.close(conn)

        if log:
            log.info("COMM", f"CSP CMD ({self._current_link} + RDP): {cmd!r}")
        return cmd

    def _manage_routing(self, log):
        """Cambia entre WiFi y UHF según la salud del enlace (Arquitectura ELANav Comms)."""
        link_lost = (time.monotonic() - self._last_recv) > GCS_LINK_LOST_S

        if self._current_link == "UDP" and link_lost and self._has_uhf:
            self._current_link = "KISS"
            self._csp.rtable_load(f"{CSP_ADDR_GCS}/0 KISS")
            if log:
                log.warning("COMM", "⚠️ Enlace WiFi perdido. Conmutando a UHF (KISS)...")
        
        elif self._current_link == "KISS" and not link_lost:
            self._current_link = "UDP"
            self._csp.rtable_load(f"{CSP_ADDR_GCS}/0 UDP")
            if log:
                log.info("COMM", "✅ Enlace WiFi recuperado. Volviendo a UDP.")

    def on_tlm(self, raw_tlm: str) -> None:
        """Envía telemetría (Best-effort para datos masivos)."""
        try:
            self._csp.sendto(
                self._gcs_addr, CSP_PORT_TM, 0,
                CSP_ADDR_HLC,
                raw_tlm.encode(),
                timeout_ms=100,
            )
        except Exception:
            pass

    def send_critical_alert(self, alert_msg: str) -> None:
        """
        Envía alertas críticas usando RDP (Garantía de entrega).
        Estrategia basada en el escenario de emergencia de ELANav Comms.
        """
        try:
            # Enviar con el flag RDP activo (protocolo confiable)
            self._csp.sendto(
                self._gcs_addr, CSP_PORT_TM, self._csp.SO_RDP,
                CSP_ADDR_HLC,
                f"CRIT:{alert_msg}".encode(),
                timeout_ms=500,
            )
        except Exception:
            pass

    @property
    def last_recv_time(self) -> float:
        return self._last_recv

    def send_probe(self) -> None:
        """Latido del enlace."""
        try:
            self._csp.sendto(
                self._gcs_addr, CSP_PORT_HB, 0,
                CSP_ADDR_HLC,
                b"HB",
                timeout_ms=50,
            )
        except Exception:
            pass

    def make_link_monitor(self) -> CommLinkMonitor:
        return self._monitor

    def close(self) -> None:
        try:
            self._csp.close(self._sock_cmd)
        except Exception:
            pass
