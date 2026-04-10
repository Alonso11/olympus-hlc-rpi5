# olympus_hlc/sources/gcs_libcsp.py — Multi-link CSP Source (WiFi + UHF/KISS)
#
# v3.1: Soporte para conmutación automática de rutas WiFi <-> UHF (SRS-001/RF-006).

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
    Fuente de comandos GCS usando libcsp nativo con soporte Multi-Link.
    
    Gestiona dinámicamente el rtable para priorizar WiFi (UDP) y 
    conmutar a UHF (KISS) en caso de pérdida de enlace.
    """

    def __init__(self,
                 gcs_ip:    str = "0.0.0.0",
                 gcs_port:  int = GCS_LISTEN_PORT,
                 tnc_dev:   str = "/dev/ttyTNC"):
        
        self._csp = _import_libcsp()

        # 1. Inicializar nodo CSP
        self._csp.init(
            addr=CSP_ADDR_HLC,
            hostname="olympus-hlc",
            model="RPi5",
            version=2,
        )
        self._csp.buffer_init(count=20, size=256)

        # 2. Registrar interfaz UDP (WiFi)
        peer = f"{gcs_ip}:{gcs_port}" if gcs_ip != "0.0.0.0" else f"0.0.0.0:{gcs_port}"
        self._csp.add_interface(self._csp.IF_UDP, peer)
        print(f"[Libcsp] Interfaz UDP (WiFi) registrada hacia {peer}")

        # 3. Registrar interfaz KISS (UHF)
        self._has_uhf = False
        try:
            # kiss_init(device, baudrate, name)
            self._csp.kiss_init(tnc_dev, 115200, "KISS")
            self._has_uhf = True
            print(f"[Libcsp] Interfaz KISS (UHF) registrada en {tnc_dev}")
        except Exception as e:
            print(f"[Libcsp] Advertencia: No se detectó TNC en {tnc_dev} ({e})")

        # 4. Configuración de rtable inicial (Prioridad WiFi)
        self._current_link = "UDP"
        self._csp.rtable_load(f"{CSP_ADDR_GCS}/0 UDP")

        # 5. Socket de servidor
        self._sock_cmd = self._csp.socket()
        self._csp.bind(self._sock_cmd, CSP_PORT_CMD)
        self._csp.listen(self._sock_cmd, 5)

        # Estado
        self._last_recv   = time.monotonic()
        self._gcs_addr    = CSP_ADDR_GCS
        self._monitor     = CommLinkMonitor(timeout_s=GCS_LINK_LOST_S)

    def next_command(self, log=None) -> "str | None":
        """Revisa comandos y gestiona la conmutación de rutas."""
        
        # Lógica de conmutación automática
        self._manage_routing(log)

        # Aceptar conexión (el kernel de CSP usa la ruta activa en rtable)
        conn = self._csp.accept(self._sock_cmd, timeout_ms=0)
        if conn is None:
            return None

        pkt = self._csp.read(conn, timeout_ms=100)
        if pkt is None:
            self._csp.close(conn)
            return None

        # Actualizar latido y monitor
        self._last_recv = time.monotonic()
        self._monitor.update()

        cmd = bytes(pkt.data[:pkt.length]).decode("utf-8", errors="replace").strip()
        self._csp.buffer_free(pkt)
        self._csp.close(conn)

        if log:
            log.info("COMM", f"CSP CMD ({self._current_link}): {cmd!r}")
        return cmd

    def _manage_routing(self, log):
        """Cambia entre WiFi y UHF según la salud del enlace."""
        link_lost = (time.monotonic() - self._last_recv) > GCS_LINK_LOST_S

        if self._current_link == "UDP" and link_lost and self._has_uhf:
            # Conmutar a UHF
            self._current_link = "KISS"
            self._csp.rtable_load(f"{CSP_ADDR_GCS}/0 KISS")
            if log:
                log.warning("COMM", "Enlace WiFi perdido. Conmutando a UHF (KISS)...")
        
        elif self._current_link == "KISS" and not link_lost:
            # Volver a WiFi si detectamos actividad (esto requiere que el GCS 
            # también intente reconectar por WiFi periódicamente)
            self._current_link = "UDP"
            self._csp.rtable_load(f"{CSP_ADDR_GCS}/0 UDP")
            if log:
                log.info("COMM", "Enlace WiFi recuperado. Conmutando a UDP...")

    def on_tlm(self, raw_tlm: str) -> None:
        """Envía TLM usando la ruta activa en libcsp."""
        try:
            self._csp.sendto(
                self._gcs_addr, CSP_PORT_TM, 0,
                CSP_ADDR_HLC,
                raw_tlm.encode(),
                timeout_ms=200,
            )
        except Exception:
            pass

    @property
    def last_recv_time(self) -> float:
        return self._last_recv

    def send_probe(self) -> None:
        """HB_REQ al GCS para mantener el enlace vivo."""
        try:
            self._csp.sendto(
                self._gcs_addr, CSP_PORT_HB, 0,
                CSP_ADDR_HLC,
                b"HB_REQ",
                timeout_ms=100,
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
