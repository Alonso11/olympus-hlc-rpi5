# olympus_hlc/interfaces.py — CommandSource abstract base class (DIP)
#
# Todas las fuentes de comandos implementan esta interfaz.
# HlcEngine depende únicamente de CommandSource — nunca de tipos concretos.
#
# Métodos con implementación por defecto (comportamiento neutro):
#   on_tlm()        — solo GCSSource reenvía TLM al GCS; otros ignoran
#   on_sys()        — solo StationSource reenvía métricas SYS: del RPi5 a la GUI
#   last_recv_time  — ManualSource/VisionSource siempre "conectados"
#   send_probe()    — solo GCSSource envía HB_REQ al GCS; otros no hacen nada
#   make_link_monitor() — solo GCSSource retorna un CommLinkMonitor
#   close()         — GCSSource cierra el socket; VisionSource libera cámara

import abc
import time


class CommandSource(abc.ABC):

    @abc.abstractmethod
    def next_command(self, log=None) -> "str | None":
        """
        Retorna el siguiente comando MSM o None si no hay nada en este ciclo.
        Debe ser no-bloqueante excepto en ManualSource (stdin interactivo).
        """

    def on_tlm(self, raw_tlm: str) -> None:  # noqa: ARG002
        """
        Llamado cuando el HLC recibe un frame TLM del Arduino.
        GCSSource lo usa para reenviar el TLM al GCS (downlink SRS-020).
        El resto de fuentes no hace nada.
        """
        pass

    def on_sys(self, sample: "object") -> None:  # noqa: ARG002
        """
        Llamado cuando SystemMonitor produce una SystemSample fresca (CPU/RAM/
        temp del RPi5). StationSource la reenvía como frame SYS: a la GUI
        (downlink de diagnóstico). El resto de fuentes no hace nada.
        sample: olympus_hlc.sysmon.SystemSample — ver to_frame().
        """
        pass

    @property
    def last_recv_time(self) -> float:
        """
        Monotonic timestamp del último paquete válido recibido.
        GCSSource retorna el timestamp real del último UDP recibido.
        ManualSource y VisionSource retornan time.monotonic() —
        se considera que siempre están "conectadas" (no hay enlace que monitorear).
        """
        return time.monotonic()

    def send_probe(self) -> None:
        """
        Envía un probe de reconexión al peer remoto.
        GCSSource envía HB_REQ al GCS. El resto no hace nada.
        Llamado por CommLinkMonitor durante la política de reintentos.
        """

    def make_link_monitor(self) -> "object | None":
        """
        Retorna una instancia de CommLinkMonitor si esta fuente soporta
        monitoreo de enlace, o None en caso contrario.
        Solo GCSSource retorna un CommLinkMonitor (SRS-013).
        """
        return None

    @property
    def safety_level(self) -> str:
        """
        Nivel de seguridad seleccionado por el operador (gobierna qué overrides
        autónomos del HLC aplican). El engine lo lee cada ciclo.
          "FULL"   — todos los overrides (retreat/slip/SafeMode/link-loss). Default.
          "ASSIST" — sin RET proactivos (retreat/slip); mantiene SafeMode y
                     link-loss→STB + la MSM del firmware.
          "MANUAL" — sin overrides del HLC (control total del operador).
        Solo StationSource lo expone configurable; el resto queda en FULL.
        """
        return "FULL"

    def on_dispatch(self, cmd: "str | None", reason: "str | None" = None) -> None:  # noqa: ARG002
        """
        Llamado por el engine con el comando FINAL despachado al Mega (ya con
        overrides aplicados) y la razón si fue un override. StationSource lo usa
        para reflejar el comando real + el evento en la GUI (evidencia). El resto
        de fuentes no hace nada.
        """
        pass

    def close(self) -> None:
        """
        Libera recursos asociados a esta fuente (sockets, procesos, handles).
        Se llama una sola vez al finalizar el programa.
        """
