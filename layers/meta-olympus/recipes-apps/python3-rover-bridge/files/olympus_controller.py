#!/usr/bin/env python3
# Version: v2.4
# Olympus HLC — Main Controller
#
# Integrates the CSI camera (or manual operator input) with the Arduino MSM
# via rover_bridge. Two modes selectable at startup:
#
#   --mode vision   Camera + cv2.dnn (YOLOv8n ONNX) → MSM commands
#   --mode manual   Operator stdin → MSM commands
#   --mode gcs      UDP commands from Ground Control Station (SRS-013)
#
# In all modes the pipeline to the Arduino is identical:
#   source.next_command() → rover_bridge.send_command() → log response
#
# The Arduino watchdog fires ERR:WDOG → FAULT if no command arrives in ~2s.
# This loop sends PING every 1s when idle to keep the watchdog alive.
#
# GCS mode uses CSP (CubeSat Space Protocol) encapsulation over UDP/IP with
# CRC-32 integrity verification (SRS-001, RF-006, SyRS-016). See CSPPacket.
#
# Usage:
#   olympus_controller.py --mode manual
#   olympus_controller.py --mode vision --model /usr/share/olympus/models/yolov8n.onnx
#   olympus_controller.py --mode gcs

import argparse
import dataclasses
import datetime
import enum
import logging
import logging.handlers
import os
import socket
import struct
import sys
import time
import zlib
import rover_bridge

# ─── Configuration loader ─────────────────────────────────────────────────────

def _load_config() -> dict:
    """Carga la configuración desde YAML si está disponible.

    Busca en orden:
      1. /etc/olympus/olympus_controller.yaml  (producción — instalado por Yocto)
      2. configs/olympus_controller.yaml       (desarrollo — junto al script)

    Si ningún archivo existe o PyYAML no está instalado, retorna {} y todos
    los parámetros usan sus valores por defecto definidos en cfg.get().
    """
    from pathlib import Path
    candidates = [
        Path("/etc/olympus/olympus_controller.yaml"),
        Path(__file__).parent / "configs" / "olympus_controller.yaml",
    ]
    try:
        import yaml
        for path in candidates:
            if path.exists():
                with open(path) as f:
                    return yaml.safe_load(f) or {}
    except ImportError:
        pass  # PyYAML no disponible — usar defaults del código
    return {}

_cfg = _load_config()

# ─── Constants ───────────────────────────────────────────────────────────────

PING_INTERVAL_S   = float(_cfg.get("ping_interval_s",   1.0))   # Max s entre comandos antes de PING
TLM_WARN_S        = float(_cfg.get("tlm_warn_s",        5.0))   # Sin TLM → advertencia de enlace degradado
TLM_RETREAT_S     = float(_cfg.get("tlm_retreat_s",     10.0))  # Sin TLM → RET al último waypoint seguro (SYS-FUN-021)
TLM_STB_S         = float(_cfg.get("tlm_stb_s",         30.0))  # Sin TLM tras RET → STB definitivo (COMM-REQ-005)
CYCLE_WARN_MS     = int  (_cfg.get("cycle_warn_ms",     1500))  # Umbral de ciclo lento (RNF-001: ≤ 2000 ms)
CYCLE_LOG_PERIOD  = int  (_cfg.get("cycle_log_period",  50))    # Cada cuántos ciclos loguear tiempo a DEBUG
RETREAT_DIST_MM   = int  (_cfg.get("retreat_dist_mm",   300))   # Distancia táctica HLC para RET (> 200 mm LLC)
MAX_WAYPOINTS     = int  (_cfg.get("max_waypoints",     5))     # Últimos N waypoints seguros (SyRS-061)
BATT_WARN_MV      = int  (_cfg.get("batt_warn_mv",      14000)) # 3.5 V/celda × 4S → advertir (EPS-REQ-001)
BATT_CRITICAL_MV  = int  (_cfg.get("batt_critical_mv",  12800)) # 3.2 V/celda × 4S → STB inmediato
FRAME_WIDTH       = int  (_cfg.get("frame_width",       640))
FRAME_HEIGHT      = int  (_cfg.get("frame_height",      480))
VISION_CONF_MIN   = float(_cfg.get("vision_conf_min",   0.5))   # Confianza mínima para actuar
VISION_AREA_MIN   = float(_cfg.get("vision_area_min",   0.05))  # Área mínima bbox como fracción del frame

# Frame zones for avoidance decision (fractions of FRAME_WIDTH)
ZONE_LEFT_END     = float(_cfg.get("zone_left_end",     0.33))  # 0–33%  → obstacle left  → AVD:R
ZONE_RIGHT_START  = float(_cfg.get("zone_right_start",  0.67))  # 67–100% → obstacle right → AVD:L
# Center zone (33–67%) → RET

SLIP_STALL_FRAMES    = int  (_cfg.get("slip_stall_frames",    2))    # Frames TLM consecutivos con stall → RET (RF-004)

TEMP_WARN_C          = int  (_cfg.get("temp_warn_c",          45))   # °C ambiente → advertencia (RNF-004)
TEMP_CRIT_C          = int  (_cfg.get("temp_crit_c",          60))   # °C ambiente → Safe Mode (RNF-004)

STORAGE_MIN_MB       = int  (_cfg.get("storage_min_mb",       50))   # MB libres mínimos en /var/log (SRS-014)
STORAGE_CHECK_CYCLES = int  (_cfg.get("storage_check_cycles", 300))  # Cada cuántos ciclos verificar disco

TLM_INTERVAL_WARN_S  = float(_cfg.get("tlm_interval_warn_s",  2.0))  # Delta entre TLMs > este valor → warn (SyRS-017)

# GCS link management (SRS-013, SYS-FUN-021)
GCS_LISTEN_PORT      = int  (_cfg.get("gcs_listen_port",       9000))  # Puerto UDP escucha HLC (comandos GCS)
GCS_REPLY_PORT       = int  (_cfg.get("gcs_reply_port",        9001))  # Puerto UDP destino GCS (TLM downlink)
GCS_BIND_ADDR        = str  (_cfg.get("gcs_bind_addr",         "0.0.0.0"))
GCS_LINK_LOST_S      = float(_cfg.get("gcs_link_lost_s",       10.0))  # Sin paquete GCS → link_lost (SYS-FUN-021)
GCS_RETRY_INTERVAL_S = float(_cfg.get("gcs_retry_interval_s",  5.0))   # Intervalo entre probes HB_REQ
GCS_MAX_RETRIES      = int  (_cfg.get("gcs_max_retries",        3))     # TBD-OP-02 (política por defecto)

# CSP encapsulation (SRS-001, RF-006, SyRS-016)
# Direcciones y puertos de servicio CSP v1.x para el enlace Rover–GCS.
# Rango de direcciones: 0–31 (5 bits). Rango de puertos: 0–63 (6 bits).
CSP_ADDR_GCS  = int (_cfg.get("csp_addr_gcs",   1))  # Nodo GCS en la red CSP
CSP_ADDR_HLC  = int (_cfg.get("csp_addr_hlc",   2))  # Nodo HLC (RPi5)
CSP_PORT_TM   = int (_cfg.get("csp_port_tm",   10))  # Puerto TM downlink (HLC→GCS)
CSP_PORT_CMD  = int (_cfg.get("csp_port_cmd",  11))  # Puerto CMD uplink  (GCS→HLC)
CSP_PORT_HB   = int (_cfg.get("csp_port_hb",    1))  # Puerto heartbeat y gestión
CSP_ENABLED   = bool(_cfg.get("csp_enabled",  True)) # False → modo ASCII legado (solo pruebas)

EXP_SPEED_L       = int  (_cfg.get("exp_speed_l",        40))   # Velocidad izquierda en exploración (-99–99)
EXP_SPEED_R       = int  (_cfg.get("exp_speed_r",        40))   # Velocidad derecha en exploración (-99–99)

# Segmentation pipeline (GNC-REQ-002) — only active when vision_mode = "segmentation"
VISION_MODE       = str  (_cfg.get("vision_mode",        "bbox"))
SEG_MODEL_PATH    = str  (_cfg.get("seg_model_path",
                         "/usr/share/olympus/models/yolov8n-seg.onnx"))
SEG_CONF_MIN      = float(_cfg.get("seg_conf_min",       0.5))  # Min detection confidence
SEG_AREA_MIN      = float(_cfg.get("seg_area_min",       0.03)) # Min mask area as frame fraction
SEG_ZONE_MIN      = float(_cfg.get("seg_zone_min",       0.05)) # Min zone coverage to trigger command
SEG_ROI_TOP       = float(_cfg.get("seg_roi_top",        0.5))  # Ignore frame above this fraction (0=full,0.5=bottom half)

# ─── Telemetry Frame ─────────────────────────────────────────────────────────

@dataclasses.dataclass
class TlmFrame:
    """
    Frame de telemetría extendida emitido por el Arduino (~1 s).
    Formato: TLM:<SAF>:<STALL>:<TS>ms:<MV>mV:<MA>mA:<I0>:<I1>:<I2>:<I3>:<I4>:<I5>:<T>C:<B0>:<B1>:<B2>:<B3>:<B4>:<B5>C:<DIST>mm
    (Ref. ICD LLC §Frame de telemetría extendida, SyRS-030)
    """
    safety:     str        # "NORMAL" | "WARN" | "LIMIT" | "FAULT"
    stall_mask: int        # 6 bits: bit5=FR … bit0=RL
    tick_ms:    int        # ms desde boot del Arduino (contador monotónico)
    batt_mv:    int        # tensión batería en mV  (0 = sin lectura)
    batt_ma:    int        # corriente batería en mA con signo (0 = sin lectura)
    currents:   list       # [FR, FL, CR, CL, RR, RL] mA
    temp_c:     int        # temperatura ambiente °C
    batt_temps: list       # [B1a, B1b, B2a, B2b, B3a, B3b] °C
    dist_mm:    int        # distancia ToF en mm (0 = sin lectura)

    @staticmethod
    def parse(raw: str):
        """
        Parsea un frame TLM crudo (sin el \\n final).
        Retorna TlmFrame o None si el formato no es válido.

        Ejemplo:
          TLM:NORMAL:000000:12340ms:11800mV:2350mA:200:210:195:205:180:190:24C:25:25:26:26:25:25C:450mm
        """
        # Índices de los campos tras split(':'):
        # 0=TLM 1=SAF 2=STALL 3=TSms 4=MVmV 5=MAma
        # 6..11=I0-I5  12=TC  13..17=B0-B4  18=B5C  19=DISTmm
        try:
            parts = raw.split(":")
            if len(parts) != 20 or parts[0] != "TLM":
                return None

            safety     = parts[1]
            stall_mask = int(parts[2], 2)          # "000101" → int
            tick_ms    = int(parts[3].rstrip("ms"))
            batt_mv    = int(parts[4].rstrip("mV"))
            batt_ma    = int(parts[5].rstrip("mA"))
            currents   = [int(parts[i]) for i in range(6, 12)]
            temp_c     = int(parts[12].rstrip("C"))
            batt_temps = [int(parts[i]) for i in range(13, 18)] + \
                         [int(parts[18].rstrip("C"))]
            dist_mm    = int(parts[19].rstrip("mm"))

            return TlmFrame(
                safety=safety, stall_mask=stall_mask, tick_ms=tick_ms,
                batt_mv=batt_mv, batt_ma=batt_ma, currents=currents,
                temp_c=temp_c, batt_temps=batt_temps, dist_mm=dist_mm,
            )
        except (ValueError, IndexError):
            return None


# ─── Waypoint Tracker ────────────────────────────────────────────────────────

@dataclasses.dataclass
class Waypoint:
    """Instantánea de un punto seguro registrado durante exploración (SyRS-061)."""
    tick_ms: int          # timestamp Arduino en ms (contador monotónico del firmware)
    state:   object       # RoverState en el momento del registro
    dist_mm: int          # distancia frontal ToF en mm
    batt_mv: int          # tensión batería en mV


class WaypointTracker:
    """
    Registra los últimos MAX_WAYPOINTS puntos seguros visitados durante EXPLORE
    con safety NORMAL (SyRS-061). Detecta condiciones tácticas de retreat a nivel
    HLC antes de que el Arduino dispare el FAULT de emergencia.

    Capas de protección de distancia:
      LLC (hardware):  < 200 mm (HC-SR04) o < 150 mm (VL53L0X) → FAULT inmediato
      HLC (táctica):   < RETREAT_DIST_MM (300 mm)               → RET proactivo
    """

    def __init__(self, max_waypoints: int = MAX_WAYPOINTS,
                 retreat_dist_mm: int = RETREAT_DIST_MM):
        self._points: list        = []
        self._max: int            = max_waypoints
        self._retreat_dist: int   = retreat_dist_mm

    # ── registro ─────────────────────────────────────────────────────────────

    def record(self, tlm, msm_state) -> None:
        """
        Guarda un waypoint si el rover está en EXPLORE con safety NORMAL.
        Mantiene como máximo MAX_WAYPOINTS entradas (FIFO).
        """
        if msm_state != RoverState.EXPLORE:
            return
        if tlm.safety != "NORMAL":
            return
        wp = Waypoint(
            tick_ms=tlm.tick_ms,
            state=msm_state,
            dist_mm=tlm.dist_mm,
            batt_mv=tlm.batt_mv,
        )
        self._points.append(wp)
        if len(self._points) > self._max:
            self._points.pop(0)

    # ── consulta ─────────────────────────────────────────────────────────────

    def last_safe(self):
        """Retorna el waypoint seguro más reciente, o None si no hay ninguno."""
        return self._points[-1] if self._points else None

    def count(self) -> int:
        return len(self._points)

    def should_retreat(self, tlm) -> bool:
        """
        True si la distancia frontal es menor que el umbral táctico HLC
        y hay una lectura válida (dist_mm > 0).
        Solo activo cuando dist_mm > 0 (0 = sin lectura del VL53L0X).
        """
        return tlm.dist_mm > 0 and tlm.dist_mm < self._retreat_dist


# ─── Energy Monitor ──────────────────────────────────────────────────────────

class EnergyLevel(enum.Enum):
    OK       = "OK"
    WARN     = "WARN"
    CRITICAL = "CRITICAL"


class EnergyMonitor:
    """
    Supervisa tensión de batería desde frames TLM (EPS-REQ-001, SyRS-017).
    Solo logea al cambiar de nivel para no saturar el log.

    Umbrales para batería 4S Li-ion:
      OK       : batt_mv ≥ BATT_WARN_MV
      WARN     : BATT_CRITICAL_MV ≤ batt_mv < BATT_WARN_MV
      CRITICAL : batt_mv < BATT_CRITICAL_MV  → forzar STB
    """

    def __init__(self,
                 warn_mv: int     = BATT_WARN_MV,
                 critical_mv: int = BATT_CRITICAL_MV):
        self._warn_mv:     int         = warn_mv
        self._critical_mv: int         = critical_mv
        self._level:       EnergyLevel = EnergyLevel.OK

    @property
    def level(self) -> EnergyLevel:
        return self._level

    def update(self, tlm) -> EnergyLevel:
        """
        Evalúa batt_mv del TLM y actualiza el nivel.
        Retorna el nivel resultante.
        batt_mv == 0 significa sin lectura — se ignora sin cambiar el nivel.
        """
        mv = tlm.batt_mv
        if mv == 0:
            return self._level

        if mv < self._critical_mv:
            new_level = EnergyLevel.CRITICAL
        elif mv < self._warn_mv:
            new_level = EnergyLevel.WARN
        else:
            new_level = EnergyLevel.OK

        self._level = new_level
        return self._level


# ─── Slip Monitor ────────────────────────────────────────────────────────────

class SlipMonitor:
    """
    Detecta slip de ruedas procesando el campo stall_mask del frame TLM (RF-004).

    El LLC pone stall_mask[i]=1 cuando el motor i está en velocidad > STALL_SPEED_MIN
    pero su encoder no registra movimiento durante > STALL_THRESHOLD ciclos (~1 s).
    Eso indica que la rueda gira sin tracción (slip) o hay bloqueo mecánico.

    Estrategia HLC:
      - Solo actúa en estado EXPLORE (en AVD/RET el stall puede ser esperado).
      - Acumula frames TLM consecutivos con stall_mask != 0.
      - Cuando el contador alcanza slip_stall_frames → override RET.
      - Se reinicia cuando stall_mask == 0 o el rover sale de EXPLORE.

    Prioridad en el loop: SafeMode > retreat (dist) > slip > comando fuente.

    # LIMITACIÓN ARQUITECTÓNICA — dead code bajo comportamiento LLC actual
    #
    # La transición stall→FAULT en el LLC es atómica: update_safety() establece
    # safety=FaultStall y state=Fault en el mismo ciclo en que stall_mask != 0
    # (rover-low-level-controller/src/state_machine/mod.rs, update_safety()).
    # Por tanto, cualquier frame TLM con stall_mask != 0 también tendrá
    # safety="FAULT", y safe_mode.update() (que verifica safety=="FAULT") se
    # activa PRIMERO en la cadena de prioridad, llamando slip.reset() antes de
    # que slip.update() pueda acumular el segundo frame.
    #
    # En la arquitectura actual, SlipMonitor._count nunca supera 0 en producción.
    #
    # Para activarlo se necesitaría uno de:
    #   a) LLC emita un nivel intermedio "WARN:STALL" (soft stall) antes del FAULT,
    #      con stall_mask != 0 pero safety todavía NORMAL/WARN. Requiere cambios
    #      en state_machine/mod.rs.
    #   b) Cambiar la prioridad y que SlipMonitor verifique safety != "FAULT" antes
    #      de acumular (pero entonces RET se enviaría a un LLC en FAULT que
    #      responderá ERR:ESTOP de todas formas).
    #
    # Actualmente la protección equivalente la provee SafeMode vía safety="FAULT".
    # SlipMonitor se mantiene para uso futuro y como documentación del requisito
    # RF-004 (slip detection).
    """

    def __init__(self, stall_frames: int = SLIP_STALL_FRAMES):
        self._threshold: int = stall_frames
        self._count:     int = 0

    def update(self, tlm, msm_state) -> bool:
        """
        Actualiza el contador con el último frame TLM.
        Retorna True si se debe emitir RET por slip persistente.

        msm_state debe ser RoverState — solo actúa en EXPLORE.
        """
        if msm_state != RoverState.EXPLORE or tlm.stall_mask == 0:
            self._count = 0
            return False

        self._count += 1
        return self._count >= self._threshold

    def reset(self) -> None:
        self._count = 0

    @property
    def stall_count(self) -> int:
        return self._count


# ─── Thermal Monitor ─────────────────────────────────────────────────────────

class ThermalLevel(enum.Enum):
    OK       = "OK"
    WARN     = "WARN"
    CRITICAL = "CRITICAL"


class ThermalMonitor:
    """
    Supervisa temperatura ambiente desde el campo temp_c del TLM (RNF-004).

    El sensor es el LM335 conectado al Arduino (A6). Umbrales por defecto
    alineados con el SRS (RNF-004):
      OK       : temp_c < TEMP_WARN_C (45 °C)
      WARN     : TEMP_WARN_C ≤ temp_c < TEMP_CRIT_C  → advertencia al operador
      CRITICAL : temp_c ≥ TEMP_CRIT_C (60 °C)        → activa SafeMode

    Solo logea al cambiar de nivel (mismo patrón que EnergyMonitor).
    temp_c == 0 significa sin lectura — se ignora sin cambiar el nivel.
    """

    def __init__(self,
                 warn_c: int = TEMP_WARN_C,
                 crit_c: int = TEMP_CRIT_C):
        self._warn_c: int          = warn_c
        self._crit_c: int          = crit_c
        self._level:  ThermalLevel = ThermalLevel.OK

    @property
    def level(self) -> ThermalLevel:
        return self._level

    def update(self, tlm) -> ThermalLevel:
        """
        Evalúa temp_c del TLM y actualiza el nivel.
        Retorna el nivel resultante.
        """
        t = tlm.temp_c
        if t == 0:
            return self._level

        if t >= self._crit_c:
            new_level = ThermalLevel.CRITICAL
        elif t >= self._warn_c:
            new_level = ThermalLevel.WARN
        else:
            new_level = ThermalLevel.OK

        self._level = new_level
        return self._level


# ─── Safe Mode ───────────────────────────────────────────────────────────────

class SafeMode:
    """
    Estado de seguridad HLC-only (SYS-FUN-040/041).

    Safe Mode es un estado gestionado por el HLC — el Arduino no lo conoce.
    Se activa ante dos condiciones definidas en el SRS:
      1. Batería crítica: EnergyLevel.CRITICAL (batt_mv < BATT_CRITICAL_MV)
      2. LLC en FAULT:    tlm.safety == "FAULT" (HC-SR04, stall o FLT forzado)

    La condición de latencia C&DH >5s (SYS-FUN-040) está cubierta por el
    escalado de link loss (tlm_retreat_s / tlm_stb_s) en el loop principal.

    Comportamiento en Safe Mode (SYS-FUN-041):
      - Solo STB y PING permitidos (actuadores de tracción desenergizados)
      - Cualquier comando de movimiento (EXP/AVD/RET) es bloqueado
      - El watchdog Arduino se mantiene vivo con PING periódicos
      - Se loguea la causa en cada activación

    Salida: solo mediante reset() explícito del operador (comando RST).
    Una vez activo, no se desactiva automáticamente aunque la batería suba
    o el LLC salga de FAULT — requiere intervención humana.

    Prioridad en el loop: SafeMode > retreat > slip > comando fuente.
    """

    def __init__(self):
        self._active:         bool = False
        self._reason:         str  = ""
        self._just_activated: bool = False

    @property
    def active(self) -> bool:
        return self._active

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def just_activated(self) -> bool:
        """True únicamente en el ciclo en que Safe Mode se activó por primera vez."""
        return self._just_activated

    def update(self, tlm, e_level: "EnergyLevel",
               t_level: "ThermalLevel") -> bool:
        """
        Evalúa condiciones de activación con el último TLM.
        Retorna True si Safe Mode está (o acaba de quedar) activo.
        Una vez activo no se desactiva aquí — usar reset().

        Condiciones de activación (SYS-FUN-040, RNF-004):
          - Batería crítica   : e_level == EnergyLevel.CRITICAL
          - LLC en FAULT      : tlm.safety == "FAULT"
          - Temperatura crítica: t_level == ThermalLevel.CRITICAL
        """
        self._just_activated = False

        if self._active:
            return True

        if e_level == EnergyLevel.CRITICAL:
            self._active         = True
            self._just_activated = True
            self._reason = f"batería crítica ({tlm.batt_mv} mV < {BATT_CRITICAL_MV} mV)"
        elif tlm.safety == "FAULT":
            self._active         = True
            self._just_activated = True
            self._reason = f"LLC en FAULT (safety={tlm.safety})"
        elif t_level == ThermalLevel.CRITICAL:
            self._active         = True
            self._just_activated = True
            self._reason = f"temperatura crítica ({tlm.temp_c} °C ≥ {TEMP_CRIT_C} °C)"

        return self._active

    def reset(self) -> None:
        """Desactiva Safe Mode — solo por reset explícito del operador (RST)."""
        self._active         = False
        self._reason         = ""
        self._just_activated = False

    def blocks_command(self, cmd: str) -> bool:
        """True si el comando debe bloquearse estando en Safe Mode."""
        return self._active and cmd not in ("STB", "PING", "RST")


# ─── GCS Comm Link Monitor ───────────────────────────────────────────────────

class CommLinkState(enum.Enum):
    COMUNICAR      = "Comunicar"
    GESTION_ENLACE = "GestiónEnlace"


class CommLinkMonitor:
    """
    Gestión de enlace GCS→HLC (SRS-013, SYS-FUN-021).

    Implementa la máquina de estados de comunicaciones (§7.3.8):
      COMUNICAR      — enlace activo; último paquete GCS ≤ GCS_LINK_LOST_S s.
      GESTION_ENLACE — enlace perdido; ejecuta política de reintentos (TBD-OP-02).

    Transiciones:
      COMUNICAR      → link_lost                     → GESTION_ENLACE
      GESTION_ENLACE → link_restored                 → COMUNICAR
      GESTION_ENLACE → reconnect_attempt_succeeded   → COMUNICAR
      GESTION_ENLACE → reconnect_attempt_failed×N    → (max_retries_exceeded → STB)

    update() retorna uno de los siguientes eventos (o None):
      "link_lost"                  — primera detección de pérdida
      "link_restored"              — recuperado sin retries
      "reconnect_attempt_failed"   — intento de reconexión fallido
      "reconnect_attempt_succeeded"— enlace recuperado tras retries
      "max_retries_exceeded"       — reintentos agotados → loop fuerza STB
    """

    def __init__(self,
                 link_lost_s:      float = GCS_LINK_LOST_S,
                 retry_interval_s: float = GCS_RETRY_INTERVAL_S,
                 max_retries:      int   = GCS_MAX_RETRIES):
        self._state          : CommLinkState = CommLinkState.COMUNICAR
        self._link_lost_s    : float         = link_lost_s
        self._retry_interval : float         = retry_interval_s
        self._max_retries    : int           = max_retries
        self._retry_count    : int           = 0
        self._last_retry_ts  : float         = 0.0

    @property
    def state(self) -> CommLinkState:
        return self._state

    @property
    def is_lost(self) -> bool:
        return self._state == CommLinkState.GESTION_ENLACE

    @property
    def retry_count(self) -> int:
        return self._retry_count

    def update(self, last_gcs_time: float, now: float,
               gcs_source=None) -> "str | None":
        """
        Evalúa el estado del enlace GCS y ejecuta la política de reintentos.

        last_gcs_time : time.monotonic() del último paquete recibido del GCS.
        now           : time.monotonic() actual.
        gcs_source    : GCSSource opcional — si se proporciona se envía HB_REQ
                        en cada intento de reconexión.

        Retorna el evento ocurrido en este ciclo, o None si no hubo cambio.
        """
        silent_s = now - last_gcs_time

        if self._state == CommLinkState.COMUNICAR:
            if silent_s > self._link_lost_s:
                self._state         = CommLinkState.GESTION_ENLACE
                self._retry_count   = 0
                self._last_retry_ts = now
                return "link_lost"
            return None

        # ── GESTION_ENLACE ──────────────────────────────────────────────────

        # Enlace recuperado (se recibió un paquete GCS reciente)
        if silent_s <= self._link_lost_s:
            had_retries       = self._retry_count > 0
            self._state       = CommLinkState.COMUNICAR
            self._retry_count = 0
            return "reconnect_attempt_succeeded" if had_retries else "link_restored"

        # Reintentos agotados — el loop principal forzará STB
        if self._retry_count >= self._max_retries:
            return "max_retries_exceeded"

        # Probe periódico de reconexión
        if now - self._last_retry_ts >= self._retry_interval:
            self._retry_count  += 1
            self._last_retry_ts = now
            if gcs_source is not None:
                gcs_source.send_probe()
            return "reconnect_attempt_failed"

        return None


# ─── Logger ──────────────────────────────────────────────────────────────────

class OlympusLogger:
    """
    Logger estructurado con timestamps ISO-8601 (SRS-061, CDH-REQ-002).
    Escribe a stdout y a LOG_PATH con rotación automática por tamaño.

    Política de rotación (CDH-REQ-002 — ~2 h de retención mínima):
      _MAX_BYTES    = 5 MB  → ~3–4 h de logs a ciclo 1 s
      _BACKUP_COUNT = 1     → hlc.log + hlc.log.1 ≈ 7–8 h en disco

    Si el archivo no es accesible, continúa solo con stdout sin abortar.
    """

    DEFAULT_LOG_PATH = "/var/log/olympus/hlc.log"
    _MAX_BYTES       = 5_000_000   # ~3–4 h de logs típicos
    _BACKUP_COUNT    = 1           # un fichero de respaldo → ~7–8 h total en disco

    def __init__(self, log_path: str = DEFAULT_LOG_PATH):
        self._handler = None
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            self._handler = logging.handlers.RotatingFileHandler(
                log_path,
                maxBytes=self._MAX_BYTES,
                backupCount=self._BACKUP_COUNT,
                encoding="utf-8",
            )
            self._handler.setFormatter(logging.Formatter("%(message)s"))
        except OSError as e:
            print(f"[Logger] Warning: no se puede abrir {log_path}: {e} — solo stdout")

    # ── escritura interna ────────────────────────────────────────────────────

    def _write(self, level: str, component: str, msg: str) -> None:
        ts   = datetime.datetime.now().isoformat(timespec="milliseconds")
        line = f"[{ts}] [{level:<5}] [{component:<7}] {msg}"
        print(line)
        if self._handler:
            self._handler.emit(logging.makeLogRecord({"msg": line}))

    # ── API pública ──────────────────────────────────────────────────────────

    def info(self, component: str, msg: str) -> None:
        self._write("INFO", component, msg)

    def warn(self, component: str, msg: str) -> None:
        self._write("WARN", component, msg)

    def log_transition(self, from_state, to_state, reason: str,
                       warn: bool = False) -> None:
        """
        Auditoría de transición de estado MSM (SRS-061).
        Registra estado_origen, estado_destino, razón y timestamp.
        """
        level = "WARN" if warn else "INFO"
        self._write(level, "MSM",
                    f"{from_state.value} → {to_state.value}  [{reason}]")

    def log_cmd(self, cmd: str, kind: str, data) -> None:
        """Log del comando enviado y la respuesta recibida del Arduino."""
        resp = f"{data}" if data else kind.upper()
        self._write("INFO", "CMD", f"{cmd:<16} → {kind}:{resp}" if data
                    else f"{cmd:<16} → {kind}")

    def log_tlm(self, tlm) -> None:
        """Log compacto del frame TLM recibido (SyRS-030)."""
        self._write("INFO", "TLM",
                    f"{tlm.safety:<6} stall={tlm.stall_mask:06b} "
                    f"batt={tlm.batt_mv}mV/{tlm.batt_ma}mA "
                    f"dist={tlm.dist_mm}mm t={tlm.tick_ms}ms")

    def log_energy(self, level, batt_mv: int) -> None:
        """Log de cambio de nivel de energía (EPS-REQ-001)."""
        lvl_str = level.value if hasattr(level, 'value') else str(level)
        msg = f"batería {batt_mv} mV — nivel {lvl_str}"
        if lvl_str == "CRITICAL":
            self._write("ERROR", "EPS", msg)
        elif lvl_str == "WARN":
            self._write("WARN",  "EPS", msg)
        else:
            self._write("INFO",  "EPS", msg)

    def log_cycle(self, cycle_ms: float) -> None:
        """Log periódico de tiempo de ciclo para verificación de RNF-001."""
        self._write("DEBUG", "CYCLE", f"{cycle_ms:.1f} ms")

    def log_link_event(self, event: str, detail: str = "") -> None:
        """
        Auditoría de evento de enlace GCS (SRS-013, §7.3.8).
        Registra transición de estado CommLink con timestamp ISO-8601.
        """
        warn = event in ("link_lost", "reconnect_attempt_failed",
                         "max_retries_exceeded")
        level = "WARN" if warn else "INFO"
        msg   = f"GCS link event={event}" + (f" — {detail}" if detail else "")
        self._write(level, "COMM", msg)

    def close(self) -> None:
        """
        Cierra el log garantizando escritura física al almacenamiento no volátil
        (SYS-FUN-050). Secuencia: flush → fsync → close.
        fsync es necesario porque el kernel puede mantener páginas sucias en
        buffer incluso después de close() — sin él el apagado abrupto puede
        truncar el fichero de log.
        """
        if self._handler:
            try:
                self._handler.flush()
                if hasattr(self._handler, "stream") and self._handler.stream:
                    os.fsync(self._handler.stream.fileno())
            except OSError:
                pass  # stream ya cerrado o fd no sincronizable (e.g. stdout)
            self._handler.close()
            self._handler = None


# ─── Rover State Machine ─────────────────────────────────────────────────────

class RoverState(enum.Enum):
    STANDBY = "STB"
    EXPLORE = "EXP"
    AVOID   = "AVD"
    RETREAT = "RET"
    FAULT   = "FLT"

    @staticmethod
    def from_ack(label: str):
        for s in RoverState:
            if s.value == label:
                return s
        return None


class RoverMSM:
    """
    Espejo local del estado MSM del Arduino (SyRS-060, SRS-061).

    Registra el estado actual y el tiempo de entrada para auditoría
    de transiciones. Solo hace transición al recibir un ACK confirmado
    del Arduino — nunca por inferencia del comando enviado.
    """

    def __init__(self):
        self._state:   RoverState = RoverState.STANDBY
        self._entered: float      = time.monotonic()

    @property
    def state(self) -> RoverState:
        return self._state

    def transition(self, new_state: RoverState) -> None:
        """Registra una transición confirmada por ACK del Arduino."""
        self._state   = new_state
        self._entered = time.monotonic()

    def time_in_state(self) -> float:
        """Segundos transcurridos en el estado actual."""
        return time.monotonic() - self._entered

    def blocks_command(self, cmd: str) -> bool:
        """
        True si el comando debe bloquearse en el estado actual.
        En FAULT solo RST y PING son válidos (ICD LLC §Tabla de estados).
        """
        if self._state == RoverState.FAULT:
            return cmd not in ("RST", "PING")
        return False


# ─── Command Sources ─────────────────────────────────────────────────────────

class ManualSource:
    """
    Reads MSM commands from stdin.
    Accepts shortcuts or full MSM protocol strings.

    Shortcuts:
      exp <l> <r>  →  EXP:<l>:<r>
      avl          →  AVD:L
      avr          →  AVD:R
      ret          →  RET
      stb          →  STB
      ping         →  PING
      rst          →  RST
      q            →  exit
    """

    def __init__(self):
        self._print_help()

    def _print_help(self):
        print("\n--- Olympus Controller — Manual Mode ---")
        print("Shortcuts: exp <l> <r> | avl | avr | ret | stb | ping | rst | q (quit)")
        print("Or type MSM commands directly: EXP:80:80 / AVD:L / RET / STB\n")

    def next_command(self, log=None):
        """
        Blocks until the operator enters a command.
        Returns the MSM command string, or None to skip this cycle.
        Raises SystemExit on 'q'.
        """
        try:
            raw = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            raise SystemExit(0)

        if not raw:
            return None

        lower = raw.lower()

        if lower == "q":
            raise SystemExit(0)
        elif lower.startswith("exp "):
            parts = lower.split()
            if len(parts) == 3:
                return f"EXP:{parts[1]}:{parts[2]}"
            print("[!] Usage: exp <left_speed> <right_speed>  e.g. exp 80 80")
            return None
        elif lower == "avl":
            return "AVD:L"
        elif lower == "avr":
            return "AVD:R"
        elif lower == "ret":
            return "RET"
        elif lower == "stb":
            return "STB"
        elif lower == "ping":
            return "PING"
        elif lower == "rst":
            return "RST"
        else:
            # Pass through as-is (full MSM command)
            return raw.upper()


class VisionSource:
    """
    Reads frames from the CSI camera and decides MSM commands via cv2.dnn.

    Two decision modes selected by VISION_MODE (olympus_controller.yaml):

      "bbox"        — YOLOv8n ONNX. Decision based on bounding box center
                      position in the frame. Stable reference implementation.

      "segmentation" — YOLOv8n-seg ONNX. Decision based on pixel mask
                       coverage per zone (GNC-REQ-002). Falls back to bbox
                       mode if the seg model file is not found.

    Frame capture uses rpicam-still --output - (one JPEG per call) instead of
    cv2.VideoCapture, because RPi5/pisp V4L2 nodes are raw CSI capture nodes
    that cannot be opened directly by OpenCV.

    Zone layout (applies to both modes):
      Left zone  (0–zone_left_end)          → AVD:R  (obstacle left, turn right)
      Center zone (zone_left_end–zone_right_start) → RET (obstacle ahead, retreat)
      Right zone (zone_right_start–1)       → AVD:L  (obstacle right, turn left)
      No detection                          → EXP:<l>:<r> (keep exploring)
    """

    # YOLOv8n-seg ONNX output layout
    # output0: [1, 116, 8400] — 4 bbox + 80 class scores + 32 mask coefficients
    # output1: [1, 32, 160, 160] — mask prototypes
    _SEG_BBOX_FIELDS  = 4
    _SEG_CLASS_FIELDS = 80
    _SEG_COEFF_FIELDS = 32
    _SEG_PROTO_SIZE   = 160

    def __init__(self, model_path):
        try:
            import cv2
            import numpy as np
            import subprocess
            self._cv2        = cv2
            self._np         = np
            self._subprocess = subprocess
        except ImportError:
            print("[ERROR] OpenCV not found. Install python3-opencv.")
            raise SystemExit(1)

        self._mode = VISION_MODE

        if self._mode == "segmentation":
            seg_path = SEG_MODEL_PATH
            if not __import__("os").path.exists(seg_path):
                print(f"[Vision] WARNING: seg model not found at {seg_path} — "
                      f"falling back to bbox mode.")
                self._mode = "bbox"
            else:
                print(f"[Vision] Loading segmentation model: {seg_path}")
                self._net = self._cv2.dnn.readNetFromONNX(seg_path)
                print(f"[Vision] Seg model loaded — {len(self._net.getLayerNames())} layers")

        if self._mode == "bbox":
            print(f"[Vision] Loading bbox model: {model_path}")
            self._net = self._cv2.dnn.readNetFromONNX(model_path)
            print(f"[Vision] Bbox model loaded — {len(self._net.getLayerNames())} layers")

        print(f"[Vision] Mode: {self._mode}. Camera ready (rpicam-still per-frame).")

    def _capture_frame(self):
        """
        Capture one JPEG via rpicam-still --output -.
        Returns a BGR numpy array or None on error.
        rpicam-vid MJPEG stdout did not flush reliably on RPi5/pisp;
        rpicam-still is simpler and sufficient for ~1–2 Hz inference.
        """
        result = self._subprocess.run(
            [
                "rpicam-still",
                "--output", "-",
                "--width",  str(FRAME_WIDTH),
                "--height", str(FRAME_HEIGHT),
                "--timeout", "1000",
                "--nopreview",
                "--encoding", "jpg",
            ],
            capture_output=True,
        )
        if result.returncode != 0 or not result.stdout:
            return None
        return self._cv2.imdecode(
            self._np.frombuffer(result.stdout, self._np.uint8),
            self._cv2.IMREAD_COLOR,
        )

    def next_command(self, log=None):
        """
        Captures a frame, runs inference, and returns an MSM command.
        Returns None on camera read error (caller will send STB).
        """
        frame = self._capture_frame()
        if frame is None:
            print("[Vision] Frame capture failed.")
            return None

        if self._mode == "segmentation":
            return self._decide_seg(frame)
        return self._decide_bbox(frame)

    # ── Bbox mode (reference) ─────────────────────────────────────────────────

    def _decide_bbox(self, frame):
        """
        YOLOv8n bbox decision. Selects the largest detection by bbox area
        and maps its center x to a zone command.
        """
        cv2 = self._cv2
        np  = self._np

        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (640, 640), swapRB=True, crop=False
        )
        self._net.setInput(blob)
        output = self._net.forward()  # shape: (1, 84, 8400)

        predictions = output[0].T     # → (8400, 84)

        best_area = 0.0
        best_cx   = None

        for pred in predictions:
            scores     = pred[4:]
            class_id   = int(np.argmax(scores))
            confidence = float(scores[class_id])

            if confidence < VISION_CONF_MIN:
                continue

            cx_norm, _, w_norm, h_norm = pred[:4]
            cx        = cx_norm / 640.0
            area_frac = (w_norm / 640.0) * (h_norm / 640.0)

            if area_frac < VISION_AREA_MIN:
                continue

            if area_frac > best_area:
                best_area = area_frac
                best_cx   = cx

        if best_cx is None:
            return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"

        if best_cx < ZONE_LEFT_END:
            return "AVD:R"
        elif best_cx > ZONE_RIGHT_START:
            return "AVD:L"
        else:
            return "RET"

    # ── Segmentation mode (GNC-REQ-002) ──────────────────────────────────────

    def _decode_masks(self, output0, output1, frame_h, frame_w):
        """
        Decode YOLOv8n-seg outputs into a list of binary masks.

        output0: ndarray [1, 116, 8400]
        output1: ndarray [1, 32, 160, 160]

        Returns list of bool ndarrays [frame_h, frame_w], one per detection
        that passes SEG_CONF_MIN and SEG_AREA_MIN filters.
        """
        np  = self._np
        cv2 = self._cv2

        B = self._SEG_BBOX_FIELDS
        C = self._SEG_CLASS_FIELDS
        K = self._SEG_COEFF_FIELDS
        P = self._SEG_PROTO_SIZE

        preds  = output0[0].T   # [8400, 116]
        protos = output1[0]     # [32, 160, 160]

        frame_area = frame_h * frame_w
        masks = []

        for pred in preds:
            scores     = pred[B : B + C]
            confidence = float(scores.max())
            if confidence < SEG_CONF_MIN:
                continue

            # bbox area filter (normalized to 640×640 input space)
            cx_n, cy_n, w_n, h_n = pred[:B]
            area_frac = (w_n / 640.0) * (h_n / 640.0)
            if area_frac < SEG_AREA_MIN:
                continue

            # Decode instance mask: coefficients × prototypes → sigmoid → resize
            coeffs   = pred[B + C : B + C + K]           # [32]
            mask_160 = coeffs @ protos.reshape(K, P * P)  # [P*P]
            mask_160 = 1.0 / (1.0 + np.exp(-mask_160))   # sigmoid
            mask_160 = mask_160.reshape(P, P).astype(np.float32)
            mask_full = cv2.resize(mask_160, (frame_w, frame_h))

            # Crop to bbox to avoid bleed outside the detected object
            x1 = max(0, int((cx_n - w_n / 2) * frame_w / 640))
            y1 = max(0, int((cy_n - h_n / 2) * frame_h / 640))
            x2 = min(frame_w, int((cx_n + w_n / 2) * frame_w / 640))
            y2 = min(frame_h, int((cy_n + h_n / 2) * frame_h / 640))

            binary = (mask_full > 0.5)
            binary[:y1, :]  = False
            binary[y2:, :]  = False
            binary[:, :x1]  = False
            binary[:, x2:]  = False

            # Final area check on actual mask (tighter than bbox)
            if binary.sum() < SEG_AREA_MIN * frame_area:
                continue

            masks.append(binary)

        return masks

    def _decide_seg(self, frame):
        """
        YOLOv8n-seg decision. Combines all decoded masks into a zone
        occupancy map and returns the command for the most-covered zone.
        Only the lower portion of the frame (y >= SEG_ROI_TOP * H) is
        evaluated — upper portion typically contains background/sky.
        """
        cv2 = self._cv2
        np  = self._np

        H, W = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (640, 640), swapRB=True, crop=False
        )
        self._net.setInput(blob)
        outputs = self._net.forward(self._net.getUnconnectedOutLayersNames())
        output0, output1 = outputs[0], outputs[1]   # [1,116,8400] and [1,32,160,160]

        masks = self._decode_masks(output0, output1, H, W)

        if not masks:
            return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"

        # ROI: only evaluate rows below SEG_ROI_TOP
        roi_y = int(SEG_ROI_TOP * H)

        # Zone pixel boundaries
        left_end    = int(ZONE_LEFT_END    * W)
        right_start = int(ZONE_RIGHT_START * W)

        # Accumulate mask coverage per zone across all detections
        combined = np.zeros((H, W), dtype=bool)
        for m in masks:
            combined |= m

        roi = combined[roi_y:, :]           # [H-roi_y, W]
        roi_area = roi.shape[0]             # rows in ROI

        left_cov   = roi[:, :left_end].sum()   / max(roi_area * left_end,        1)
        center_cov = roi[:, left_end:right_start].sum() / max(
                         roi_area * (right_start - left_end), 1)
        right_cov  = roi[:, right_start:].sum() / max(roi_area * (W - right_start), 1)

        # Any zone above threshold? Pick the most covered one.
        candidates = {
            "AVD:R": left_cov,
            "RET":   center_cov,
            "AVD:L": right_cov,
        }
        best_cmd, best_cov = max(candidates.items(), key=lambda kv: kv[1])

        if best_cov < SEG_ZONE_MIN:
            return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"

        return best_cmd

    def release(self):
        pass  # No persistent process to clean up with rpicam-still


# ─── CSP Encapsulation ───────────────────────────────────────────────────────

class CSPPacket:
    """
    Encapsulamiento CSP v1.x (CubeSat Space Protocol) sobre UDP/IP (SRS-001).

    Implementa el subconjunto mínimo de CSP necesario para satisfacer los
    requisitos de integridad y encapsulamiento del SRS sin depender de libcsp:

      SRS-001  — encapsular/decapsular TM/CMD usando CSP sobre UDP/IP
      RF-006   — integridad de datos garantizada con CRC-32
      SyRS-016 — 100% de mensajes TM/CMD con CSP válido

    Razón de no usar libcsp completo (Nivel 2):
      - libcsp requiere receta Yocto adicional y bindings C/Python (ctypes).
      - El canal UHF/RDP está marcado como TBD-OP-02 en el SRS; sin hardware
        UHF disponible, el routing CSP completo no aporta valor verificable.
      - Esta implementación cubre todos los requisitos de integridad del SRS
        con stdlib Python (struct + zlib), sin dependencias externas.

    Formato de paquete (total = 4 + len(payload) + 4 bytes):
      ┌──────────────────────────────────────────────────────────┐
      │  CSP Header (32 bits, big-endian)                        │
      │  bits 31-30: priority  (2=NORM)                          │
      │  bits 29-25: src addr  (5 bits, 0–31)                    │
      │  bits 24-20: dst addr  (5 bits, 0–31)                    │
      │  bits 19-14: dst port  (6 bits, 0–63)                    │
      │  bits 13- 8: src port  (6 bits, 0–63)                    │
      │  bits  7- 2: reserved                                    │
      │  bit      1: CRC32 flag (siempre 1 en esta impl.)        │
      │  bit      0: reserved                                    │
      ├──────────────────────────────────────────────────────────┤
      │  Payload (N bytes, ASCII UTF-8)                          │
      ├──────────────────────────────────────────────────────────┤
      │  CRC-32 (4 bytes, big-endian, sobre header + payload)    │
      └──────────────────────────────────────────────────────────┘

    Prioridades CSP:
      0=CRITICAL  1=HIGH  2=NORM  3=LOW
    """

    PRIO_NORM   = 2
    FLAG_CRC32  = 0b10   # bit 1 del campo flags CSP

    # Tamaño mínimo de un paquete válido: 4B header + 0B payload + 4B CRC
    MIN_SIZE = 8

    @staticmethod
    def pack(src: int, dst: int, dport: int, sport: int,
             payload: bytes, prio: int = 2) -> bytes:
        """
        Construye un paquete CSP con CRC-32.

        src, dst  : direcciones CSP (0–31)
        dport     : puerto destino (0–63)
        sport     : puerto origen  (0–63)
        payload   : datos en bytes (ASCII TLM o comando MSM)
        prio      : prioridad CSP (defecto NORM=2)

        Retorna bytes listos para enviar por UDP.
        """
        header = (
            ((prio  & 0x03) << 30) |
            ((src   & 0x1F) << 25) |
            ((dst   & 0x1F) << 20) |
            ((dport & 0x3F) << 14) |
            ((sport & 0x3F) <<  8) |
            CSPPacket.FLAG_CRC32
        )
        raw = struct.pack(">I", header) + payload
        crc = struct.pack(">I", zlib.crc32(raw) & 0xFFFFFFFF)
        return raw + crc

    @staticmethod
    def unpack(data: bytes) -> "tuple[int | None, bytes | None]":
        """
        Valida y decapsula un paquete CSP.

        Retorna (header: int, payload: bytes) si el paquete es válido.
        Retorna (None, None) si el tamaño es insuficiente o el CRC falla.
        El CRC-32 se verifica sobre header + payload antes de retornar.
        """
        if len(data) < CSPPacket.MIN_SIZE:
            return None, None

        raw, crc_recv = data[:-4], data[-4:]
        crc_calc = struct.pack(">I", zlib.crc32(raw) & 0xFFFFFFFF)
        if crc_calc != crc_recv:
            return None, None   # RF-006: integridad comprometida

        header  = struct.unpack(">I", raw[:4])[0]
        payload = raw[4:]
        return header, payload

    @staticmethod
    def dst_port(header: int) -> int:
        """Extrae el puerto destino del header CSP."""
        return (header >> 14) & 0x3F

    @staticmethod
    def src_port(header: int) -> int:
        """Extrae el puerto origen del header CSP."""
        return (header >> 8) & 0x3F

    @staticmethod
    def src_addr(header: int) -> int:
        """Extrae la dirección origen del header CSP."""
        return (header >> 25) & 0x1F


class GCSSource:
    """
    Recibe comandos y heartbeats del GCS vía UDP no-bloqueante (SRS-013).

    Cuando CSP_ENABLED=True (defecto) todos los paquetes UDP están
    encapsulados en CSP con CRC-32 (SRS-001, RF-006):

      GCS→HLC (puerto GCS_LISTEN_PORT):
        CSP[src=GCS, dst=HLC, dport=CSP_PORT_CMD] + payload "EXP:40:40"
        CSP[src=GCS, dst=HLC, dport=CSP_PORT_HB]  + payload "HB" | "HB_ACK:<seq>"

      HLC→GCS (puerto GCS_REPLY_PORT):
        CSP[src=HLC, dst=GCS, dport=CSP_PORT_TM]  + payload <línea TLM cruda>
        CSP[src=HLC, dst=GCS, dport=CSP_PORT_HB]  + payload "HB_REQ:<seq>"

    Cuando CSP_ENABLED=False (modo legado para pruebas sin GCS completo):
      Protocolo ASCII plano — CMD:<cmd>, HB, HB_ACK:<seq>.

    La dirección del GCS se aprende dinámicamente del primer paquete recibido.
    next_command() es no-bloqueante: retorna None si no hay paquete disponible.
    Paquetes con CRC-32 inválido son descartados y logueados (RF-006).
    """

    def __init__(self,
                 bind_addr:   str = GCS_BIND_ADDR,
                 listen_port: int = GCS_LISTEN_PORT,
                 reply_port:  int = GCS_REPLY_PORT):
        self._sock        = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_addr, listen_port))
        self._sock.setblocking(False)
        self._reply_port  = reply_port
        self._gcs_addr    = None   # (ip, reply_port) — aprendida dinámicamente
        self._last_recv   = time.monotonic()
        self._probe_seq   = 0
        csp_str = "CSP+CRC32" if CSP_ENABLED else "ASCII (legado)"
        print(f"[GCSSource] UDP {bind_addr}:{listen_port} → :{reply_port} "
              f"[{csp_str}]")

    @property
    def last_recv_time(self) -> float:
        """Monotonic timestamp del último paquete válido recibido del GCS."""
        return self._last_recv

    def next_command(self, log=None) -> "str | None":
        """
        Intenta leer un paquete UDP del GCS (no-bloqueante).

        Con CSP_ENABLED: decapsula CSP, verifica CRC-32 (RF-006), despacha
        por puerto de servicio. Paquetes con CRC inválido se descartan.
        Con CSP_ENABLED=False: modo ASCII legado para pruebas rápidas.

        Retorna la cadena de comando MSM o None si no hay comando.
        log: OlympusLogger opcional para registrar errores de integridad.
        """
        try:
            data, addr = self._sock.recvfrom(512)
        except BlockingIOError:
            return None
        except OSError:
            return None

        if CSP_ENABLED:
            header, payload = CSPPacket.unpack(data)
            if header is None:
                if log:
                    log.warn("COMM",
                             f"CSP CRC-32 inválido — paquete de {addr[0]} descartado "
                             f"(RF-006 integridad comprometida)")
                return None   # no actualizar last_recv con paquetes corruptos

            self._last_recv = time.monotonic()
            self._gcs_addr  = (addr[0], self._reply_port)

            dport = CSPPacket.dst_port(header)
            if dport == CSP_PORT_CMD:
                return payload.decode("utf-8", errors="replace").strip()
            # CSP_PORT_HB u otros — solo actualiza last_recv_time
            return None
        else:
            # Modo ASCII legado
            self._last_recv = time.monotonic()
            self._gcs_addr  = (addr[0], self._reply_port)
            msg = data.decode("utf-8", errors="replace").strip()
            if msg.startswith("CMD:"):
                return msg[4:]
            return None

    def send_probe(self) -> None:
        """
        Envía probe de reconexión HB_REQ al GCS (CommLinkMonitor, SRS-013).
        Con CSP: encapsulado en CSP[dport=CSP_PORT_HB] con CRC-32.
        """
        if self._gcs_addr is None:
            return
        self._probe_seq += 1
        payload = f"HB_REQ:{self._probe_seq}".encode()
        msg = (CSPPacket.pack(CSP_ADDR_HLC, CSP_ADDR_GCS,
                              CSP_PORT_HB, 0, payload)
               if CSP_ENABLED
               else payload + b"\n")
        try:
            self._sock.sendto(msg, self._gcs_addr)
        except OSError:
            pass

    def forward_tlm(self, raw_tlm: str) -> None:
        """
        Reenvía frame TLM al GCS vía UDP (downlink SRS-020).
        Con CSP: encapsulado en CSP[dport=CSP_PORT_TM] con CRC-32 (RF-006).
        """
        if self._gcs_addr is None:
            return
        payload = raw_tlm.encode()
        msg = (CSPPacket.pack(CSP_ADDR_HLC, CSP_ADDR_GCS,
                              CSP_PORT_TM, 0, payload)
               if CSP_ENABLED
               else payload + b"\n")
        try:
            self._sock.sendto(msg, self._gcs_addr)
        except OSError:
            pass

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


# ─── Response parser ─────────────────────────────────────────────────────────

# Kinds returned by parse_response():
#   "pong"        — PING keepalive reply
#   "ack"         — ACK:<STATE>  data = state string ("STB","EXP","AVD","RET","FLT")
#   "err_estop"   — ERR:ESTOP   rover is in FAULT, command rejected
#   "err_wdog"    — ERR:WDOG    watchdog fired, rover went to FAULT
#   "err_unknown" — ERR:UNKNOWN command not recognised by firmware
#   "unknown"     — unrecognised frame (data = raw string)

def parse_response(resp):
    if resp == "PONG":
        return ("pong", None)
    if resp.startswith("ACK:"):
        return ("ack", resp[4:])
    if resp == "ERR:ESTOP":
        return ("err_estop", None)
    if resp == "ERR:WDOG":
        return ("err_wdog", None)
    if resp == "ERR:UNKNOWN":
        return ("err_unknown", None)
    return ("unknown", resp)


# ─── Dry-run mock ────────────────────────────────────────────────────────────

class DryRunRover:
    """Simulates Arduino responses following the MSM protocol."""

    _CMD_TO_STATE = {
        "STB": "STB", "RET": "RET", "FLT": "FLT",
        "RST": "STB", "AVD:L": "AVD", "AVD:R": "AVD",
    }

    # Synthetic TLM emitted every ~1 s to avoid false link-loss warnings
    # during dry-run testing (recv_tlm returning None would trigger COMM-REQ-005
    # after TLM_TIMEOUT_S seconds, which is confusing when there is no real link).
    _TLM_INTERVAL_S = 1.0
    _TLM_TEMPLATE   = (
        "TLM:NORMAL:000000:{tick}ms:16000mV:500mA:"
        "100:100:100:100:100:100:25C:25:25:25:25:25:25C:1000mm"
    )

    def __init__(self):
        self._state        = "STB"
        self._tick_ms      = 0
        self._last_tlm_ts  = time.monotonic()

    def send_command(self, cmd):
        if cmd == "PING":
            return "PONG"
        if cmd.startswith("EXP:"):
            self._state = "EXP"
            return "ACK:EXP"
        new_state = self._CMD_TO_STATE.get(cmd)
        if new_state is not None:
            self._state = new_state
            return f"ACK:{new_state}"
        return "ERR:UNKNOWN"

    def recv_tlm(self):
        """
        Emits a synthetic TLM frame every _TLM_INTERVAL_S seconds so that
        the link-loss watchdog (TLM_TIMEOUT_S) does not fire during dry-run.
        Returns None between intervals, mimicking the async behaviour of the LLC.
        """
        now = time.monotonic()
        if now - self._last_tlm_ts >= self._TLM_INTERVAL_S:
            self._tick_ms     += int((now - self._last_tlm_ts) * 1000)
            self._last_tlm_ts  = now
            return self._TLM_TEMPLATE.format(tick=self._tick_ms)
        return None


# ─── Main loop ───────────────────────────────────────────────────────────────

def _send(rover, cmd, log=None):
    """
    Envía cmd, parsea respuesta, retorna (kind, data).
    Si se pasa log (OlympusLogger) registra el intercambio.
    Retorna ("timeout", None) o ("error", str) en caso de fallo.
    """
    try:
        raw = rover.send_command(cmd)
        kind, data = parse_response(raw)
        if log:
            log.log_cmd(cmd, kind, data)
        return kind, data
    except TimeoutError as e:
        if log:
            log.warn("CMD", f"{cmd:<16} → TIMEOUT: {e}")
        return "timeout", None
    except Exception as e:
        if log:
            log.warn("CMD", f"{cmd:<16} → ERROR: {e}")
        return "error", str(e)


def run(rover, source, mode, log_path=OlympusLogger.DEFAULT_LOG_PATH):
    log = OlympusLogger(log_path)
    log.info("CTRL", f"Starting in {mode.upper()} mode")

    last_cmd_time   = time.monotonic()
    last_tlm_time   = time.monotonic()
    tlm_loss_level  = 0   # 0=ok  1=warn  2=retreat  3=stb
    msm             = RoverMSM()
    tracker         = WaypointTracker()
    energy          = EnergyMonitor()
    slip            = SlipMonitor()
    thermal         = ThermalMonitor()
    safe_mode       = SafeMode()
    prev_energy     = EnergyLevel.OK
    prev_thermal    = ThermalLevel.OK
    last_tlm_ts     = time.monotonic()   # para SyRS-017 (delta entre TLMs)
    last_storage_check = 0               # ciclo del último check de disco
    cycle_count     = 0

    # SRS-013 — CommLinkMonitor activo solo en modo GCS
    comm_link        = CommLinkMonitor() if isinstance(source, GCSSource) else None
    gcs_stb_forced   = False   # True cuando max_retries agotados → STB permanente

    try:
        while True:
            cycle_start = time.monotonic()

            # Drenar TLM asíncrono pendiente antes de cualquier comando
            raw_tlm = rover.recv_tlm()
            tlm_override = None

            # SRS-013 / SRS-020 — reenviar TLM al GCS (downlink) y evaluar enlace
            if raw_tlm and isinstance(source, GCSSource):
                source.forward_tlm(raw_tlm)

            if comm_link is not None:
                now = time.monotonic()
                link_event = comm_link.update(source.last_recv_time, now, source)
                if link_event == "link_lost":
                    log.log_link_event(link_event,
                                       f">{GCS_LINK_LOST_S:.0f}s sin paquete GCS — "
                                       f"transición a {CommLinkState.GESTION_ENLACE.value} "
                                       f"(SRS-013)")
                elif link_event in ("link_restored", "reconnect_attempt_succeeded"):
                    log.log_link_event(link_event,
                                       f"retorno a {CommLinkState.COMUNICAR.value} "
                                       f"(SRS-013)")
                    gcs_stb_forced = False
                elif link_event == "reconnect_attempt_failed":
                    log.log_link_event(link_event,
                                       f"intento {comm_link.retry_count}/{GCS_MAX_RETRIES} "
                                       f"— HB_REQ enviado")
                elif link_event == "max_retries_exceeded":
                    if not gcs_stb_forced:
                        log.log_link_event(link_event,
                                           f"reintentos agotados ({GCS_MAX_RETRIES}) — "
                                           f"forzando STB permanente (SRS-013)")
                        gcs_stb_forced = True

                # SYS-FUN-021: enlace perdido → forzar STB si reintentos agotados
                if gcs_stb_forced:
                    tlm_override = "STB"

            if raw_tlm:
                last_tlm_time = time.monotonic()
                if tlm_loss_level > 0:
                    log.info("COMM", "TLM restablecido — enlace recuperado")
                    tlm_loss_level = 0
                tlm = TlmFrame.parse(raw_tlm)
                if tlm:
                    log.log_tlm(tlm)
                    tracker.record(tlm, msm.state)

                    # SyRS-017 — verificar frecuencia TLM ≥ 1 Hz
                    now_ts      = time.monotonic()
                    tlm_delta_s = now_ts - last_tlm_ts
                    last_tlm_ts = now_ts
                    if tlm_delta_s > TLM_INTERVAL_WARN_S:
                        log.warn("COMM",
                                 f"TLM tardío: delta={tlm_delta_s:.1f} s "
                                 f"(esperado ≤ {TLM_INTERVAL_WARN_S:.0f} s) — "
                                 f"posible degradación de enlace (SyRS-017)")

                    # Supervisión de energía — logea solo al cambiar de nivel
                    e_level = energy.update(tlm)
                    if e_level != prev_energy:
                        log.log_energy(e_level, tlm.batt_mv)
                        prev_energy = e_level

                    # Supervisión térmica — logea solo al cambiar de nivel (RNF-004)
                    t_level = thermal.update(tlm)
                    if t_level != prev_thermal:
                        lvl_str = t_level.value
                        msg = f"temperatura {tlm.temp_c} °C — nivel {lvl_str}"
                        if t_level == ThermalLevel.CRITICAL:
                            log.warn("THERM", msg)
                        elif t_level == ThermalLevel.WARN:
                            log.warn("THERM", msg)
                        else:
                            log.info("THERM", msg)
                        prev_thermal = t_level

                    # Prioridad de override: SafeMode > retreat > slip
                    if safe_mode.update(tlm, e_level, t_level):
                        if safe_mode.just_activated:
                            log.warn("EPS",
                                     f"SAFE MODE activado — {safe_mode.reason} "
                                     f"(SYS-FUN-040) — solo STB/PING permitidos")
                        tlm_override = "STB"
                        slip.reset()
                    elif tracker.should_retreat(tlm):
                        wp = tracker.last_safe()
                        wp_info = (f"last_safe tick={wp.tick_ms}ms dist={wp.dist_mm}mm"
                                   if wp else "no waypoint previo")
                        log.warn("NAV",
                                 f"obstáculo táctico a {tlm.dist_mm} mm "
                                 f"(< {RETREAT_DIST_MM} mm) — forzando RET "
                                 f"[{wp_info}]")
                        tlm_override = "RET"
                        slip.reset()
                    elif slip.update(tlm, msm.state):
                        log.warn("NAV",
                                 f"slip detectado — stall_mask={tlm.stall_mask:06b} "
                                 f"durante {slip.stall_count} frames TLM — forzando RET (RF-004)")
                        tlm_override = "RET"
            else:
                # Link loss escalation — three levels (SYS-FUN-021 / COMM-REQ-005)
                silent_s = time.monotonic() - last_tlm_time
                if silent_s > TLM_STB_S:
                    if tlm_loss_level < 3:
                        log.warn("COMM",
                                 f"sin TLM por {TLM_STB_S:.0f}+ s — "
                                 f"forzando STB definitivo (COMM-REQ-005)")
                        tlm_loss_level = 3
                    tlm_override = "STB"
                elif silent_s > TLM_RETREAT_S:
                    if tlm_loss_level < 2:
                        wp = tracker.last_safe()
                        log.warn("COMM",
                                 f"sin TLM por {TLM_RETREAT_S:.0f}+ s — "
                                 f"RET al último waypoint seguro "
                                 f"{wp} (SYS-FUN-021)")
                        tlm_loss_level = 2
                    tlm_override = "RET"
                elif silent_s > TLM_WARN_S:
                    if tlm_loss_level < 1:
                        log.warn("COMM",
                                 f"sin TLM por {TLM_WARN_S:.0f}+ s — "
                                 f"enlace degradado")
                        tlm_loss_level = 1

            cmd = tlm_override or source.next_command(log)

            # Vision mode: camera error → safe stop
            if cmd is None and mode == "vision":
                log.warn("CTRL", "Camera error — sending STB")
                cmd = "STB"

            if cmd is not None:
                if safe_mode.blocks_command(cmd):
                    log.warn("CMD", f"{cmd:<16} → BLOCKED (Safe Mode activo — {safe_mode.reason})")
                elif msm.blocks_command(cmd):
                    log.warn("CMD", f"{cmd:<16} → BLOCKED (rover in FAULT, send RST)")
                else:
                    kind, data = _send(rover, cmd, log)
                    last_cmd_time = time.monotonic()

                    if kind == "ack" and data is not None:
                        new_state = RoverState.from_ack(data)
                        if new_state is not None:
                            log.log_transition(msm.state, new_state, f"ACK:{data}")
                            msm.transition(new_state)
                        if cmd == "RST":
                            safe_mode.reset()
                            log.info("EPS", "Safe Mode desactivado por RST del operador")
                    elif kind == "err_wdog":
                        log.log_transition(msm.state, RoverState.FAULT,
                                           "ERR:WDOG", warn=True)
                        msm.transition(RoverState.FAULT)
                        log.info("CTRL", "Auto-sending RST to recover from watchdog")
                        kind2, data2 = _send(rover, "RST", log)
                        if kind2 == "ack" and data2 is not None:
                            new_state = RoverState.from_ack(data2)
                            if new_state is not None:
                                log.log_transition(msm.state, new_state, "RST")
                                msm.transition(new_state)
                    elif kind == "err_estop":
                        log.log_transition(msm.state, RoverState.FAULT,
                                           "ERR:ESTOP", warn=True)
                        msm.transition(RoverState.FAULT)
                    elif kind == "err_unknown":
                        log.warn("CMD", f"{cmd:<16} → ERR:UNKNOWN (comando no reconocido por firmware)")

            # Keepalive: PING if no command sent in the last PING_INTERVAL_S
            if time.monotonic() - last_cmd_time >= PING_INTERVAL_S:
                _send(rover, "PING", log)
                last_cmd_time = time.monotonic()

            # In vision mode sleep briefly between frames
            if mode == "vision":
                time.sleep(0.05)  # ~20 Hz max loop rate

            # Medición de ciclo (RNF-001: ≤ 2000 ms)
            cycle_ms = (time.monotonic() - cycle_start) * 1000
            cycle_count += 1
            if cycle_ms > CYCLE_WARN_MS:
                log.warn("CYCLE",
                         f"ciclo lento: {cycle_ms:.1f} ms "
                         f"(umbral {CYCLE_WARN_MS} ms, RNF-001)")
            elif cycle_count % CYCLE_LOG_PERIOD == 0:
                log.log_cycle(cycle_ms)

            # SRS-014 — monitoreo de espacio en disco cada STORAGE_CHECK_CYCLES ciclos
            if cycle_count - last_storage_check >= STORAGE_CHECK_CYCLES:
                last_storage_check = cycle_count
                try:
                    log_dir = os.path.dirname(OlympusLogger.DEFAULT_LOG_PATH)
                    st      = os.statvfs(log_dir)
                    free_mb = (st.f_bavail * st.f_frsize) / 1_000_000
                    if free_mb < STORAGE_MIN_MB:
                        log.warn("CDH",
                                 f"espacio en disco bajo: {free_mb:.1f} MB libres "
                                 f"(mínimo {STORAGE_MIN_MB} MB) — "
                                 f"riesgo de pérdida de logs (SRS-014)")
                except OSError:
                    pass  # sistema de archivos no accesible

    except (KeyboardInterrupt, SystemExit):
        # ── Shutdown sequence (SYS-FUN-050 / SYS-FUN-051) ───────────────────
        # Step 1 — parking: send STB and wait for ACK:STB before exiting
        log.info("CTRL", "Shutdown iniciado — enviando STB (SYS-FUN-051)")
        _parked = False
        try:
            resp = rover.send_command("STB")
            if isinstance(resp, str) and "ACK:STB" in resp:
                _parked = True
                log.info("CTRL", "Parking confirmado (ACK:STB)")
            else:
                log.warn("CTRL",
                         f"ACK:STB no recibido (resp={resp!r}) — "
                         f"asumiendo parado por timeout")
        except Exception as exc:
            log.warn("CTRL", f"Error enviando STB en shutdown: {exc}")

        # Step 2 — log sync: escribir entrada final y forzar fsync (SYS-FUN-050)
        log.info("CTRL",
                 f"READY_FOR_POWEROFF — parked={_parked} "
                 f"(logs sincronizados a almacenamiento no volátil)")
        print("READY_FOR_POWEROFF")
    finally:
        log.close()   # flush + fsync + close


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Olympus HLC Controller — vision or manual mode"
    )
    parser.add_argument(
        "--mode",
        choices=["vision", "manual", "gcs"],
        required=True,
        help="Command source: 'vision' (camera+YOLOv8n), 'manual' (stdin) "
             "or 'gcs' (UDP commands from Ground Control Station, SRS-013)"
    )
    parser.add_argument(
        "--model",
        default="/usr/share/olympus/models/yolov8n.onnx",
        help="Path to YOLOv8n ONNX model (vision mode only)"
    )
    parser.add_argument(
        "--port",
        default="/dev/arduino_mega",
        help="Serial port for Arduino (default: /dev/arduino_mega)"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Skip Arduino connection; print commands to stdout (testing without hardware)"
    )
    parser.add_argument(
        "--log-path",
        default=OlympusLogger.DEFAULT_LOG_PATH,
        help=f"Path for the HLC log file (default: {OlympusLogger.DEFAULT_LOG_PATH})"
    )
    args = parser.parse_args()

    if args.dry_run:
        print("[Controller] DRY-RUN mode — Arduino not required.")
        rover = DryRunRover()
    else:
        print(f"[Controller] Connecting to Arduino on {args.port} @ {args.baud}...")
        try:
            rover = rover_bridge.Rover(args.port, args.baud)
            print("[Controller] Connected.")
        except Exception as e:
            print(f"[ERROR] Cannot open rover bridge: {e}")
            sys.exit(1)

    if args.mode == "manual":
        source = ManualSource()
    elif args.mode == "gcs":
        source = GCSSource()
    else:
        source = VisionSource(args.model)

    try:
        run(rover, source, args.mode, log_path=args.log_path)
    finally:
        if isinstance(source, VisionSource):
            source.release()
        elif isinstance(source, GCSSource):
            source.close()


if __name__ == "__main__":
    main()
