# olympus_hlc/monitors.py — Supervision monitors (energy, thermal, slip, safe mode, comm link, waypoints)

from .config import (
    BATT_WARN_MV, BATT_CRITICAL_MV,
    SLIP_STALL_FRAMES,
    TEMP_WARN_C, TEMP_CRIT_C,
    MAX_WAYPOINTS, RETREAT_DIST_MM,
    GCS_LINK_LOST_S, GCS_RETRY_INTERVAL_S, GCS_MAX_RETRIES,
)
from .models import (
    EnergyLevel, ThermalLevel, CommLinkState, RoverState, Waypoint,
)


# ─── Waypoint Tracker ────────────────────────────────────────────────────────

class WaypointTracker:
    """
    Registra los últimos MAX_WAYPOINTS puntos seguros visitados durante EXPLORE
    con safety NORMAL (SyRS-061). Detecta condiciones tácticas de retreat a
    nivel HLC antes de que el Arduino dispare el FAULT de emergencia.

    Capas de protección de distancia:
      LLC (hardware): < 200 mm → FAULT inmediato
      HLC (táctica):  < RETREAT_DIST_MM (300 mm) → RET proactivo
    """

    def __init__(self, max_waypoints: int = MAX_WAYPOINTS,
                 retreat_dist_mm: int = RETREAT_DIST_MM):
        self._points:       list = []
        self._max:          int  = max_waypoints
        self._retreat_dist: int  = retreat_dist_mm

    def record(self, tlm, msm_state) -> None:
        """Guarda un waypoint si el rover está en EXPLORE con safety NORMAL."""
        if msm_state != RoverState.EXPLORE or tlm.safety != "NORMAL":
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

    def last_safe(self) -> "Waypoint | None":
        return self._points[-1] if self._points else None

    def count(self) -> int:
        return len(self._points)

    def should_retreat(self, tlm) -> bool:
        """True si la distancia frontal es menor que el umbral táctico HLC."""
        return tlm.dist_mm > 0 and tlm.dist_mm < self._retreat_dist


# ─── Energy Monitor ──────────────────────────────────────────────────────────

class EnergyMonitor:
    """
    Supervisa tensión de batería desde frames TLM (EPS-REQ-001, SyRS-017).

    Umbrales para batería 4S Li-ion:
      OK       : batt_mv ≥ BATT_WARN_MV
      WARN     : BATT_CRITICAL_MV ≤ batt_mv < BATT_WARN_MV
      CRITICAL : batt_mv < BATT_CRITICAL_MV → forzar STB
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
        Evalúa batt_mv del TLM y actualiza el nivel. Retorna el nivel resultante.
        batt_mv == 0 significa sin lectura — se ignora.
        """
        mv = tlm.batt_mv
        if mv == 0:
            return self._level

        if mv < self._critical_mv:
            self._level = EnergyLevel.CRITICAL
        elif mv < self._warn_mv:
            self._level = EnergyLevel.WARN
        else:
            self._level = EnergyLevel.OK

        return self._level


# ─── Slip Monitor ────────────────────────────────────────────────────────────

class SlipMonitor:
    """
    Detecta slip de ruedas procesando stall_mask del frame TLM (RF-004).

    Solo actúa en EXPLORE. Acumula frames TLM consecutivos con stall_mask != 0.
    Cuando el contador alcanza slip_stall_frames → override RET.

    LIMITACIÓN ARQUITECTÓNICA — ver olympus_controller.py SlipMonitor para
    descripción completa. En resumen: la transición stall→FAULT del LLC es
    atómica (state_machine/mod.rs update_safety()), por lo que cualquier frame
    con stall_mask != 0 también lleva safety="FAULT". SafeMode se activa antes
    en la cadena de prioridad y llama slip.reset(), impidiendo que el contador
    llegue a slip_stall_frames. Actualmente es dead code; se mantiene para uso
    futuro cuando el LLC implemente señalización gradual de stall (RF-004).
    """

    def __init__(self, stall_frames: int = SLIP_STALL_FRAMES):
        self._threshold: int = stall_frames
        self._count:     int = 0

    def update(self, tlm, msm_state) -> bool:
        """
        Actualiza el contador con el último frame TLM.
        Retorna True si se debe emitir RET por slip persistente.
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

class ThermalMonitor:
    """
    Supervisa temperatura ambiente desde temp_c del TLM (RNF-004).

    Umbrales:
      OK       : temp_c < TEMP_WARN_C (45 °C)
      WARN     : TEMP_WARN_C ≤ temp_c < TEMP_CRIT_C
      CRITICAL : temp_c ≥ TEMP_CRIT_C (60 °C) → activa SafeMode
    """

    def __init__(self, warn_c: int = TEMP_WARN_C, crit_c: int = TEMP_CRIT_C):
        self._warn_c: int          = warn_c
        self._crit_c: int          = crit_c
        self._level:  ThermalLevel = ThermalLevel.OK

    @property
    def level(self) -> ThermalLevel:
        return self._level

    def update(self, tlm) -> ThermalLevel:
        """Evalúa temp_c del TLM. temp_c == 0 = sin lectura → ignorar."""
        t = tlm.temp_c
        if t == 0:
            return self._level

        if t >= self._crit_c:
            self._level = ThermalLevel.CRITICAL
        elif t >= self._warn_c:
            self._level = ThermalLevel.WARN
        else:
            self._level = ThermalLevel.OK

        return self._level


# ─── Safe Mode ───────────────────────────────────────────────────────────────

class SafeMode:
    """
    Estado de seguridad HLC-only (SYS-FUN-040/041).

    Se activa ante: batería crítica, LLC en FAULT, o temperatura crítica.
    Solo STB y PING permitidos mientras esté activo.
    Sale únicamente por reset() explícito del operador (RST).
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

    def update(self, tlm, e_level: EnergyLevel,
               t_level: ThermalLevel) -> bool:
        """
        Evalúa condiciones de activación (SYS-FUN-040, RNF-004).
        Retorna True si Safe Mode está activo. Una vez activo, no se desactiva
        aquí — usar reset().
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
        """
        True si el comando debe bloquearse estando en Safe Mode.
        BNK:0 siempre permitido — corte de emergencia del relay (ICD-LLC-001).
        """
        return self._active and cmd not in ("STB", "PING", "RST", "BNK:0")


# ─── GCS Comm Link Monitor ───────────────────────────────────────────────────

class CommLinkMonitor:
    """
    Gestión de enlace GCS→HLC (SRS-013, SYS-FUN-021).

    Máquina de estados (§7.3.8):
      COMUNICAR      — enlace activo
      GESTION_ENLACE — enlace perdido; política de reintentos (TBD-OP-02)

    update() retorna el evento ocurrido en este ciclo o None.
    """

    def __init__(self,
                 link_lost_s:      float = GCS_LINK_LOST_S,
                 retry_interval_s: float = GCS_RETRY_INTERVAL_S,
                 max_retries:      int   = GCS_MAX_RETRIES):
        self._state         : CommLinkState = CommLinkState.COMUNICAR
        self._link_lost_s   : float         = link_lost_s
        self._retry_interval: float         = retry_interval_s
        self._max_retries   : int           = max_retries
        self._retry_count   : int           = 0
        self._last_retry_ts : float         = 0.0

    @property
    def state(self) -> CommLinkState:
        return self._state

    @property
    def is_lost(self) -> bool:
        return self._state == CommLinkState.GESTION_ENLACE

    @property
    def retry_count(self) -> int:
        return self._retry_count

    def update(self, last_recv_time: float, now: float,
               source=None) -> "str | None":
        """
        Evalúa el estado del enlace y ejecuta la política de reintentos.

        last_recv_time : monotonic del último paquete recibido.
        now            : monotonic actual.
        source         : CommandSource — se llama send_probe() en cada intento.

        Retorna el evento ocurrido o None.
        """
        silent_s = now - last_recv_time

        if self._state == CommLinkState.COMUNICAR:
            if silent_s > self._link_lost_s:
                self._state         = CommLinkState.GESTION_ENLACE
                self._retry_count   = 0
                self._last_retry_ts = now
                return "link_lost"
            return None

        # ── GESTION_ENLACE ──────────────────────────────────────────────────

        if silent_s <= self._link_lost_s:
            had_retries       = self._retry_count > 0
            self._state       = CommLinkState.COMUNICAR
            self._retry_count = 0
            return "reconnect_attempt_succeeded" if had_retries else "link_restored"

        if self._retry_count >= self._max_retries:
            return "max_retries_exceeded"

        if now - self._last_retry_ts >= self._retry_interval:
            self._retry_count  += 1
            self._last_retry_ts = now
            if source is not None:
                source.send_probe()
            return "reconnect_attempt_failed"

        return None
