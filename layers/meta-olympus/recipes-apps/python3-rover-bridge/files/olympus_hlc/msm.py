# olympus_hlc/msm.py — MSM mirror, dry-run mock, response parser and _send helper

import time

from .models import RoverState


# ─── Rover MSM mirror ────────────────────────────────────────────────────────

class RoverMSM:
    """
    Espejo local del estado MSM del Arduino (SyRS-060, SRS-061).
    Solo hace transición al recibir un ACK confirmado del Arduino.
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
        return time.monotonic() - self._entered

    def blocks_command(self, cmd: str) -> bool:
        """
        En FAULT o SAFE solo RST, PING y BNK:0 son válidos.
        BNK:0 corta ambos bancos de batería — siempre permitido (ICD LLC §Tabla de estados).
        """
        if self._state in (RoverState.FAULT, RoverState.SAFE):
            return cmd not in ("RST", "PING", "BNK:0")
        return False


# ─── Dry-run mock ────────────────────────────────────────────────────────────

class DryRunRover:
    """Simula respuestas del Arduino siguiendo el protocolo MSM."""

    _CMD_TO_STATE = {
        "STB": "STB", "RET": "RET", "FLT": "FLT", "SAFE": "SFE",
        "RST": "STB", "AVD:L": "AVD", "AVD:R": "AVD",
    }

    _TLM_INTERVAL_S = 1.0
    _TLM_TEMPLATE   = (
        "TLM:NORMAL:000000:{tick}ms:16000mV:500mA:"
        "100:100:100:100:100:100:25C:25:25:25:25:25:25C:1000mm:0:0:0:0:0"
    )

    def __init__(self):
        self._state       = "STB"
        self._tick_ms     = 0
        self._last_tlm_ts = time.monotonic()

    def send_command(self, cmd: str) -> str:
        if cmd == "PING":
            return "PONG"
        if cmd.startswith("EXP:"):
            self._state = "EXP"
            return "ACK:EXP"
        if cmd.startswith("BNK:"):
            bank = cmd[4:]
            if bank in ("0", "2", "3", "12"):
                return f"ACK:BNK:{bank}"
            return "ERR:UNKNOWN"
        new_state = self._CMD_TO_STATE.get(cmd)
        if new_state is not None:
            self._state = new_state
            return f"ACK:{new_state}"
        return "ERR:UNKNOWN"

    def recv_tlm(self) -> "str | None":
        """
        Emite un frame TLM sintético cada _TLM_INTERVAL_S segundos para evitar
        que el watchdog de link-loss se dispare durante pruebas dry-run.
        """
        now = time.monotonic()
        if now - self._last_tlm_ts >= self._TLM_INTERVAL_S:
            self._tick_ms    += int((now - self._last_tlm_ts) * 1000)
            self._last_tlm_ts = now
            return self._TLM_TEMPLATE.format(tick=self._tick_ms)
        return None


# ─── Response parser ─────────────────────────────────────────────────────────

def parse_response(resp: str) -> "tuple[str, str | None]":
    """
    Parsea la respuesta raw del Arduino.

    Retorna uno de:
      ("pong",        None)
      ("ack",         "<STATE>")   e.g. "STB","EXP","AVD","RET","FLT"
      ("err_estop",   None)
      ("err_wdog",    None)
      ("err_unknown", None)
      ("unknown",     <raw>)
    """
    if resp == "PONG":
        return ("pong", None)
    if resp.startswith("ACK:BNK:"):
        return ("bank_ack", resp[8:])   # "0", "2", "3" or "12"
    if resp.startswith("ACK:"):
        return ("ack", resp[4:])
    if resp == "ERR:ESTOP":
        return ("err_estop", None)
    if resp == "ERR:WDOG":
        return ("err_wdog", None)
    if resp == "ERR:UNKNOWN":
        return ("err_unknown", None)
    return ("unknown", resp)


# ─── Send helper ─────────────────────────────────────────────────────────────

def _send(rover, cmd: str, log=None) -> "tuple[str, str | None]":
    """
    Envía cmd al rover, parsea la respuesta y retorna (kind, data).
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
