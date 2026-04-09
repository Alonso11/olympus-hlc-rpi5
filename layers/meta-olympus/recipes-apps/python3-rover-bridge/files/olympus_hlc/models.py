# olympus_hlc/models.py — Pure data models (dataclasses and enums)
#
# No lógica de negocio aquí. Todos los módulos pueden importar desde este
# archivo sin riesgo de dependencias circulares.

import dataclasses
import enum


# ─── Telemetry Frame ─────────────────────────────────────────────────────────

@dataclasses.dataclass
class TlmFrame:
    """
    Frame de telemetría extendida emitido por el Arduino (~1 s).
    Formato: TLM:<SAF>:<STALL>:<TS>ms:<MV>mV:<MA>mA:<I0>:<I1>:<I2>:<I3>:<I4>:<I5>:<T>C:<B0>:<B1>:<B2>:<B3>:<B4>:<B5>C:<DIST>mm:<EL>:<ER>
    (Ref. ICD LLC §Frame de telemetría extendida, SyRS-030)
    """
    safety:     str    # "NORMAL" | "WARN" | "LIMIT" | "FAULT"
    stall_mask: int    # 6 bits: bit5=FR … bit0=RL
    tick_ms:    int    # ms desde boot del Arduino (contador monotónico)
    batt_mv:    int    # tensión batería en mV  (0 = sin lectura)
    batt_ma:    int    # corriente batería en mA con signo
    currents:   list   # [FR, FL, CR, CL, RR, RL] mA
    temp_c:     int    # temperatura ambiente °C
    batt_temps: list   # [B1a, B1b, B2a, B2b, B3a, B3b] °C
    dist_mm:    int    # distancia ToF en mm (0 = sin lectura)
    enc_left:   int    # acumulador pulsos encoder izquierdo (FL+CL+RL)
    enc_right:  int    # acumulador pulsos encoder derecho  (FR+CR+RR)
    x_mm:       int    # Posición X (EKF)
    y_mm:       int    # Posición Y (EKF)
    theta_mrad: int    # Orientación milirrad (EKF)

    @staticmethod
    def parse(raw: str) -> "TlmFrame | None":
        """
        Parsea un frame TLM crudo (sin el \\n final).
        Retorna TlmFrame o None si el formato no es válido.

        Ejemplo:
          TLM:NORMAL:000000:12340ms:11800mV:2350mA:200:210:195:205:180:190:24C:25:25:26:26:25:25C:450mm:60:62
        """
        try:
            parts = raw.split(":")
            if len(parts) < 22 or parts[0] != "TLM":
                return None

            safety     = parts[1]
            stall_mask = int(parts[2], 2)
            tick_ms    = int(parts[3].rstrip("ms"))
            batt_mv    = int(parts[4].rstrip("mV"))
            batt_ma    = int(parts[5].rstrip("mA"))
            currents   = [int(parts[i]) for i in range(6, 12)]
            temp_c     = int(parts[12].rstrip("C"))
            batt_temps = [int(parts[i]) for i in range(13, 18)] + \
                         [int(parts[18].rstrip("C"))]
            dist_mm    = int(parts[19].rstrip("mm"))
            enc_left   = int(parts[20])
            enc_right  = int(parts[21])

            return TlmFrame(
                safety=safety, stall_mask=stall_mask, tick_ms=tick_ms,
                batt_mv=batt_mv, batt_ma=batt_ma, currents=currents,
                temp_c=temp_c, batt_temps=batt_temps, dist_mm=dist_mm,
                enc_left=enc_left, enc_right=enc_right,
                x_mm=int(parts[22]), y_mm=int(parts[23]),
                theta_mrad=int(parts[24]),
            )
        except (ValueError, IndexError):
            return None


# ─── Waypoint ────────────────────────────────────────────────────────────────

@dataclasses.dataclass
class Waypoint:
    """Instantánea de un punto seguro registrado durante exploración (SyRS-061)."""
    tick_ms: int    # timestamp Arduino en ms
    state:   object # RoverState en el momento del registro
    dist_mm: int    # distancia frontal ToF en mm
    batt_mv: int    # tensión batería en mV


# ─── Rover State Machine ─────────────────────────────────────────────────────

class RoverState(enum.Enum):
    STANDBY = "STB"
    EXPLORE = "EXP"
    AVOID   = "AVD"
    RETREAT = "RET"
    FAULT   = "FLT"
    # Safe Mode: iniciado por HLC ante batería/temperatura crítica (SYS-FUN-040).
    # El LLC bloquea todos los comandos de movimiento hasta RST explícito.
    # Diferencia con FAULT: SAFE es energético/térmico, no un fallo hardware del LLC.
    SAFE    = "SFE"

    @staticmethod
    def from_ack(label: str) -> "RoverState | None":
        for s in RoverState:
            if s.value == label:
                return s
        return None


# ─── Energy / Thermal / Comm enums ───────────────────────────────────────────

class EnergyLevel(enum.Enum):
    OK       = "OK"
    WARN     = "WARN"
    CRITICAL = "CRITICAL"


class ThermalLevel(enum.Enum):
    OK       = "OK"
    WARN     = "WARN"
    CRITICAL = "CRITICAL"


class CommLinkState(enum.Enum):
    COMUNICAR      = "Comunicar"
    GESTION_ENLACE = "GestiónEnlace"
