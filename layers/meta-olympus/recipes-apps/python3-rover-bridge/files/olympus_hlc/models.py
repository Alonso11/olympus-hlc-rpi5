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
    Formato v2.15:
      TLM:<SAF>:<STALL>:<TS>ms:<MV>mV:<MA>mA:<I0..I5>:<T>C:<B0..B5>C:<DIST>mm:<EL>:<ER>:<X>:<Y>:<TH>:<DIST_FAR>mm
    (Ref. ICD LLC §Frame de telemetría extendida, SyRS-030)
    """
    safety:       str    # "NORMAL" | "WARN" | "LIMIT" | "FAULT"
    stall_mask:   int    # 6 bits: bit5=FR … bit0=RL
    tick_ms:      int    # ms desde boot del Arduino (contador monotónico)
    batt_mv:      int    # tensión batería en mV  (0 = sin lectura)
    batt_ma:      int    # corriente batería en mA con signo
    currents:     list   # [FR, FL, CR, CL, RR, RL] mA
    temp_c:       int    # temperatura ambiente °C
    batt_temps:   list   # [B1a, B1b, B2a, B2b, B3a, B3b] °C
    dist_mm:      int    # distancia VL53L0X en mm — rango 3 cm–2 m (0 = sin lectura)
    enc_left:     int    # acumulador pulsos encoder izquierdo (FL+CL+RL)
    enc_right:    int    # acumulador pulsos encoder derecho  (FR+CR+RR)
    x_mm:         int    # Posición X (EKF)
    y_mm:         int    # Posición Y (EKF)
    theta_mrad:   int    # Orientación milirrad (EKF)
    dist_far_mm:  int    # distancia TF02 en mm — rango 40 cm–22 m (0 = sin lectura)

    @staticmethod
    def parse(raw: str) -> "TlmFrame | None":
        """
        Parsea un frame TLM crudo (sin el \\n final).
        Retorna TlmFrame o None si el formato no es válido.

        Ejemplo v2.15:
          TLM:NORMAL:000000:12340ms:11800mV:2350mA:200:210:195:205:180:190:24C:25:25:26:26:25:25C:450mm:60:62:120:-45:31:3200mm
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

            # EKF fields (x_mm, y_mm, theta_mrad) are optional — older firmware
            # and test fixtures emit only 22 fields (ICD §Frame extendido ≥ v1.1).
            x_mm       = int(parts[22]) if len(parts) > 22 else 0
            y_mm       = int(parts[23]) if len(parts) > 23 else 0
            theta_mrad = int(parts[24]) if len(parts) > 24 else 0
            # dist_far_mm: TF02 LiDAR largo alcance (≥ v2.15, campo 25)
            dist_far_mm = int(parts[25].rstrip("mm")) if len(parts) > 25 else 0
            return TlmFrame(
                safety=safety, stall_mask=stall_mask, tick_ms=tick_ms,
                batt_mv=batt_mv, batt_ma=batt_ma, currents=currents,
                temp_c=temp_c, batt_temps=batt_temps, dist_mm=dist_mm,
                enc_left=enc_left, enc_right=enc_right,
                x_mm=x_mm, y_mm=y_mm, theta_mrad=theta_mrad,
                dist_far_mm=dist_far_mm,
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
    # Modo escalada: velocidad diferencial con umbrales de proximidad relajados
    # (LLC CLB_HC=60mm, CLB_TOF=50mm) y stall extendido (CLB_STALL=150 ciclos).
    # Comando: CLB:L:R → ACK:CLB. Bloqueado en FAULT/SAFE igual que EXP.
    CLIMB   = "CLB"
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


# ─── Battery Bank Mode ────────────────────────────────────────────────────────

class BankMode(enum.Enum):
    """
    Modo de banco de batería para los puentes H (LLC §relay, ICD-LLC-001).

    | BNK:N | D40 | D41 | Resultado                        |
    |-------|-----|-----|----------------------------------|
    |   0   | HI  | HI  | Ambos OFF — emergencia / apagado |
    |   2   | LO  | HI  | Bank 2 activo — operación normal |
    |   3   | HI  | LO  | Bank 3 activo — failover manual  |
    |  12   | LO  | LO  | Ambos activos — máx. corriente   |
    """
    ALL_OFF    = "0"   # emergencia: ambos bancos cortados
    BANK2_ONLY = "2"   # operación normal
    BANK3_ONLY = "3"   # failover manual
    BOTH_BANKS = "12"  # paralelo — máxima corriente
