# olympus_hlc/engine.py — HlcEngine: main control loop (refactored from run())
#
# La función run() original (~250 líneas) se divide en métodos con
# responsabilidad única, eliminando los isinstance() y facilitando los tests.

import os
import time

from .config import (
    PING_INTERVAL_S,
    TLM_WARN_S, TLM_RETREAT_S, TLM_STB_S,
    TLM_INTERVAL_WARN_S,
    CYCLE_WARN_MS, CYCLE_LOG_PERIOD,
    STORAGE_MIN_MB, STORAGE_CHECK_CYCLES,
    RETREAT_DIST_MM,
    GCS_LINK_LOST_S, GCS_MAX_RETRIES,
    POWEROFF_DELAY_S, POWEROFF_ENABLED,
)
from .interfaces import CommandSource
from .logger import OlympusLogger
from .models import BankMode, EnergyLevel, RoverState, ThermalLevel, TlmFrame
from .monitors import (
    EnergyMonitor, SafeMode, SlipMonitor, ThermalMonitor, WaypointTracker,
)
from .msm import RoverMSM, _send
from .odometry import OdometryTracker


class HlcEngine:
    """
    Motor principal del HLC. Orquesta el bucle de control sin conocer los
    tipos concretos de fuente de comandos (SRP, DIP, OCP).

    Prioridad de overrides en cada ciclo:
      1. GCS link lost (STB permanente)
      2. Safe Mode (STB)
      3. Retreat táctica (RET)
      4. Slip detectado (RET)
      5. TLM link loss escalation (STB | RET)
      6. Comando de la fuente
    """

    def __init__(self, rover, source: CommandSource, mode: str,
                 log_path: str = OlympusLogger.DEFAULT_LOG_PATH,
                 bench: bool = False):
        self._rover  = rover
        self._source = source
        self._mode   = mode
        self._log    = OlympusLogger(log_path)

        # Banco de caracterización: el HLC lee y registra la telemetría
        # (encoders/corrientes para medir) pero NUNCA inyecta overrides
        # autónomos (RET/STB por TLM-loss, retreat táctico, slip, SafeMode,
        # ni STB del monitor de enlace GCS). El operador maneja el rover
        # directamente; la MSM del firmware sigue protegiendo el hardware.
        # Se activa siempre en modo manual, o con --bench en cualquier modo
        # (p.ej. 'station' para manejar desde la GUI sin que el watchdog
        # pise los comandos). Esto DESACTIVA las protecciones autónomas del
        # HLC — usar solo para pruebas/caracterización, nunca en campo.
        self._manual_bench = (mode == "manual") or bench

        # MSM y monitores
        self._msm         = RoverMSM()
        self._tracker     = WaypointTracker()
        self._energy      = EnergyMonitor()
        self._slip        = SlipMonitor()
        self._thermal     = ThermalMonitor()
        self._safe_mode   = SafeMode()
        self._odometry    = OdometryTracker()
        self._prev_energy  = EnergyLevel.OK
        self._prev_thermal = ThermalLevel.OK

        # Timing
        self._last_cmd_time  = time.monotonic()
        self._last_tlm_time  = time.monotonic()
        self._last_tlm_ts    = time.monotonic()
        self._tlm_loss_level = 0   # 0=ok  1=warn  2=retreat  3=stb
        self._override_reason: "str | None" = None  # razón del último override (→GUI)

        # Storage check
        self._cycle_count        = 0
        self._last_storage_check = 0

        # Power management (SYS-FUN-040)
        self._poweroff_at: float = 0.0  # monotonic del apagado programado, 0 = ninguno

        # Bank mode mirror (ICD-LLC-001 §relay)
        self._bank_mode: BankMode = BankMode.BANK2_ONLY

        # CommLink (solo para GCSSource, None para las demás)
        self._comm_link     = source.make_link_monitor()
        self._gcs_stb_forced = False

    # ── Bucle principal ───────────────────────────────────────────────────────

    def run(self) -> None:
        self._log.info("CTRL", f"Starting in {self._mode.upper()} mode")
        if self._manual_bench and self._mode != "manual":
            self._log.warn(
                "CTRL",
                f"BANCO DE CARACTERIZACIÓN ({self._mode}) — overrides autónomos "
                f"del HLC DESACTIVADOS (RET/STB/SafeMode/enlace). Solo pruebas."
            )
        try:
            while True:
                # Power management: apagar OS si la batería activó SafeMode (SYS-FUN-040)
                if self._poweroff_at > 0 and time.monotonic() >= self._poweroff_at:
                    self._trigger_poweroff()

                cycle_start  = time.monotonic()
                tlm_override = self._tick_telemetry()
                if tlm_override is not None:
                    cmd    = tlm_override
                    reason = self._override_reason or f"override HLC → {tlm_override}"
                else:
                    cmd    = self._source.next_command(self._log)
                    reason = None
                self._override_reason = None

                # Vision: error de cámara → STB seguro
                if cmd is None and self._mode == "vision":
                    self._log.warn("CTRL", "Camera error — sending STB")
                    cmd = "STB"

                if cmd is not None:
                    self._dispatch(cmd)
                    # Eco del comando REAL (con override) + razón a la fuente/GUI
                    # (evidencia). No-op salvo en StationSource.
                    self._source.on_dispatch(cmd, reason)

                self._keepalive()

                # Vision: pausa entre frames (~20 Hz máximo)
                if self._mode == "vision":
                    time.sleep(0.05)

                self._check_cycle(cycle_start)
                self._check_storage()

        except (KeyboardInterrupt, SystemExit):
            self._shutdown()
        finally:
            self._log.close()

    # ── Telemetría y CommLink ─────────────────────────────────────────────────

    def _tick_telemetry(self) -> "str | None":
        """
        Drena TLM, reenvía al peer (on_tlm), actualiza CommLink y monitores.
        Retorna el override de comando más prioritario, o None.
        """
        raw_tlm = self._rover.recv_tlm()
        tlm_override = None

        if raw_tlm:
            self._source.on_tlm(raw_tlm)  # GCSSource reenvía; otros no hacen nada

        # Banco de caracterización (modo manual): registrar TLM para medir
        # encoders/corrientes, pero sin ningún override autónomo. El operador
        # tiene control total — no hay RET/STB inyectado por el HLC.
        # Nivel MANUAL (selector de la GUI) = control total del operador: sin
        # ningún override autónomo del HLC, igual que el banco de caracterización.
        if self._manual_bench or self._source.safety_level == "MANUAL":
            if raw_tlm:
                self._log_tlm_passive(raw_tlm)
            return None

        # CommLink — solo activo cuando la fuente tiene monitor (GCSSource)
        if self._comm_link is not None:
            now        = time.monotonic()
            link_event = self._comm_link.update(
                self._source.last_recv_time, now, self._source
            )
            self._handle_link_event(link_event)
            # Robustez: si el enlace está vivo de nuevo (monitor en COMUNICAR),
            # levantar el STB forzado aunque no se haya emitido el evento de
            # recuperación en este ciclo (p.ej. un cliente nuevo que refresca
            # last_recv tras un período idle que latcheó max_retries_exceeded).
            # Sin esto, el station queda pegado en STB y nunca despacha EXP.
            if self._gcs_stb_forced and not self._comm_link.is_lost:
                self._gcs_stb_forced = False
            if self._gcs_stb_forced:
                tlm_override = "STB"

        # Monitores TLM — prioridad #1 es GCS link lost (STB); monitores no pueden
        # sobreescribirlo porque "RET" táctica < "STB" por enlace perdido (SRS-013).
        if raw_tlm:
            monitor_override = self._process_tlm_frame(raw_tlm)
            if monitor_override is not None and tlm_override is None:
                tlm_override = monitor_override
        else:
            loss_override = self._handle_tlm_loss()
            if loss_override is not None and tlm_override is None:
                tlm_override = loss_override

        return tlm_override

    def _log_tlm_passive(self, raw_tlm: str) -> None:
        """
        Registra TLM y odometría SIN activar monitores ni overrides.
        Usado en modo manual (caracterización): conserva la evidencia de
        encoders/corrientes en el log y la odometría, pero el HLC no toma
        ninguna decisión autónoma sobre el movimiento.
        """
        self._last_tlm_time = time.monotonic()
        tlm = TlmFrame.parse(raw_tlm)
        if tlm is None:
            return
        self._log.log_tlm(tlm)
        self._tracker.record(tlm, self._msm.state)
        self._odometry.update(tlm.enc_left, tlm.enc_right)

    def _handle_link_event(self, event: "str | None") -> None:
        """Loguea y actúa sobre eventos del CommLinkMonitor."""
        if event is None:
            return

        if event == "link_lost":
            self._log.log_link_event(
                event,
                f">{GCS_LINK_LOST_S:.0f}s sin paquete GCS — "
                f"transición a GestiónEnlace (SRS-013)"
            )
        elif event in ("link_restored", "reconnect_attempt_succeeded"):
            self._log.log_link_event(
                event, "retorno a Comunicar (SRS-013)"
            )
            self._gcs_stb_forced = False
        elif event == "reconnect_attempt_failed":
            self._log.log_link_event(
                event,
                f"intento {self._comm_link.retry_count}/{GCS_MAX_RETRIES} "
                f"— HB_REQ enviado"
            )
        elif event == "max_retries_exceeded":
            if not self._gcs_stb_forced:
                self._log.log_link_event(
                    event,
                    f"reintentos agotados ({GCS_MAX_RETRIES}) — "
                    f"forzando STB permanente (SRS-013)"
                )
                self._gcs_stb_forced = True

    def _process_tlm_frame(self, raw_tlm: str) -> "str | None":
        """
        Parsea el frame TLM, actualiza todos los monitores y retorna override o None.
        Prioridad: SafeMode > retreat táctica > slip.
        """
        self._last_tlm_time = time.monotonic()
        if self._tlm_loss_level > 0:
            self._log.info("COMM", "TLM restablecido — enlace recuperado")
            self._tlm_loss_level = 0

        tlm = TlmFrame.parse(raw_tlm)
        if tlm is None:
            return None

        self._log.log_tlm(tlm)
        self._tracker.record(tlm, self._msm.state)
        self._odometry.update(tlm.enc_left, tlm.enc_right)

        # SyRS-017 — verificar frecuencia TLM ≥ 1 Hz
        now_ts      = time.monotonic()
        tlm_delta_s = now_ts - self._last_tlm_ts
        self._last_tlm_ts = now_ts
        if tlm_delta_s > TLM_INTERVAL_WARN_S:
            self._log.warn(
                "COMM",
                f"TLM tardío: delta={tlm_delta_s:.1f} s "
                f"(esperado ≤ {TLM_INTERVAL_WARN_S:.0f} s) — "
                f"posible degradación de enlace (SyRS-017)"
            )

        # Energía — logea solo al cambiar de nivel (EPS-REQ-001)
        e_level = self._energy.update(tlm)
        if e_level != self._prev_energy:
            self._log.log_energy(e_level, tlm.batt_mv)
            self._prev_energy = e_level

        # Térmica — logea solo al cambiar de nivel (RNF-004)
        t_level = self._thermal.update(tlm)
        if t_level != self._prev_thermal:
            lvl_str = t_level.value
            msg = f"temperatura {tlm.temp_c} °C — nivel {lvl_str}"
            if t_level in (ThermalLevel.CRITICAL, ThermalLevel.WARN):
                self._log.warn("THERM", msg)
            else:
                self._log.info("THERM", msg)
            self._prev_thermal = t_level

        # Prioridad SafeMode > retreat > slip
        if self._safe_mode.update(tlm, e_level, t_level):
            if self._safe_mode.just_activated:
                self._log.warn(
                    "EPS",
                    f"SAFE MODE activado — {self._safe_mode.reason} "
                    f"(SYS-FUN-040) — solo STB/PING permitidos"
                )
                self._slip.reset()
                # Batería crítica → programar apagado del OS (SYS-FUN-040).
                # El delay da tiempo al LLC para procesar SAFE y al log para
                # sincronizarse antes de cortar la alimentación.
                if "batería" in self._safe_mode.reason and self._poweroff_at == 0.0:
                    self._poweroff_at = time.monotonic() + POWEROFF_DELAY_S
                    self._log.warn(
                        "EPS",
                        f"Poweroff programado en {POWEROFF_DELAY_S} s "
                        f"(POWEROFF_ENABLED={POWEROFF_ENABLED})"
                    )
                # Primera activación: notificar al LLC via Command::Safe (ICD-LLC-001).
                # El LLC entra en RoverState::Safe y bloquea todo movimiento hasta RST.
                # En ciclos posteriores (just_activated=False), solo el keepalive PING
                # del engine loop llega al LLC — sin comandos adicionales.
                # Cortar bancos de batería (relay) — BNK:0 siempre permitido en FAULT/SAFE.
                _send(self._rover, "BNK:0", self._log)
                self._bank_mode = BankMode.ALL_OFF
                self._log.warn("EPS", "BNK:0 enviado — relay: ambos bancos OFF")
                self._override_reason = f"SafeMode: {self._safe_mode.reason}"
                return "SAFE"
            self._slip.reset()
            return None  # Safe Mode ya activo: engine solo envía PING keepalive

        # En CLIMB el terreno inclinado queda a < 300 mm del sensor frontal —
        # suprimir el retreat táctico para evitar falsos positivos (CLB_TOF=50mm).
        # Retreat y slip son RET PROACTIVOS (mueven el rover solo): solo en nivel
        # PLENA. En ASISTIDA el operador no quiere retrocesos sorpresa; SafeMode y
        # link-loss (arriba/abajo) sí se mantienen.
        full = self._source.safety_level == "FULL"

        if full and self._msm.state != RoverState.CLIMB and self._tracker.should_retreat(tlm):
            wp = self._tracker.last_safe()
            wp_info = (f"last_safe tick={wp.tick_ms}ms dist={wp.dist_mm}mm"
                       if wp else "no waypoint previo")
            self._log.warn(
                "NAV",
                f"obstáculo táctico a {tlm.dist_mm} mm "
                f"(< {RETREAT_DIST_MM} mm) — forzando RET [{wp_info}]"
            )
            self._override_reason = (
                f"retreat: obstáculo a {tlm.dist_mm} mm (< {RETREAT_DIST_MM} mm)")
            self._slip.reset()
            return "RET"

        if full and self._slip.update(tlm, self._msm.state):
            self._log.warn(
                "NAV",
                f"slip detectado — stall_mask={tlm.stall_mask:06b} "
                f"durante {self._slip.stall_count} frames TLM — forzando RET (RF-004)"
            )
            self._override_reason = (
                f"slip: stall_mask={tlm.stall_mask:06b} ({self._slip.stall_count} frames)")
            return "RET"

        return None

    def _handle_tlm_loss(self) -> "str | None":
        """
        Escalado de pérdida de enlace TLM (SYS-FUN-021, COMM-REQ-005).
        Retorna override "STB" o "RET" según el tiempo sin TLM, o None.
        """
        silent_s = time.monotonic() - self._last_tlm_time

        if silent_s > TLM_STB_S:
            if self._tlm_loss_level < 3:
                self._log.warn(
                    "COMM",
                    f"sin TLM por {TLM_STB_S:.0f}+ s — "
                    f"forzando STB definitivo (COMM-REQ-005)"
                )
                self._tlm_loss_level = 3
            self._override_reason = f"link-loss: sin TLM >{TLM_STB_S:.0f}s → STB"
            return "STB"

        # El RET por TLM-loss (retroceder al último waypoint) es proactivo → solo
        # en PLENA. En ASISTIDA el STB de arriba sigue protegiendo, sin retroceso.
        if self._source.safety_level == "FULL" and silent_s > TLM_RETREAT_S:
            if self._tlm_loss_level < 2:
                wp = self._tracker.last_safe()
                self._log.warn(
                    "COMM",
                    f"sin TLM por {TLM_RETREAT_S:.0f}+ s — "
                    f"RET al último waypoint seguro {wp} (SYS-FUN-021)"
                )
                self._tlm_loss_level = 2
            self._override_reason = f"link-loss: sin TLM >{TLM_RETREAT_S:.0f}s → RET waypoint"
            return "RET"

        if silent_s > TLM_WARN_S:
            if self._tlm_loss_level < 1:
                self._log.warn("COMM",
                               f"sin TLM por {TLM_WARN_S:.0f}+ s — enlace degradado")
                self._tlm_loss_level = 1

        return None

    # ── Despacho de comandos ──────────────────────────────────────────────────

    def _dispatch(self, cmd: str) -> None:
        """Envía el comando al Arduino, maneja ACK/ERR y actualiza el MSM."""
        if self._safe_mode.blocks_command(cmd):
            self._log.warn(
                "CMD",
                f"{cmd:<16} → BLOCKED (Safe Mode activo — {self._safe_mode.reason})"
            )
            return

        if self._msm.blocks_command(cmd) and not self._manual_bench:
            self._log.warn("CMD", f"{cmd:<16} → BLOCKED (rover in FAULT, send RST)")
            return

        kind, data = _send(self._rover, cmd, self._log)
        self._last_cmd_time = time.monotonic()

        if kind == "bank_ack" and data is not None:
            try:
                self._bank_mode = BankMode(data)
                self._log.info("EPS", f"relay → BNK:{data} ({self._bank_mode.name})")
            except ValueError:
                self._log.warn("CMD", f"ACK:BNK:{data} — valor desconocido")

        elif kind == "ack" and data is not None:
            new_state = RoverState.from_ack(data)
            if new_state is not None:
                self._log.log_transition(self._msm.state, new_state, f"ACK:{data}")
                self._msm.transition(new_state)
            if cmd == "RST":
                self._safe_mode.reset()
                self._log.info("EPS", "Safe Mode desactivado por RST del operador")
                # Restaurar banco primario tras salir de SAFE/FAULT (ICD-LLC-001).
                _send(self._rover, "BNK:2", self._log)
                self._bank_mode = BankMode.BANK2_ONLY
                self._log.info("EPS", "relay → BNK:2 (Bank2Only restaurado)")

        elif kind == "err_wdog":
            self._log.log_transition(
                self._msm.state, RoverState.FAULT, "ERR:WDOG", warn=True
            )
            self._msm.transition(RoverState.FAULT)
            self._log.info("CTRL", "Auto-sending RST to recover from watchdog")
            kind2, data2 = _send(self._rover, "RST", self._log)
            if kind2 == "ack" and data2 is not None:
                new_state = RoverState.from_ack(data2)
                if new_state is not None:
                    self._log.log_transition(self._msm.state, new_state, "RST")
                    self._msm.transition(new_state)

        elif kind == "err_estop":
            self._log.log_transition(
                self._msm.state, RoverState.FAULT, "ERR:ESTOP", warn=True
            )
            self._msm.transition(RoverState.FAULT)

        elif kind == "err_unknown":
            self._log.warn(
                "CMD",
                f"{cmd:<16} → ERR:UNKNOWN (comando no reconocido por firmware)"
            )

    # ── Auxiliares del bucle ──────────────────────────────────────────────────

    def _keepalive(self) -> None:
        """Envía PING si no se envió ningún comando en los últimos PING_INTERVAL_S."""
        if time.monotonic() - self._last_cmd_time >= PING_INTERVAL_S:
            _send(self._rover, "PING", self._log)
            self._last_cmd_time = time.monotonic()

    def _check_cycle(self, cycle_start: float) -> None:
        """Mide y loguea el tiempo de ciclo (RNF-001: ≤ 2000 ms)."""
        cycle_ms = (time.monotonic() - cycle_start) * 1000
        self._cycle_count += 1
        if cycle_ms > CYCLE_WARN_MS:
            self._log.warn(
                "CYCLE",
                f"ciclo lento: {cycle_ms:.1f} ms (umbral {CYCLE_WARN_MS} ms, RNF-001)"
            )
        elif self._cycle_count % CYCLE_LOG_PERIOD == 0:
            self._log.log_cycle(cycle_ms)

    def _check_storage(self) -> None:
        """Verifica espacio en disco cada STORAGE_CHECK_CYCLES ciclos (SRS-014)."""
        if self._cycle_count - self._last_storage_check < STORAGE_CHECK_CYCLES:
            return
        self._last_storage_check = self._cycle_count
        try:
            log_dir = os.path.dirname(OlympusLogger.DEFAULT_LOG_PATH)
            st      = os.statvfs(log_dir)
            free_mb = (st.f_bavail * st.f_frsize) / 1_000_000
            if free_mb < STORAGE_MIN_MB:
                self._log.warn(
                    "CDH",
                    f"espacio en disco bajo: {free_mb:.1f} MB libres "
                    f"(mínimo {STORAGE_MIN_MB} MB) — "
                    f"riesgo de pérdida de logs (SRS-014)"
                )
        except OSError:
            pass

    def _trigger_poweroff(self) -> None:
        """
        Apagado seguro del OS ante batería crítica (SYS-FUN-040).

        Secuencia:
          1. STB al LLC + sync de logs (_shutdown)
          2. systemctl poweroff — apaga el OS limpiamente para proteger la SD
          3. SystemExit(0) como red de seguridad si poweroff tarda o está desactivado

        POWEROFF_ENABLED=False: omite el systemctl (dry-run / tests).
        """
        self._log.warn(
            "EPS",
            f"POWEROFF — apagando sistema operativo "
            f"(POWEROFF_ENABLED={POWEROFF_ENABLED}, SYS-FUN-040)"
        )
        self._shutdown()
        if POWEROFF_ENABLED:
            import subprocess
            subprocess.run(["systemctl", "poweroff"], check=False)
        raise SystemExit(0)

    def _shutdown(self) -> None:
        """Secuencia de apagado seguro (SYS-FUN-050, SYS-FUN-051)."""
        self._log.info("CTRL", "Shutdown iniciado — enviando STB (SYS-FUN-051)")
        parked = False
        try:
            resp = self._rover.send_command("STB")
            if isinstance(resp, str) and "ACK:STB" in resp:
                parked = True
                self._log.info("CTRL", "Parking confirmado (ACK:STB)")
            else:
                self._log.warn(
                    "CTRL",
                    f"ACK:STB no recibido (resp={resp!r}) — "
                    f"asumiendo parado por timeout"
                )
        except Exception as exc:
            self._log.warn("CTRL", f"Error enviando STB en shutdown: {exc}")

        self._log.info(
            "CTRL",
            f"READY_FOR_POWEROFF — parked={parked} "
            f"(logs sincronizados a almacenamiento no volátil)"
        )
        print("READY_FOR_POWEROFF")
