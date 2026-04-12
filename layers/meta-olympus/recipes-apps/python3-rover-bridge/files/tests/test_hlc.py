"""
Tests unitarios para olympus_hlc — ejecutar con:
    cd files/
    pytest tests/ -v
"""

import sys
import time
import pytest

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent.parent))

import math

from olympus_hlc.models import (
    TlmFrame, Waypoint, RoverState, EnergyLevel, ThermalLevel, CommLinkState,
)
from olympus_hlc.odometry import OdometryTracker
from olympus_hlc.csp import CSPPacket
from olympus_hlc.msm import RoverMSM, DryRunRover, parse_response, _send
from olympus_hlc.monitors import (
    EnergyMonitor, SlipMonitor, ThermalMonitor, SafeMode,
    WaypointTracker, CommLinkMonitor,
)
from olympus_hlc.engine import HlcEngine


# ─── Fixtures ────────────────────────────────────────────────────────────────

TLM_NORMAL = (
    "TLM:NORMAL:000000:12340ms:15000mV:500mA:"
    "100:100:100:100:100:100:25C:25:25:25:25:25:25C:800mm:0:0"
)
TLM_CRITICAL_BATT = (
    "TLM:NORMAL:000000:1000ms:12000mV:500mA:"
    "100:100:100:100:100:100:25C:25:25:25:25:25:25C:800mm:0:0"
)
TLM_WARN_BATT = (
    "TLM:NORMAL:000000:1000ms:13500mV:500mA:"
    "100:100:100:100:100:100:25C:25:25:25:25:25:25C:800mm:0:0"
)
TLM_HOT = (
    "TLM:NORMAL:000000:1000ms:15000mV:500mA:"
    "100:100:100:100:100:100:65C:25:25:25:25:25:25C:800mm:0:0"
)
TLM_WARM = (
    "TLM:NORMAL:000000:1000ms:15000mV:500mA:"
    "100:100:100:100:100:100:48C:25:25:25:25:25:25C:800mm:0:0"
)
TLM_LLC_FAULT = (
    "TLM:FAULT:000000:1000ms:15000mV:500mA:"
    "100:100:100:100:100:100:25C:25:25:25:25:25:25C:800mm:0:0"
)
TLM_STALL = (
    "TLM:NORMAL:100000:1000ms:15000mV:500mA:"
    "100:100:100:100:100:100:25C:25:25:25:25:25:25C:150mm:0:0"
)
TLM_CLOSE = (
    "TLM:NORMAL:000000:1000ms:15000mV:500mA:"
    "100:100:100:100:100:100:25C:25:25:25:25:25:25C:200mm:0:0"
)
TLM_NO_READING = (
    "TLM:NORMAL:000000:1000ms:0mV:0mA:"
    "0:0:0:0:0:0:0C:0:0:0:0:0:0C:0mm:0:0"
)


def parse(raw: str) -> TlmFrame:
    tlm = TlmFrame.parse(raw)
    assert tlm is not None, f"parse failed for: {raw}"
    return tlm


# ─── TlmFrame ────────────────────────────────────────────────────────────────

class TestTlmFrame:

    def test_parse_valid(self):
        tlm = parse(TLM_NORMAL)
        assert tlm.safety    == "NORMAL"
        assert tlm.stall_mask == 0
        assert tlm.tick_ms   == 12340
        assert tlm.batt_mv   == 15000
        assert tlm.batt_ma   == 500
        assert tlm.currents  == [100, 100, 100, 100, 100, 100]
        assert tlm.temp_c    == 25
        assert tlm.batt_temps == [25, 25, 25, 25, 25, 25]
        assert tlm.dist_mm   == 800
        assert tlm.enc_left  == 0
        assert tlm.enc_right == 0

    def test_parse_enc_nonzero(self):
        raw = (
            "TLM:NORMAL:000000:1000ms:15000mV:500mA:"
            "100:100:100:100:100:100:25C:25:25:25:25:25:25C:800mm:120:-45"
        )
        tlm = TlmFrame.parse(raw)
        assert tlm is not None
        assert tlm.enc_left  == 120
        assert tlm.enc_right == -45

    def test_parse_returns_none_old_format(self):
        # Frame de 20 campos (sin enc) debe retornar None
        old = (
            "TLM:NORMAL:000000:1000ms:15000mV:500mA:"
            "100:100:100:100:100:100:25C:25:25:25:25:25:25C:800mm"
        )
        assert TlmFrame.parse(old) is None

    def test_parse_stall_mask(self):
        tlm = parse(TLM_STALL)
        assert tlm.stall_mask == 0b100000  # bit5=FR activo

    def test_parse_returns_none_bad_prefix(self):
        assert TlmFrame.parse("FOO:NORMAL:000000:0ms:0mV:0mA:0:0:0:0:0:0:0C:0:0:0:0:0:0C:0mm") is None

    def test_parse_returns_none_too_few_fields(self):
        assert TlmFrame.parse("TLM:NORMAL:000000") is None

    def test_parse_returns_none_garbage(self):
        assert TlmFrame.parse("not a tlm frame at all") is None

    def test_parse_returns_none_empty(self):
        assert TlmFrame.parse("") is None

    def test_parse_zero_readings(self):
        tlm = parse(TLM_NO_READING)
        assert tlm.batt_mv == 0
        assert tlm.dist_mm == 0
        assert tlm.temp_c  == 0


# ─── RoverState ──────────────────────────────────────────────────────────────

class TestRoverState:

    def test_from_ack_all_states(self):
        assert RoverState.from_ack("STB") == RoverState.STANDBY
        assert RoverState.from_ack("EXP") == RoverState.EXPLORE
        assert RoverState.from_ack("AVD") == RoverState.AVOID
        assert RoverState.from_ack("RET") == RoverState.RETREAT
        assert RoverState.from_ack("FLT") == RoverState.FAULT

    def test_from_ack_unknown_returns_none(self):
        assert RoverState.from_ack("XXX") is None
        assert RoverState.from_ack("")    is None


# ─── EnergyMonitor ───────────────────────────────────────────────────────────

class TestEnergyMonitor:

    def setup_method(self):
        self.em = EnergyMonitor(warn_mv=14000, critical_mv=12800)

    def test_ok_level(self):
        assert self.em.update(parse(TLM_NORMAL)) == EnergyLevel.OK

    def test_warn_level(self):
        assert self.em.update(parse(TLM_WARN_BATT)) == EnergyLevel.WARN

    def test_critical_level(self):
        assert self.em.update(parse(TLM_CRITICAL_BATT)) == EnergyLevel.CRITICAL

    def test_zero_mv_ignored(self):
        self.em.update(parse(TLM_CRITICAL_BATT))
        assert self.em.level == EnergyLevel.CRITICAL
        # batt_mv=0 → sin lectura, el nivel no cambia
        assert self.em.update(parse(TLM_NO_READING)) == EnergyLevel.CRITICAL

    def test_level_property(self):
        self.em.update(parse(TLM_WARN_BATT))
        assert self.em.level == EnergyLevel.WARN


# ─── ThermalMonitor ──────────────────────────────────────────────────────────

class TestThermalMonitor:

    def setup_method(self):
        self.tm = ThermalMonitor(warn_c=45, crit_c=60)

    def test_ok_level(self):
        assert self.tm.update(parse(TLM_NORMAL)) == ThermalLevel.OK

    def test_warn_level(self):
        assert self.tm.update(parse(TLM_WARM)) == ThermalLevel.WARN

    def test_critical_level(self):
        assert self.tm.update(parse(TLM_HOT)) == ThermalLevel.CRITICAL

    def test_zero_temp_ignored(self):
        self.tm.update(parse(TLM_HOT))
        assert self.tm.level == ThermalLevel.CRITICAL
        assert self.tm.update(parse(TLM_NO_READING)) == ThermalLevel.CRITICAL


# ─── SlipMonitor ─────────────────────────────────────────────────────────────

class TestSlipMonitor:

    def setup_method(self):
        self.sm = SlipMonitor(stall_frames=2)

    def test_no_slip_in_explore(self):
        tlm = parse(TLM_NORMAL)
        assert not self.sm.update(tlm, RoverState.EXPLORE)

    def test_slip_triggers_after_threshold(self):
        tlm = parse(TLM_STALL)
        assert not self.sm.update(tlm, RoverState.EXPLORE)  # frame 1
        assert     self.sm.update(tlm, RoverState.EXPLORE)  # frame 2 → threshold

    def test_no_slip_outside_explore(self):
        tlm = parse(TLM_STALL)
        self.sm.update(tlm, RoverState.EXPLORE)
        # Fuera de EXPLORE → counter reset
        assert not self.sm.update(tlm, RoverState.STANDBY)
        assert self.sm.stall_count == 0

    def test_reset_clears_count(self):
        tlm = parse(TLM_STALL)
        self.sm.update(tlm, RoverState.EXPLORE)
        self.sm.reset()
        assert self.sm.stall_count == 0

    def test_stall_mask_zero_resets(self):
        tlm_stall  = parse(TLM_STALL)
        tlm_normal = parse(TLM_NORMAL)
        self.sm.update(tlm_stall, RoverState.EXPLORE)
        assert self.sm.stall_count == 1
        self.sm.update(tlm_normal, RoverState.EXPLORE)  # stall_mask=0 → reset
        assert self.sm.stall_count == 0


# ─── SafeMode ────────────────────────────────────────────────────────────────

class TestSafeMode:

    def setup_method(self):
        self.safe = SafeMode()

    def test_inactive_by_default(self):
        assert not self.safe.active

    def test_activates_on_critical_battery(self):
        assert self.safe.update(parse(TLM_NORMAL), EnergyLevel.CRITICAL, ThermalLevel.OK)
        assert self.safe.active
        assert self.safe.just_activated

    def test_activates_on_llc_fault(self):
        assert self.safe.update(parse(TLM_LLC_FAULT), EnergyLevel.OK, ThermalLevel.OK)
        assert self.safe.active

    def test_activates_on_critical_thermal(self):
        assert self.safe.update(parse(TLM_NORMAL), EnergyLevel.OK, ThermalLevel.CRITICAL)
        assert self.safe.active

    def test_stays_active_once_triggered(self):
        self.safe.update(parse(TLM_NORMAL), EnergyLevel.CRITICAL, ThermalLevel.OK)
        assert self.safe.just_activated
        # segundo ciclo: ya activo, just_activated=False
        assert self.safe.update(parse(TLM_NORMAL), EnergyLevel.OK, ThermalLevel.OK)
        assert not self.safe.just_activated

    def test_reset_deactivates(self):
        self.safe.update(parse(TLM_NORMAL), EnergyLevel.CRITICAL, ThermalLevel.OK)
        self.safe.reset()
        assert not self.safe.active
        assert not self.safe.just_activated

    def test_blocks_movement_commands(self):
        self.safe.update(parse(TLM_NORMAL), EnergyLevel.CRITICAL, ThermalLevel.OK)
        assert     self.safe.blocks_command("EXP:40:40")
        assert     self.safe.blocks_command("AVD:L")
        assert     self.safe.blocks_command("RET")
        assert not self.safe.blocks_command("STB")
        assert not self.safe.blocks_command("PING")
        assert not self.safe.blocks_command("RST")

    def test_does_not_block_when_inactive(self):
        assert not self.safe.blocks_command("EXP:40:40")


# ─── WaypointTracker ─────────────────────────────────────────────────────────

class TestWaypointTracker:

    def setup_method(self):
        self.wt = WaypointTracker(max_waypoints=3, retreat_dist_mm=300)

    def test_record_only_in_explore_normal(self):
        tlm = parse(TLM_NORMAL)
        self.wt.record(tlm, RoverState.STANDBY)
        assert self.wt.count() == 0
        self.wt.record(tlm, RoverState.EXPLORE)
        assert self.wt.count() == 1

    def test_record_ignores_non_normal_safety(self):
        tlm = parse(TLM_LLC_FAULT)
        self.wt.record(tlm, RoverState.EXPLORE)
        assert self.wt.count() == 0

    def test_fifo_max_waypoints(self):
        tlm = parse(TLM_NORMAL)
        for _ in range(5):
            self.wt.record(tlm, RoverState.EXPLORE)
        assert self.wt.count() == 3  # max_waypoints=3

    def test_last_safe_returns_most_recent(self):
        tlm = parse(TLM_NORMAL)
        self.wt.record(tlm, RoverState.EXPLORE)
        wp = self.wt.last_safe()
        assert wp is not None
        assert wp.dist_mm == 800
        assert wp.batt_mv == 15000

    def test_last_safe_returns_none_when_empty(self):
        assert self.wt.last_safe() is None

    def test_should_retreat_below_threshold(self):
        assert     self.wt.should_retreat(parse(TLM_CLOSE))   # 200mm < 300mm
        assert not self.wt.should_retreat(parse(TLM_NORMAL))  # 800mm > 300mm

    def test_should_not_retreat_on_zero_dist(self):
        assert not self.wt.should_retreat(parse(TLM_NO_READING))  # dist=0 → sin lectura


# ─── CommLinkMonitor ─────────────────────────────────────────────────────────

class TestCommLinkMonitor:

    def setup_method(self):
        self.clm = CommLinkMonitor(link_lost_s=10.0, retry_interval_s=5.0, max_retries=3)

    def test_no_event_while_link_active(self):
        now = time.monotonic()
        assert self.clm.update(now, now + 1) is None

    def test_link_lost_event(self):
        now = time.monotonic()
        event = self.clm.update(now, now + 11)
        assert event == "link_lost"
        assert self.clm.is_lost

    def test_link_restored_without_retries(self):
        now = time.monotonic()
        self.clm.update(now, now + 11)            # → link_lost
        event = self.clm.update(now + 15, now + 15)  # paquete reciente → restored
        assert event == "link_restored"
        assert not self.clm.is_lost

    def test_reconnect_attempt_failed(self):
        now = time.monotonic()
        self.clm.update(now, now + 11)            # → link_lost
        event = self.clm.update(now, now + 17)    # 6s desde last_retry → intento
        assert event == "reconnect_attempt_failed"
        assert self.clm.retry_count == 1

    def test_max_retries_exceeded(self):
        now = time.monotonic()
        self.clm.update(now, now + 11)            # link_lost
        self.clm.update(now, now + 17)            # retry 1
        self.clm.update(now, now + 23)            # retry 2
        self.clm.update(now, now + 29)            # retry 3 → max_retries
        event = self.clm.update(now, now + 35)
        assert event == "max_retries_exceeded"

    def test_reconnect_succeeded_after_retries(self):
        now = time.monotonic()
        self.clm.update(now, now + 11)            # link_lost
        self.clm.update(now, now + 17)            # retry 1
        # paquete reciente recibido
        event = self.clm.update(now + 20, now + 20)
        assert event == "reconnect_attempt_succeeded"
        assert not self.clm.is_lost

    def test_send_probe_called_on_retry(self):
        probes = []

        class FakeSource:
            def send_probe(self):
                probes.append(1)

        now = time.monotonic()
        self.clm.update(now, now + 11)
        self.clm.update(now, now + 17, source=FakeSource())
        assert len(probes) == 1


# ─── CSPPacket ───────────────────────────────────────────────────────────────

class TestCSPPacket:

    def test_pack_unpack_round_trip(self):
        payload = b"EXP:40:40"
        packet  = CSPPacket.pack(src=2, dst=1, dport=11, sport=0, payload=payload)
        header, out = CSPPacket.unpack(packet)
        assert header is not None
        assert out == payload

    def test_dst_port_extracted_correctly(self):
        packet = CSPPacket.pack(src=2, dst=1, dport=11, sport=0, payload=b"X")
        header, _ = CSPPacket.unpack(packet)
        assert CSPPacket.dst_port(header) == 11

    def test_src_addr_extracted_correctly(self):
        packet = CSPPacket.pack(src=2, dst=1, dport=11, sport=0, payload=b"X")
        header, _ = CSPPacket.unpack(packet)
        assert CSPPacket.src_addr(header) == 2

    def test_bad_crc_rejected(self):
        packet = bytearray(CSPPacket.pack(src=2, dst=1, dport=11, sport=0, payload=b"X"))
        packet[-1] ^= 0xFF
        assert CSPPacket.unpack(bytes(packet)) == (None, None)

    def test_too_short_rejected(self):
        assert CSPPacket.unpack(b"short") == (None, None)
        assert CSPPacket.unpack(b"") == (None, None)

    def test_empty_payload(self):
        packet = CSPPacket.pack(src=2, dst=1, dport=10, sport=0, payload=b"")
        header, out = CSPPacket.unpack(packet)
        assert out == b""


# ─── parse_response ──────────────────────────────────────────────────────────

class TestParseResponse:

    def test_pong(self):
        assert parse_response("PONG") == ("pong", None)

    def test_ack_stb(self):
        assert parse_response("ACK:STB") == ("ack", "STB")

    def test_ack_exp(self):
        assert parse_response("ACK:EXP") == ("ack", "EXP")

    def test_err_estop(self):
        assert parse_response("ERR:ESTOP") == ("err_estop", None)

    def test_err_wdog(self):
        assert parse_response("ERR:WDOG") == ("err_wdog", None)

    def test_err_unknown(self):
        assert parse_response("ERR:UNKNOWN") == ("err_unknown", None)

    def test_unknown_frame(self):
        kind, data = parse_response("GARBAGE")
        assert kind == "unknown"
        assert data == "GARBAGE"


# ─── RoverMSM ────────────────────────────────────────────────────────────────

class TestRoverMSM:

    def test_initial_state_is_standby(self):
        msm = RoverMSM()
        assert msm.state == RoverState.STANDBY

    def test_transition_updates_state(self):
        msm = RoverMSM()
        msm.transition(RoverState.EXPLORE)
        assert msm.state == RoverState.EXPLORE

    def test_blocks_movement_in_fault(self):
        msm = RoverMSM()
        msm.transition(RoverState.FAULT)
        assert     msm.blocks_command("EXP:40:40")
        assert     msm.blocks_command("STB")
        assert not msm.blocks_command("RST")
        assert not msm.blocks_command("PING")

    def test_does_not_block_outside_fault(self):
        msm = RoverMSM()
        assert not msm.blocks_command("EXP:40:40")

    def test_time_in_state_increases(self):
        msm = RoverMSM()
        time.sleep(0.05)
        assert msm.time_in_state() >= 0.05


# ─── DryRunRover ─────────────────────────────────────────────────────────────

class TestDryRunRover:

    def setup_method(self):
        self.rover = DryRunRover()

    def test_ping_returns_pong(self):
        assert self.rover.send_command("PING") == "PONG"

    def test_exp_returns_ack_exp(self):
        assert self.rover.send_command("EXP:40:40") == "ACK:EXP"
        assert self.rover.send_command("EXP:80:80") == "ACK:EXP"

    def test_stb_returns_ack_stb(self):
        assert self.rover.send_command("STB") == "ACK:STB"

    def test_rst_returns_ack_stb(self):
        assert self.rover.send_command("RST") == "ACK:STB"

    def test_avd_commands(self):
        assert self.rover.send_command("AVD:L") == "ACK:AVD"
        assert self.rover.send_command("AVD:R") == "ACK:AVD"

    def test_ret_returns_ack_ret(self):
        assert self.rover.send_command("RET") == "ACK:RET"

    def test_unknown_returns_err(self):
        assert self.rover.send_command("BOGUS") == "ERR:UNKNOWN"

    def test_recv_tlm_none_before_interval(self):
        assert self.rover.recv_tlm() is None

    def test_recv_tlm_returns_frame_after_interval(self):
        time.sleep(1.1)
        raw = self.rover.recv_tlm()
        assert raw is not None
        assert raw.startswith("TLM:")
        assert TlmFrame.parse(raw) is not None


# ─── HlcEngine — integración ─────────────────────────────────────────────────

class _LimitedRover(DryRunRover):
    """DryRunRover que emite TLM desde el primer ciclo y se detiene tras N frames."""

    def __init__(self, raw_tlm: str, max_cycles: int):
        super().__init__()
        self._raw     = raw_tlm
        self._max     = max_cycles
        self._count   = 0

    def recv_tlm(self) -> "str | None":
        self._count += 1
        if self._count > self._max:
            raise KeyboardInterrupt
        return self._raw


class _NullSource:
    """Fuente de comandos que siempre retorna el mismo comando."""

    def __init__(self, cmd="EXP:40:40"):
        self._cmd = cmd

    def next_command(self, log=None):
        return self._cmd

    def on_tlm(self, raw): pass

    @property
    def last_recv_time(self):
        return time.monotonic()

    def send_probe(self): pass
    def make_link_monitor(self): return None
    def close(self): pass


class TestHlcEngine:

    def test_engine_starts_and_shuts_down(self, tmp_path):
        rover  = _LimitedRover(TLM_NORMAL, max_cycles=2)
        source = _NullSource("PING")
        engine = HlcEngine(rover, source, "manual",
                           log_path=str(tmp_path / "hlc.log"))
        engine.run()  # debe terminar limpiamente

    def test_safe_mode_overrides_exp(self, tmp_path):
        """Con batería crítica el engine debe mandar STB aunque la fuente pida EXP."""
        rover  = _LimitedRover(TLM_CRITICAL_BATT, max_cycles=3)
        source = _NullSource("EXP:40:40")
        engine = HlcEngine(rover, source, "manual",
                           log_path=str(tmp_path / "hlc.log"))
        engine.run()
        # Si SafeMode no funcionara, EXP:40:40 llegaría al rover y
        # DryRunRover pondría state="EXP". Verificamos que se quedó en STB.
        assert engine._msm.state == RoverState.STANDBY

    def test_retreat_on_close_obstacle(self, tmp_path):
        """Con dist_mm < 300mm el engine debe mandar RET."""
        rover  = _LimitedRover(TLM_CLOSE, max_cycles=2)
        source = _NullSource("EXP:40:40")
        engine = HlcEngine(rover, source, "manual",
                           log_path=str(tmp_path / "hlc.log"))
        engine.run()
        assert engine._msm.state == RoverState.RETREAT

    def test_log_file_created(self, tmp_path):
        log = tmp_path / "test.log"
        rover  = _LimitedRover(TLM_NORMAL, max_cycles=1)
        source = _NullSource("STB")
        HlcEngine(rover, source, "manual", log_path=str(log)).run()
        assert log.exists()
        assert log.stat().st_size > 0

    def test_poweroff_scheduled_on_critical_battery(self, tmp_path, monkeypatch):
        """
        Con batería crítica el engine debe programar el poweroff OS (SYS-FUN-040).
        monkeypatch desactiva la ejecución real de systemctl para el test.
        """
        import olympus_hlc.engine as eng_mod
        monkeypatch.setattr(eng_mod, "POWEROFF_ENABLED", False)

        rover  = _LimitedRover(TLM_CRITICAL_BATT, max_cycles=2)
        source = _NullSource("STB")
        engine = HlcEngine(rover, source, "manual",
                           log_path=str(tmp_path / "hlc.log"))
        engine.run()

        assert engine._poweroff_at > 0, \
            "_poweroff_at debe quedar > 0 tras recibir TLM con batería crítica"

    def test_poweroff_not_scheduled_on_thermal_or_fault(self, tmp_path, monkeypatch):
        """
        Safe Mode por LLC FAULT o temperatura NO debe programar poweroff.
        Solo la causa de batería crítica justifica apagar el OS.
        """
        import olympus_hlc.engine as eng_mod
        monkeypatch.setattr(eng_mod, "POWEROFF_ENABLED", False)

        for tlm in (TLM_LLC_FAULT, TLM_HOT):
            rover  = _LimitedRover(tlm, max_cycles=2)
            source = _NullSource("STB")
            engine = HlcEngine(rover, source, "manual",
                               log_path=str(tmp_path / "hlc.log"))
            engine.run()
            assert engine._poweroff_at == 0.0, \
                f"poweroff NO debe programarse para tlm={tlm[:40]}..."


# ─── OdometryTracker ─────────────────────────────────────────────────────────

class TestOdometryTracker:
    """
    Valida el modelo cinemático diferencial con constantes conocidas.
    Usa TICKS_PER_REV=10, WHEEL_RADIUS_MM=100, WHEEL_BASE_MM=200
    para hacer los cálculos verificables a mano.
    """

    def _make_tracker(self, ticks_per_rev=10, radius_mm=100, base_mm=200):
        tracker = OdometryTracker.__new__(OdometryTracker)
        tracker.x_mm      = 0.0
        tracker.y_mm      = 0.0
        tracker.theta_rad = 0.0
        tracker._last_enc_left  = float("nan")
        tracker._last_enc_right = float("nan")
        tracker._mm_per_tick = (2.0 * math.pi * radius_mm) / (3.0 * ticks_per_rev)
        tracker._wheel_base  = float(base_mm)
        return tracker

    def test_initial_pose_is_zero(self):
        t = self._make_tracker()
        x, y, th = t.pose()
        assert x == 0.0 and y == 0.0 and th == 0.0

    def test_first_update_only_sets_reference(self):
        """El primer frame sólo inicializa la referencia — pose no cambia."""
        t = self._make_tracker()
        t.update(100, 100)
        assert t.x_mm == 0.0 and t.y_mm == 0.0

    def test_straight_forward(self):
        """
        Ambos lados avanzan el mismo número de ticks → movimiento recto.
        Con ticks=10, radius=100: mm_per_tick = 2π*100/(3*10) ≈ 20.944
        10 ticks por lado × 3 ruedas = 30 ticks totales por lado.
        d_left = d_right = 30 * 20.944 ≈ 628.3 mm = 2π*100
        dθ = 0 → x += 628.3, y ≈ 0.
        """
        t = self._make_tracker()
        t.update(0, 0)            # frame 0 — referencia
        t.update(30, 30)          # frame 1 — avance recto
        expected = 2 * math.pi * 100  # una vuelta completa
        assert abs(t.x_mm - expected) < 0.01
        assert abs(t.y_mm) < 0.01
        assert abs(t.theta_rad) < 1e-9

    def test_spin_in_place(self):
        """
        Izquierda retrocede, derecha avanza igual → giro en el sitio.
        d_center = 0, dθ = (d_right - d_left) / base.
        Con 30 ticks derecha, -30 ticks izquierda:
          d_right = +628.3 mm, d_left = -628.3 mm
          dθ = 1256.6 / 200 ≈ 2π rad (vuelta completa)
          x y deben ser ~0.
        """
        t = self._make_tracker()
        t.update(0, 0)
        t.update(-30, 30)
        assert abs(t.x_mm) < 0.01
        assert abs(t.y_mm) < 0.01
        assert abs(t.theta_rad - 2 * math.pi) < 1e-6

    def test_reset_clears_pose_and_reference(self):
        t = self._make_tracker()
        t.update(0, 0)
        t.update(30, 30)
        t.reset()
        assert t.x_mm == 0.0 and t.y_mm == 0.0 and t.theta_rad == 0.0
        # Después del reset, el siguiente update solo fija referencia
        t.update(50, 50)
        assert t.x_mm == 0.0

    def test_incremental_updates_accumulate(self):
        """Dos pasos de 15 ticks rectos = un paso de 30 ticks rectos."""
        t1 = self._make_tracker()
        t1.update(0, 0)
        t1.update(30, 30)
        x1, y1, _ = t1.pose()

        t2 = self._make_tracker()
        t2.update(0, 0)
        t2.update(15, 15)
        t2.update(30, 30)
        x2, y2, _ = t2.pose()

        assert abs(x1 - x2) < 0.01
        assert abs(y1 - y2) < 0.01
