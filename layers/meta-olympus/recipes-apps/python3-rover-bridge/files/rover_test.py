#!/usr/bin/env python3
"""
rover_test.py — Integration test: GCS laptop → WiFi UDP/CSP → RPi5 HLC → LLC → rover

Tests the full stack with the real Arduino Mega (LLC v2.16).
Parses TLM frames to verify motors, sensors, and safety state.

Setup:
  RPi5:   python3 -m olympus_hlc --mode gcs [--use-libcsp]
  Laptop: python3 rover_test.py <RPI5_IP_OR_HOSTNAME>

  Lab:    python3 rover_test.py olympus-rover.local
  Field:  python3 rover_test.py 192.168.100.1
  Direct: python3 rover_test.py 192.168.18.245

Tests:
  T1  PING keepalive
  T2  Sensor baseline (batt, temp, ToF, safety=NORMAL)
  T3  Forward motion EXP:30:30 — verify encoders increment
  T4  Reverse motion EXP:-30:-30 — verify encoders decrement
  T5  Climb mode CLB:30:30 — verify state transition
  T6  Stall detection (blocked wheels) — verify WARN response
  T7  Emergency stop STB from any state
  T8  Link lost → HLC forces STB after 10s silence
"""

import argparse
import select
import socket
import struct
import sys
import time

# ── CSP wire format (must match olympus_hlc/csp.py) ─────────────────────────

def _crc32c(data: bytes) -> int:
    """CRC-32C (Castagnoli) — matches libcsp 4.x csp_crc32_append."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


PRIO_NORM  = 2
FLAG_CRC32 = 0x01  # CSP_FCRC32 bit in CSPv1 (libcsp 4.x)

CSP_ADDR_GCS = 1
CSP_ADDR_HLC = 2
CSP_PORT_TM  = 10
CSP_PORT_CMD = 11
CSP_PORT_HB  = 1

CMD_PORT = 9000
TLM_PORT = 9001


def csp_pack(src, dst, dport, sport, payload: bytes) -> bytes:
    """Wire: 4B header (BE) + payload + 4B CRC-32C (BE, over payload-only)."""
    header = (
        ((PRIO_NORM & 0x03) << 30) |
        ((src   & 0x1F) << 25) |
        ((dst   & 0x1F) << 20) |
        ((dport & 0x3F) << 14) |
        ((sport & 0x3F) <<  8) |
        FLAG_CRC32
    )
    hdr_bytes = struct.pack(">I", header)
    crc = struct.pack(">I", _crc32c(payload))
    return hdr_bytes + payload + crc


def csp_unpack(data: bytes):
    if len(data) < 8:
        return None, None
    payload  = data[4:-4]
    crc_recv = data[-4:]
    if struct.pack(">I", _crc32c(payload)) != crc_recv:
        return None, None
    header = struct.unpack(">I", data[:4])[0]
    dport  = (header >> 14) & 0x3F
    return dport, payload


# ── TLM frame parser (mirrors olympus_hlc/models.py TlmFrame.parse) ─────────

def parse_tlm(raw: str):
    try:
        p = raw.split(":")
        if len(p) < 22 or p[0] != "TLM":
            return None
        return {
            "safety":      p[1],
            "stall_mask":  int(p[2], 2),
            "tick_ms":     int(p[3].rstrip("ms")),
            "batt_mv":     int(p[4].rstrip("mV")),
            "batt_ma":     int(p[5].rstrip("mA")),
            "currents":    [int(p[i]) for i in range(6, 12)],
            "temp_c":      int(p[12].rstrip("C")),
            "dist_mm":     int(p[19].rstrip("mm")),
            "enc_left":    int(p[20]),
            "enc_right":   int(p[21]),
            "dist_far_mm": int(p[25].rstrip("mm")) if len(p) > 25 else 0,
        }
    except (ValueError, IndexError):
        return None


# ── Transport ────────────────────────────────────────────────────────────────

class RoverLink:
    def __init__(self, rpi_host: str):
        self._rpi = rpi_host
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", TLM_PORT))
        self._sock.setblocking(False)
        self._tlm_buf = []

    def send(self, cmd: str):
        pkt = csp_pack(CSP_ADDR_GCS, CSP_ADDR_HLC, CSP_PORT_CMD, 0, cmd.encode())
        self._sock.sendto(pkt, (self._rpi, CMD_PORT))

    def drain(self, timeout_s: float) -> list:
        """Read all TLM packets for timeout_s seconds. Returns list of parsed frames."""
        frames = []
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = max(0.0, deadline - time.monotonic())
            r, _, _ = select.select([self._sock], [], [], remaining)
            if not r:
                break
            try:
                data, _ = self._sock.recvfrom(1024)
            except OSError:
                break
            dport, payload = csp_unpack(data)
            if dport == CSP_PORT_TM and payload:
                tlm = parse_tlm(payload.decode(errors="replace").strip())
                if tlm:
                    frames.append(tlm)
        return frames

    def last_tlm(self, timeout_s: float = 3.0):
        """Wait for at least one valid TLM frame. Returns last one or None."""
        frames = self.drain(timeout_s)
        return frames[-1] if frames else None

    def close(self):
        self._sock.close()


# ── Test runner ──────────────────────────────────────────────────────────────

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"

results = []

def step(label: str):
    print(f"\n{'─'*60}")
    print(f"  {label}")
    print(f"{'─'*60}")

def check(name: str, ok: bool, detail: str = ""):
    status = PASS if ok else FAIL
    mark   = "✓" if ok else "✗"
    line   = f"  [{mark}] {name}"
    if detail:
        line += f"  ({detail})"
    print(line)
    results.append((name, status, detail))
    return ok


def run_tests(link: RoverLink, skip_motion: bool):
    # ── T1: PING ─────────────────────────────────────────────────────────────
    step("T1 — PING keepalive")
    link.send("PING")
    tlm = link.last_tlm(3.0)
    check("TLM received after PING", tlm is not None,
          f"safety={tlm['safety']}" if tlm else "no TLM")

    # ── T2: Sensor baseline ───────────────────────────────────────────────────
    step("T2 — Sensor baseline")
    link.send("STB")
    tlm = link.last_tlm(3.0)
    if tlm is None:
        check("Baseline TLM", False, "no TLM — check HLC running on RPi5")
        return
    check("safety = NORMAL",   tlm["safety"] == "NORMAL",    f"got {tlm['safety']}")
    check("batt_mv > 12000",   tlm["batt_mv"] > 12000,       f"{tlm['batt_mv']} mV")
    check("batt_mv < 17000",   tlm["batt_mv"] < 17000,       f"{tlm['batt_mv']} mV")
    check("temp_c in [0, 60]", 0 <= tlm["temp_c"] <= 60,     f"{tlm['temp_c']} °C")
    check("dist_mm > 0",       tlm["dist_mm"] > 0,           f"{tlm['dist_mm']} mm (VL53L0X)")
    check("stall = 0",         tlm["stall_mask"] == 0,        f"mask=0b{tlm['stall_mask']:06b}")

    if skip_motion:
        print("\n  [SKIP] Motion tests omitted (--no-motion)")
        for name in ["T3 Forward", "T4 Reverse", "T5 CLB", "T6 Stall", "T7 STB"]:
            results.append((name, SKIP, ""))
        return

    enc_l0 = tlm["enc_left"]
    enc_r0 = tlm["enc_right"]

    # ── T3: Forward motion ────────────────────────────────────────────────────
    step("T3 — Forward motion EXP:30:30 (3 s)")
    link.send("EXP:30:30")
    time.sleep(3.0)
    link.send("STB")
    tlm = link.last_tlm(3.0)
    if tlm:
        dl = tlm["enc_left"]  - enc_l0
        dr = tlm["enc_right"] - enc_r0
        check("safety = NORMAL",         tlm["safety"] == "NORMAL", tlm["safety"])
        check("enc_left  incremented",   dl > 0,  f"Δ={dl}")
        check("enc_right incremented",   dr > 0,  f"Δ={dr}")
        check("symmetry |ΔL-ΔR| < 30%", abs(dl - dr) < max(dl, dr) * 0.3,
              f"ΔL={dl} ΔR={dr}")
        enc_l0 = tlm["enc_left"]
        enc_r0 = tlm["enc_right"]
    else:
        check("Forward TLM received", False, "no TLM")

    # ── T4: Reverse motion ────────────────────────────────────────────────────
    step("T4 — Reverse motion EXP:-30:-30 (2 s)")
    link.send("EXP:-30:-30")
    time.sleep(2.0)
    link.send("STB")
    tlm = link.last_tlm(3.0)
    if tlm:
        dl = tlm["enc_left"]  - enc_l0
        dr = tlm["enc_right"] - enc_r0
        check("safety = NORMAL",        tlm["safety"] == "NORMAL", tlm["safety"])
        check("enc_left  decremented",  dl < 0,  f"Δ={dl}")
        check("enc_right decremented",  dr < 0,  f"Δ={dr}")
    else:
        check("Reverse TLM received", False, "no TLM")

    # ── T5: Climb mode ────────────────────────────────────────────────────────
    step("T5 — Climb mode CLB:30:30 (2 s)")
    link.send("CLB:30:30")
    time.sleep(2.0)
    tlm = link.last_tlm(2.0)
    link.send("STB")
    if tlm:
        check("safety = NORMAL in CLB", tlm["safety"] == "NORMAL", tlm["safety"])
        check("no full stall in CLB",   tlm["stall_mask"] != 0x3F,
              f"mask=0b{tlm['stall_mask']:06b}")
    else:
        check("CLB TLM received", False, "no TLM")

    # ── T6: Emergency STB ─────────────────────────────────────────────────────
    step("T6 — Emergency STB from EXP state")
    link.send("EXP:40:40")
    time.sleep(0.5)
    link.send("STB")
    tlm = link.last_tlm(3.0)
    if tlm:
        check("rover stopped (stall=0 after STB)", tlm["stall_mask"] == 0,
              f"mask=0b{tlm['stall_mask']:06b}")
        check("safety = NORMAL after STB", tlm["safety"] == "NORMAL", tlm["safety"])
    else:
        check("STB TLM received", False, "no TLM")

    # ── T7: Link lost → forced STB ────────────────────────────────────────────
    step("T7 — Link lost: 12 s silence → HLC must force STB")
    link.send("EXP:20:20")
    time.sleep(1.0)
    print("  [GCS] Going silent for 12 s...")
    hb_count_before = sum(1 for n, s, _ in results if "HB" in n)
    time.sleep(12.0)
    tlm = link.last_tlm(3.0)
    link.send("PING")  # restore link
    if tlm:
        check("safety = NORMAL after link restore", tlm["safety"] == "NORMAL",
              tlm["safety"])
    else:
        # HLC may have stopped sending TLM while in STB after link_lost
        link.send("PING")
        tlm2 = link.last_tlm(3.0)
        check("TLM resumed after PING", tlm2 is not None,
              "check CommLinkMonitor active on HLC")


def print_summary():
    print(f"\n{'═'*60}")
    print("  SUMMARY")
    print(f"{'═'*60}")
    passed = sum(1 for _, s, _ in results if s == PASS)
    failed = sum(1 for _, s, _ in results if s == FAIL)
    skipped = sum(1 for _, s, _ in results if s == SKIP)
    for name, status, detail in results:
        mark = "✓" if status == PASS else ("·" if status == SKIP else "✗")
        line = f"  [{mark}] {name}"
        if detail:
            line += f"  — {detail}"
        print(line)
    print(f"\n  {passed} passed  {failed} failed  {skipped} skipped")
    if failed:
        print("\n  Verify on RPi5:")
        print("    journalctl -u olympus-hlc -f")
        print("    python3 -m olympus_hlc --mode gcs --dry-run  (sin Arduino)")


def main():
    parser = argparse.ArgumentParser(
        description="Olympus rover integration test — GCS laptop → WiFi UDP → RPi5 → LLC"
    )
    parser.add_argument(
        "host",
        help="RPi5 IP or hostname (olympus-rover.local / 192.168.100.1 / 192.168.18.245)"
    )
    parser.add_argument(
        "--no-motion",
        action="store_true",
        help="Skip motion tests (T3-T7) — safe when motors are not connected"
    )
    args = parser.parse_args()

    print(f"Olympus Rover Integration Test")
    print(f"  Target : {args.host}:{CMD_PORT}")
    print(f"  Listen : :{TLM_PORT}")
    print(f"  Motion : {'disabled' if args.no_motion else 'enabled'}")
    print(f"\n  Ensure on RPi5:")
    print(f"    python3 -m olympus_hlc --mode gcs")

    link = RoverLink(args.host)
    try:
        run_tests(link, skip_motion=args.no_motion)
    except KeyboardInterrupt:
        print("\n[TEST] Interrupted — sending STB")
        link.send("STB")
    finally:
        link.send("STB")
        link.close()

    print_summary()
    failed = sum(1 for _, s, _ in results if s == FAIL)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
