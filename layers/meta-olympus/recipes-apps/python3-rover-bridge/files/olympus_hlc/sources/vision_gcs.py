# olympus_hlc/sources/vision_gcs.py — VisionGCSSource: supervisory control
#
# Combines VisionSource (YOLO autonomous) + LibcspGCSSource (GCS link).
#
# Command priority:
#   1. Emergency (STB / RST / ABORT / FLT) — always executed, any mode
#   2. Mode switch (MODE:AUTO / MODE:TELEOP) — changes command source
#   3. Motion (EXP / CLB / AVD / RET)       — only forwarded in TELEOP
#
# TLM always forwarded to GCS regardless of mode (downlink SRS-020).
# CommLinkMonitor active in both modes: link_lost forces STB.

from ..interfaces import CommandSource
from .gcs_libcsp import LibcspGCSSource
from .vision import VisionSource

_EMERGENCY = {"STB", "RST", "ABORT", "FLT"}


class VisionGCSSource(CommandSource):

    def __init__(self, model_path: str, gcs_host: str = ""):
        self._gcs    = LibcspGCSSource(gcs_host=gcs_host)
        self._vision = VisionSource(model_path)
        self._mode   = "auto"  # "auto" | "teleop"
        print("[VisionGCSSource] Supervisory mode — starting AUTO")

    def next_command(self, log=None) -> "str | None":
        gcs_cmd = self._gcs.next_command(log)  # non-blocking (timeout=0)

        # Priority 1: emergency overrides — always execute
        if gcs_cmd in _EMERGENCY:
            return gcs_cmd

        # Priority 2: mode switch
        if gcs_cmd == "MODE:AUTO":
            self._mode = "auto"
            print("[VisionGCSSource] → AUTO")
            return None
        if gcs_cmd == "MODE:TELEOP":
            self._mode = "teleop"
            print("[VisionGCSSource] → TELEOP")
            return None

        # Priority 3: motion commands — only in TELEOP
        if self._mode == "teleop":
            return gcs_cmd  # None if no GCS cmd this cycle

        # AUTO: YOLO decides
        return self._vision.next_command(log)

    def on_tlm(self, raw_tlm: str) -> None:
        self._gcs.on_tlm(raw_tlm)

    @property
    def last_recv_time(self) -> float:
        return self._gcs.last_recv_time

    def send_probe(self) -> None:
        self._gcs.send_probe()

    def make_link_monitor(self):
        return self._gcs.make_link_monitor()

    def close(self) -> None:
        self._gcs.close()
        self._vision.close()
