#!/usr/bin/env python3
# olympus_hlc/__main__.py — Entry point: argparse + rover/source setup
#
# Invocable como:
#   python3 -m olympus_hlc --mode manual
#   python3 -m olympus_hlc --mode vision --model /usr/share/olympus/models/yolov8n.onnx
#   python3 -m olympus_hlc --mode gcs
#
# O mediante el entry point installado por Yocto:
#   olympus_controller --mode gcs

import argparse
import sys

from .engine import HlcEngine
from .logger import OlympusLogger
from .msm import DryRunRover
from .sources.gcs import GCSSource
from .sources.manual import ManualSource
from .sources.vision import VisionSource
from .sources.gcs_libcsp import LibcspGCSSource


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Olympus HLC Controller — vision, manual or gcs mode"
    )
    parser.add_argument(
        "--mode",
        choices=["vision", "manual", "gcs"],
        required=True,
        help="Command source: 'vision' (camera+YOLOv8n), 'manual' (stdin) "
             "or 'gcs' (UDP commands from Ground Control Station, SRS-013)",
    )
    parser.add_argument(
        "--model",
        default="/usr/share/olympus/models/yolov8n.onnx",
        help="Path to YOLOv8n ONNX model (vision mode only)",
    )
    parser.add_argument(
        "--port",
        default="/dev/arduino_mega",
        help="Serial port for Arduino (default: /dev/arduino_mega)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Skip Arduino connection; simulate responses (testing without hardware)",
    )
    parser.add_argument(
        "--use-libcsp",
        action="store_true",
        help="Use native libcsp socket API instead of raw UDP+CSPPacket (requires "
             "libcsp Python bindings in site-packages). Enables real CSP routing "
             "for future UHF interface integration.",
    )
    parser.add_argument(
        "--log-path",
        default=OlympusLogger.DEFAULT_LOG_PATH,
        help=f"Path for the HLC log file (default: {OlympusLogger.DEFAULT_LOG_PATH})",
    )
    args = parser.parse_args()

    # ── Rover connection ──────────────────────────────────────────────────────

    if args.dry_run:
        print("[Controller] DRY-RUN mode — Arduino not required.")
        rover = DryRunRover()
    else:
        print(f"[Controller] Connecting to Arduino on {args.port} @ {args.baud}...")
        try:
            import rover_bridge
            rover = rover_bridge.Rover(args.port, args.baud)
            print("[Controller] Connected.")
        except Exception as e:
            print(f"[ERROR] Cannot open rover bridge: {e}")
            sys.exit(1)

    # ── Command source ────────────────────────────────────────────────────────

    if args.mode == "manual":
        source = ManualSource()
    elif args.mode == "gcs":
        if args.use_libcsp:
            source = LibcspGCSSource()
        else:
            source = GCSSource()
    else:
        source = VisionSource(args.model)

    # ── Run ───────────────────────────────────────────────────────────────────

    engine = HlcEngine(rover, source, args.mode, log_path=args.log_path)
    try:
        engine.run()
    finally:
        source.close()


if __name__ == "__main__":
    main()
