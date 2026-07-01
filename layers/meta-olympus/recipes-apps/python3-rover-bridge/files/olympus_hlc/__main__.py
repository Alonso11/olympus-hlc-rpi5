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
from .sources.gcs_libcsp import LibcspGCSSource
from .sources.manual import ManualSource
from .sources.manual_stream import ManualStreamSource
from .sources.station import StationSource
from .sources.vision import VisionSource
from .sources.vision_gcs import VisionGCSSource
from .sources.vision_nav import VisionNavSource


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Olympus HLC Controller — vision, manual or gcs mode"
    )
    parser.add_argument(
        "--mode",
        choices=["vision", "vision-nav", "vision-gcs", "manual", "gcs", "station"],
        required=True,
        help="Command source: 'vision' (camera+YOLOv8n), 'vision-nav' (lunar seg), "
             "'manual' (stdin), "
             "'gcs' (UDP+CSP desde GCS, SRS-013), o 'station' (GUI por TCP — "
             "consolidación A: reemplaza el daemon ground_station/olympus_station.py)",
    )
    parser.add_argument(
        "--model",
        default="/usr/share/olympus/models/yolov8n.onnx",
        help="Path to ONNX model (vision: YOLOv8n, vision-nav: lunar UNetMobileNet)",
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
        "--bench",
        action="store_true",
        help="Banco de caracterización: registra TLM pero DESACTIVA los overrides "
             "autónomos del HLC (RET/STB por TLM-loss, retreat, slip, SafeMode, "
             "monitor de enlace GCS). Permite manejar desde la GUI (--mode station) "
             "sin que el watchdog pise los comandos. Solo pruebas, nunca en campo. "
             "(En --mode manual el banco ya está siempre activo.)",
    )
    parser.add_argument(
        "--gcs-host",
        default="",
        help="IP o hostname del GCS (default: vacío = aprender del primer CMD recibido). "
             "Ej: 192.168.100.10 en campo, olympus-rover.local no aplica aquí.",
    )
    parser.add_argument(
        "--log-path",
        default=OlympusLogger.DEFAULT_LOG_PATH,
        help=f"Path for the HLC log file (default: {OlympusLogger.DEFAULT_LOG_PATH})",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="(manual mode) Habilita servidor TCP de cámara cruda para stream_view.py",
    )
    parser.add_argument(
        "--stream-port",
        type=int,
        default=5005,
        help="Puerto TCP del stream de cámara (default: 5005)",
    )
    parser.add_argument(
        "--stream-fps",
        type=int,
        default=5,
        help="Framerate del stream de cámara (default: 5)",
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

    if args.mode == "manual" and args.stream:
        source = ManualStreamSource(
            stream_port=args.stream_port,
            fps=args.stream_fps,
        )
    elif args.mode == "manual":
        source = ManualSource()
    elif args.mode == "gcs":
        source = LibcspGCSSource(gcs_host=args.gcs_host)
    elif args.mode == "station":
        # Puertos fijos 5006 (control) / 5005 (video) — mismo contrato que la GUI.
        # --model habilita AUTO (YOLO a bordo); si no existe, opera solo MANUAL.
        source = StationSource(model_path=args.model)
    elif args.mode == "vision-gcs":
        source = VisionGCSSource(args.model, gcs_host=args.gcs_host)
    elif args.mode == "vision-nav":
        source = VisionNavSource(args.model)
    else:
        source = VisionSource(args.model)

    # ── Run ───────────────────────────────────────────────────────────────────

    engine = HlcEngine(rover, source, args.mode, log_path=args.log_path,
                       bench=args.bench)
    try:
        engine.run()
    finally:
        source.close()


if __name__ == "__main__":
    main()
