#!/usr/bin/env python3
"""
debug_vision.py — Pipeline de visión con anotaciones, stream via SSH pipe.

Corre en el RPi5. Por cada frame capturado ejecuta inferencia YOLOv8n,
anota el resultado y escribe a stdout:
  [4 bytes big-endian: longitud JPEG] [N bytes JPEG]

Uso en el RPi5:
  python3 debug_vision.py [--mode bbox|seg] [--capture still|vid] [--model PATH] [--frames N]

Uso desde el PC via SSH:
  ssh root@rpi5 "python3 /usr/bin/debug_vision.py --mode seg" | python3 debug_view.py
  ssh root@rpi5 "python3 /usr/bin/debug_vision.py --mode seg --capture vid --fps 10" | python3 debug_view.py

Dependencias RPi5: python3-opencv, python3-numpy (incluidos en imagen Yocto Olympus)
"""

import argparse
import struct
import subprocess
import sys

# ── Constantes por defecto (espejo de olympus_hlc/config.py) ─────────────────

FRAME_WIDTH      = 640
FRAME_HEIGHT     = 480
ZONE_LEFT_END    = 0.33
ZONE_RIGHT_START = 0.67
SEG_ROI_TOP      = 0.50
VISION_CONF_MIN  = 0.50
VISION_AREA_MIN  = 0.05
SEG_CONF_MIN     = 0.50
SEG_AREA_MIN     = 0.03
SEG_ZONE_MIN     = 0.05

BBOX_MODEL_DEFAULT = "/usr/share/olympus/models/yolov8n.onnx"
SEG_MODEL_DEFAULT  = "/usr/share/olympus/models/yolov8n-seg.onnx"

# ── Colores BGR ───────────────────────────────────────────────────────────────

COLOR_GREEN  = (0, 200, 0)
COLOR_YELLOW = (0, 200, 200)
COLOR_RED    = (0, 0, 220)
COLOR_WHITE  = (255, 255, 255)
COLOR_BLACK  = (0, 0, 0)
COLOR_ZONE   = (200, 200, 0)   # líneas de zona
COLOR_ROI    = (180, 180, 180) # línea ROI

FONT       = None   # cv2.FONT_HERSHEY_SIMPLEX — asignado tras import cv2
FONT_SCALE = 0.55
FONT_THICK = 1

# ── Captura: modo still ───────────────────────────────────────────────────────

def capture_frame(cv2, np, width, height):
    """Captura un JPEG via rpicam-still. Retorna ndarray BGR o None."""
    result = subprocess.run(
        [
            "rpicam-still",
            "--output",   "-",
            "--width",    str(width),
            "--height",   str(height),
            "--timeout",  "200",
            "--nopreview",
            "--encoding", "jpg",
        ],
        capture_output=True,
    )
    if result.returncode != 0 or not result.stdout:
        return None
    return cv2.imdecode(
        np.frombuffer(result.stdout, np.uint8),
        cv2.IMREAD_COLOR,
    )

# ── Captura: modo vid (generador MJPEG continuo) ──────────────────────────────

def mjpeg_frames(cv2, np, width, height, fps):
    """
    Genera ndarray BGR frames leyendo el stream MJPEG de rpicam-vid.
    rpicam-vid escribe JPEG raw (boundaries SOI=\\xff\\xd8, EOI=\\xff\\xd9).
    """
    proc = subprocess.Popen(
        [
            "rpicam-vid",
            "--codec",     "mjpeg",
            "--output",    "-",
            "--width",     str(width),
            "--height",    str(height),
            "--framerate", str(fps),
            "--timeout",   "0",        # sin límite de tiempo
            "--nopreview",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    buf = b""
    SOI = b"\xff\xd8"
    EOI = b"\xff\xd9"

    assert proc.stdout is not None
    try:
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk

            # Extraer todos los JPEG completos del buffer acumulado
            while True:
                start = buf.find(SOI)
                if start == -1:
                    buf = b""
                    break
                end = buf.find(EOI, start + 2)
                if end == -1:
                    buf = buf[start:]   # conservar desde SOI
                    break
                jpeg = buf[start: end + 2]
                buf  = buf[end + 2:]

                frame = cv2.imdecode(
                    np.frombuffer(jpeg, np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if frame is not None:
                    yield frame
    finally:
        proc.terminate()
        proc.wait()

# ── Anotación común: líneas de zona ──────────────────────────────────────────

def draw_zones(cv2, frame, cmd):
    H, W = frame.shape[:2]
    lx = int(ZONE_LEFT_END    * W)
    rx = int(ZONE_RIGHT_START * W)

    cv2.line(frame, (lx, 0), (lx, H), COLOR_ZONE, 1)
    cv2.line(frame, (rx, 0), (rx, H), COLOR_ZONE, 1)

    cv2.putText(frame, "IZQ",    (4,     14), FONT, FONT_SCALE, COLOR_ZONE, FONT_THICK)
    cv2.putText(frame, "CENTRO", (lx+4,  14), FONT, FONT_SCALE, COLOR_ZONE, FONT_THICK)
    cv2.putText(frame, "DER",    (rx+4,  14), FONT, FONT_SCALE, COLOR_ZONE, FONT_THICK)

    color = COLOR_GREEN if cmd.startswith("EXP") else COLOR_RED
    cv2.putText(frame, f"CMD: {cmd}", (4, H - 8), FONT, 0.65, color, 2)

# ── Modo bbox ─────────────────────────────────────────────────────────────────

def run_bbox(cv2, np, net, frame):
    H, W = frame.shape[:2]

    blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (640, 640), swapRB=True, crop=False)
    net.setInput(blob)
    output = net.forward()          # (1, 84, 8400)
    preds  = output[0].T            # (8400, 84)

    best_area = 0.0
    best_box  = None
    best_cx   = None
    best_conf = 0.0

    for pred in preds:
        scores     = pred[4:]
        class_id   = int(np.argmax(scores))
        confidence = float(scores[class_id])
        if confidence < VISION_CONF_MIN:
            continue

        cx_n, cy_n, w_n, h_n = pred[:4]
        area_frac = (w_n / 640.0) * (h_n / 640.0)
        if area_frac < VISION_AREA_MIN:
            continue

        if area_frac > best_area:
            best_area = area_frac
            best_conf = confidence
            best_cx   = cx_n / 640.0
            x1 = max(0, int((cx_n - w_n / 2) * W / 640))
            y1 = max(0, int((cy_n - h_n / 2) * H / 640))
            x2 = min(W, int((cx_n + w_n / 2) * W / 640))
            y2 = min(H, int((cy_n + h_n / 2) * H / 640))
            best_box = (x1, y1, x2, y2)

    if best_box is not None:
        x1, y1, x2, y2 = best_box
        cv2.rectangle(frame, (x1, y1), (x2, y2), COLOR_GREEN, 2)
        cx_px = int(best_cx * W)
        cv2.line(frame, (cx_px, y1), (cx_px, y2), COLOR_YELLOW, 1)
        cv2.putText(frame, f"{best_conf:.2f}", (x1, max(y1 - 4, 10)),
                    FONT, FONT_SCALE, COLOR_GREEN, FONT_THICK)

    if best_cx is None:
        cmd = "EXP:40:40"
    elif best_cx < ZONE_LEFT_END:
        cmd = "AVD:R"
    elif best_cx > ZONE_RIGHT_START:
        cmd = "AVD:L"
    else:
        cmd = "RET"

    draw_zones(cv2, frame, cmd)
    return cmd

# ── Modo segmentation ─────────────────────────────────────────────────────────

def decode_masks(np, cv2, output0, output1, H, W):
    B, C, K, P = 4, 80, 32, 160
    preds  = output0[0].T    # (8400, 116)
    protos = output1[0]      # (32, 160, 160)
    masks  = []

    for pred in preds:
        confidence = float(pred[B: B + C].max())
        if confidence < SEG_CONF_MIN:
            continue

        cx_n, cy_n, w_n, h_n = pred[:B]
        area_frac = (w_n / 640.0) * (h_n / 640.0)
        if area_frac < SEG_AREA_MIN:
            continue

        coeffs   = pred[B + C: B + C + K]
        mask_160 = coeffs @ protos.reshape(K, P * P)
        mask_160 = 1.0 / (1.0 + np.exp(-mask_160))
        mask_160 = mask_160.reshape(P, P).astype(np.float32)
        mask_full = cv2.resize(mask_160, (W, H))

        x1 = max(0, int((cx_n - w_n / 2) * W / 640))
        y1 = max(0, int((cy_n - h_n / 2) * H / 640))
        x2 = min(W, int((cx_n + w_n / 2) * W / 640))
        y2 = min(H, int((cy_n + h_n / 2) * H / 640))

        binary = (mask_full > 0.5)
        binary[:y1, :]  = False
        binary[y2:, :]  = False
        binary[:, :x1]  = False
        binary[:, x2:]  = False

        frame_area = H * W
        if binary.sum() < SEG_AREA_MIN * frame_area:
            continue

        masks.append(binary)

    return masks


def run_seg(cv2, np, net, frame):
    H, W = frame.shape[:2]

    blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (640, 640), swapRB=True, crop=False)
    net.setInput(blob)
    outputs          = net.forward(net.getUnconnectedOutLayersNames())
    output0, output1 = outputs[0], outputs[1]

    masks = decode_masks(np, cv2, output0, output1, H, W)

    lx     = int(ZONE_LEFT_END    * W)
    rx     = int(ZONE_RIGHT_START * W)
    roi_y  = int(SEG_ROI_TOP      * H)

    # Overlay máscara combinada en verde semitransparente
    if masks:
        combined = np.zeros((H, W), dtype=bool)
        for m in masks:
            combined |= m
        overlay = frame.copy()
        overlay[combined] = (0, 180, 0)
        cv2.addWeighted(overlay, 0.40, frame, 0.60, 0, frame)

    # Línea ROI
    cv2.line(frame, (0, roi_y), (W, roi_y), COLOR_ROI, 1)
    cv2.putText(frame, "ROI", (4, roi_y - 4), FONT, 0.45, COLOR_ROI, 1)

    # Calcular coberturas y decidir comando
    if masks:
        combined_roi = np.zeros((H, W), dtype=bool)
        for m in masks:
            combined_roi |= m
        roi = combined_roi[roi_y:, :]
        roi_h = roi.shape[0]

        left_cov   = roi[:, :lx].sum()      / max(roi_h * lx,          1)
        center_cov = roi[:, lx:rx].sum()    / max(roi_h * (rx - lx),   1)
        right_cov  = roi[:, rx:].sum()      / max(roi_h * (W - rx),    1)

        candidates = {"AVD:R": left_cov, "RET": center_cov, "AVD:L": right_cov}
        best_cmd, best_cov = max(candidates.items(), key=lambda kv: kv[1])
        cmd = best_cmd if best_cov >= SEG_ZONE_MIN else "EXP:40:40"

        # Texto de cobertura por zona
        cv2.putText(frame, f"{left_cov:.0%}",   (4,      roi_y + 16), FONT, 0.50, COLOR_WHITE, 1)
        cv2.putText(frame, f"{center_cov:.0%}", (lx + 4, roi_y + 16), FONT, 0.50, COLOR_WHITE, 1)
        cv2.putText(frame, f"{right_cov:.0%}",  (rx + 4, roi_y + 16), FONT, 0.50, COLOR_WHITE, 1)
    else:
        cmd = "EXP:40:40"

    draw_zones(cv2, frame, cmd)
    return cmd

# ── Write frame to stdout ─────────────────────────────────────────────────────

def write_frame(cv2, frame):
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return
    data = buf.tobytes()
    sys.stdout.buffer.write(struct.pack(">I", len(data)))
    sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global FONT

    ap = argparse.ArgumentParser(description="Debug vision stream via SSH pipe")
    ap.add_argument("--mode",    default="bbox", choices=["bbox", "seg"],
                    help="Modo de inferencia (default: bbox)")
    ap.add_argument("--capture", default="still", choices=["still", "vid"],
                    help="Fuente de captura: still=rpicam-still (~1fps), vid=rpicam-vid MJPEG (default: still)")
    ap.add_argument("--fps",     type=int, default=5,
                    help="Framerate para --capture vid (default: 5)")
    ap.add_argument("--model",   default=None,
                    help="Ruta al modelo ONNX (default: según modo)")
    ap.add_argument("--frames",  type=int, default=0,
                    help="Número de frames (0 = infinito, default: 0)")
    args = ap.parse_args()

    try:
        import cv2
        import numpy as np
    except ImportError as e:
        print(f"[ERROR] Dependencia faltante: {e}", file=sys.stderr)
        sys.exit(1)

    FONT = cv2.FONT_HERSHEY_SIMPLEX

    model_path = args.model or (SEG_MODEL_DEFAULT if args.mode == "seg" else BBOX_MODEL_DEFAULT)

    import os
    if not os.path.exists(model_path):
        print(f"[ERROR] Modelo no encontrado: {model_path}", file=sys.stderr)
        sys.exit(1)

    print(f"[debug_vision] mode={args.mode} capture={args.capture} model={model_path}", file=sys.stderr)
    net = cv2.dnn.readNetFromONNX(model_path)
    print(f"[debug_vision] Modelo cargado ({len(net.getLayerNames())} capas)", file=sys.stderr)

    def infer(frame):
        if args.mode == "seg":
            return run_seg(cv2, np, net, frame)
        return run_bbox(cv2, np, net, frame)

    def error_frame(msg):
        f = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
        cv2.putText(f, msg, (40, FRAME_HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOR_RED, 2)
        return f

    count = 0

    if args.capture == "vid":
        print(f"[debug_vision] rpicam-vid MJPEG @ {args.fps}fps", file=sys.stderr)
        for frame in mjpeg_frames(cv2, np, FRAME_WIDTH, FRAME_HEIGHT, args.fps):
            try:
                cmd = infer(frame)
            except Exception as e:
                frame = error_frame(f"INFER ERR: {e}")
                cmd = "ERR"
            print(f"[debug_vision] frame={count} cmd={cmd}", file=sys.stderr)
            write_frame(cv2, frame)
            count += 1
            if args.frames and count >= args.frames:
                break

    else:  # still
        while args.frames == 0 or count < args.frames:
            frame = capture_frame(cv2, np, FRAME_WIDTH, FRAME_HEIGHT)
            if frame is None:
                frame = error_frame("CAPTURE FAILED")
                write_frame(cv2, frame)
                count += 1
                continue
            try:
                cmd = infer(frame)
            except Exception as e:
                frame = error_frame(f"INFER ERR: {e}")
                cmd = "ERR"
            print(f"[debug_vision] frame={count} cmd={cmd}", file=sys.stderr)
            write_frame(cv2, frame)
            count += 1


if __name__ == "__main__":
    main()
