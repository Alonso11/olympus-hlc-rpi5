# olympus_hlc/sources/vision.py — VisionSource: CSI camera + YOLOv8n inference

from ..interfaces import CommandSource
from ..config import (
    FRAME_WIDTH, FRAME_HEIGHT,
    VISION_CONF_MIN, VISION_AREA_MIN,
    ZONE_LEFT_END, ZONE_RIGHT_START,
    EXP_SPEED_L, EXP_SPEED_R,
    VISION_MODE, SEG_MODEL_PATH,
    SEG_CONF_MIN, SEG_AREA_MIN, SEG_ZONE_MIN, SEG_ROI_TOP, SEG_MASK_THRESHOLD,
    GOVERNOR_PIN, GOVERNOR_VISION, GOVERNOR_DEFAULT,
    CAPTURE_METHOD, CAPTURE_FRAMERATE, CAPTURE_TIMEOUT_S,
    INFERENCE_BACKEND, INFER_INPUT_SIZE,
)


class _MjpegStream:
    """
    Stream MJPEG persistente sobre un único `rpicam-vid --codec mjpeg --output -`.
    Mantiene el nodo libcamera caliente entre frames (recomendación C): la cámara
    AWB/AEC converge una sola vez y el subprocess se reutiliza, evitando el
    init de ~300–700 ms por frame de `rpicam-still` por llamada.
    """

    _SOI = b"\xff\xd8"   # JPEG Start Of Image
    _EOI = b"\xff\xd9"   # JPEG End Of Image

    def __init__(self, cmd, timeout_s, cv2mod, npmod):
        import os, select, subprocess, time
        self._os, self._select, self._subprocess, self._time = os, select, subprocess, time
        self._cv2, self._np = cv2mod, npmod
        self._timeout_s = timeout_s
        self._buf = bytearray()
        self._popen = self._subprocess.Popen(cmd, stdout=self._subprocess.PIPE,
                                             stderr=self._subprocess.DEVNULL)
        self._fd = self._popen.stdout.fileno()
        self._closed = False

    def next_jpeg(self):
        """Retorna los bytes de un JPEG completo, o None si fallback/cierre."""
        deadline = self._time.monotonic() + self._timeout_s
        while not self._closed:
            eoi = self._buf.find(self._EOI)
            if eoi != -1:
                soi = self._buf.find(self._SOI, 0, eoi)
                if soi != -1:
                    jpg = bytes(self._buf[soi : eoi + 2])
                    del self._buf[: eoi + 2]
                    return jpg
            remaining = deadline - self._time.monotonic()
            if remaining <= 0:
                return None
            r, _, _ = self._select.select([self._popen.stdout], [], [], min(remaining, 0.5))
            if not r:
                continue
            try:
                chunk = self._os.read(self._fd, 65536)
            except OSError:
                return None
            if not chunk:
                self._closed = True
                return None
            self._buf.extend(chunk)
        return None

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._popen.stdout.close()
        except OSError:
            pass
        if self._popen.poll() is None:
            self._popen.terminate()
            try:
                self._popen.wait(timeout=2)
            except Exception:
                self._popen.kill()


class VisionSource(CommandSource):
    """
    Lee frames de la cámara CSI y decide comandos MSM vía inferencia ONNX.

    Dos modos seleccionables por VISION_MODE (olympus_controller.yaml):
      "bbox"         — YOLOv8n ONNX, decisión por centro del bounding box.
      "segmentation" — YOLOv8n-seg ONNX, decisión por cobertura de máscara
                       por zona (GNC-REQ-002). Cae a bbox si el modelo no existe.

    Capture backend (CAPTURE_METHOD, recomendación C):
      "rpicam-vid"  (default) — stream MJPEG persistente; cámara caliente.
      "rpicam-still"          — subprocess por frame heredado; fallback auto.

    Inference backend (INFERENCE_BACKEND, recomendación E):
      "opencv" (default)      — cv2.dnn.readNetFromONNX (solo FP32, <2 MB extra).
      "onnxruntime"          — ort.InferenceSession (MLAS + thread pool;
                               necesita `onnxruntime` en la imagen). Cae a
                               opencv con warning si onnxruntime no importable.

    CPU governor (GOVERNOR_PIN, recomendaciones B+D): fija governor a
    "performance" y floor scaling_min_freq = max_freq durante la sesión; se
    revierte en close(). arm_freq mismo es config de boot (firmware) y no se
    puede subir en runtime — para > 1500 MHz usar RPI_EXTRA_CONFIG + reflash.

    El tamaño del blob (INFER_INPUT_SIZE, recomendación F) debe coincidir con
    la forma de exportación del ONNX; bajarlo exige re-exportar el modelo.

    Frame capture evita los nodos V4L2 de RPi5/pisp (no abribles con OpenCV
    directamente — de ahí el uso de rpicam-* vía stdout).

    Zonas (aplica a ambos modos):
      Izquierda  (0–zone_left_end)               → AVD:R
      Centro     (zone_left_end–zone_right_start) → RET
      Derecha    (zone_right_start–1)             → AVD:L
      Sin detección                               → EXP:<l>:<r>
    """

    _SEG_BBOX_FIELDS  = 4
    _SEG_CLASS_FIELDS = 80
    _SEG_COEFF_FIELDS = 32
    _SEG_PROTO_SIZE   = 160

    def __init__(self, model_path: str):
        try:
            import cv2
            import numpy as np
            self._cv2 = cv2
            self._np  = np
        except ImportError:
            print("[ERROR] OpenCV not found. Install python3-opencv.")
            raise SystemExit(1)
        import subprocess  # solo el path de fallback rpicam-still lo usa en caliente
        self._subprocess = subprocess

        self._mode = VISION_MODE
        self._backend = INFERENCE_BACKEND
        self._capture = CAPTURE_METHOD
        self._stream = None
        self._net = None
        self._ort = None
        self._in_name = None
        self._prev_gov = []
        self._prev_min = []

        # Auto-detect segmentation model from filename regardless of VISION_MODE config.
        # yolov8n-seg.onnx has 116-column output (4+80+32); _decide_bbox expects 84.
        if self._mode == "bbox" and "seg" in model_path.lower():
            print(f"[Vision] seg model detected in path — switching mode to segmentation")
            self._mode = "segmentation"

        # Resolver ruta del modelo efectiva.
        eff_path = model_path
        if self._mode == "segmentation":
            import os
            seg_path = model_path if "seg" in model_path.lower() else SEG_MODEL_PATH
            if not os.path.exists(seg_path):
                print(f"[Vision] WARNING: seg model not found at {seg_path} — "
                      f"falling back to bbox mode.")
                self._mode = "bbox"
            else:
                eff_path = seg_path

        # Cargar el backend de inferencia.
        self._load_backend(eff_path)

        # Performance: pin governor + freq floor (B + D).
        self._pin_cpu()

        # Captura: arrancar el stream persistente si corresponde (C).
        if self._capture == "rpicam-vid":
            self._start_stream()

        print(f"[Vision] Mode: {self._mode}. Backend: {self._backend}. "
              f"Capture: {self._capture} (input {INFER_INPUT_SIZE}²). "
              f"Camera {('warm (rpicam-vid)' if self._stream else 'per-frame (rpicam-still)')}.")

    # ── Backend de inferencia (E) ─────────────────────────────────────────────

    def _load_backend(self, model_path: str):
        # Intento onnxruntime primero si se pidió; fallback gracioso a opencv.
        if self._backend == "onnxruntime":
            try:
                import onnxruntime as ort
                so = ort.SessionOptions()
                so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                so.intra_op_num_threads = 0   # 0 = que ORT elija (#cores)
                self._ort = ort.InferenceSession(model_path, so,
                                                 providers=["CPUExecutionProvider"])
                self._in_name = self._ort.get_inputs()[0].name
                if self._mode == "segmentation":
                    print(f"[Vision] ORT seg model loaded — "
                          f"{len(self._ort.get_outputs())} outputs, in='{self._in_name}'")
                else:
                    print(f"[Vision] ORT bbox model loaded — in='{self._in_name}'")
                return
            except ImportError:
                print(f"[Vision] WARNING: onnxruntime not importable "
                      f"(¿paquete en IMAGE_INSTALL?) — falling back to opencv.")
                self._backend = "opencv"
            except Exception as e:
                print(f"[Vision] WARNING: onnxruntime load failed ({e!r}) — "
                      f"falling back to opencv.")
                self._backend = "opencv"

        # opencv (default o fallback).
        self._net = self._cv2.dnn.readNetFromONNX(model_path)
        nlayers = len(self._net.getLayerNames())
        print(f"[Vision] {'Seg' if self._mode == 'segmentation' else 'Bbox'} model "
              f"loaded (cv2.dnn) — {nlayers} layers")

    def _forward(self, blob):
        """Ejecuta el backend y retorna SIEMPRE una lista de ndarray outputs."""
        if self._backend == "onnxruntime":
            return list(self._ort.run(None, {self._in_name: blob}))
        self._net.setInput(blob)
        if self._mode == "segmentation":
            return list(self._net.forward(self._net.getUnconnectedOutLayersNames()))
        return [self._net.forward()]

    # ── CPU governor / freq floor (B + D) ──────────────────────────────────────

    @staticmethod
    def _read_sys(path):
        try:
            with open(path) as f:
                return f.read().strip()
        except OSError:
            return None

    @staticmethod
    def _write_sys(path, val):
        try:
            with open(path, "w") as f:
                f.write(val)
            print(f"[Vision] wrote '{val}' -> {path}")
            return True
        except OSError as e:
            print(f"[Vision] (warn) cannot write {path}: {e}")
            return False

    def _pin_cpu(self):
        import glob
        if not GOVERNOR_PIN:
            return
        pinned = 0
        for path in glob.glob("/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"):
            prev = self._read_sys(path)
            self._prev_gov.append((path, prev))
            if self._write_sys(path, GOVERNOR_VISION):
                pinned += 1
        # Floor: scaling_min_freq = scaling_max_freq (D) — sienta el cl el en su
        # cap de boot; no sube el cap (eso es boot-time, ver local.conf).
        for path in glob.glob("/sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq"):
            prev = self._read_sys(path)
            self._prev_min.append((path, prev))
            mxpath = path.replace("scaling_min_freq", "scaling_max_freq")
            mx = self._read_sys(mxpath)
            if mx:
                self._write_sys(path, mx)
        if pinned == 0:
            print(f"[Vision] (warn) governor pin had no effect "
                  f"(¿sin root / regla udev?). Continuando con governor actual.")

    def _unpin_cpu(self):
        for path, prev in self._prev_gov:
            if prev is not None:
                self._write_sys(path, prev)
        for path, prev in self._prev_min:
            if prev is not None:
                self._write_sys(path, prev)

    # ── Capture (C) ────────────────────────────────────────────────────────────

    def _start_stream(self):
        cmd = [
            "rpicam-vid",
            "--codec", "mjpeg",
            "--width",  str(FRAME_WIDTH),
            "--height", str(FRAME_HEIGHT),
            "--framerate", str(CAPTURE_FRAMERATE),
            "--rotation", "180",
            "--timeout", "0",      # hasta close()
            "--nopreview",
            "--output", "-",
        ]
        try:
            self._stream = _MjpegStream(cmd, CAPTURE_TIMEOUT_S, self._cv2, self._np)
            print(f"[Vision] rpicam-vid stream started ({CAPTURE_FRAMERATE} fps).")
        except FileNotFoundError:
            print(f"[Vision] rpicam-vid not found — falling back to rpicam-still.")
            self._capture = "rpicam-still"
            self._stream = None

    def _capture_still(self):
        """Captura un JPEG via rpicam-still --output -. Retorna ndarray BGR o None."""
        result = self._subprocess.run(
            [
                "rpicam-still",
                "--output", "-",
                "--width",  str(FRAME_WIDTH),
                "--height", str(FRAME_HEIGHT),
                "--rotation", "180",
                "--timeout", "1000",
                "--nopreview",
                "--encoding", "jpg",
            ],
            capture_output=True,
        )
        if result.returncode != 0 or not result.stdout:
            return None
        return self._cv2.imdecode(
            self._np.frombuffer(result.stdout, self._np.uint8),
            self._cv2.IMREAD_COLOR,
        )

    def _capture_frame(self):
        """Retorna un ndarray BGR o None, usando el backend de capture activo."""
        if self._capture == "rpicam-vid" and self._stream is not None:
            jpg = self._stream.next_jpeg()
            if jpg is None:
                print("[Vision] rpicam-vid stream stalled/ended — fallback rpicam-still.")
                self._stream.close()
                self._stream = None
                self._capture = "rpicam-still"
                return self._capture_still()
            return self._cv2.imdecode(
                self._np.frombuffer(jpg, self._np.uint8),
                self._cv2.IMREAD_COLOR,
            )
        return self._capture_still()

    def next_command(self, log=None) -> "str | None":
        """Captura un frame, ejecuta inferencia y retorna un comando MSM."""
        frame = self._capture_frame()
        if frame is None:
            print("[Vision] Frame capture failed.")
            return None
        cmd, _dets = self.infer(frame)
        return cmd

    def infer(self, frame) -> "tuple[str, list]":
        """
        Punto único de decisión YOLO reutilizable: retorna (comando_MSM, dets).
        `dets` es una lista de cajas normalizadas (x1, y1, x2, y2, conf) en [0,1]
        para que la GUI dibuje el overlay (StationSource las reenvía como DET:).
        En modo segmentación no se exponen cajas → dets vacío.
        """
        if self._mode == "segmentation":
            return (self._decide_seg(frame), [])
        return self._decide_bbox(frame)

    # ── Bbox mode ─────────────────────────────────────────────────────────────

    def _decide_bbox(self, frame) -> "tuple[str, list]":
        """YOLOv8n bbox: selecciona la detección más grande y mapea su cx a zona.

        Retorna (comando, dets) — dets son todas las cajas sobre umbral, para el
        overlay de la GUI; el comando se decide por el centro de la caja mayor.
        """
        cv2 = self._cv2
        np  = self._np

        sz = INFER_INPUT_SIZE
        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (sz, sz), swapRB=True, crop=False
        )
        output = self._forward(blob)[0]   # (1, 84, 8400)
        predictions = output[0].T        # (8400, 84)

        best_area = 0.0
        best_cx   = None
        dets: list = []

        for pred in predictions:
            scores     = pred[4:]
            class_id   = int(np.argmax(scores))
            confidence = float(scores[class_id])

            if confidence < VISION_CONF_MIN:
                continue

            cx_norm, cy_norm, w_norm, h_norm = pred[:4]
            cx        = cx_norm / sz
            area_frac = (w_norm / sz) * (h_norm / sz)

            if area_frac < VISION_AREA_MIN:
                continue

            # Caja normalizada (esquinas) para el overlay de la GUI.
            dets.append((
                (cx_norm - w_norm / 2) / sz,
                (cy_norm - h_norm / 2) / sz,
                (cx_norm + w_norm / 2) / sz,
                (cy_norm + h_norm / 2) / sz,
                confidence,
            ))

            if area_frac > best_area:
                best_area = area_frac
                best_cx   = cx

        if best_cx is None:
            return (f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}", dets)

        if best_cx < ZONE_LEFT_END:
            return ("AVD:R", dets)
        elif best_cx > ZONE_RIGHT_START:
            return ("AVD:L", dets)
        else:
            return ("RET", dets)

    # ── Segmentation mode (GNC-REQ-002) ──────────────────────────────────────

    def _decode_masks(self, output0, output1, frame_h: int, frame_w: int) -> list:
        """Decodifica salidas YOLOv8n-seg en máscaras binarias por detección."""
        np  = self._np
        cv2 = self._cv2
        sz  = INFER_INPUT_SIZE

        B = self._SEG_BBOX_FIELDS
        C = self._SEG_CLASS_FIELDS
        K = self._SEG_COEFF_FIELDS
        P = self._SEG_PROTO_SIZE

        preds      = output0[0].T   # [8400, 116]
        protos     = output1[0]     # [32, 160, 160]
        frame_area = frame_h * frame_w
        masks      = []

        for pred in preds:
            scores     = pred[B : B + C]
            confidence = float(scores.max())
            if confidence < SEG_CONF_MIN:
                continue

            cx_n, cy_n, w_n, h_n = pred[:B]
            area_frac = (w_n / sz) * (h_n / sz)
            if area_frac < SEG_AREA_MIN:
                continue

            coeffs   = pred[B + C : B + C + K]
            mask_160 = coeffs @ protos.reshape(K, P * P)
            mask_160 = 1.0 / (1.0 + np.exp(-mask_160))
            mask_160 = mask_160.reshape(P, P).astype(np.float32)
            mask_full = cv2.resize(mask_160, (frame_w, frame_h))

            x1 = max(0, int((cx_n - w_n / 2) * frame_w / sz))
            y1 = max(0, int((cy_n - h_n / 2) * frame_h / sz))
            x2 = min(frame_w, int((cx_n + w_n / 2) * frame_w / sz))
            y2 = min(frame_h, int((cy_n + h_n / 2) * frame_h / sz))

            binary = (mask_full > SEG_MASK_THRESHOLD)
            binary[:y1, :]  = False
            binary[y2:, :]  = False
            binary[:, :x1]  = False
            binary[:, x2:]  = False

            if binary.sum() < SEG_AREA_MIN * frame_area:
                continue

            masks.append(binary)

        return masks

    def _decide_seg(self, frame) -> str:
        """YOLOv8n-seg: cobertura de máscara por zona en el ROI inferior."""
        cv2 = self._cv2
        np  = self._np

        H, W = frame.shape[:2]

        sz = INFER_INPUT_SIZE
        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (sz, sz), swapRB=True, crop=False
        )
        outputs          = self._forward(blob)
        output0, output1   = outputs[0], outputs[1]

        masks = self._decode_masks(output0, output1, H, W)

        if not masks:
            return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"

        roi_y       = int(SEG_ROI_TOP * H)
        left_end    = int(ZONE_LEFT_END    * W)
        right_start = int(ZONE_RIGHT_START * W)

        combined = np.zeros((H, W), dtype=bool)
        for m in masks:
            combined |= m

        roi      = combined[roi_y:, :]
        roi_area = roi.shape[0]

        left_cov   = roi[:, :left_end].sum()  / max(roi_area * left_end, 1)
        center_cov = roi[:, left_end:right_start].sum() / max(
                         roi_area * (right_start - left_end), 1)
        right_cov  = roi[:, right_start:].sum() / max(roi_area * (W - right_start), 1)

        candidates = {"AVD:R": left_cov, "RET": center_cov, "AVD:L": right_cov}
        best_cmd, best_cov = max(candidates.items(), key=lambda kv: kv[1])

        if best_cov < SEG_ZONE_MIN:
            return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"

        return best_cmd

    def close(self) -> None:
        # (C) cerrar el stream MJPEG persistente si estaba activo.
        if self._stream is not None:
            self._stream.close()
            self._stream = None
        # (B+D) revertir governor / freq floor a sus valores previos.
        self._unpin_cpu()
