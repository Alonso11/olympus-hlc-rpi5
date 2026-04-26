# olympus_hlc/sources/vision.py — VisionSource: CSI camera + YOLOv8n inference

from ..interfaces import CommandSource
from ..config import (
    FRAME_WIDTH, FRAME_HEIGHT,
    VISION_CONF_MIN, VISION_AREA_MIN,
    ZONE_LEFT_END, ZONE_RIGHT_START,
    EXP_SPEED_L, EXP_SPEED_R,
    VISION_MODE, SEG_MODEL_PATH,
    SEG_CONF_MIN, SEG_AREA_MIN, SEG_ZONE_MIN, SEG_ROI_TOP, SEG_MASK_THRESHOLD,
)


class VisionSource(CommandSource):
    """
    Lee frames de la cámara CSI y decide comandos MSM vía cv2.dnn (YOLOv8n).

    Dos modos seleccionables por VISION_MODE (olympus_controller.yaml):
      "bbox"         — YOLOv8n ONNX, decisión por centro del bounding box.
      "segmentation" — YOLOv8n-seg ONNX, decisión por cobertura de máscara
                       por zona (GNC-REQ-002). Cae a bbox si el modelo no existe.

    Frame capture usa rpicam-still --output - (un JPEG por llamada) porque
    los nodos V4L2 de RPi5/pisp no se pueden abrir directamente con OpenCV.

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
            import subprocess
            self._cv2        = cv2
            self._np         = np
            self._subprocess = subprocess
        except ImportError:
            print("[ERROR] OpenCV not found. Install python3-opencv.")
            raise SystemExit(1)

        self._mode = VISION_MODE

        # Auto-detect segmentation model from filename regardless of VISION_MODE config.
        # yolov8n-seg.onnx has 116-column output (4+80+32); _decide_bbox expects 84.
        if self._mode == "bbox" and "seg" in model_path.lower():
            print(f"[Vision] seg model detected in path — switching mode to segmentation")
            self._mode = "segmentation"

        if self._mode == "segmentation":
            import os
            seg_path = model_path if "seg" in model_path.lower() else SEG_MODEL_PATH
            if not os.path.exists(seg_path):
                print(f"[Vision] WARNING: seg model not found at {seg_path} — "
                      f"falling back to bbox mode.")
                self._mode = "bbox"
            else:
                print(f"[Vision] Loading segmentation model: {seg_path}")
                self._net = self._cv2.dnn.readNetFromONNX(seg_path)
                print(f"[Vision] Seg model loaded — {len(self._net.getLayerNames())} layers")

        if self._mode == "bbox":
            print(f"[Vision] Loading bbox model: {model_path}")
            self._net = self._cv2.dnn.readNetFromONNX(model_path)
            print(f"[Vision] Bbox model loaded — {len(self._net.getLayerNames())} layers")

        print(f"[Vision] Mode: {self._mode}. Camera ready (rpicam-still per-frame).")

    def _capture_frame(self):
        """Captura un JPEG via rpicam-still --output -. Retorna ndarray BGR o None."""
        result = self._subprocess.run(
            [
                "rpicam-still",
                "--output", "-",
                "--width",  str(FRAME_WIDTH),
                "--height", str(FRAME_HEIGHT),
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

    def next_command(self, log=None) -> "str | None":
        """Captura un frame, ejecuta inferencia y retorna un comando MSM."""
        frame = self._capture_frame()
        if frame is None:
            print("[Vision] Frame capture failed.")
            return None

        if self._mode == "segmentation":
            return self._decide_seg(frame)
        return self._decide_bbox(frame)

    # ── Bbox mode ─────────────────────────────────────────────────────────────

    def _decide_bbox(self, frame) -> str:
        """YOLOv8n bbox: selecciona la detección más grande y mapea su cx a zona."""
        cv2 = self._cv2
        np  = self._np

        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (640, 640), swapRB=True, crop=False
        )
        self._net.setInput(blob)
        output = self._net.forward()     # (1, 84, 8400)
        predictions = output[0].T        # (8400, 84)

        best_area = 0.0
        best_cx   = None

        for pred in predictions:
            scores     = pred[4:]
            class_id   = int(np.argmax(scores))
            confidence = float(scores[class_id])

            if confidence < VISION_CONF_MIN:
                continue

            cx_norm, _, w_norm, h_norm = pred[:4]
            cx        = cx_norm / 640.0
            area_frac = (w_norm / 640.0) * (h_norm / 640.0)

            if area_frac < VISION_AREA_MIN:
                continue

            if area_frac > best_area:
                best_area = area_frac
                best_cx   = cx

        if best_cx is None:
            return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"

        if best_cx < ZONE_LEFT_END:
            return "AVD:R"
        elif best_cx > ZONE_RIGHT_START:
            return "AVD:L"
        else:
            return "RET"

    # ── Segmentation mode (GNC-REQ-002) ──────────────────────────────────────

    def _decode_masks(self, output0, output1, frame_h: int, frame_w: int) -> list:
        """Decodifica salidas YOLOv8n-seg en máscaras binarias por detección."""
        np  = self._np
        cv2 = self._cv2

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
            area_frac = (w_n / 640.0) * (h_n / 640.0)
            if area_frac < SEG_AREA_MIN:
                continue

            coeffs   = pred[B + C : B + C + K]
            mask_160 = coeffs @ protos.reshape(K, P * P)
            mask_160 = 1.0 / (1.0 + np.exp(-mask_160))
            mask_160 = mask_160.reshape(P, P).astype(np.float32)
            mask_full = cv2.resize(mask_160, (frame_w, frame_h))

            x1 = max(0, int((cx_n - w_n / 2) * frame_w / 640))
            y1 = max(0, int((cy_n - h_n / 2) * frame_h / 640))
            x2 = min(frame_w, int((cx_n + w_n / 2) * frame_w / 640))
            y2 = min(frame_h, int((cy_n + h_n / 2) * frame_h / 640))

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

        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (640, 640), swapRB=True, crop=False
        )
        self._net.setInput(blob)
        outputs        = self._net.forward(self._net.getUnconnectedOutLayersNames())
        output0, output1 = outputs[0], outputs[1]

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
        pass  # rpicam-still no deja procesos persistentes
