# olympus_hlc/sources/vision_nav.py — VisionNavSource: lunar semantic segmentation
#
# Integracion del modelo de segmentacion semantica lunar del TFG de Carlos
# Alfaro (repo TFG_Quillo_CEA_ITCR) en el proyecto ELANAV (Olympus HLC).
#
# El modelo UNetMobileNet (384x384, 5 clases lunares) fue entrenado y exportado
# a ONNX por Carlos en su TFG. La logica de navegacion (decide_direction con
# thirds + near/far weighting) y el preprocesado (CHW, /255, trapezoid ROI)
# se adaptaron de sus modulos navigation.py, perception.py y model.py.
#
# Cambios principales respecto al codigo original de Carlos:
#   - Runtime: TFLite Interpreter → cv2.dnn (ONNX) — sin dependencias nuevas
#   - Captura: ESP32-CAM MJPEG → rpicam-still CSI (camara OV5647 directa)
#   - Comando: WebSocket→ESP32→Mega (K/Q) → MSM directo al Arduino (FWD/AVD/RET)
#   - Source: integra con CommandSource del HLC, respeta overrides del HlcEngine
#
# Modelo ONNX:
#   Input:  [1, 3, 384, 384] float32 — NCHW, RGB, /255
#   Output: [1, 5, 384, 384] float32 — logits por pixel, 5 clases:
#     0=Regolith (navegable), 1=Crater, 2=Rock, 3=Mountain, 4=Sky
#
# Diferencias vs VisionSource (YOLOv8n):
#   - Semantic segmentation (pixel-level), no object detection (bbox)
#   - Input 384x384 (no 640x640), sin blobFromImage (preproc manual NCHW)
#   - Decision por transitabilidad: sigue terrain navegable, no evita objetos
#   - Logica de decide_direction thirds 40/60% + near/far weighting
#
# Mapeo de comandos:
#   ADELANTE → EXP:l:r   (avanzar recto)
#   IZQUIERDA → AVD:L    (girar izquierda, hacia terrain navegable)
#   DERECHA  → AVD:R     (girar derecha, hacia terrain navegable)

from ..interfaces import CommandSource
from ..config import (
    FRAME_WIDTH, FRAME_HEIGHT,
    EXP_SPEED_L, EXP_SPEED_R,
    LUNAR_MODEL_PATH,
    LUNAR_MODEL_H, LUNAR_MODEL_W,
    LUNAR_NAV_CLASSES,
    LUNAR_MIN_FORWARD, LUNAR_DELTA_SIDE, LUNAR_DELTA_CENTER,
    LUNAR_USE_TRAPEZOID,
    LUNAR_ZONE_LEFT_END, LUNAR_ZONE_RIGHT_START,
)


class VisionNavSource(CommandSource):
    """
    Lee frames de la camara CSI y decide comandos MSM via segmentacion
    semantica lunar (UNetMobileNet ONNX, cv2.dnn).

    Pipeline:
      1. Captura JPEG via rpicam-still (CSI OV5647)
      2. Preprocesado: resize 384x384, BGR→RGB, /255, HWC→CHW, expand batch
      3. Inferencia: cv2.dnn.forward() → [1,5,384,384] logits
      4. Postprocesado: argmax channel → mascara de clases → isin(NAV_CLASSES)
         → mascara binaria navegable → resize NEAREST al frame original
      5. Trapezoid ROI (opcional): recorta bordes superiores del frame
      6. decide_direction: thirds 40/60% + near/far weighting → comando MSM

    La mascara navegable identifica pixeles donde el modelo predice Regolith
    (clase 0) — el terrain por donde el rover puede transitar.
    """

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

        import os
        path = model_path if model_path else LUNAR_MODEL_PATH
        if not os.path.exists(path):
            print(f"[VisionNav] WARNING: lunar model not found at {path} — "
                  f"using default {LUNAR_MODEL_PATH}")
            path = LUNAR_MODEL_PATH
            if not os.path.exists(path):
                print(f"[VisionNav] ERROR: default model not found either.")
                raise SystemExit(1)

        print(f"[VisionNav] Loading lunar segmentation model: {path}")
        self._net = self._cv2.dnn.readNetFromONNX(path)
        print(f"[VisionNav] Model loaded — {len(self._net.getLayerNames())} layers")
        print(f"[VisionNav] Input: [1,3,{LUNAR_MODEL_H},{LUNAR_MODEL_W}] "
              f"Output: [1,5,{LUNAR_MODEL_H},{LUNAR_MODEL_W}]")
        print(f"[VisionNav] Camera ready (rpicam-still per-frame).")

        # Ultima mascara de clases (para SLAM — el engine la lee despues de
        # next_command() y la pasa a SemanticSLAM.integrate_observation()).
        self._last_class_mask = None

    @property
    def last_class_mask(self):
        """Mascara de clases [H, W] uint8 del ultimo frame inferido (para SLAM)."""
        return self._last_class_mask

    def _capture_frame(self):
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

    def _preprocess(self, frame):
        """Preprocesa el frame BGR al formato de entrada del modelo.

        Pasos:
          1. Resize a (MODEL_W, MODEL_H) = 384x384
          2. BGR → RGB
          3. Cast float32, /255
          4. HWC → CHW (transpose)
          5. Expand dims batch → [1, 3, 384, 384]

        Retorna ndarray float32 [1,3,384,384].
        """
        np = self._np
        cv2 = self._cv2

        img = cv2.resize(frame, (LUNAR_MODEL_W, LUNAR_MODEL_H))
        img = cv2.cvtColor(img, self._cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, 0)

        return img

    def _infer(self, blob):
        """Ejecuta forward pass y retorna mascara de clases [384,384] (int)."""
        np = self._np
        cv2 = self._cv2

        self._net.setInput(blob)
        output = self._net.forward()

        # output: [1, 5, 384, 384] — logits por clase por pixel
        # argmax sobre axis=1 (canal de clases) → [384, 384] indice de clase
        mask = np.argmax(output[0], axis=0)
        return mask.astype(np.uint8)

    def _create_nav_mask(self, class_mask):
        """Convierte mascara de clases a mascara binaria navegable.

        NAV_CLASSES = [0] (Regolith) → 1 = navegable, 0 = obstaculo.
        """
        return self._np.isin(class_mask, LUNAR_NAV_CLASSES).astype(self._np.uint8)

    def _trapezoid_roi(self, shape):
        """Genera mascara trapezoidal para enfocar el ROI central del frame.

        El trapecio descarta los bordes laterales y la parte superior del frame
        donde suelen aparecer falsos positivos (bordes de la camara, cielo).
        Si LUNAR_USE_TRAPEZOID es False, retorna mascara de unos.
        """
        np = self._np
        cv2 = self._cv2
        h, w = shape

        if not LUNAR_USE_TRAPEZOID:
            return np.ones((h, w), dtype=np.uint8)

        mask = np.zeros((h, w), dtype=np.uint8)
        pts = np.array([
            (int(w * 0.05), int(h * 0.10)),
            (int(w * 0.95), int(h * 0.10)),
            (int(w * 0.98), int(h * 0.50)),
            (int(w * 0.02), int(h * 0.50)),
        ], dtype=np.int32)
        cv2.fillPoly(mask, [pts], 1)
        return mask

    def _decide_direction(self, nav_mask, roi_mask):
        """Decide el comando MSM basado en la cobertura navegable por zona.

        Zonas (adaptadas de Carlos, navigation.py):
          Left   (0–40%)    — si mas navegable aqui → girar izquierda (AVD:L)
          Center (40–60%)   — si navegable aqui → avanzar (EXP:l:r)
          Right  (60–100%)  — si mas navegable aqui → girar derecha (AVD:R)

        Weighting near/far en el centro:
          70% peso mitad inferior (near — terreno inmediato)
          30% peso mitad superior (far — terreno lejano)

        Umbrales (de config.py, calibrados por Carlos):
          MIN_FORWARD  — ratio minimo en centro para avanzar
          DELTA_CENTER — margen: si un lateral es mucho mejor que el centro, girar
          DELTA_SIDE   — diferencia minima entre laterales para decidir giro

        Retorna (comando_msm, center_ratio, left_ratio, right_ratio).
        """
        np = self._np

        region = nav_mask * roi_mask
        h, w = region.shape

        left_end = int(w * LUNAR_ZONE_LEFT_END)
        right_start = int(w * LUNAR_ZONE_RIGHT_START)

        left   = region[:, :left_end]
        center = region[:, left_end:right_start]
        right  = region[:, right_start:]

        near_h = int(h * 0.6)

        center_ratio = (
            0.7 * np.mean(center[near_h:, :]) +
            0.3 * np.mean(center[:near_h, :])
        )

        left_ratio  = np.mean(left)
        right_ratio = np.mean(right)

        best_side = max(left_ratio, right_ratio)
        side_diff = abs(left_ratio - right_ratio)

        if center_ratio > LUNAR_MIN_FORWARD and (best_side - center_ratio) < LUNAR_DELTA_CENTER:
            decision = "ADELANTE"
        elif side_diff > LUNAR_DELTA_SIDE:
            decision = "IZQUIERDA" if left_ratio > right_ratio else "DERECHA"
        else:
            decision = "ADELANTE"

        return decision, center_ratio, left_ratio, right_ratio

    def _decision_to_msm(self, decision: str) -> str:
        """Mapea decision de navegacion a comando MSM."""
        if decision == "ADELANTE":
            return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"
        elif decision == "IZQUIERDA":
            return "AVD:L"
        elif decision == "DERECHA":
            return "AVD:R"
        return f"EXP:{EXP_SPEED_L}:{EXP_SPEED_R}"

    def next_command(self, log=None) -> "str | None":
        """Captura un frame, ejecuta segmentacion lunar y retorna comando MSM."""
        frame = self._capture_frame()
        if frame is None:
            print("[VisionNav] Frame capture failed.")
            return None

        cmd, _mask = self.infer(frame)
        return cmd

    def infer(self, frame) -> "tuple[str, object]":
        """
        Punto unico de decision reutilizable: retorna (comando_MSM, nav_mask).
        nav_mask es un ndarray uint8 [H, W] con 1=navegable, 0=obstaculo,
        para que la GUI dibuje el overlay (StationSource).
        """
        np = self._np
        cv2 = self._cv2

        H, W = frame.shape[:2]

        blob = self._preprocess(frame)
        class_mask = self._infer(blob)

        # Resize mascara de clases al tamaño del frame original
        class_mask_full = cv2.resize(
            class_mask, (W, H), interpolation=cv2.INTER_NEAREST
        )

        # Guardar mascara de clases para SLAM (el engine la lee via property)
        self._last_class_mask = class_mask_full

        # Mascara binaria navegable
        nav_mask = self._create_nav_mask(class_mask_full)

        # Trapezoid ROI
        roi_mask = self._trapezoid_roi((H, W))

        # Decision de navegacion
        decision, c_ratio, l_ratio, r_ratio = self._decide_direction(
            nav_mask, roi_mask
        )

        cmd = self._decision_to_msm(decision)

        if log:
            log.info(
                "VISION_NAV",
                f"decision={decision} cmd={cmd} "
                f"L={l_ratio:.2f} C={c_ratio:.2f} R={r_ratio:.2f}"
            )

        return (cmd, nav_mask)

    def close(self) -> None:
        pass
