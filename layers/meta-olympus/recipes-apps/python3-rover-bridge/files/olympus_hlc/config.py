# olympus_hlc/config.py — Configuration loader and runtime constants
#
# Busca la configuración en orden:
#   1. /etc/olympus/olympus_controller.yaml  (producción — instalado por Yocto)
#   2. configs/olympus_controller.yaml       (desarrollo — junto al paquete)
#
# Si ningún archivo existe o PyYAML no está disponible, todos los parámetros
# usan los valores por defecto definidos aquí.

from pathlib import Path


def _load_config() -> dict:
    candidates = [
        Path("/etc/olympus/olympus_controller.yaml"),
        Path(__file__).parent.parent / "configs" / "olympus_controller.yaml",
    ]
    try:
        import yaml
        for path in candidates:
            if path.exists():
                with open(path) as f:
                    return yaml.safe_load(f) or {}
    except ImportError:
        pass
    return {}


_cfg = _load_config()

# ─── Timing ──────────────────────────────────────────────────────────────────

PING_INTERVAL_S   = float(_cfg.get("ping_interval_s",   1.0))   # Max s entre comandos antes de PING
TLM_WARN_S        = float(_cfg.get("tlm_warn_s",        5.0))   # Sin TLM → advertencia
TLM_RETREAT_S     = float(_cfg.get("tlm_retreat_s",     10.0))  # Sin TLM → RET (SYS-FUN-021)
TLM_STB_S         = float(_cfg.get("tlm_stb_s",         30.0))  # Sin TLM → STB (COMM-REQ-005)
CYCLE_WARN_MS     = int  (_cfg.get("cycle_warn_ms",     1500))  # Umbral ciclo lento (RNF-001)
CYCLE_LOG_PERIOD  = int  (_cfg.get("cycle_log_period",  50))    # Cada N ciclos loguear tiempo
TLM_INTERVAL_WARN_S = float(_cfg.get("tlm_interval_warn_s", 2.0))  # Delta TLMs (SyRS-017)

# ─── Navigation ──────────────────────────────────────────────────────────────

RETREAT_DIST_MM   = int  (_cfg.get("retreat_dist_mm",   300))   # Distancia táctica HLC (SyRS-061)
MAX_WAYPOINTS     = int  (_cfg.get("max_waypoints",     5))     # Últimos N waypoints (SyRS-061)
SLIP_STALL_FRAMES = int  (_cfg.get("slip_stall_frames", 2))     # Frames consecutivos stall → RET (RF-004)

# ─── Energy / Thermal ────────────────────────────────────────────────────────
#
# Batería: pack 4S Li-ion 18650 NMC (ej. Samsung INR18650-30Q, Panasonic NCR18650B).
#
# Curva SoC-V para celdas NMC 18650 a descarga 1C (aprox.):
#   4.20 V/celda → 100 % SoC  (carga completa)
#   3.80 V/celda →  60 % SoC
#   3.60 V/celda →  30 % SoC  (voltaje nominal)
#   3.50 V/celda →  20 % SoC  ← BATT_WARN  (tiempo para estacionar)
#   3.20 V/celda →   5 % SoC  ← BATT_CRITICAL (daño inminente por subdescarga)
#   3.00 V/celda →   0 % SoC  (corte absoluto del BMS)
#
# Ref.: Samsung SDI. (2015). INR18650-30Q Specification Sheet, Fig. 3 —
#       Discharge Curve at 25 °C, 0.2C/1C/2C.
# Ref.: Plett, G. L. (2015). Battery Management Systems, Vol. I: Battery
#       Modeling. Artech House. §1.3.2 — OCV-SoC curve for NMC chemistry.
#
# BATT_WARN_MV = 14 000 mV (3.5 V/celda × 4S):
#   ~20 % SoC → el rover tiene autonomía para volver a base de forma ordenada.
#
# BATT_CRITICAL_MV = 12 800 mV (3.2 V/celda × 4S):
#   ~5 % SoC → parada de emergencia; continuar descargando bajo 3.0 V/celda
#   causa reducción irreversible de capacidad en celdas NMC por deposición
#   de litio metálico en el ánodo de grafito.
#   Ref.: Vetter, J. et al. (2005). "Ageing mechanisms in lithium-ion batteries."
#   Journal of Power Sources, 147(1-2), 269-281. §3.1 — Li plating at low SoC.
#
# Temperatura: TEMP_WARN_C / TEMP_CRIT_C comparan contra temp_c del TLM,
#   que es la temperatura AMBIENTE medida por el LM335 (sensor en el PCB del
#   LLC, no en la superficie de la celda). El LLC usa sensores NTC en celda
#   para sus propios umbrales (BATT_WARN_C=45 / BATT_LIMIT_C=55 / BATT_FAULT_C=65).
#
#   Relación entre sensores bajo carga:
#     T_celda ≈ T_ambiente + 10–15 °C  (conducción + convección natural)
#   Por tanto, cuando T_ambiente (LM335) llega a TEMP_CRIT_C = 60 °C,
#   las celdas ya están a ~70–75 °C, lo que significa que el LLC ya habrá
#   disparado BATT_FAULT_C = 65 °C y SafeMode se habrá activado por
#   tlm.safety == "FAULT" ANTES de que TEMP_CRIT_C se alcance en condiciones
#   normales. TEMP_CRIT_C actúa como red de seguridad secundaria para el
#   caso en que el sensor NTC de celda falle (pin ADC flotante o desconexión).
#   Ref.: IEC 62133-2:2017 §4.3.8 — Temperature limits for secondary
#   lithium cells in portable equipment.

BATT_WARN_MV      = int  (_cfg.get("batt_warn_mv",      14000)) # 3.5 V/celda × 4S ≈ 20 % SoC (EPS-REQ-001)
BATT_CRITICAL_MV  = int  (_cfg.get("batt_critical_mv",  12800)) # 3.2 V/celda × 4S ≈  5 % SoC → STB inmediato
TEMP_WARN_C       = int  (_cfg.get("temp_warn_c",       45))    # Temperatura AMBIENTE (LM335) → advertencia (RNF-004)
TEMP_CRIT_C       = int  (_cfg.get("temp_crit_c",       60))    # Temperatura AMBIENTE (LM335) → Safe Mode; red secundaria (ver nota arriba)

# ─── Power management (SYS-FUN-040) ─────────────────────────────────────────
#
# Cuando SafeMode se activa por batería crítica, el HLC envía STB al LLC y
# luego programa el apagado del sistema operativo (RPi5) tras POWEROFF_DELAY_S
# segundos. El delay da tiempo al LLC para procesar el comando SAFE y al log
# para sincronizarse a almacenamiento no volátil antes de cortar la alimentación.
#
# POWEROFF_ENABLED = false desactiva el poweroff — OBLIGATORIO en dry-run y
# tests para evitar apagar la máquina de desarrollo durante pytest.
#
# Ref.: ESA PSS-05-0 Issue 2 (1991) §6.2.3 — Orderly system shutdown on
#       power-fail detection. Equivalente en sistemas embebidos modernos:
#       "soft poweroff with state persistence before rail collapse."

POWEROFF_DELAY_S = int (_cfg.get("poweroff_delay_s", 5))    # s entre SafeMode y apagado OS
POWEROFF_ENABLED = bool(_cfg.get("poweroff_enabled", True)) # False en dry-run/testing

# ─── Storage ─────────────────────────────────────────────────────────────────

STORAGE_MIN_MB       = int(_cfg.get("storage_min_mb",       50))   # MB libres mínimos (SRS-014)
STORAGE_CHECK_CYCLES = int(_cfg.get("storage_check_cycles", 300))  # Cada N ciclos verificar disco

# ─── Vision ──────────────────────────────────────────────────────────────────
#
# Modo bbox (VISION_MODE="bbox"):
#   Filtro único: fracción de área del bounding box respecto al blob 640×640 del
#   modelo ≥ VISION_AREA_MIN. No hay validación secundaria; la decisión la toma
#   la detección con mayor área. Por eso el umbral es estricto (5 %).
#   Ref.: Jocher, G. et al. (2023). Ultralytics YOLOv8. GitHub.
#   https://github.com/ultralytics/ultralytics — conf=0.5 por defecto para
#   producción; area_min del 5 % descarta detecciones de fondo/ruido sin perder
#   obstáculos a < 1.5 m (≈ 5 s de reacción a 0.3 m/s).
#
# Modo segmentación (VISION_MODE="segmentation"):
#   Filtrado en dos etapas:
#     1. PRE-FILTRO (SEG_AREA_MIN=0.03): descarta detecciones con bbox area < 3 %
#        ANTES del decode costoso (sigmoid + resize de la máscara 160×160).
#        Umbral más laxo que bbox porque la decisión final la da la etapa 2.
#     2. VALIDACIÓN DE ZONA (SEG_ZONE_MIN=0.05): cobertura de máscara combinada
#        en el ROI inferior. Una detección puede pasar el pre-filtro (3 %) pero
#        no generar comando si no cubre el 5 % de ninguna zona.
#   Ref.: Bolya, D. et al. (2022). "YOLACT++: Better Real-time Instance
#   Segmentation." IEEE TPAMI 44(2), 1108-1121. — Tabla 3, mask-quality
#   filtering with area thresholds in two-stage pipelines.
#   Ref.: Jocher, G. et al. (2023). Ultralytics YOLOv8. GitHub. §Segmentation.
#
# SEG_MASK_THRESHOLD: umbral de binarización de la máscara sigmoidal (0–1 → bool).
#   0.5 es el valor canónico del clasificador binario (probabilidad > 50 %).
#   Ref.: Redmon, J. & Farhadi, A. (2018). "YOLOv3: An Incremental Improvement."
#   arXiv:1804.02767. §2.2 — binary cross-entropy mask threshold.

FRAME_WIDTH       = int  (_cfg.get("frame_width",       640))
FRAME_HEIGHT      = int  (_cfg.get("frame_height",      480))
VISION_CONF_MIN   = float(_cfg.get("vision_conf_min",   0.5))    # Confianza mínima detección
VISION_AREA_MIN   = float(_cfg.get("vision_area_min",   0.05))   # Área bbox / 640² ≥ 5 % (único filtro en bbox mode)
ZONE_LEFT_END     = float(_cfg.get("zone_left_end",     0.33))   # 0–33 % ancho frame → AVD:R
ZONE_RIGHT_START  = float(_cfg.get("zone_right_start",  0.67))   # 67–100 % ancho frame → AVD:L
EXP_SPEED_L       = int  (_cfg.get("exp_speed_l",       25))
EXP_SPEED_R       = int  (_cfg.get("exp_speed_r",       25))

# Segmentation pipeline (GNC-REQ-002)
VISION_MODE       = str  (_cfg.get("vision_mode",       "bbox"))
SEG_MODEL_PATH    = str  (_cfg.get("seg_model_path",
                          "/usr/share/olympus/models/yolov8n-seg.onnx"))
SEG_CONF_MIN      = float(_cfg.get("seg_conf_min",      0.5))    # Conf. mínima (igual que bbox)
SEG_AREA_MIN      = float(_cfg.get("seg_area_min",      0.03))   # Pre-filtro bbox / 640² ≥ 3 % (antes de decode de máscara)
SEG_ZONE_MIN      = float(_cfg.get("seg_zone_min",      0.05))   # Cobertura de zona ≥ 5 % para emitir comando (validación final)
SEG_ROI_TOP       = float(_cfg.get("seg_roi_top",       0.5))    # Ignorar mitad superior del frame (obstáculos siempre en mitad inferior)
SEG_MASK_THRESHOLD = float(_cfg.get("seg_mask_threshold", 0.5))  # Binarización sigmoide → bool (umbral canónico clasificador binario)

# ─── Performance tuning (vision mode) ─────────────────────────────────────────
#
# CPU governor pinning (sistema de recomendaciones B + D). arm_freq está FIJO
# en boot por config.txt (firmware) y NO se puede subir en runtime — para
# pasar de 1500 a 2400 MHz hay que poner
#     arm_freq=2400\nover_voltage=2
# en RPI_EXTRA_CONFIG (build/conf/local.conf) y re-flashear. El knob de runtime
# honesto es (B) fijar el governor a "performance" para que el CPU no baje de
# frecuencia entre frames, y (D) poner scaling_min_freq = scaling_max_freq para
# que la frecuencia se siente en su cap de boot durante toda la sesión de
# visión. Ambos se revierten al valor previo en VisionSource.close().
#
# Requieren acceso de escritura a /sys/devices/system/cpu/cpu*/cpufreq/*: en
# producción el HLC corre como root (ssh root@<IP>, seud olympus_hlc), así que
# funciona sin reglas extra. Las custom-udev-rules que se despachan hoy NO
# cubren cpufreq sysfs (solo tty e i2c); si se lanza el controlador como usuario
# no-root, extremeá una regla udev adicional (RUN+= chmod) — fuera del alcance
# actual. Sin acceso de escritura, _pin_cpu lo loguea como warning y continúa.
#
# Ref.: Linux CPUFreq governor documentation, §2.4 "performance governor".

GOVERNOR_PIN       = bool(_cfg.get("governor_pin",       True))   # Activar pinning en init / revert en close
GOVERNOR_VISION    = str (_cfg.get("governor_vision",    "performance"))  # Governor durante visión
GOVERNOR_DEFAULT   = str (_cfg.get("governor_default",   "ondemand"))     # Restaurado al cerrar

# ─── Capture backend ─────────────────────────────────────────────────────────
#
# Recomendación C: mantener el nodo libcamera caliente entre frames en lugar de
# spawnear un `rpicam-still` POR frame (~300–700 ms de init de libcamera cada
# vez). `rpicam-vid --codec mjpeg --output -` streamea bytes MJPEG a stdout que
# parseamos incrementalmente; la cámara AWB/AEC converge una sola vez y se
# mantiene caliente → esperado ~30–80 ms por frame.
#
# "rpicam-vid"  (default) — stream MJPEG persistente sobre un único Popen.
# "rpicam-still"         — subprocess por frame heredado (fallback, ~500 ms/fr).
# Si el stream rpicam-vid no entrega un frame en CAPTURE_TIMEOUT_S, la fuente
# cae automáticamente a rpicam-still por el resto de la sesión.
#
# Ref.: Raspberry Pi libcamera-apps, `rpicam-vid(1)` §Output, "--codec mjpeg".
CAPTURE_METHOD     = str (_cfg.get("capture_method",      "rpicam-vid"))
CAPTURE_FRAMERATE  = int (_cfg.get("capture_framerate",    4))       # fps del stream
CAPTURE_TIMEOUT_S  = float(_cfg.get("capture_timeout_s",   3.0))     # s antes de fallback

# ─── Inference backend (recomendación E) ──────────────────────────────────────
#
# "opencv" (default)     — cv2.dnn.readNetFromONNX; funciona con la imagen
#                          mínima actual (solo python3-opencv). Solo FP32.
# "onnxruntime"          — ort.InferenceSession(providers=["CPUExecutionProvider"]);
#                          más rápido (conv fusionadas MLAS + thread pool) y
#                          NECESARIO para cargar los artefactos *_int8.onnx en
#                          files/models-optimized/. Requiere `onnxruntime` en
#                          IMAGE_INSTALL (meta-onnxruntime) — rebuild obligatorio
#                          para habilitarlo (do_configure[network]=1 en la receta).
#                          Si la importación falla en runtime, cae a opencv con
#                          un warning.
#
# Ref.: onnxruntime Python API, `InferenceSession`, CPUExecutionProvider;
#       MLAS fused aarch64 conv kernels.
INFERENCE_BACKEND   = str (_cfg.get("inference_backend",  "opencv"))

# INFER_INPUT_SIZE: lado del blob alimentado al modelo. DEBE coincidir con la
# forma de exportación del ONNX (640 para yolov8n / yolov8n-seg, 384 lunar_seg).
# Bajarlo a 480 (recomendación F) exige RE-EXPORTAR el modelo con ese imgsz —
# no se puede alimentar 480×480 a un grafo ONNX de 640×640. El default mantiene
# los modelos despachados hoy.
INFER_INPUT_SIZE    = int (_cfg.get("infer_input_size",    640))

# ─── System monitor (recursos del RPi5 → GUI) ─────────────────────────────────
#
# SystemMonitor (olympus_hlc/sysmon.py) muestrea CPU %, RAM usada/total y
# temperatura del SoC del RPi5 leyendo directamente de /proc/stat, /proc/meminfo
# y /sys/class/thermal/* — SIN psutil — y publica a la GUI vía frames SYS:
# (formato: "SYS:<cpu%>,<ram_used_mb>,<ram_total_mb>,<temp_c>"). Sin dependencias
# extra en la imagen Yocto (python3-core basta).
#
# sys_sample_s: intervalo de muestreo. La primera lectura de CPU % requiere dos
# lecturas de /proc/stat; el baseline se toma en __init__, así que el primer
# sample() ya entrega un valor válido. 2.0 s es ~0.5 Hz — barato y suficiente
# para diagnóstico de latencia del vision loop.
SYS_MON_ENABLED = bool(_cfg.get("sys_mon_enabled", True))
SYS_SAMPLE_S    = float(_cfg.get("sys_sample_s",    2.0))

# ─── Navegacion lunar (modo --mode vision-nav) ──────────────────────────────
#
# Integracion del modelo de segmentacion semantica lunar del TFG de Carlos
# Alfaro (repo TFG_Quillo_CEA_ITCR) en el proyecto ELANAV (Olympus HLC).
#
# Segmentacion semantica lunar con UNetMobileNet (5 clases, 384² input).
# Modelo entrenado y exportado a ONNX por Carlos; la logica de navegacion
# (umbrales MIN_FORWARD, DELTA_SIDE, DELTA_CENTER y zonas 40/60%) se calibro
# en su dataset lunar y se reutiliza aqui sin modificaciones.
#
# Clases del modelo:
#   0 = Regolith  (navegable — terrain por donde el rover transita)
#   1 = Crater    (obstáculo — depresión, riesgo de atrapamiento)
#   2 = Rock      (obstáculo — roca, riesgo de colisión)
#   3 = Mountain  (obstáculo — pared/elevación infranqueable)
#   4 = Sky       (irrelevante — horizonte, no navegable ni obstáculo)
#
# NAV_CLASSES define qué clases se consideran transitables. Por defecto solo
# Regolith (0). En terreno con cráters muy suaves se podría añadir 1, pero
# es riesgoso — calibrar en campo.
#
# Umbrales de decisión (calibrados por Carlos en dataset lunar):
#   MIN_FORWARD   — ratio mín. de navegable en centro para avanzar (0.18)
#   DELTA_CENTER  — margen: si un lateral supera al centro por esto, girar (0.06)
#   DELTA_SIDE    — diff. mín. entre laterales para decidir giro (0.05)
#
# Zonas de decisión (fracciones de ancho del frame):
#   0 ─── 0.40 ─── 0.60 ─── 1
#      IZQUIERDA  CENTRO  DERECHA
#   Más anchas que las de YOLO (0.33/0.67) porque el modelo lunar prioriza
#   el centro — si hay terrain navegable al frente, avanzar.

LUNAR_MODEL_PATH    = str  (_cfg.get("lunar_model_path",
                              "/usr/share/olympus/models/lunar_seg.onnx"))
LUNAR_MODEL_H       = int  (_cfg.get("lunar_model_h",       384))
LUNAR_MODEL_W       = int  (_cfg.get("lunar_model_w",       384))
LUNAR_NAV_CLASSES   = list (_cfg.get("lunar_nav_classes",   [0]))
LUNAR_MIN_FORWARD   = float(_cfg.get("lunar_min_forward",   0.18))
LUNAR_DELTA_SIDE    = float(_cfg.get("lunar_delta_side",    0.05))
LUNAR_DELTA_CENTER  = float(_cfg.get("lunar_delta_center",  0.06))
LUNAR_USE_TRAPEZOID = bool (_cfg.get("lunar_use_trapezoid", True))
LUNAR_ZONE_LEFT_END  = float(_cfg.get("lunar_zone_left_end",  0.40))
LUNAR_ZONE_RIGHT_START = float(_cfg.get("lunar_zone_right_start", 0.60))

# ─── SLAM semantico (modo --mode vision-nav) ────────────────────────────────
#
# Mapa de ocupacion semantico grid-based que integra mascaras de segmentacion
# lunar proyectadas a coordenadas mundo. Adaptado del TFG de Carlos Alfaro.
#
# La pose del rover se obtiene de OdometryTracker (encoders, TLM v1.1) o EKF
# (TLM v1.2, pendiente de integrar). El SLAM recibe la pose externamente —
# diseno pluggable para cambiar la fuente sin modificar el modulo.
#
# CELL_M: tamano de celda en metros. 0.5m es suficiente para navegacion lunar
#   (obstaculos > 0.5m son relevantes; menores se ignora).
# MAP_W_M / MAP_H_M: dimensiones iniciales del mapa en metros. La grid se
#   expande dinamicamente si el rover se acerca al borde.

SLAM_CELL_M  = float(_cfg.get("slam_cell_m",  0.5))
SLAM_MAP_W_M = float(_cfg.get("slam_map_w_m", 40.0))
SLAM_MAP_H_M = float(_cfg.get("slam_map_h_m", 40.0))

# ─── GCS link (SRS-013, SYS-FUN-021) ─────────────────────────────────────────

GCS_LISTEN_PORT      = int  (_cfg.get("gcs_listen_port",      9000))
GCS_REPLY_PORT       = int  (_cfg.get("gcs_reply_port",       9001))
GCS_BIND_ADDR        = str  (_cfg.get("gcs_bind_addr",        "0.0.0.0"))
GCS_LINK_LOST_S      = float(_cfg.get("gcs_link_lost_s",      10.0))
GCS_RETRY_INTERVAL_S = float(_cfg.get("gcs_retry_interval_s", 5.0))
GCS_MAX_RETRIES      = int  (_cfg.get("gcs_max_retries",      3))

# ─── CSP (SRS-001, RF-006, SyRS-016) ─────────────────────────────────────────

CSP_ADDR_GCS     = int (_cfg.get("csp_addr_gcs",     1))
CSP_ADDR_HLC     = int (_cfg.get("csp_addr_hlc",     2))
CSP_ADDR_GCS_UHF = int (_cfg.get("csp_addr_gcs_uhf", 3))  # GCS via radio UHF (KISS)
CSP_PORT_TM      = int (_cfg.get("csp_port_tm",      10))
CSP_PORT_CMD     = int (_cfg.get("csp_port_cmd",     11))
CSP_PORT_HB      = int (_cfg.get("csp_port_hb",       1))
CSP_ENABLED      = bool(_cfg.get("csp_enabled",    True))
CSP_UHF_DEVICE   = str (_cfg.get("csp_uhf_device", "/dev/ttyTNC"))  # TNC/radio-módem UART
CSP_UHF_BAUD     = int (_cfg.get("csp_uhf_baud",    9600))

# ─── LibcspGCSSource UDP peer (ICD-CSP-001) ──────────────────────────────────
# IP del GCS que LibcspGCSSource usa como destino TX (telemetría).
# En campo: IP WiFi del laptop del operador.
# En desarrollo: "127.0.0.1" si GCS mock corre en la misma máquina.
# Se puede sobreescribir con --gcs-host <ip> al lanzar el HLC.
CSP_UDP_GCS_HOST = str(_cfg.get("csp_udp_gcs_host", ""))

# ─── Odometry (RNF-003) ───────────────────────────────────────────────────────
# Valores TBD — calibrar con hardware real (ver LLC config.rs comentarios).
# WHEEL_RADIUS_MM: radio de la rueda impresa PLA MAX (medir con calibrador).
# TICKS_PER_REV:   pulsos encoder Phase-A por vuelta eje salida (NFP-5840-31ZY-EN, TBD).
# WHEEL_BASE_MM:   distancia entre centros de contacto izq/der (track width).

WHEEL_RADIUS_MM  = int  (_cfg.get("wheel_radius_mm",  50))   # TBD — calibrar
TICKS_PER_REV    = int  (_cfg.get("ticks_per_rev",    20))   # TBD — calibrar
WHEEL_BASE_MM    = int  (_cfg.get("wheel_base_mm",    280))  # TBD — calibrar
