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
EXP_SPEED_L       = int  (_cfg.get("exp_speed_l",       40))
EXP_SPEED_R       = int  (_cfg.get("exp_speed_r",       40))

# Segmentation pipeline (GNC-REQ-002)
VISION_MODE       = str  (_cfg.get("vision_mode",       "bbox"))
SEG_MODEL_PATH    = str  (_cfg.get("seg_model_path",
                          "/usr/share/olympus/models/yolov8n-seg.onnx"))
SEG_CONF_MIN      = float(_cfg.get("seg_conf_min",      0.5))    # Conf. mínima (igual que bbox)
SEG_AREA_MIN      = float(_cfg.get("seg_area_min",      0.03))   # Pre-filtro bbox / 640² ≥ 3 % (antes de decode de máscara)
SEG_ZONE_MIN      = float(_cfg.get("seg_zone_min",      0.05))   # Cobertura de zona ≥ 5 % para emitir comando (validación final)
SEG_ROI_TOP       = float(_cfg.get("seg_roi_top",       0.5))    # Ignorar mitad superior del frame (obstáculos siempre en mitad inferior)
SEG_MASK_THRESHOLD = float(_cfg.get("seg_mask_threshold", 0.5))  # Binarización sigmoide → bool (umbral canónico clasificador binario)

# ─── GCS link (SRS-013, SYS-FUN-021) ─────────────────────────────────────────

GCS_LISTEN_PORT      = int  (_cfg.get("gcs_listen_port",      9000))
GCS_REPLY_PORT       = int  (_cfg.get("gcs_reply_port",       9001))
GCS_BIND_ADDR        = str  (_cfg.get("gcs_bind_addr",        "0.0.0.0"))
GCS_LINK_LOST_S      = float(_cfg.get("gcs_link_lost_s",      10.0))
GCS_RETRY_INTERVAL_S = float(_cfg.get("gcs_retry_interval_s", 5.0))
GCS_MAX_RETRIES      = int  (_cfg.get("gcs_max_retries",      3))

# ─── CSP (SRS-001, RF-006, SyRS-016) ─────────────────────────────────────────

CSP_ADDR_GCS  = int (_cfg.get("csp_addr_gcs",  1))
CSP_ADDR_HLC  = int (_cfg.get("csp_addr_hlc",  2))
CSP_PORT_TM   = int (_cfg.get("csp_port_tm",  10))
CSP_PORT_CMD  = int (_cfg.get("csp_port_cmd", 11))
CSP_PORT_HB   = int (_cfg.get("csp_port_hb",   1))
CSP_ENABLED   = bool(_cfg.get("csp_enabled", True))

# ─── Odometry (RNF-003) ───────────────────────────────────────────────────────
# Valores TBD — calibrar con hardware real (ver LLC config.rs comentarios).
# WHEEL_RADIUS_MM: radio de la rueda impresa PLA MAX (medir con calibrador).
# TICKS_PER_REV:   pulsos encoder Phase-A por vuelta eje salida (NFP-5840-31ZY-EN, TBD).
# WHEEL_BASE_MM:   distancia entre centros de contacto izq/der (track width).

WHEEL_RADIUS_MM  = int  (_cfg.get("wheel_radius_mm",  50))   # TBD — calibrar
TICKS_PER_REV    = int  (_cfg.get("ticks_per_rev",    20))   # TBD — calibrar
WHEEL_BASE_MM    = int  (_cfg.get("wheel_base_mm",    280))  # TBD — calibrar
