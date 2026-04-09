# olympus_hlc/odometry.py — Receptor del EKF del LLC + ground-truth comparison (v3.1)
#
# Recibe la pose filtrada (x, y, theta) directamente del LLC vía UART.
# El LLC ejecuta el EKF a 50 Hz con fusión de encoders + IMU (MPU-6050).
#
# Formato TLM esperado:
#   TLM:<SAFETY>:<STALL>:<TS>ms:<MV>mV:<MA>mA:...:<EL>:<ER>:<X_mm>:<Y_mm>:<Theta_mrad>\n
#
# Ground truth (RF-004-R1, RNF-003):
#   La comparación con ground truth se realiza midiendo una distancia física
#   conocida (cinta métrica) y comparando con segment_distance_mm().
#   error_vs_ground_truth(gt_mm) retorna el error relativo (%) para registrar
#   en verificacion.tex §subsec:vv_odometria.

import math


class OdometryTracker:
    """
    Mantiene la pose estimada del rover (x, y, theta) recibida del EKF del LLC
    y provee las operaciones de comparación con ground truth necesarias para
    verificar RF-004-R1 (error odométrico < 5 % en distancia total).
    """

    def __init__(self) -> None:
        self.x_mm:      float = 0.0
        self.y_mm:      float = 0.0
        self.theta_rad: float = 0.0

        # Punto de inicio del segmento de validación actual
        self._seg_x0: float = 0.0
        self._seg_y0: float = 0.0
        self._seg_active: bool = False

    # ── Actualización desde TLM ───────────────────────────────────────────────

    def update_from_ekf(self, x_mm: int, y_mm: int, theta_mrad: int) -> None:
        """Actualiza la pose directamente desde el frame TLM del LLC."""
        self.x_mm      = float(x_mm)
        self.y_mm      = float(y_mm)
        self.theta_rad = float(theta_mrad) / 1000.0

    def pose(self) -> tuple[float, float, float]:
        """Retorna (x_mm, y_mm, theta_rad) — pose actual estimada por EKF."""
        return (self.x_mm, self.y_mm, self.theta_rad)

    def reset(self) -> None:
        """Reinicia la pose a origen. Llamar al inicio de cada misión."""
        self.x_mm      = 0.0
        self.y_mm      = 0.0
        self.theta_rad = 0.0
        self._seg_active = False

    # ── Ground truth (RF-004-R1) ──────────────────────────────────────────────

    def start_segment(self) -> None:
        """
        Marca el inicio de un segmento de validación de odometría.

        Procedimiento en campo:
          1. Colocar el rover en el punto de inicio marcado con cinta.
          2. Llamar a start_segment().
          3. Desplazar el rover la distancia de referencia.
          4. Llamar a error_vs_ground_truth(distancia_real_mm).
        """
        self._seg_x0     = self.x_mm
        self._seg_y0     = self.y_mm
        self._seg_active = True

    def segment_distance_mm(self) -> float:
        """
        Distancia Euclidiana (mm) recorrida desde start_segment().
        Retorna 0.0 si no hay segmento activo.
        """
        if not self._seg_active:
            return 0.0
        dx = self.x_mm - self._seg_x0
        dy = self.y_mm - self._seg_y0
        return math.sqrt(dx * dx + dy * dy)

    def error_vs_ground_truth(self, ground_truth_mm: float) -> float:
        """
        Error relativo (%) entre la distancia estimada por odometría y la
        distancia medida con cinta métrica (ground truth).

        Criterio de aceptación RF-004-R1 / RNF-003: retorno < 5.0 %.

        Args:
            ground_truth_mm: distancia física real medida con cinta (mm).

        Returns:
            Error relativo en porcentaje. 0.0 si ground_truth_mm == 0.
        """
        if ground_truth_mm == 0.0:
            return 0.0
        estimated = self.segment_distance_mm()
        return abs(estimated - ground_truth_mm) / ground_truth_mm * 100.0

    def log_ground_truth_result(self, ground_truth_mm: float, log=None) -> dict:
        """
        Calcula y registra el resultado de la prueba de ground truth.

        Retorna un dict con los campos necesarios para la tabla de resultados
        de verificacion.tex §subsec:vv_odometria:
          {estimated_mm, ground_truth_mm, error_pct, pass_rf004}
        """
        estimated = self.segment_distance_mm()
        error_pct = self.error_vs_ground_truth(ground_truth_mm)
        result = {
            "estimated_mm":    round(estimated, 1),
            "ground_truth_mm": round(ground_truth_mm, 1),
            "error_pct":       round(error_pct, 2),
            "pass_rf004":      error_pct < 5.0,
        }
        if log:
            status = "PASS" if result["pass_rf004"] else "FAIL"
            log.info(
                "NAV",
                f"RF-004-R1 [{status}] estimado={estimated:.1f}mm "
                f"gt={ground_truth_mm:.1f}mm error={error_pct:.2f}%"
            )
        return result
