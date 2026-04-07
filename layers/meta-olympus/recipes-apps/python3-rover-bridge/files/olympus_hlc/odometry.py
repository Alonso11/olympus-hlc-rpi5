# olympus_hlc/odometry.py — Receptor del EKF del LLC (v3.0)
#
# Recibe la pose filtrada (x, y, theta) directamente del LLC vía UART.
# El LLC ejecuta un EKF a 50Hz con fusión de encoders + IMU.
#
# Formato TLM esperado: 
# TLM:<SAFETY>:<STALL>:<TS>ms:<MV>mV:<MA>mA:...:<EL>:<ER>:<X_mm>:<Y_mm>:<Theta_mrad>\n

import math

class OdometryTracker:
    def __init__(self) -> None:
        self.x_mm:      float = 0.0
        self.y_mm:      float = 0.0
        self.theta_rad: float = 0.0
        self.uncertainty: float = 0.0

    def update_from_ekf(self, x_mm: int, y_mm: int, theta_mrad: int) -> None:
        # Actualización directa desde el filtro de bajo nivel
        self.x_mm = float(x_mm)
        self.y_mm = float(y_mm)
        self.theta_rad = float(theta_mrad) / 1000.0

    def pose(self) -> tuple[float, float, float]:
        return (self.x_mm, self.y_mm, self.theta_rad)

    def reset(self) -> None:
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
