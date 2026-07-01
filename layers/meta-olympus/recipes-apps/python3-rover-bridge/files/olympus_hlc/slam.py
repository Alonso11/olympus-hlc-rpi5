# olympus_hlc/slam.py — Semantic SLAM: occupancy grid with lunar terrain classes
#
# Integracion del SLAM semantico del TFG de Carlos Alfaro (TFG_Quillo_CEA_ITCR)
# en el proyecto ELANAV (Olympus HLC).
#
# Mapa de ocupacion semantico grid-based que integra las mascaras de
# segmentacion lunar proyectadas a coordenadas mundo usando la pose del rover.
#
# Diferencias vs slam.py original de Carlos:
#   - Pose: dead-reckoning por comandos → OdometryTracker (encoders, TLM v1.1)
#     o EKF (TLM v1.2). La pose se recibe externamente, no se calcula aqui.
#   - Sin estado global: la grid, trayectoria y pose son atributos de instancia
#   - Diseno pluggable: integrate_observation(class_mask, x_m, y_m, theta_rad)
#     recibe la pose como parametro — cuando el EKF este listo solo se cambia
#     la fuente de pose en el engine, el SLAM no cambia.
#
# Grid de ocupacion:
#   0-4   = clases del modelo (0=Regolith, 1=Crater, 2=Rock, 3=Mountain, 4=Sky)
#   5     = FREE (espacio libre confirmado por raycasting)
#   255   = UNKNOWN (no observado)
#
# En integrate_observation, la clase 0 (Regolith) se salta — no aporta info
# al mapa. El espacio libre se pinta por raycasting desde el rover hacia cada
# pixel observado. Los obstaculos (1-3) se pintan sobre el free space.

import math

import numpy as np

from .config import (
    SLAM_CELL_M, SLAM_MAP_W_M, SLAM_MAP_H_M,
)


# Clases del modelo lunar (deben coincidir con config.py LUNAR_NAV_CLASSES)
CLASS_REGOLITH  = 0
CLASS_CRATER    = 1
CLASS_ROCK      = 2
CLASS_MOUNTAIN  = 3
CLASS_SKY       = 4

# Valores especiales en la grid
VAL_UNKNOWN = 255
VAL_FREE    = 5


class SemanticSLAM:
    """
    SLAM semantico grid-based para navegacion lunar.

    Mantiene un mapa de ocupacion 2D donde cada celda almacena la clase
    semantica del terrain (crater, rock, mountain) o espacio libre confirmado.
    La pose del rover se recibe de OdometryTracker (encoders) o EKF.

    Uso tipico desde HlcEngine:
      slam = SemanticSLAM()
      # cada ciclo de vision:
      slam.integrate_observation(class_mask, x_m, y_m, theta_rad)
      # consultar mapa:
      cell = slam.world_to_cell(x_m, y_m)
      val  = slam.grid[ry, rx]
    """

    def __init__(self, cell_m: float = SLAM_CELL_M,
                 map_w_m: float = SLAM_MAP_W_M,
                 map_h_m: float = SLAM_MAP_H_M):
        self.cell_m: float = cell_m

        self.map_w: int = int(map_w_m / cell_m)
        self.map_h: int = int(map_h_m / cell_m)

        self.grid: np.ndarray = np.full(
            (self.map_h, self.map_w), VAL_UNKNOWN, dtype=np.uint8
        )

        self.origin_x: int = self.map_w // 2
        self.origin_y: int = self.map_h // 2

        self.trajectory: list = [(0.0, 0.0, math.pi / 2)]

        self._pad: int = 120
        self._margin: int = 40

    # ── Coordenadas ───────────────────────────────────────────────────────────

    def world_to_cell(self, x_m: float, y_m: float) -> "tuple[int, int]":
        """Convierte coordenadas mundo (metros) a celda (rx, ry) en la grid."""
        rx = int(self.origin_x + x_m / self.cell_m)
        ry = int(self.origin_y - y_m / self.cell_m)
        return rx, ry

    # ── Expansion dinamica ────────────────────────────────────────────────────

    def _expand_map_if_needed(self, rx: int, ry: int) -> None:
        """Expande la grid si el rover se acerca al borde."""
        expand_left   = rx < self._margin
        expand_right  = rx > self.map_w - self._margin - 1
        expand_top    = ry < self._margin
        expand_bottom = ry > self.map_h - self._margin - 1

        if not (expand_left or expand_right or expand_top or expand_bottom):
            return

        old_h, old_w = self.grid.shape

        new_h = old_h + self._pad * (expand_top + expand_bottom)
        new_w = old_w + self._pad * (expand_left + expand_right)

        new_grid = np.full((new_h, new_w), VAL_UNKNOWN, dtype=np.uint8)

        off_x = self._pad if expand_left else 0
        off_y = self._pad if expand_top else 0

        new_grid[off_y:off_y + old_h, off_x:off_x + old_w] = self.grid

        self.grid = new_grid
        self.origin_x += off_x
        self.origin_y += off_y
        self.map_h, self.map_w = new_grid.shape

    # ── Integracion de observaciones ──────────────────────────────────────────

    def integrate_observation(self, class_mask: np.ndarray,
                              x_m: float, y_m: float,
                              theta_rad: float) -> None:
        """
        Integra una mascara de segmentacion (clases por pixel) al mapa global.

        Parametros:
          class_mask : ndarray [H, W] uint8 con clases 0-4 por pixel
                        (salida de VisionNavSource.last_class_mask)
          x_m, y_m   : pose del rover en metros (de OdometryTracker.pose())
          theta_rad  : heading del rover en radianes

        Proceso:
          1. Convertir pose a celda en la grid
          2. Expandir grid si el rover esta cerca del borde
          3. Recortar mitad superior del frame (suelo, no cielo)
          4. Downsamplear la mascara (cada 10 px) para reducir costo
          5. Para cada pixel observado:
             - Saltar Regolith (clase 0) — no aporta info al mapa
             - Raycast desde rover al pixel: pintar camino como FREE
             - Pintar el pixel como obstaculo si es Crater/Rock/Mountain
             - Saltar Sky (clase 4) — no es obstaculo ni navegable
          6. Registrar trayectoria
        """
        np = self._np if hasattr(self, '_np') else __import__('numpy')

        self.trajectory.append((x_m, y_m, theta_rad))

        rx, ry = self.world_to_cell(x_m, y_m)
        self._expand_map_if_needed(rx, ry)
        rx, ry = self.world_to_cell(x_m, y_m)

        h, w = self.grid.shape
        if not (0 <= rx < w and 0 <= ry < h):
            return

        # Recortar mitad superior (suelo)
        start_row = int(class_mask.shape[0] * 0.45)
        cropped = class_mask[start_row:, :]

        # Downsample
        mini = cropped[::10, ::10]
        mh, mw = mini.shape

        DEPTH_SCALE = 0.7
        LATERAL_SCALE = 0.9

        for r in range(mh):
            for c in range(mw):
                val = int(mini[r, c])

                # Saltar Regolith (clase 0) — no aporta info al mapa
                if val == CLASS_REGOLITH:
                    continue

                is_obstacle = (val != CLASS_SKY)

                # Profundidad relativa (pixeles lejanos = mas lejos)
                depth = (mh - r) * DEPTH_SCALE

                # Desplazamiento lateral
                lateral = (mw / 2 - c) * LATERAL_SCALE

                # Coordenadas mundo relativas al rover
                wx = depth * math.cos(theta_rad) - lateral * math.sin(theta_rad)
                wy = depth * math.sin(theta_rad) + lateral * math.cos(theta_rad)

                # Celda global objetivo
                gx = int(rx + wx)
                gy = int(ry - wy)

                # Raycasting: pintar camino como FREE
                steps = max(1, int(depth))
                for s in range(steps):
                    fx = int(rx + (wx * s / depth))
                    fy = int(ry - (wy * s / depth))

                    if 0 <= fx < w and 0 <= fy < h:
                        # Expandir free space localmente (radio 2)
                        R = 2
                        for oy in range(-R, R + 1):
                            for ox in range(-R, R + 1):
                                nx = fx + ox
                                ny = fy + oy
                                if 0 <= nx < w and 0 <= ny < h:
                                    if self.grid[ny, nx] == VAL_UNKNOWN:
                                        self.grid[ny, nx] = VAL_FREE

                # Pintar obstaculo
                if is_obstacle:
                    if 0 <= gx < w and 0 <= gy < h:
                        self.grid[gy, gx] = val

    # ── Consultas ─────────────────────────────────────────────────────────────

    def cell_value(self, x_m: float, y_m: float) -> int:
        """Retorna el valor de la celda en (x_m, y_m) o VAL_UNKNOWN si fuera."""
        rx, ry = self.world_to_cell(x_m, y_m)
        if 0 <= rx < self.map_w and 0 <= ry < self.map_h:
            return int(self.grid[ry, rx])
        return VAL_UNKNOWN

    def is_free(self, x_m: float, y_m: float) -> bool:
        """True si la celda en (x_m, y_m) es espacio libre confirmado."""
        return self.cell_value(x_m, y_m) == VAL_FREE

    def is_obstacle(self, x_m: float, y_m: float) -> bool:
        """True si la celda en (x_m, y_m) contiene un obstaculo (Crater/Rock/Mountain)."""
        v = self.cell_value(x_m, y_m)
        return v in (CLASS_CRATER, CLASS_ROCK, CLASS_MOUNTAIN)

    def is_unknown(self, x_m: float, y_m: float) -> bool:
        """True si la celda en (x_m, y_m) no ha sido observada."""
        return self.cell_value(x_m, y_m) == VAL_UNKNOWN

    # ── Reset ─────────────────────────────────────────────────────────────────

    def reset(self) -> None:
        """Reinicia el mapa a estado vacio. Llamar al inicio de cada mision."""
        self.grid = np.full(
            (self.map_h, self.map_w), VAL_UNKNOWN, dtype=np.uint8
        )
        self.trajectory = [(0.0, 0.0, math.pi / 2)]
