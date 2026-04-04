#!/usr/bin/env python3
"""
debug_view.py — Visor local del stream de visión via SSH pipe.

Corre en el PC. Lee frames JPEG de stdin (escritos por debug_vision.py en el RPi5)
y los muestra en una ventana con matplotlib (TkAgg).

Protocolo de pipe:
  [4 bytes big-endian: longitud JPEG] [N bytes JPEG]

Uso:
  ssh root@<IP_RPi5> "python3 /usr/bin/debug_vision.py --mode seg" | python3 debug_view.py

Presionar 'q' o cerrar la ventana para salir.

Dependencias PC: opencv-python (cualquier build — solo se usa para imdecode),
                 matplotlib, numpy
  uv run python3 debug_view.py   # dentro del directorio con pyproject.toml
"""

import struct
import sys


def read_exact(stream, n: int) -> bytes:
    """Lee exactamente n bytes de stream. Retorna bytes vacío si EOF."""
    buf = b""
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf


def main():
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("[ERROR] opencv-python/numpy no encontrado. Instalar con: uv sync --dev")
        sys.exit(1)

    try:
        import matplotlib
        # Intentar QtAgg (PyQt6) primero, TkAgg como fallback, Agg sin display
        for backend in ("QtAgg", "TkAgg", "Agg"):
            try:
                matplotlib.use(backend)
                break
            except Exception:
                continue
        import matplotlib.pyplot as plt
    except ImportError:
        print("[ERROR] matplotlib no encontrado. Instalar con: uv sync --dev")
        sys.exit(1)

    stdin = sys.stdin.buffer
    frame_count = 0

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 6))
    if fig.canvas.manager is not None:
        fig.canvas.manager.set_window_title("Olympus Vision Debug (cierra ventana para salir)")
    ax.axis("off")
    img_handle = None

    print("[debug_view] Esperando frames... (cierra ventana para salir)")

    while True:
        # Leer cabecera de 4 bytes
        header = read_exact(stdin, 4)
        if not header:
            print("[debug_view] Stream terminado (EOF).")
            break

        length = struct.unpack(">I", header)[0]
        if length == 0 or length > 10_000_000:
            print(f"[debug_view] Longitud inválida: {length}, abortando.")
            break

        # Leer JPEG
        data = read_exact(stdin, length)
        if not data:
            print("[debug_view] Stream cortado leyendo frame, EOF.")
            break

        # Decodificar BGR → RGB para matplotlib
        frame_bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame_bgr is None:
            print(f"[debug_view] frame={frame_count} — fallo al decodificar JPEG, saltando.")
            frame_count += 1
            continue

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_count += 1

        # Mostrar o actualizar frame en la misma ventana
        if img_handle is None:
            img_handle = ax.imshow(frame_rgb)
        else:
            img_handle.set_data(frame_rgb)

        ax.set_title(f"frame {frame_count}", fontsize=9)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        # Salir si la ventana fue cerrada
        if not plt.fignum_exists(fig.number):
            print("[debug_view] Ventana cerrada.")
            break

    plt.close("all")
    print(f"[debug_view] Total frames recibidos: {frame_count}")


if __name__ == "__main__":
    main()
