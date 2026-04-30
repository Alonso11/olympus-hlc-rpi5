#!/usr/bin/env python3
"""
stream_view.py — Visor de cámara cruda del rover Olympus (modo manual --stream).

Se conecta al TCP server del rover y muestra frames JPEG sin inferencia.
El operador ve el feed de cámara y escribe comandos en otra terminal SSH.

Protocolo: [4 bytes big-endian: longitud JPEG] [N bytes JPEG]

Uso:
    python3 stream_view.py <ROVER_IP> [<PORT>]
    python3 stream_view.py 192.168.100.1          # puerto 5005 por defecto
    python3 stream_view.py 192.168.100.1 5005
    python3 stream_view.py olympus-rover.local

Presionar 'q' o cerrar la ventana para salir.

Dependencias: opencv-python, matplotlib, numpy
    uv run python3 stream_view.py 192.168.100.1
"""

import socket
import struct
import sys


STREAM_PORT_DEFAULT = 5005


def read_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf


def main() -> None:
    if len(sys.argv) < 2:
        print("Uso: python3 stream_view.py <ROVER_IP> [<PORT>]")
        print("     python3 stream_view.py 192.168.100.1")
        sys.exit(1)

    host = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else STREAM_PORT_DEFAULT

    try:
        import cv2
        import numpy as np
    except ImportError:
        print("[ERROR] opencv-python / numpy no encontrado. Instalar: uv sync --dev")
        sys.exit(1)

    try:
        import matplotlib
        for backend in ("QtAgg", "TkAgg", "Agg"):
            try:
                matplotlib.use(backend)
                break
            except Exception:
                continue
        import matplotlib.pyplot as plt
    except ImportError:
        print("[ERROR] matplotlib no encontrado. Instalar: uv sync --dev")
        sys.exit(1)

    print(f"[stream_view] Conectando a {host}:{port} ...")
    try:
        sock = socket.create_connection((host, port), timeout=10)
    except Exception as e:
        print(f"[ERROR] No se pudo conectar: {e}")
        sys.exit(1)

    print("[stream_view] Conectado. Mostrando stream — cierra la ventana para salir.")

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 6))
    if fig.canvas.manager is not None:
        fig.canvas.manager.set_window_title(f"Olympus Camera  {host}:{port}")
    ax.axis("off")
    img_handle = None
    frame_count = 0

    try:
        while True:
            header = read_exact(sock, 4)
            if not header:
                print("[stream_view] Conexión cerrada por el rover.")
                break

            length = struct.unpack(">I", header)[0]
            if length == 0 or length > 10_000_000:
                print(f"[stream_view] Longitud inválida: {length} — abortando.")
                break

            data = read_exact(sock, length)
            if not data:
                print("[stream_view] EOF leyendo frame.")
                break

            frame_bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame_bgr is None:
                continue

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            frame_count += 1

            if img_handle is None:
                img_handle = ax.imshow(frame_rgb)
            else:
                img_handle.set_data(frame_rgb)

            ax.set_title(f"frame {frame_count}", fontsize=9)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            if not plt.fignum_exists(fig.number):
                print("[stream_view] Ventana cerrada.")
                break

    finally:
        sock.close()
        plt.close("all")

    print(f"[stream_view] Total frames recibidos: {frame_count}")


if __name__ == "__main__":
    main()
