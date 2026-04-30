# olympus_hlc/sources/manual_stream.py — ManualStreamSource: stdin + TCP camera stream

import socket
import struct
import subprocess
import threading

from ..config import FRAME_WIDTH, FRAME_HEIGHT
from .manual import ManualSource


class ManualStreamSource(ManualSource):
    """
    ManualSource con un servidor TCP de cámara en background.

    El rover escucha en 0.0.0.0:<stream_port>. El laptop se conecta y recibe
    frames JPEG crudos (sin inferencia) usando el protocolo:
        [4 bytes big-endian: longitud] [N bytes JPEG]

    Compatible con stream_view.py y debug_view.py en el lado del laptop.

    Uso en RPi5:
        python3 -m olympus_hlc --mode manual --stream --stream-port 5005 --stream-fps 5

    Uso en laptop (terminal separado):
        python3 stream_view.py 192.168.100.1 5005
    """

    def __init__(self, stream_port: int = 5005, fps: int = 5):
        super().__init__()
        self._stream_port = stream_port
        self._fps = fps
        self._running = True
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("0.0.0.0", stream_port))
        self._srv.listen(1)
        self._srv.settimeout(1.0)
        self._srv_thread = threading.Thread(target=self._server_loop, daemon=True)
        self._srv_thread.start()
        print(f"[ManualStream] Camera TCP server on :{stream_port} @ {fps} fps")
        print(f"[ManualStream] Laptop: python3 stream_view.py <ROVER_IP> {stream_port}")

    def _server_loop(self) -> None:
        while self._running:
            try:
                conn, addr = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"[ManualStream] Viewer connected: {addr[0]}:{addr[1]}")
            try:
                self._stream_to(conn)
            finally:
                conn.close()
                print("[ManualStream] Viewer disconnected.")
        self._srv.close()

    def _stream_to(self, conn: socket.socket) -> None:
        """Captura MJPEG con rpicam-vid y reenvía frames al cliente conectado."""
        proc = subprocess.Popen(
            [
                "rpicam-vid",
                "--codec",     "mjpeg",
                "--output",    "-",
                "--width",     str(FRAME_WIDTH),
                "--height",    str(FRAME_HEIGHT),
                "--framerate", str(self._fps),
                "--timeout",   "0",
                "--nopreview",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

        SOI = b"\xff\xd8"
        EOI = b"\xff\xd9"
        buf = b""

        assert proc.stdout is not None
        try:
            while self._running:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk

                # Extraer todos los JPEG completos del buffer acumulado
                while True:
                    start = buf.find(SOI)
                    if start == -1:
                        buf = b""
                        break
                    end = buf.find(EOI, start + 2)
                    if end == -1:
                        buf = buf[start:]
                        break
                    jpeg = buf[start: end + 2]
                    buf = buf[end + 2:]
                    try:
                        conn.sendall(struct.pack(">I", len(jpeg)) + jpeg)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return
        finally:
            proc.terminate()
            proc.wait()

    def close(self) -> None:
        self._running = False
