# olympus_hlc/sources/station.py — StationSource: GUI por TCP (consolidación A)
#
# Gemelo TCP de GCSSource (que es UDP+CSP). Reemplaza al daemon
# `ground_station/olympus_station.py`: en vez de que ESE proceso abra el Mega y
# duplique keepalive/dead-man/monitores, este CommandSource hace SOLO el
# transporte hacia la GUI del portátil; el engine HLC es el único dueño del Mega.
#
# Por qué TCP plano (no CSP/UDP como el path `gcs`): el WiFi del campus bloquea
# UDP pero no TCP. La GUI (`ground_station/olympus_gui.py`) habla el mismo
# protocolo de líneas que el daemon viejo, así que no hay que cambiarla.
#
# Protocolo (idéntico al daemon retirado):
#   Puerto 5006 (control, bidireccional, texto por líneas)
#       GUI → HLC : "MODE:MANUAL" | "MODE:AUTO" | "MODEL:YOLO" | "MODEL:LUNAR" |
#                    "EXP:l:r" | "STB" | "RST" | "AVD:L" | "SAFE:FULL" ...
#       HLC → GUI : "TLM:<frame>"  "CMD:<cmd>"  "EVT:<msg>"
#   Puerto 5005 (video, HLC → GUI): [4 bytes len big-endian][JPEG]
#
# Alcance MVP (Fase 0+1): manual + TLM + video. El modo AUTO (YOLO a bordo) es
# Fase 2 — aquí se acepta MODE:AUTO pero se fuerza MANUAL con un EVT de aviso.
#
# Capas de seguridad (sin duplicar al engine):
#   - dead-man RÁPIDO (local): STB si no llega EXP/CLB en DEADMAN_S (~1.2 s).
#   - pérdida de enlace LENTA: la maneja el engine vía make_link_monitor()
#     (CommLinkMonitor, SRS-013) → STB permanente si la GUI se desconecta.

import os
import re
import socket
import struct
import subprocess
import threading
import time

from ..config import LUNAR_MODEL_PATH
from ..interfaces import CommandSource
from ..monitors import CommLinkMonitor

# ── Parámetros (antes constantes del daemon) ──────────────────────────────────
CTRL_PORT  = 5006
VIDEO_PORT = 5005
DEADMAN_S  = 6.0  # AUTO: tolera latencia de YOLO en RPi5 (3-5s/inferencia)
CAM_W, CAM_H, CAM_FPS = 640, 480, 10
INFER_EVERY = 1   # AUTO: inferir en cada frame (máxima capacidad de respuesta)

# ── Helpers CSV (portados del daemon: pipeline de evidencia del TFG) ──────────
_SAFETY_N = {"NORMAL": 0, "WARN": 1, "LIMIT": 2, "FAULT": 3}
_CSV_COLS = ("t_ms,safety,i_fr,i_fl,i_cr,i_cl,i_rr,i_rl,"
             "ntc1,ntc2,ntc3,ntc4,tf_mm,enc_l,enc_r,unix_ms")


def _num(s):
    m = re.match(r"-?\d+", s.strip())
    return m.group(0) if m else "0"


def _tlm_row(line):
    f = line.split(":")
    if len(f) < 26:
        return None
    try:
        row = [_num(f[3]), _SAFETY_N.get(f[1], -1)]
        row += [_num(f[i]) for i in range(6, 12)]    # corrientes ACS712
        row += [_num(f[i]) for i in range(13, 17)]   # ntc1..ntc4
        row += [_num(f[25]), _num(f[20]), _num(f[21])]  # tf_mm, enc_l, enc_r
        row += [int(time.time() * 1000)]
        return ",".join(str(v) for v in row)
    except (IndexError, ValueError):
        return None


def _open_csv():
    for d in (os.path.expanduser("~/evidencia"), "/var/log/olympus", "/tmp"):
        try:
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, "station_tlm_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
            fh = open(path, "w")
            fh.write("# csv_cols=" + _CSV_COLS + "\n")
            fh.flush()
            return fh, path
        except OSError:
            continue
    return None, None


class StationSource(CommandSource):
    """
    Fuente de comandos por TCP para la GUI del portátil. El engine HLC es el
    único que abre el Mega; esta clase solo transporta comandos/TLM/video.
    """

    def __init__(self, ctrl_port: int = CTRL_PORT, video_port: int = VIDEO_PORT,
                 deadman_s: float = DEADMAN_S, enable_video: bool = True,
                 model_path: "str | None" = None, infer_every: int = INFER_EVERY):
        self._ctrl_port  = ctrl_port
        self._video_port = video_port
        self._deadman_s  = deadman_s
        self._enable_video = enable_video
        self._infer_every = max(1, infer_every)

        self._lock = threading.Lock()
        self._pending: list = []          # cola FIFO de comandos de la GUI (cap 16)
        self._mode   = "MANUAL"
        self._safety_level = "FULL"   # FULL | ASSIST | MANUAL (selector GUI)
        self._last_cmd = "STB"
        self._drive_active = False
        self._last_drive   = 0.0
        self._last_recv    = time.monotonic()   # liveness del cliente TCP (engine)
        self._latest_jpeg  = b""

        # AUTO (Fase 2): decisión YOLO a bordo. Reutiliza VisionSource (misma
        # lógica que `--mode vision`) sobre el frame compartido del MJPEG, sin
        # doble captura. Si no hay modelo/cv2, AUTO queda deshabilitado y solo
        # opera MANUAL (degradación elegante).
        #
        # Modelo lunar (TFG Carlos Alfaro): VisionNavSource con segmentación
        # semántica UNetMobileNet. La GUI selecciona entre YOLO y lunar con
        # "MODEL:YOLO" / "MODEL:LUNAR"; _infer_auto brancea según _model_mode.
        self._vision = None
        self._vision_nav = None
        self._model_mode = "YOLO"   # "YOLO" | "LUNAR" (selector GUI)
        self._auto_cmd   = "STB"
        self._auto_fresh = False
        self._auto_dets: list = []
        if model_path:
            try:
                from .vision import VisionSource
                self._vision = VisionSource(model_path)
                print(f"[StationSource] AUTO habilitado — modelo YOLO {model_path}")
            except (SystemExit, Exception) as ex:  # noqa: BLE001
                self._vision = None
                print(f"[StationSource] YOLO deshabilitado (modelo/cv2 no disponible): {ex}")
        # Modelo lunar (siempre intentar cargarlo — la GUI puede seleccionarlo)
        try:
            from .vision_nav import VisionNavSource
            self._vision_nav = VisionNavSource(LUNAR_MODEL_PATH)
            print(f"[StationSource] Lunar habilitado — modelo {LUNAR_MODEL_PATH}")
        except (SystemExit, Exception) as ex:  # noqa: BLE001
            self._vision_nav = None
            print(f"[StationSource] Lunar deshabilitado (modelo/cv2 no disponible): {ex}")

        self._ctrl_conn = None            # socket del cliente de control activo
        self._stop = threading.Event()

        self._csv, csv_path = _open_csv()

        # Threads de fondo (daemon: mueren con el proceso).
        threading.Thread(target=self._ctrl_server, daemon=True).start()
        if self._enable_video:
            threading.Thread(target=self._camera_loop, daemon=True).start()
            threading.Thread(target=self._video_server, daemon=True).start()
            # La inferencia YOLO corre en su PROPIO hilo (desacoplada del bombeo
            # de frames): en la RPi5 el forward tarda cientos de ms y, si corriera
            # dentro de _camera_loop, taparía el pipe de rpicam-vid y congelaría el
            # video en cuanto se entra a AUTO.
            if self._vision is not None or self._vision_nav is not None:
                threading.Thread(target=self._inference_loop, daemon=True).start()

        csv_str = csv_path or "deshabilitado"
        print(f"[StationSource] TCP control :{ctrl_port}  video :{video_port}"
              f"  dead-man {deadman_s:.1f}s  CSV {csv_str}")

    # ── CommandSource interface ───────────────────────────────────────────────

    def next_command(self, log=None) -> "str | None":
        """No-bloqueante: en AUTO devuelve la decisión YOLO; en MANUAL drena la
        cola de la GUI o aplica el dead-man rápido."""
        now = time.monotonic()
        with self._lock:
            # AUTO: el loop de cámara produce el comando; lo devolvemos cuando es
            # nuevo (uno por inferencia) para no inundar al engine cada ciclo.
            active = self._vision if self._model_mode == "YOLO" else self._vision_nav
            if self._mode == "AUTO" and active is not None:
                if self._auto_fresh:
                    self._auto_fresh = False
                    cmd = self._auto_cmd
                    self._last_cmd = cmd
                    if cmd.startswith("EXP:") or cmd.startswith("CLB:") or cmd.startswith("AVD"):
                        self._drive_active = True
                        self._last_drive = now
                    elif cmd in ("STB", "RST", "FLT"):
                        self._drive_active = False
                    return cmd
                # sin inferencia fresca: dead-man si la cámara dejó de producir
                if self._drive_active and (now - self._last_drive) > self._deadman_s:
                    self._drive_active = False
                    self._last_cmd = "STB"
                    if log:
                        log.warn("STATION", "AUTO sin inferencia fresca → STB")
                    return "STB"
                return None
            if self._pending:
                cmd = self._pending.pop(0)
                self._last_cmd = cmd
                if cmd.startswith("EXP:") or cmd.startswith("CLB:"):
                    self._drive_active = True
                    self._last_drive = now
                elif cmd in ("STB", "RST", "FLT") or cmd.startswith("AVD"):
                    self._drive_active = False
                return cmd
            # dead-man rápido: cortar si se dejó de recibir EXP/CLB
            if self._drive_active and (now - self._last_drive) > self._deadman_s:
                self._drive_active = False
                self._last_cmd = "STB"
                if log:
                    log.warn("STATION", f"dead-man {self._deadman_s:.1f}s sin drive → STB")
                return "STB"
        return None

    def on_tlm(self, raw_tlm: str) -> None:
        """Reenvía el frame TLM a la GUI (downlink) y lo registra en CSV."""
        line = raw_tlm.strip()
        self._send_gui(line + "\n")
        # Eco del último comando para que la GUI muestre el estado.
        with self._lock:
            cmd = self._last_cmd
        self._send_gui("CMD:" + cmd + "\n")
        if self._csv:
            row = _tlm_row(line)
            if row:
                try:
                    self._csv.write(row + "\n")
                    self._csv.flush()
                except OSError:
                    pass

    @property
    def last_recv_time(self) -> float:
        # Mientras haya un cliente TCP conectado, el enlace está VIVO aunque la
        # GUI no esté mandando comandos (puede estar quieta entre EXP). Reportar
        # 'ahora' evita el falso `link_lost` del CommLinkMonitor que latcheaba
        # STB permanente y bloqueaba todos los EXP (bug 2026-06-02). Cuando NO
        # hay cliente (_ctrl_conn=None) se usa el último recv real → el monitor
        # sí escala a STB (seguridad correcta: sin GUI no se maneja).
        with self._lock:
            if self._ctrl_conn is not None:
                return time.monotonic()
            return self._last_recv

    @property
    def safety_level(self) -> str:
        """Nivel de seguridad activo (selector de la GUI vía 'SAFE:<nivel>')."""
        with self._lock:
            return self._safety_level

    def on_dispatch(self, cmd: "str | None", reason: "str | None" = None) -> None:
        """El engine reporta el comando FINAL (con override) + su razón. Reflejamos
        el comando REAL en la GUI (antes mostraba el del operador, no el override)
        y mandamos la razón como evento para evidencia/log."""
        if cmd is None:
            return
        with self._lock:
            self._last_cmd = cmd
        if reason:
            self._send_gui("EVT:OVR " + reason + "\n")

    def make_link_monitor(self) -> CommLinkMonitor:
        """
        El engine vigila el enlace con la GUI (SRS-013). En teleop, link_lost →
        STB permanente (lo correcto: NO RET autónomo si se cae la GUI).
        """
        return CommLinkMonitor()

    def send_probe(self) -> None:
        """Probe de reconexión: avisa a la GUI. No-op si no hay cliente."""
        self._send_gui("EVT:HB_REQ\n")

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            conn = self._ctrl_conn
        for s in (conn,):
            try:
                if s:
                    s.close()
            except OSError:
                pass
        if self._csv:
            try:
                self._csv.close()
            except OSError:
                pass

    # ── Internos: transporte ──────────────────────────────────────────────────

    def _enqueue(self, cmd: str) -> None:
        with self._lock:
            self._pending.append(cmd)
            if len(self._pending) > 16:
                self._pending.pop(0)   # bound: descartar el más viejo

    def _send_gui(self, text: str) -> None:
        """Envía una línea al cliente de control; marca el enlace como vivo."""
        with self._lock:
            conn = self._ctrl_conn
        if conn is None:
            return
        try:
            conn.sendall(text.encode())
            # liveness: si el socket acepta escritura, el enlace está vivo.
            self._last_recv = time.monotonic()
        except OSError:
            pass

    def _ctrl_server(self) -> None:
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._ctrl_port))
        srv.listen(1)
        print(f"[StationSource] control en :{self._ctrl_port}")
        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            print(f"[StationSource] ctrl client: {addr}")
            with self._lock:
                self._ctrl_conn = conn
                self._last_recv = time.monotonic()
                # Sesión nueva limpia: descartar comandos viejos y resetear estado,
                # para no arrastrar un STB/AUTO de una conexión previa. El refresco
                # de `_last_recv` además recupera el enlace si quedó en STB latcheado
                # (CommLinkMonitor → "reconnect_attempt_succeeded" → engine limpia).
                self._pending.clear()
                self._drive_active = False
                self._mode = "MANUAL"
                self._safety_level = "FULL"   # default seguro; la GUI re-sincroniza
            try:
                # Lectura por recv línea-a-línea (NO `makefile().__iter__`, cuyo
                # read-ahead puede retener líneas y hacer que comandos como EXP no
                # se procesen a tiempo; MODE pasaba pero EXP quedaba en el buffer).
                # Cada byte recibido refresca `_last_recv` → mantiene vivo el enlace
                # (CommLinkMonitor) y permite recuperarse de un STB latcheado.
                conn.settimeout(1.0)
                rxbuf = b""
                while not self._stop.is_set():
                    try:
                        data = conn.recv(1024)
                    except socket.timeout:
                        continue
                    if not data:
                        break  # el cliente cerró la conexión
                    self._last_recv = time.monotonic()
                    rxbuf += data
                    while b"\n" in rxbuf:
                        line, rxbuf = rxbuf.split(b"\n", 1)
                        c = line.decode("ascii", "replace").strip()
                        if not c:
                            continue
                        self._last_recv = time.monotonic()
                        if c.startswith("MODE:"):
                            m = c[5:].upper()
                            active = self._vision if self._model_mode == "YOLO" else self._vision_nav
                            if m == "AUTO" and active is not None:
                                with self._lock:
                                    self._mode = "AUTO"
                                    self._drive_active = False
                                    self._auto_fresh = False
                                self._enqueue("STB")  # parada limpia antes de ceder a YOLO
                                self._send_gui("EVT:MODE_AUTO\n")
                            elif m == "AUTO":
                                # Sin modelo/cv2 → AUTO no disponible; quedarse en MANUAL.
                                self._send_gui("EVT:AUTO_UNAVAILABLE_NO_MODEL\n")
                                with self._lock:
                                    self._mode = "MANUAL"
                                self._enqueue("STB")
                            else:
                                with self._lock:
                                    self._mode = "MANUAL"
                                    self._drive_active = False
                                self._enqueue("STB")
                                self._send_gui("EVT:MODE_MANUAL\n")
                            continue
                        if c.startswith("SAFE:"):
                            lvl = c[5:].upper()
                            if lvl in ("FULL", "ASSIST", "MANUAL"):
                                with self._lock:
                                    self._safety_level = lvl
                                self._send_gui(f"EVT:SAFETY_{lvl}\n")
                            continue
                        if c.startswith("MODEL:"):
                            m = c[6:].upper()
                            if m in ("YOLO", "LUNAR"):
                                target = self._vision if m == "YOLO" else self._vision_nav
                                if target is None:
                                    self._send_gui(f"EVT:{m}_UNAVAILABLE_NO_MODEL\n")
                                else:
                                    with self._lock:
                                        self._model_mode = m
                                        self._auto_fresh = False
                                        self._auto_dets = []
                                    self._send_gui(f"EVT:MODEL_{m}\n")
                            continue
                        if c in ("STB", "FLT"):
                            # Paro de emergencia: en AUTO, next_command devuelve la
                            # decisión de YOLO e IGNORA la cola _pending, así que un
                            # STB encolado no frenaría (la siguiente inferencia
                            # re-emite EXP). Forzamos salir de AUTO → el comando se
                            # procesa y la autonomía queda desarmada hasta que el
                            # operador la re-active deliberadamente.
                            with self._lock:
                                self._mode = "MANUAL"
                                self._drive_active = False
                            self._enqueue(c)
                            self._send_gui("EVT:MODE_MANUAL\n")
                            continue
                        self._enqueue(c)
            except OSError:
                pass
            finally:
                # Desconexión de la GUI → STB inmediato; el engine además
                # escalará a STB permanente vía CommLinkMonitor.
                self._enqueue("STB")
                with self._lock:
                    if self._ctrl_conn is conn:
                        self._ctrl_conn = None
                    self._drive_active = False
                try:
                    conn.close()
                except OSError:
                    pass
                print("[StationSource] ctrl desconectado → STB")

    # ── Internos: cámara/video (MVP: solo stream, sin YOLO) ───────────────────

    def _camera_loop(self) -> None:
        while not self._stop.is_set():
            try:
                proc = subprocess.Popen(
                    ["rpicam-vid", "--codec", "mjpeg", "--output", "-",
                     "--width", str(CAM_W), "--height", str(CAM_H),
                     "--framerate", str(CAM_FPS), "--rotation", "180",
                     "--timeout", "0", "--nopreview"],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            except FileNotFoundError:
                print("[StationSource] rpicam-vid no disponible — video deshabilitado")
                return
            buf = b""
            try:
                if proc.stdout is None:
                    raise RuntimeError("rpicam-vid sin stdout")
                while not self._stop.is_set():
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while True:
                        s = buf.find(b"\xff\xd8")
                        if s == -1:
                            buf = b""
                            break
                        e = buf.find(b"\xff\xd9", s + 2)
                        if e == -1:
                            buf = buf[s:]
                            break
                        jpeg = buf[s:e + 2]
                        buf = buf[e + 2:]
                        with self._lock:
                            self._latest_jpeg = jpeg
                        # NOTA: la inferencia NO se hace aquí. Solo bombeamos frames
                        # (latest_jpeg) lo más rápido posible; _inference_loop consume
                        # el último frame a su ritmo. Ver __init__.
            except Exception as ex:  # noqa: BLE001
                print(f"[StationSource] cam err: {ex}")
            finally:
                try:
                    proc.terminate()
                    proc.wait()
                except Exception:  # noqa: BLE001
                    pass
            time.sleep(1)   # rpicam murió: reintentar

    def _inference_loop(self) -> None:
        """Hilo dedicado de inferencia: en AUTO corre el modelo activo (YOLO o
        lunar) sobre el ÚLTIMO frame disponible, a su propio ritmo (la RPi5 da
        ~1-3 inf/s) sin bloquear el video. Salta frames intermedios: siempre
        infiere sobre el más reciente."""
        last_seen = None
        while not self._stop.is_set():
            with self._lock:
                mode = self._mode
                model_mode = self._model_mode
                jpeg = self._latest_jpeg
            active = self._vision if model_mode == "YOLO" else self._vision_nav
            if mode != "AUTO" or active is None or not jpeg or jpeg is last_seen:
                time.sleep(0.03)        # nada nuevo / no-AUTO: no quemar CPU
                continue
            last_seen = jpeg
            self._infer_auto(jpeg)

    def _infer_auto(self, jpeg: bytes) -> None:
        """Decodifica el JPEG, corre la decisión del modelo activo (YOLO o lunar)
        y publica comando + overlay (DET: para YOLO; el lunar no tiene bboxes)."""
        # ── Lunar: segmentación semántica (TFG Carlos Alfaro) ──
        if self._model_mode == "LUNAR" and self._vision_nav is not None:
            vn = self._vision_nav
            try:
                frame = vn._cv2.imdecode(
                    vn._np.frombuffer(jpeg, vn._np.uint8), vn._cv2.IMREAD_COLOR)
                if frame is None:
                    return
                cmd, nav_mask = vn.infer(frame)
            except Exception as ex:  # noqa: BLE001
                print(f"[StationSource] lunar infer err: {ex}")
                return
            with self._lock:
                self._auto_cmd   = cmd
                self._auto_fresh = True
            return
        # ── YOLO: object detection ──
        v = self._vision
        if v is None:
            return
        try:
            frame = v._cv2.imdecode(
                v._np.frombuffer(jpeg, v._np.uint8), v._cv2.IMREAD_COLOR)
            if frame is None:
                return
            cmd, dets = v.infer(frame)
        except Exception as ex:  # noqa: BLE001
            print(f"[StationSource] infer err: {ex}")
            return
        with self._lock:
            self._auto_cmd   = cmd
            self._auto_dets  = dets
            self._auto_fresh = True
        # Overlay para la GUI (mismo formato que el daemon retirado).
        ds = ";".join("%.4f,%.4f,%.4f,%.4f,%.2f" % d for d in dets)
        self._send_gui("DET:" + ds + "\n")

    def _video_server(self) -> None:
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._video_port))
        srv.listen(1)
        print(f"[StationSource] video en :{self._video_port}")
        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            print(f"[StationSource] video viewer: {addr}")
            try:
                while not self._stop.is_set():
                    with self._lock:
                        j = self._latest_jpeg
                    if j:
                        conn.sendall(struct.pack(">I", len(j)) + j)
                    time.sleep(1.0 / CAM_FPS)
            except OSError:
                pass
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
