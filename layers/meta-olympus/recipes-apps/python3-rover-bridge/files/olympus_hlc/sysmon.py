# olympus_hlc/sysmon.py — SystemMonitor: RPi5 resource sampling → GUI
#
# Muestrea CPU %, RAM usada/total y temperatura del SoC del RPi5 leyendo
# directamente de ABI estables del kernel Linux (/proc, /sys) — SIN psutil, sin
# dependencias adicionales en la imagen Yocto.
#
# Engine invoca sample() en cada ciclo y llama a source.on_sys() cuando hay una
# muestra fresca; StationSource la reenvía como frame SYS: a la GUI (formato
# idéntico a TLM:, ver station.py). Sin /proc ni /sys (ej. en pytest aislado),
# sample() retorna valores 0.0 — el control loop no se ve afectado.
#
# Fuentes (Linux kernel ABI estables):
#   CPU  — /proc/stat la línea "cpu  user nice system idle iowait irq softirq ...
#           CPU% = (1 - Δidle/Δtotal) × 100  sobre dos lecturas.
#   RAM  — /proc/meminfo: MemTotal, MemAvailable.  used = total - available.
#   Temp — /sys/class/thermal/thermal_zone*/temp (miligrados C). En RPi5 el primer
#           thermal_zone (cpu_thermal del bcm2712) es el del SoC.
#
# Ref.: proc(5), Linux kernel Documentation/filesystems/proc.rst §meminfo;
#       Documentation/admin-guide/thermal/. thermal_zone0 en bcm2712 = SoC.

import glob
import time

from .config import SYS_MON_ENABLED, SYS_SAMPLE_S


class SystemSample:
    """Instantánea de recursos del RPi5 transmitida a la GUI.

    Formato de frame SYS: (los campos viajan como números, la GUI resuelve las
    unidades — ver olympus_gui.py):
        SYS:<cpu_pct>,<ram_used_mb>,<ram_total_mb>,<temp_c|0>

    temp_c = 0 indica "sin lectura" (mismo convenio que dist_mm en TlmFrame).
    """

    __slots__ = ("cpu_pct", "ram_used_mb", "ram_total_mb", "temp_c")

    def __init__(self, cpu_pct, ram_used_mb, ram_total_mb, temp_c):
        self.cpu_pct      = cpu_pct
        self.ram_used_mb  = ram_used_mb
        self.ram_total_mb = ram_total_mb
        self.temp_c       = temp_c

    def to_frame(self) -> str:
        return (f"SYS:{self.cpu_pct:.0f},"
                f"{self.ram_used_mb:.0f},"
                f"{self.ram_total_mb:.0f},"
                f"{self.temp_c:.0f}")


class SystemMonitor:
    """Muestrea recursos del RPi5 con throttling (una lectura fresca / sys_sample_s).

    Uso desde el engine:
        mon = SystemMonitor()
        ...
        s = mon.sample()                       # None si throttled
        if s is not None: source.on_sys(s)     # StationSource reenvía SYS:

    CPU% requiere dos lecturas de /proc/stat; el baseline se toma en __init__, así
    que la primera llamada a sample() ya entrega un valor válido (no 0 como psutil).
    """

    def __init__(self, enabled: bool = SYS_MON_ENABLED, sample_s: float = SYS_SAMPLE_S):
        self._enabled  = enabled
        self._sample_s = sample_s
        self._last_t   = 0.0
        self._prev_cpu = None    # (total_jiffies, idle_jiffies) del último sample
        self._temp_paths = None  # cached glob resultado

        if enabled:
            # Baseline de CPU para que el primer sample tenga un Δfinite.
            self._prev_cpu = self._read_cpu_jiffies()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def sample(self) -> "SystemSample | None":
        """Retorna SystemSample si han pasado sys_sample_s desde la última, o None."""
        if not self._enabled:
            return None
        now = time.monotonic()
        if now - self._last_t < self._sample_s:
            return None
        self._last_t = now
        return SystemSample(
            cpu_pct      = self._read_cpu(),
            ram_used_mb  = self._read_ram_used(),
            ram_total_mb = self._read_ram_total(),
            temp_c       = self._read_temp(),
        )

    # ── Readers (cada uno robusto: nunca levanta — retorna 0 sino) ─────────────

    @staticmethod
    def _read_cpu_jiffies():
        """Lee la primera línea "cpu" de /proc/stat → (total, idle) o None."""
        try:
            with open("/proc/stat") as f:
                line = f.readline()
            if not line.startswith("cpu"):
                return None
            # "cpu  user nice system idle iowait irq softirq steal guest guest_nice"
            vals = [int(x) for x in line.split()[1:]]
            idle     = vals[3]
            total    = sum(vals)
            return (total, idle)
        except (OSError, ValueError, IndexError):
            return None

    def _read_cpu(self) -> float:
        cur = self._read_cpu_jiffies()
        if cur is None or self._prev_cpu is None:
            return 0.0
        dt = cur[0] - self._prev_cpu[0]
        di = cur[1] - self._prev_cpu[1]
        self._prev_cpu = cur
        if dt <= 0:
            return 0.0
        return max(0.0, (1.0 - di / dt) * 100.0)

    @staticmethod
    def _read_meminfo():
        """Retorna (mem_total_kb, mem_avail_kb) o (0, 0)."""
        total = avail = 0
        try:
            with open("/proc/meminfo") as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        total = int(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        avail = int(line.split()[1])
                        break                   # MemAvailable va después de MemTotal
            return (total, avail)
        except (OSError, ValueError, IndexError):
            return (0, 0)

    def _read_ram_total(self) -> float:
        total, _ = self._read_meminfo()
        return total / 1024.0  # KB → MB

    def _read_ram_used(self) -> float:
        total, avail = self._read_meminfo()
        return max(0.0, (total - avail)) / 1024.0  # KB → MB

    def _read_temp(self) -> float:
        """Lee /sys/class/thermal/thermal_zone*/temp (miligrados C)."""
        if self._temp_paths is None:
            self._temp_paths = glob.glob("/sys/class/thermal/thermal_zone*/temp")
        for path in self._temp_paths:
            try:
                with open(path) as f:
                    return float(f.read().strip()) / 1000.0
            except (OSError, ValueError):
                continue
        return 0.0