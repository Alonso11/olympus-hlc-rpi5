#!/usr/bin/env python3
# oled_status.py — Muestra el SSID de la red WiFi y la IP del rover en el
# display OLED SSD1306 (I2C 0x3C, bus i2c-1, GPIO2/GPIO3).
#
# Uso:
#   oled_status.py             # refresca cada 3 s (bucle)
#   oled_status.py --once      # una sola actualizacion (p.ej. al arranque)
#   oled_status.py --interval 5
#
# No requiere root: la regla udev 99-i2c.rules da modo 0666 a /dev/i2c-1.

import argparse
import socket
import subprocess
import sys
import time


def get_default_ip(iface: str = "wlan0") -> str:
    """IP de la interfaz de red activa."""
    for name in (iface, "eth0", "wlan0"):
        try:
            out = subprocess.run(
                ["ip", "-4", "addr", "show", name],
                capture_output=True, text=True, timeout=2,
            )
            for line in out.stdout.splitlines():
                line = line.strip()
                if line.startswith("inet "):
                    return line.split()[1].split("/")[0]
        except (subprocess.TimeoutExpired, FileNotFoundError):
            continue
    # Fallback: UDP socket trick
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return ""


def get_wifi_ssid(iface: str = "wlan0") -> str:
    """SSID de la red WiFi (cliente o AP), o '' si no hay tools."""
    # 1) Modo AP:  iw dev <iface> info  (linea "ssid <nombre>")
    try:
        out = subprocess.run(
            ["iw", "dev", iface, "info"],
            capture_output=True, text=True, timeout=2,
        )
        for line in out.stdout.splitlines():
            line = line.strip()
            if line.startswith("ssid"):
                return line.split(None, 1)[1]
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # 2) Modo cliente: iwgetid -r
    try:
        out = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True, text=True, timeout=2,
        )
        ssid = out.stdout.strip()
        if ssid:
            return ssid
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # 3) Modo cliente (nl80211):  iw dev <iface> link
    try:
        out = subprocess.run(
            ["iw", "dev", iface, "link"],
            capture_output=True, text=True, timeout=2,
        )
        for line in out.stdout.splitlines():
            line = line.strip()
            if line.startswith("SSID:"):
                return line.split(":", 1)[1].strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # 4) Sin tools: ver /proc/net/wireless
    try:
        with open("/proc/net/wireless") as f:
            for line in f:
                if iface in line:
                    return "WIFI LINK"
    except OSError:
        pass

    return ""


def refresh(disp) -> None:
    import socket
    hostname = socket.gethostname()
    ssid = get_wifi_ssid()
    ip = get_default_ip()
    disp.clear()
    disp.draw_text(0, 0, "OLYMPUS ROVER")
    disp.draw_text(0, 1, hostname[:16])
    disp.draw_text(0, 3, ("NET:" + (ssid or "NO WIFI"))[:16])
    disp.draw_text(0, 4, ("IP:" + (ip or "0.0.0.0"))[:16])
    disp.draw_text(0, 6, "i2c 0x3C")
    disp.flush()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Muestra SSID + IP del rover en el OLED SSD1306 (I2C 0x3C)"
    )
    parser.add_argument("--once", action="store_true",
                        help="Actualiza una sola vez y sale (p.ej. al arranque)")
    parser.add_argument("--interval", type=float, default=3.0,
                        help="Periodo de refresco en segundos (default 3.0)")
    args = parser.parse_args()

    try:
        import rover_bridge
        disp = rover_bridge.OledDisplay()
    except Exception as e:
        print(f"[OLED] No se pudo abrir el display: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        refresh(disp)
        if args.once:
            return
        while True:
            time.sleep(args.interval)
            try:
                refresh(disp)
            except Exception as e:
                print(f"[OLED] error de refresco: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        if not args.once:
            try:
                disp.power_off()
            except Exception:
                pass


if __name__ == "__main__":
    main()
