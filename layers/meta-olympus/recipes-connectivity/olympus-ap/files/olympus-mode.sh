#!/bin/sh
# olympus-mode.sh — cambia entre modo lab (WiFi cliente) y field (AP hotspot)
#
# Uso:
#   olympus-mode field    → fuerza AP Olympus-Rover (192.168.100.1)
#   olympus-mode lab      → intenta cliente WiFi (estudiantes.ie)
#   olympus-mode status   → muestra modo activo e IP actual
#
# Funciona con sysvinit (init.d), no requiere systemctl.

case "${1:-status}" in

  field)
    echo "[olympus-mode] Activando modo FIELD (AP hotspot)"
    /etc/init.d/wifi-connect ap
    echo "[olympus-mode] Listo"
    echo "  SSID     : Olympus-Rover"
    echo "  Password : olympus2026"
    echo "  IP rover : 192.168.100.1"
    echo "  GCS cmd  : python3 gcs_mock.py 192.168.100.1"
    ;;

  lab)
    echo "[olympus-mode] Activando modo LAB (WiFi cliente)"
    /etc/init.d/wifi-connect restart
    sleep 3
    IP=$(ip -4 addr show wlan0 2>/dev/null | awk '/inet / {split($2,a,"/"); print a[1]; exit}' || echo "pendiente")
    echo "[olympus-mode] Listo"
    echo "  IP rover : ${IP}"
    echo "  GCS cmd  : python3 gcs_mock.py olympus-rover.local"
    ;;

  status)
    /etc/init.d/wifi-connect status
    ;;

  *)
    echo "Uso: olympus-mode field|lab|status"
    exit 1
    ;;
esac
