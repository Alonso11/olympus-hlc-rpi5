#!/bin/bash
# olympus-mode.sh — cambia entre modo lab (WiFi cliente) y field (AP hotspot)
#
# Uso:
#   olympus-mode field    → crea red Olympus-Rover (192.168.100.1)
#   olympus-mode lab      → conecta a red del lab via wpa_supplicant
#   olympus-mode status   → muestra modo activo e IP actual

set -e

case "${1:-status}" in

  field)
    echo "[olympus-mode] Activando modo FIELD (AP hotspot)"
    systemctl disable --now wpa_supplicant@wlan0.service 2>/dev/null || true
    systemctl enable --now olympus-ap.service
    sleep 2
    echo "[olympus-mode] Listo"
    echo "  SSID     : Olympus-Rover"
    echo "  Password : olympus2026"
    echo "  IP rover : 192.168.100.1"
    echo "  GCS cmd  : python3 gcs_mock.py 192.168.100.1"
    ;;

  lab)
    echo "[olympus-mode] Activando modo LAB (WiFi cliente)"
    systemctl disable --now olympus-ap.service 2>/dev/null || true
    systemctl enable --now wpa_supplicant@wlan0.service
    sleep 3
    IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "pendiente")
    echo "[olympus-mode] Listo"
    echo "  IP rover : ${IP}"
    echo "  GCS cmd  : python3 gcs_mock.py olympus-rover.local"
    ;;

  status)
    if systemctl is-active --quiet olympus-ap.service 2>/dev/null; then
      echo "FIELD — AP activo (Olympus-Rover / 192.168.100.1)"
    elif systemctl is-active --quiet wpa_supplicant@wlan0.service 2>/dev/null; then
      IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "?")
      echo "LAB — WiFi cliente activo (IP: ${IP})"
    else
      echo "SIN CONEXION — ningún servicio WiFi activo"
    fi
    ;;

  *)
    echo "Uso: olympus-mode field|lab|status"
    exit 1
    ;;
esac
