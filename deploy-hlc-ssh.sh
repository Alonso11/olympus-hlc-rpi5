#!/bin/bash
# deploy-hlc-ssh.sh — Copia el paquete olympus_hlc actualizado a una RPi5 que ya
# corre la imagen Yocto, SIN reconstruir/reflashear (scp/tar por SSH).
#
# Por qué: la imagen Yocto instala olympus_hlc en site-packages; para iterar el
# código Python basta sobreescribir esos .py (rover_bridge.so y libcsp_py3 NO
# cambian, así que no hay que recompilar la imagen).
#
# Uso:
#   ./deploy-hlc-ssh.sh <host_pi> [usuario]
#   ./deploy-hlc-ssh.sh 192.168.100.1            # usuario root por defecto
#   ./deploy-hlc-ssh.sh olympus-rover.local root
#
# Qué copia: olympus_hlc/ (incluye sources/station.py, vision.py refactor, etc.)
# desde el árbol fuente del repo hacia ${site-packages}/olympus_hlc/ en la Pi.

set -euo pipefail

HOST="${1:?Uso: $0 <host_pi> [usuario]}"
USER_PI="${2:-root}"
TGT="${USER_PI}@${HOST}"

# Árbol fuente canónico (lo que empaqueta la receta Yocto).
SRC_DIR="$(cd "$(dirname "$0")" && pwd)/layers/meta-olympus/recipes-apps/python3-rover-bridge/files"
if [ ! -d "$SRC_DIR/olympus_hlc" ]; then
    echo "ERROR: no encuentro $SRC_DIR/olympus_hlc" >&2
    exit 1
fi

echo "==> Pi destino: $TGT"
echo "==> Detectando site-packages en la Pi..."
SITEPKG="$(ssh "$TGT" 'ls -d /usr/lib/python3*/site-packages 2>/dev/null | head -1')"
if [ -z "$SITEPKG" ]; then
    echo "ERROR: no pude detectar site-packages en la Pi (¿SSH OK? ¿imagen Yocto?)" >&2
    exit 1
fi
echo "==> site-packages: $SITEPKG"

if [ ! -e "$SITEPKG/olympus_hlc/__main__.py" ] 2>/dev/null; then :; fi
echo "==> Respaldo del olympus_hlc actual en la Pi (~/.olympus_hlc.bak.tgz)..."
ssh "$TGT" "tar -C '$SITEPKG' -czf ~/.olympus_hlc.bak.tgz olympus_hlc 2>/dev/null || true"

echo "==> Copiando olympus_hlc/ → $SITEPKG/ (tar sobre SSH)..."
tar -C "$SRC_DIR" -cf - olympus_hlc | ssh "$TGT" "tar -C '$SITEPKG' -xf -"

echo "==> Verificando que --mode station existe en la Pi..."
ssh "$TGT" "python3 -c 'import olympus_hlc.sources.station as s; print(\"station OK:\", s.StationSource.__name__)'" \
    || echo "AVISO: import falló (revisar dependencias en la Pi: cv2/yaml/etc.)"

echo "==> Listo. Para validar CSP:"
echo "    Pi:     olympus_hlc --mode gcs --dry-run     # o: python3 -m olympus_hlc --mode gcs --dry-run"
echo "    Laptop: python3 gcs_mock.py $HOST"
echo "    (Restaurar si algo sale mal:  ssh $TGT 'tar -C $SITEPKG -xzf ~/.olympus_hlc.bak.tgz')"
