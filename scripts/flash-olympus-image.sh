#!/bin/bash
# flash-olympus-image.sh
# Flashea la imagen Yocto a una microSD usando bmaptool o dd como fallback.

set -euo pipefail

IMAGE_DIR="${1:-./olympus-image}"
TARGET_DEV="${2:-/dev/sdb}"

# --- Validaciones ---
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Ejecuta el script con sudo."
    exit 1
fi

if [ ! -b "$TARGET_DEV" ]; then
    echo "ERROR: Dispositivo '$TARGET_DEV' no encontrado. Conecta la microSD e intentalo de nuevo."
    exit 1
fi

# Buscar imagen .wic.bz2
IMAGE_FILE=$(ls "$IMAGE_DIR"/*.wic.bz2 2>/dev/null | head -n1)
if [ -z "$IMAGE_FILE" ]; then
    echo "ERROR: No se encontro ninguna imagen .wic.bz2 en '$IMAGE_DIR'."
    echo "Ejecuta primero deploy-olympus-image.sh"
    exit 1
fi

echo "==> Imagen:      $IMAGE_FILE"
echo "==> Dispositivo: $TARGET_DEV"
echo ""
read -r -p "ATENCION: Esto borrara TODO el contenido de $TARGET_DEV. Continuar? [s/N] " CONFIRM
if [[ ! "$CONFIRM" =~ ^[sSyY]$ ]]; then
    echo "Cancelado."
    exit 0
fi

# Desmontar particiones del dispositivo si estan montadas
echo "==> Desmontando particiones de $TARGET_DEV..."
for part in "${TARGET_DEV}"[0-9]*; do
    [ -b "$part" ] && umount "$part" 2>/dev/null || true
done

# Flashear con bmaptool si esta disponible, si no usar dd
if command -v bmaptool &>/dev/null; then
    BMAP_FILE=$(ls "$IMAGE_DIR"/*.wic.bmap 2>/dev/null | head -n1 || true)
    if [ -n "$BMAP_FILE" ]; then
        echo "==> Flasheando con bmaptool (bmap: $BMAP_FILE)..."
        bmaptool copy --bmap "$BMAP_FILE" "$IMAGE_FILE" "$TARGET_DEV"
    else
        echo "==> Flasheando con bmaptool (sin bmap)..."
        bmaptool copy --nobmap "$IMAGE_FILE" "$TARGET_DEV"
    fi
else
    echo "==> bmaptool no encontrado, usando dd (puede tardar mas)..."
    bzip2 -dc "$IMAGE_FILE" | dd of="$TARGET_DEV" bs=4M status=progress conv=fsync
    # pipefail propaga error de bzip2 si la imagen esta corrupta
fi

sync
echo ""
echo "==> Flash completado. Ya puedes retirar la microSD."
