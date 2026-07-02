#!/bin/bash
# deploy-olympus-image.sh
# Descarga la imagen Yocto compilada desde la VM de GCP a la maquina local.

VM_NAME="instance-20260309-151629"
VM_ZONE="us-central1-a"
VM_IMAGE_PATH="/home/m_r_homero11_2002/rpi5-yocto-project/rpi5-optim-for-olympus-image/build/tmp/deploy/images/raspberrypi5"
LOCAL_DEST="${1:-./olympus-image}"

mkdir -p "$LOCAL_DEST"

echo "==> Descargando imagen desde $VM_NAME ($VM_ZONE)..."
gcloud compute scp \
    --zone="$VM_ZONE" \
    --recurse \
    --ssh-key-file="$HOME/.ssh/google_compute_engine" \
    "m_r_homero11_2002@${VM_NAME}:${VM_IMAGE_PATH}/olympus-image-raspberrypi5.rootfs.wic.bz2" \
    "m_r_homero11_2002@${VM_NAME}:${VM_IMAGE_PATH}/olympus-image-raspberrypi5.rootfs.wic.bmap" \
    "$LOCAL_DEST/"

if [ $? -eq 0 ]; then
    echo "==> Imagen descargada en: $LOCAL_DEST"
    ls -lh "$LOCAL_DEST"
else
    echo "ERROR: Fallo la descarga. Revisa que la imagen este compilada en la VM."
    exit 1
fi
