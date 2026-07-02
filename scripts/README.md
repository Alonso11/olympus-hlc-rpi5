# Scripts de Olympus para RPi5

## Requisitos

- **SO:** Linux (probado en Ubuntu 22.04 / Debian 12)
- **Dependencias:**
  - `gcloud` CLI (solo para `deploy-olympus-image.sh`)
  - `bmaptool` (opcional, acelera el flasheo) — `sudo apt install bmaptool`
  - `bzip2` — `sudo apt install bzip2`
  - `sudo` (para flashear la microSD)

## Scripts

### `setup-env.sh`

Clona o actualiza las capas Yocto necesarias (poky, meta-raspberrypi, meta-openembedded, meta-tensorflow-lite, meta-onnxruntime).

```bash
./scripts/setup-env.sh
```

### `deploy-olympus-image.sh`

Descarga la imagen Yocto compilada desde una VM de GCP a la máquina local.

```bash
./scripts/deploy-olympus-image.sh [directorio_destino]
```

- `directorio_destino`: opcional, por defecto `./olympus-image`

### `flash-olympus-image.sh`

Flashea una imagen `.wic.bz2` a una microSD.

```bash
sudo ./scripts/flash-olympus-image.sh [directorio_imagen] [dispositivo]
```

- `directorio_imagen`: directorio con los archivos `.wic.bz2` y `.wic.bmap`. Por defecto `./olympus-image`.
- `dispositivo`: dispositivo de la microSD (ej. `/dev/sda`, `/dev/mmcblk0`). Por defecto `/dev/sdb`.

**Advertencia:** Borra todo el contenido del dispositivo. Verifica con `lsblk` antes.

## Flujo típico con compilación local

```bash
# 1. Preparar entorno (solo una vez)
./scripts/setup-env.sh

# 2. Cargar entorno y compilar
source layers/poky/oe-init-build-env build
bitbake olympus-image

# 3. Flashear la imagen a la microSD
sudo ./scripts/flash-olympus-image.sh \
  ./build/tmp/deploy/images/raspberrypi5 \
  /dev/sda
```
