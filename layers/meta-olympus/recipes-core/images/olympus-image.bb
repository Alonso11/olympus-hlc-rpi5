# Version: v1.5
SUMMARY = "Olympus Image: WiFi, UART, Sensors and Vision"
LICENSE = "MIT"

inherit core-image

IMAGE_FSTYPES = "wic.bz2 wic.bmap"

# Añadir soporte para WiFi, UART, SSH, Redimensionamiento, Sensores y Vision.
#
# onnxruntime (recomendación E): habilita el backend de inferencia ort
# (INFERENCE_BACKEND="onnxruntime" en olympus_controller.yaml) y permite cargar
# los artefactos *_int8.onnx en files/models-optimized/. La receta vive en
# meta-onnxruntime y declara do_configure[network]=1 — trae dependencias a la
# red durante el build. Con INFERENCE_BACKEND="opencv" (default) este paquete no
# se usa en runtime, pero lo dejamos instalado para que el flip sea solo YAML.
#
# python3-psutil: NO NECESARIO. SystemMonitor (olympus_hlc/sysmon.py) lee
# CPU/RAM/temp directamente de /proc y /sys — sin dependencias extra. La
# receta python3-psutil existe en poky/meta pero su package feed no estaba
# listo en la pasada anterior; quitarla evita una dependencia innecesaria.
IMAGE_INSTALL:append = " \
    libcsp \
    libcsp-dev \
    csp-sfp-rover \
    avahi-daemon \
    avahi-olympus \
    hostapd \
    dnsmasq \
    olympus-ap \
    custom-udev-rules \
    resize-rootfs \
    wifi-config \
    packagegroup-core-boot \
    kernel-modules \
    kernel-module-cdc-acm \
    iw \
    wpa-supplicant \
    linux-firmware-rpidistro-bcm43455 \
    python3-core \
    python3-pyserial \
    python3-numpy \
    python3-opencv \
    onnxruntime \
    libpisp \
    libcamera \
    libcamera-apps \
    v4l-utils \
    libudev \
    bash \
    cpufrequtils \
    powertop \
    python3-rover-bridge \
    openssh \
    openssh-sftp-server \
"

# Habilitar login root sin contraseña para desarrollo
EXTRA_IMAGE_FEATURES += "debug-tweaks ssh-server-openssh"

# Mantenemos WiFi, pero eliminamos Gráficos y Bluetooth para ahorrar energía
DISTRO_FEATURES:append = " wifi"
DISTRO_FEATURES:remove = "x11 wayland vulkan opengl bluetooth"
