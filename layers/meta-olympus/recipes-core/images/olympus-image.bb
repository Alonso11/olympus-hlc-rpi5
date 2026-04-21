# Version: v1.4
SUMMARY = "Imagen Olympus: WiFi, UART, Sensores y Vision"
LICENSE = "MIT"

inherit core-image

# Añadir soporte para WiFi, UART, SSH, Redimensionamiento, Sensores y Vision
IMAGE_INSTALL:append = " \
    libcsp \
    libcsp-dev \
    zeromq \
    csp-zmqproxy \
    custom-udev-rules \
    resize-rootfs \
    wifi-config \
    wifi-power-save \
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
