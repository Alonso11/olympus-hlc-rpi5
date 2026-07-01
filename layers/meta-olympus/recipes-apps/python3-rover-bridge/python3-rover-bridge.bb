# Version: v1.5
SUMMARY = "Extensión nativa de Python en Rust para control de Rover (Olympus Bridge)"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://rover-bridge/ \
           file://test_rover.py \
           file://test_bridge.py \
           file://test_bridge_interactive.py \
           file://test_ultrasonic_rpi.py \
           file://test_opencv_camera.py \
           file://olympus_controller.py \
           file://oled_status.py \
           file://olympus_hlc/ \
           file://test_smoke.py \
           file://debug_vision.py \
           file://yolov8n.onnx \
           file://yolov8n-seg.onnx \
           file://configs/olympus_controller.yaml"

# El código está en la subcarpeta rover-bridge
S = "${WORKDIR}/rover-bridge"

inherit cargo python3native python3-dir pkgconfig

# Dependencias para compilar la extensión nativa (necesita udev para serialport)
DEPENDS += "python3 python3-setuptools-native udev"
RDEPENDS:${PN} += "python3-core python3-pyserial python3-pyyaml udev libcsp custom-udev-rules"

# Configuración para usar las fuentes vendoreadas incluidas en el repo
do_configure:prepend() {
    # Bitbake's cargo class expects offline crates in this specific directory
    mkdir -p ${WORKDIR}/cargo_home/bitbake
    # Symlink all vendored crates to where Bitbake expects them
    if [ -d "${S}/vendor" ]; then
        ln -sf ${S}/vendor/* ${WORKDIR}/cargo_home/bitbake/
    fi
}

# Forzamos a Cargo a trabajar offline
export CARGO_OFFLINE = "1"

# Variables para compilación cruzada de PyO3
export PYO3_CROSS = "1"
export PYO3_CROSS_PYTHON_VERSION = "3.12"
export PYO3_CROSS_LIB_DIR = "${STAGING_LIBDIR}"
export PYO3_CONFIG_INTERPRETER = "${PYTHON}"

# Forzamos la instalación de la librería dinámica (.so) en el directorio de paquetes de Python
do_install() {
    install -d ${D}${PYTHON_SITEPACKAGES_DIR}
    # Buscamos el archivo .so generado por Cargo y lo movemos a site-packages
    install -m 0755 ${B}/target/${CARGO_TARGET_SUBDIR}/librover_bridge.so ${D}${PYTHON_SITEPACKAGES_DIR}/rover_bridge.so

    # Instalamos los scripts de prueba en /usr/bin de la RPi
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/test_smoke.py ${D}${bindir}/test_smoke.py
    install -m 0755 ${WORKDIR}/test_rover.py ${D}${bindir}/test_rover.py
    install -m 0755 ${WORKDIR}/test_bridge.py ${D}${bindir}/test_bridge.py
    install -m 0755 ${WORKDIR}/test_bridge_interactive.py ${D}${bindir}/test_bridge_interactive.py
    install -m 0755 ${WORKDIR}/test_ultrasonic_rpi.py ${D}${bindir}/test_ultrasonic_rpi.py
    install -m 0755 ${WORKDIR}/test_opencv_camera.py ${D}${bindir}/test_opencv_camera.py
    install -m 0755 ${WORKDIR}/olympus_controller.py ${D}${bindir}/olympus_controller.py
    install -m 0755 ${WORKDIR}/debug_vision.py ${D}${bindir}/debug_vision.py
    install -m 0755 ${WORKDIR}/oled_status.py ${D}${bindir}/oled_status.py

    # Instalar el paquete olympus_hlc (refactorización SOLID, v3.0)
    install -d ${D}${PYTHON_SITEPACKAGES_DIR}/olympus_hlc/sources
    cp -r ${WORKDIR}/olympus_hlc/. ${D}${PYTHON_SITEPACKAGES_DIR}/olympus_hlc/
    # Entry point del paquete (invocable como script)
    ln -sf ${PYTHON_SITEPACKAGES_DIR}/olympus_hlc/__main__.py \
           ${D}${bindir}/olympus_hlc

    install -d ${D}${datadir}/olympus/models
    install -m 0644 ${WORKDIR}/yolov8n.onnx ${D}${datadir}/olympus/models/yolov8n.onnx
    install -m 0644 ${WORKDIR}/yolov8n-seg.onnx ${D}${datadir}/olympus/models/yolov8n-seg.onnx

    install -d ${D}${sysconfdir}/olympus
    install -m 0644 ${WORKDIR}/configs/olympus_controller.yaml ${D}${sysconfdir}/olympus/olympus_controller.yaml
}

FILES:${PN} += "${PYTHON_SITEPACKAGES_DIR}/rover_bridge.so \
                ${PYTHON_SITEPACKAGES_DIR}/olympus_hlc/ \
                ${bindir}/test_smoke.py \
                ${bindir}/debug_vision.py \
                ${bindir}/test_rover.py \
                ${bindir}/test_bridge.py \
                ${bindir}/test_bridge_interactive.py \
                ${bindir}/test_ultrasonic_rpi.py \
                ${bindir}/test_opencv_camera.py \
                ${bindir}/olympus_controller.py \
                ${bindir}/oled_status.py \
                ${bindir}/olympus_hlc \
                ${datadir}/olympus/models/yolov8n.onnx \
                ${datadir}/olympus/models/yolov8n-seg.onnx \
                ${sysconfdir}/olympus/olympus_controller.yaml"
