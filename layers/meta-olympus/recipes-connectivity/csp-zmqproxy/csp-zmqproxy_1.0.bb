SUMMARY = "CSP ZMQ Proxy systemd service — libcsp routing hub para WiFi"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

# El binario csp_zmqproxy lo instala libcsp. Esta receta solo instala
# el servicio systemd que lo arranca al boot.

inherit systemd

SRC_URI = "file://csp-zmqproxy.service"
S = "${WORKDIR}"

SYSTEMD_SERVICE:${PN} = "csp-zmqproxy.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_install() {
    install -d ${D}${systemd_unitdir}/system
    install -m 0644 ${WORKDIR}/csp-zmqproxy.service \
        ${D}${systemd_unitdir}/system/csp-zmqproxy.service
}

FILES:${PN} = "${systemd_unitdir}/system/csp-zmqproxy.service"

RDEPENDS:${PN} = "libcsp zeromq"
