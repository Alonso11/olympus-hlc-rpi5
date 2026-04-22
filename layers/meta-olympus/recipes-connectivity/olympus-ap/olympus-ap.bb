SUMMARY = "Olympus AP — hotspot WiFi para operación en campo"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://hostapd.conf \
    file://dnsmasq-ap.conf \
    file://olympus-ap.service \
    file://olympus-mode.sh \
"

S = "${WORKDIR}"

inherit systemd

RDEPENDS:${PN} = "hostapd dnsmasq"

# Deshabilitado por defecto — activar con: systemctl enable olympus-ap
# o con el script: olympus-mode field
SYSTEMD_SERVICE:${PN} = "olympus-ap.service"
SYSTEMD_AUTO_ENABLE:${PN} = "disable"

do_install() {
    # hostapd config
    install -d ${D}${sysconfdir}/hostapd
    install -m 0600 ${WORKDIR}/hostapd.conf \
        ${D}${sysconfdir}/hostapd/hostapd.conf

    # dnsmasq config para el AP (no sobreescribe el dnsmasq.conf del sistema)
    install -m 0644 ${WORKDIR}/dnsmasq-ap.conf \
        ${D}${sysconfdir}/dnsmasq-ap.conf

    # systemd service
    install -d ${D}${systemd_unitdir}/system
    install -m 0644 ${WORKDIR}/olympus-ap.service \
        ${D}${systemd_unitdir}/system/olympus-ap.service

    # Script de cambio de modo
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/olympus-mode.sh \
        ${D}${sbindir}/olympus-mode
}

FILES:${PN} = " \
    ${sysconfdir}/hostapd/hostapd.conf \
    ${sysconfdir}/dnsmasq-ap.conf \
    ${systemd_unitdir}/system/olympus-ap.service \
    ${sbindir}/olympus-mode \
"
