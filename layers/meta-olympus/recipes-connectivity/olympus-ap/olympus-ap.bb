# Version: v2.0
SUMMARY = "Olympus AP config — hostapd/dnsmasq para hotspot (field mode)"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

# Solo configs — el init.d script wifi-connect gestiona el ciclo de vida.
SRC_URI = "file://hostapd.conf \
           file://dnsmasq-ap.conf \
           file://olympus-mode.sh"

S = "${WORKDIR}"

RDEPENDS:${PN} = "hostapd dnsmasq wifi-config"

do_install() {
    # hostapd config
    install -d ${D}${sysconfdir}/hostapd
    install -m 0600 ${WORKDIR}/hostapd.conf \
        ${D}${sysconfdir}/hostapd/hostapd.conf

    # dnsmasq config para el AP (no sobreescribe el dnsmasq.conf del sistema)
    install -m 0644 ${WORKDIR}/dnsmasq-ap.conf \
        ${D}${sysconfdir}/dnsmasq-ap.conf

    # Script de cambio de modo (usa init.d, no systemctl)
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/olympus-mode.sh \
        ${D}${sbindir}/olympus-mode
}

FILES:${PN} = " \
    ${sysconfdir}/hostapd/hostapd.conf \
    ${sysconfdir}/dnsmasq-ap.conf \
    ${sbindir}/olympus-mode \
"
