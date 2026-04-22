SUMMARY = "Avahi mDNS service descriptor para Olympus Rover"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://olympus.service"

S = "${WORKDIR}"

RDEPENDS:${PN} = "avahi-daemon"

do_install() {
    install -d ${D}${sysconfdir}/avahi/services
    install -m 0644 ${WORKDIR}/olympus.service \
        ${D}${sysconfdir}/avahi/services/olympus.service
}

FILES:${PN} = "${sysconfdir}/avahi/services/olympus.service"
