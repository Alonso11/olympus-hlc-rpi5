SUMMARY = "CSP SFP Rover — ELANav ground rover node with SFP telemetry/images/video"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=2915dc85ab8fd26629e560d023ef175c"

SRC_URI = "git://github.com/CiQiuu/libcsp-ELANav.git;protocol=https;branch=develop \
           file://csp-rover.service \
           file://ninotnc-probe.c \
"

SRCREV = "88556daef794d08b6f412624ef40acf170c1fe23"
S = "${WORKDIR}/git"

DEPENDS = "libcsp"

inherit systemd

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE:${PN} = "csp-rover.service"

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} \
        -I${S}/include \
        -I${RECIPE_SYSROOT}/usr/include \
        -L${RECIPE_SYSROOT}/usr/lib \
        ${S}/examples/csp_sfp_rover.c \
        ${S}/examples/csp_posix_helper.c \
        -lcsp \
        -lpthread \
        -o ${B}/csp_sfp_rover

    ${CC} ${CFLAGS} ${LDFLAGS} \
        ${WORKDIR}/ninotnc-probe.c \
        -o ${B}/ninotnc-probe
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${B}/csp_sfp_rover ${D}${bindir}/csp_sfp_rover
    install -m 0755 ${B}/ninotnc-probe ${D}${bindir}/ninotnc-probe

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/csp-rover.service ${D}${systemd_system_unitdir}/csp-rover.service
}

FILES:${PN} += " \
    ${bindir}/csp_sfp_rover \
    ${bindir}/ninotnc-probe \
    ${systemd_system_unitdir}/csp-rover.service \
"
