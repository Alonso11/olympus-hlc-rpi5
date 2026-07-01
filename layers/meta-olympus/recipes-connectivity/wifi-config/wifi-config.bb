# Version: v2.0
SUMMARY = "WiFi client with AP fallback for Olympus Rover (sysvinit)"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://wpa_supplicant.conf \
           file://wifi-connect"

S = "${WORKDIR}"

# sysvinit: usar update-rc.d para instalar symlinks de runlevel
inherit update-rc.d

# wpa_supplicant (cliente), hostapd + dnsmasq (AP fallback), iw (modo/config)
RDEPENDS:${PN} += "wpa-supplicant hostapd dnsmasq iw"

# Arrancar al boot (S=stop en 0/1/6, S=start en 2/3/4/5), prioridad 99 (ultimo)
INITSCRIPT_NAME = "wifi-connect"
INITSCRIPT_PARAMS = "defaults 99 10"

do_install() {
    # wpa_supplicant.conf en /etc/wpa_supplicant/ (evita conflicto con
    # /etc/wpa_supplicant.conf que instala el paquete wpa-supplicant de poky)
    install -d ${D}${sysconfdir}/wpa_supplicant
    install -m 0600 ${WORKDIR}/wpa_supplicant.conf ${D}${sysconfdir}/wpa_supplicant/wpa_supplicant.conf

    # init.d script
    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/wifi-connect ${D}${sysconfdir}/init.d/wifi-connect
}

FILES:${PN} += "${sysconfdir}/wpa_supplicant/wpa_supplicant.conf \
                ${sysconfdir}/init.d/wifi-connect"
