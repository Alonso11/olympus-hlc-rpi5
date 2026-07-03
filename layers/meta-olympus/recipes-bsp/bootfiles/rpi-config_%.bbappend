# IMX219 (Camera Module v2) en CAM0 (conector derecho).
#   CAM0 (right connector) = CSI1 / 1f00150000 / i2c@90000
RPI_EXTRA_CONFIG += "\ndtoverlay=imx219,cam0"

# OV5647 (Camera Module v1) en CAM1 (conector izquierdo).
#   CAM1 (left connector) = CSI0 / 1f00128000 / i2c@80000
# camera_auto_detect=0: módulos sin EEPROM requieren overlay explícito.
# RPI_EXTRA_CONFIG must be set here (rpi-config scope), not in the image recipe.
RPI_EXTRA_CONFIG += "\ndtoverlay=ov5647,cam1"

# --- I2C bus 1 (GPIO2=SDA, GPIO3=SCL, pines 3/5 del header de 40 pines) ---
# Necesario para el display OLED SSD1306 (dir 0x3C) del rover.
# dtparam=i2c_arm=on activa /dev/i2c-1 en el BCM2712 (RPi5).
RPI_EXTRA_CONFIG += "\ndtparam=i2c_arm=on"

# THISDIR en shell functions de bbappend apunta al directorio de la receta base,
# no al del bbappend. Usamos FILESEXTRAPATHS + SRC_URI para localizar el .dtbo.
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "file://ov5647.dtbo"

# La receta base (rpi-config_git.bb) procesa RPI_EXTRA_CONFIG en do_deploy,
# NO en do_install. Por eso el anterior do_install:append() JAMÁS se ejecutaba
# y camera_auto_detect=1 de local.conf quedaba intacto.
# Forzamos camera_auto_detect=0 al final de do_deploy para que sea el valor
# definitivo (módulos sin EEPROM como OV5647 necesitan dtoverlay explícito).
do_deploy:append() {
    CONFIG="${DEPLOYDIR}/${BOOTFILES_DIR_NAME}/config.txt"
    if [ -f "$CONFIG" ]; then
        sed -i '/camera_auto_detect/d' "$CONFIG"
        echo "camera_auto_detect=0" >> "$CONFIG"
    fi

    # Instalar overlay OV5647 flat en DEPLOYDIR (como hacen los overlays del
    # kernel). Luego IMAGE_BOOT_FILES lo mapea a overlays/ov5647.dtbo en la
    # particion boot. No crear subdirectorio overlays/ dentro de BOOTFILES_DIR_NAME
    # porque el glob BOOTFILES_DIR_NAME/* en WIC no maneja subdirectorios.
    install -m 0644 ${WORKDIR}/ov5647.dtbo "${DEPLOYDIR}/ov5647.dtbo"
}
