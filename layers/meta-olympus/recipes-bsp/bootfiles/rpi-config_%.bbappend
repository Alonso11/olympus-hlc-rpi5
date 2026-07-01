# OV5647 (Camera Module v1) en CAM1 (conector izquierdo).
#   CAM1 (left connector) = CSI0 / 1f00128000 / i2c@80000
# camera_auto_detect=0: módulos sin EEPROM requieren overlay explícito.
# RPI_EXTRA_CONFIG must be set here (rpi-config scope), not in the image recipe.
RPI_EXTRA_CONFIG += "dtoverlay=ov5647,cam1"

# --- I2C bus 1 (GPIO2=SDA, GPIO3=SCL, pines 3/5 del header de 40 pines) ---
# Necesario para el display OLED SSD1306 (dir 0x3C) del rover.
# dtparam=i2c_arm=on activa /dev/i2c-1 en el BCM2712 (RPi5).
RPI_EXTRA_CONFIG += "dtparam=i2c_arm=on"

# meta-raspberrypi appends camera_auto_detect=1 via its own RPI_EXTRA_CONFIG
# after ours, overriding our setting. We strip all occurrences in do_install
# and append camera_auto_detect=0 last so it is the definitive value.
do_install:append() {
    config_file=$(find ${D} -name "config.txt" 2>/dev/null | head -n 1)
    if [ -n "$config_file" ]; then
        sed -i '/camera_auto_detect/d' "$config_file"
        echo "camera_auto_detect=0" >> "$config_file"
    fi

    # Instalar overlay OV5647 al directorio de overlays del boot
    overlays_dir=$(find ${D} -type d -name "overlays" 2>/dev/null | head -n 1)
    if [ -n "$overlays_dir" ]; then
        install -m 0644 ${THISDIR}/files/ov5647.dtbo "$overlays_dir/ov5647.dtbo"
    fi
}
