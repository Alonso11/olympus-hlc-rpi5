SUMMARY = "CubeSat Space Protocol library"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=2915dc85ab8fd26629e560d023ef175c"

SRC_URI = "git://github.com/CiQiuu/libcsp-ELANav.git;protocol=https;branch=develop \
           file://0001-bindings-python-add-udp-init-wrapper.patch \
           file://0002-if-udp-dynamic-peer-learning.patch \
           file://0003-do-not-build-examples-by-default.patch \
"
SRCREV = "88556daef794d08b6f412624ef40acf170c1fe23"

S = "${WORKDIR}/git"

DEPENDS = " \
    python3 \
    python3-native \
    cmake-native \
"

inherit cmake python3native

# Disable CMake's Python binding — it cross-compiles for the build host (x86_64)
# instead of the target (aarch64). We compile it manually in do_compile:append.
EXTRA_OECMAKE = " \
    -DCSP_ENABLE_PYTHON3_BINDINGS=OFF \
    -DCSP_IF_UDP=ON \
    -DCSP_USE_RTABLE=ON \
    -DCSP_HAVE_STDIO=ON \
    -DCSP_BUFFER_COUNT=20 \
    -DCSP_BUFFER_SIZE=256 \
"

SOLIBS = ".so"
FILES_SOLIBSDEV = ""

do_compile:append() {
    libcsp_so=$(find ${B} -name "libcsp.so" | head -1)
    libcsp_dir=$(dirname ${libcsp_so})
    ${CC} ${CFLAGS} ${LDFLAGS} -shared -fPIC \
        -I${STAGING_INCDIR}/python${PYTHON_BASEVERSION} \
        -I${S}/include \
        -I${B}/include \
        ${S}/src/bindings/python/pycsp.c \
        -L${libcsp_dir} -lcsp \
        -o ${B}/libcsp_py3.so
}

do_install:append() {
    install -d ${D}${PYTHON_SITEPACKAGES_DIR}
    install -m 0755 ${B}/libcsp_py3.so ${D}${PYTHON_SITEPACKAGES_DIR}/libcsp_py3.so
}

FILES:${PN} += " \
    ${libdir}/libcsp.so* \
    ${PYTHON_SITEPACKAGES_DIR}/libcsp_py3* \
"

FILES:${PN}-dev += " \
    ${includedir}/csp/* \
"

RDEPENDS:${PN} = " \
    python3-core \
    python3-ctypes \
"

INSANE_SKIP:${PN}-dev = "dev-elf"
