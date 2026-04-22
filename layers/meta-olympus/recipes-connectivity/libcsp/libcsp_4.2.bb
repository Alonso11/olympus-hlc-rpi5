SUMMARY = "CubeSat Space Protocol library"
LICENSE = "LGPL-2.1-only"
LIC_FILES_CHKSUM = "file://LICENSE;md5=e38286c6cb20ecbf85b80bb4af68efdc"

SRC_URI = "git://github.com/libcsp/libcsp.git;protocol=https;branch=develop \
           file://0001-add-udp-python-binding.patch \
           file://0002-udp-dynamic-peer-learning.patch \
"
SRCREV = "51628cd7a208edff81eff9b2b6fadc70dea5c5a4"

S = "${WORKDIR}/git"

DEPENDS = " \
    python3-native \
    cmake-native \
"

inherit cmake python3native

EXTRA_OECMAKE = " \
    -DCSP_ENABLE_PYTHON3_BINDINGS=ON \
    -DCSP_IF_UDP=ON \
    -DCSP_USE_RTABLE=ON \
    -DCSP_HAVE_STDIO=ON \
    -DPYTHON_EXECUTABLE=${PYTHON} \
    -DCSP_BUFFER_COUNT=20 \
    -DCSP_BUFFER_SIZE=256 \
"

SOLIBS = ".so"
FILES_SOLIBSDEV = ""

FILES:${PN} += " \
    ${libdir}/libcsp.so* \
    ${libdir}/python3*/site-packages/libcsp_py3* \
    ${libdir}/python3*/site-packages/csp* \
    ${libdir}/python3*/site-packages/_csp* \
"

FILES:${PN}-dev += " \
    ${includedir}/csp/* \
"

RDEPENDS:${PN} = " \
    python3-core \
    python3-ctypes \
"

INSANE_SKIP:${PN}-dev = "dev-elf"
