FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "file://radiotea5767-overlay.dts;subdir=git/arch/arm/boot/dts/overlays \
            "

KERNEL_DEVICETREE:append = " overlays/radiotea5767.dtbo"

