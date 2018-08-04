#!/bin/bash

# Generate the BOOT.BIN to be placed on the SD card.

# Copy U-Boot and PMU firmware for TE0808 board
if [ $BOARD == te0808 ]; then
  echo "Copying prebuilt U-Boot and PMU firwmare images for TE0808"
  cp ${PREBUILT_PATH}/os/petalinux/default/u-boot.elf                ${IMAGE_PATH}/.
  cp ${PREBUILT_PATH}/software/te0808_2es2_tebf0808/zynqmp_pmufw.elf ${IMAGE_PATH}/.
fi

# Copy custom FSBL
if [ $BOARD == te0808 ]; then
  echo "Copying custom FSBL for TE0808"
  cp ${VIVADO_EXPORT_PATH}/FSBL/Debug/FSBL.elf ${IMAGE_PATH}/zynqmp_fsbl.elf
fi


# Copy the patched ARM Trusted Firmware
cp ../../components/ext_sources/arm-trusted-firmware/build/zynqmp/release/bl31/bl31.elf ${IMAGE_PATH}/.
cp ../../components/ext_sources/arm-trusted-firmware/build/zynqmp/release/bl31.bin      ${IMAGE_PATH}/.

# Copy bitstream
cp ${VIVADO_EXPORT_PATH}/bigpulp-zux.bit ${IMAGE_PATH}/.

# Go to image folder and generate
cd ${IMAGE_PATH}
petalinux-package --boot --fsbl zynqmp_fsbl.elf --fpga bigpulp-zux.bit --u-boot u-boot.elf --pmufw zynqmp_pmufw.elf --force --bif bootgen.bif
