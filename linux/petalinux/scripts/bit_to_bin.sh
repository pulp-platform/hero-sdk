#!/bin/bash

# Convert bitstream to bin file for on-line reocnfiguration

# Copy bitstream
cp ${VIVADO_EXPORT_PATH}/bigpulp-zux.bit ${IMAGE_PATH}/.

# Generate the bin file
bootgen -image boot_a53_bin_pl_all.bif -arch zynqmp -process_bitstream bin -w on

# upload to NFS
scp update_bitstream.sh ${SCP_TARGET_MACHINE}:${SCP_TARGET_PATH}/bitstream/.
scp bigpulp-zux.bit.bin ${SCP_TARGET_MACHINE}:${SCP_TARGET_PATH}/bitstream/.
