#!/bin/bash

set -e

if [ "${PLATFORM}" -eq "4" ]; then # JUNO
  # Make
  make

  # .read
  ${KERNEL_CROSS_COMPILE}objdump -DS pulp.ko > pulp.read
else # ZYNQ
  # Make
  ${VIVADO_VERSION} make

  # .read
  ${VIVADO_VERSION} ${KERNEL_CROSS_COMPILE}objdump -DS pulp.ko > pulp.read
fi

# Create folder on target
ssh ${SCP_TARGET_MACHINE} "mkdir -p ${SCP_TARGET_PATH_DRIVERS}"

# Copy
scp *.ko ${SCP_TARGET_MACHINE}:${SCP_TARGET_PATH_DRIVERS}/.
