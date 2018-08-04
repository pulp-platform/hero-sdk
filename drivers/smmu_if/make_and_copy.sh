#!/bin/bash

set -e

# Make
make

# .read
${KERNEL_CROSS_COMPILE}objdump -DS smmu_if.ko > smmu_if.read

# Create folder on target
ssh ${SCP_TARGET_MACHINE} "mkdir -p ${SCP_TARGET_PATH_DRIVERS}"

# Copy
scp *.ko ${SCP_TARGET_MACHINE}:${SCP_TARGET_PATH_DRIVERS}/.
