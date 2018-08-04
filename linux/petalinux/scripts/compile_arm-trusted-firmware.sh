#!/bin/bash

# Generate patched ARM Trusted Firmware

# Navigate to source folder
cd ../../components/ext_sources/arm-trusted-firmware

# Set CROSS_COMPILE (needs absolute path to compiler)
CROSS_COMPILE=`which aarch64-linux-gnu-gcc`
TMP=${CROSS_COMPILE::-3}
export CROSS_COMPILE=$TMP

# Do it!
make PLAT=zynqmp RESET_TO_BL31=1 all
