#!/bin/bash

# Compile script for U-Boot

# Info
echo "-----------------------------------------"
echo "-    Executing U-Boot compile script    -"
echo "-----------------------------------------"

# Add kernel scripts to PATH - dtc required
Scripts="linux-xlnx/scripts/dtc"
if [ -d "../"${Scripts} ]; then
    echo "Adding kernel scripts to PATH"
    CurrentPath=`pwd`
    cd "../"${Scripts}
    export PATH=`pwd`:${PATH}
    cd ${CurrentPath}
else
    echo "ERROR: Kernel scripts not found"
    echo "Check for the folder ../"${Scripts}
    exit 1
fi

# Compile U-Boot
echo "Comiling U-Boot"
make CROSS_COMPILE=arm-xilinx-linux-gnueabi- zynq_zc706_config
make CROSS_COMPILE=arm-xilinx-linux-gnueabi- $1

# Copy
echo "Copying u-boot to ../sdk/u-boot.elf"
cp u-boot ../sdk/u-boot.elf
