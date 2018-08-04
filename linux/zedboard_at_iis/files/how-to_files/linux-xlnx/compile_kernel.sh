#!/bin/bash

# Compile script for the Linux kernel
# Call using vivado-2014.1 ./compile_kernel.sh

# Info
echo "-----------------------------------------"
echo "- Executing Linux kernel compile script -"
echo "-----------------------------------------"

# Add U-Boot tools to PATH - mkimage required for building uImage
Tools="u-boot-xlnx/tools"
if [ -d "../"${Tools} ]; then
    echo "Adding U-Boot tools to PATH"
    CurrentPath=`pwd`
    cd "../"${Tools}
    export PATH=`pwd`:${PATH}
    cd ${CurrentPath}
else
    echo "ERROR: U-Boot tools not found"
    echo "Check for the folder ../"${tools}
    exit 1
fi

# Compile the Linux kernel
echo "Comiling the Linux kernel"
make ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi- $1 UIMAGE_LOADADDR=0x8000 uImage

# Copy
echo "Copying uImage to ../sd_image/uImage"
cp arch/arm/boot/uImage ../sd_image/uImage
