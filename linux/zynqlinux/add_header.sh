#!/bin/bash

# This script adds the U-Boot header to the root filesystem image

# Info
echo "-----------------------------------------"
echo "-          Adding U-Boot Header         -"
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
    echo "Check for the folder "${Tools}
    exit 1
fi

# Add the header
echo "Adding the header"
mkimage -A arm -T ramdisk -C gzip -d rootfs.cpio.gz uramdisk.image.gz

# Copy the created image to sd_image
echo "Copying uramdisk.image.gz to ../sd_image"
cp uramdisk.image.gz ../sd_image/uramdisk.image.gz

